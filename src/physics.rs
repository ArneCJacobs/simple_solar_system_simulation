use bevy::{prelude::*, ecs::query::WorldQuery};
use itertools::izip;
use crate::Settings;
use bevy_inspector_egui::Inspectable;
use core::ops::{Add, Mul};
use ndarray::{Ix1, Array};
use std::iter::zip;


const GRAVITY_CONSTANT: f32 = 0.03;
pub const DELTA_TIME: f64 = 0.01;


#[derive(Component, Reflect)]
pub struct Star;

#[derive(Component, Default, Clone)]
pub struct PredictedPath {
    pub pos_vec: Vec<Vec3>
}

#[derive(Reflect, Inspectable, Component, Default, Debug, Clone)]
#[reflect(Component)]
pub struct CelestialBody {
    pub mass: f32,
    pub vel: Vec3, 
    pub acc: Vec3, 
    pub prev_acc: Vec3,
}

pub struct Consts {
    pub mass: f32,
    pub entity: Entity,
}


pub struct CelestialBodyCalcBody {
    entity: Entity,
    celestial_body: CelestialBody,
    pos: Transform,
}

trait CalcBody<'a> {
    fn fields(&'a mut self) -> (Entity, &'a mut Vec3, &'a mut CelestialBody);
}

impl From<&CelestialBodyCalcBody> for Consts {
    fn from(cb: &CelestialBodyCalcBody) -> Self {
        Consts {
            entity: cb.entity.clone(),
            mass: cb.celestial_body.mass,
        } 
    }
}


impl From<CelestialBodyQueryItem<'_>> for CelestialBodyCalcBody {
   fn from(item: CelestialBodyQueryItem<'_>) -> Self {
       CelestialBodyCalcBody { 
           entity: item.entity.clone(), 
           celestial_body: item.celestial_body.clone(), 
           pos: item.pos.clone(),
       }
   } 
}

impl<'a> CalcBody<'a> for CelestialBodyCalcBody {
    fn fields(&'a mut self) -> (Entity, &'a mut Vec3, &'a mut CelestialBody) {
        (self.entity, &mut self.pos.translation, &mut self.celestial_body)
    }
}

impl<'a> CalcBody<'a> for CelestialBodyQueryItem<'_> {
    fn fields(&'a mut self) -> (Entity, &'a mut Vec3, &'a mut CelestialBody) {
        (self.entity, &mut self.pos.translation, &mut self.celestial_body)
    }
}

trait StepCalcBody<'a> {
    fn interact(&'a mut self, other: &'a mut Self);
    fn integrate(&'a mut self, dt: f32, f: impl Integrator);
}

fn interact_gravity(
    pos1: &Vec3, _vel1: &Vec3, acc1: &mut Vec3, _consts1: &Consts,
    pos2: &Vec3, _vel2: &Vec3, consts2: &Consts,
) {
    let delta = *pos2 - *pos1;
    let distance_sq: f32 = delta.length_squared();

    let f = GRAVITY_CONSTANT / distance_sq;
    let force_unit_mass = delta * f;
    *acc1 += force_unit_mass * consts2.mass;
    // cb2.acc -= force_unit_mass * consts1.mass;
    
}

#[derive(WorldQuery)]
#[world_query(mutable)]
pub struct CelestialBodyQuery {
    entity: Entity,
    celestial_body: &'static mut CelestialBody,
    pos: &'static mut Transform, 
}
trait Integrator = Fn(f32, Vec3, Vec3, Vec3, Vec3) -> (Vec3, Vec3);

pub fn step_system(
    mut query: Query<CelestialBodyQuery>,
    settings: Res<Settings>,
    mut time_passed: Local<f32>
) {
    if !settings.play {
        return;
    }

    let dt = DELTA_TIME as f32;

    *time_passed += dt;
    let mut bodies: Vec<CelestialBodyCalcBody> = vec![]; 

    for body in &mut query {
        bodies.push(body.into());
    }

    let mut system = System::new(&bodies);
    let (new_poss, new_vels) = system.step(dt, *time_passed);
    for (body, new_pos, new_vel) in izip!(bodies, new_poss, new_vels) {
        let mut query_body = query.get_mut(body.entity).unwrap();
        query_body.celestial_body.vel = new_vel;
        query_body.pos.translation = new_pos;
    }
}


pub fn estimate_paths(
    mut query: Query<CelestialBodyQuery>,
    mut paths_query: Query<&mut PredictedPath>,
    settings: Res<Settings>,
) {
    let mut bodies: Vec<CelestialBodyCalcBody> = vec![];
    let mut paths: Vec<PredictedPath> = vec![];

    let dt = DELTA_TIME as f32;
    let time_passed = 0.0; // TODO make time_passed a resource and update it from step_system

    for body in &mut query {
        bodies.push(body.into());
        paths.push(PredictedPath{ pos_vec: vec![]});
    }

    let mut system = System::new(&bodies);

    for i in 0..(settings.trail_length * settings.trail_interval) {
        let (new_poss, _) = system.step(dt, time_passed);

        for (path, new_pos) in zip(&mut paths, new_poss) { 
            if (i % settings.trail_interval) == 0 {
                path.pos_vec.push(new_pos);
            }
        }
    }

    for (body, path) in zip(&mut bodies, &paths) { 
        let mut query_path = paths_query.get_mut(body.entity).unwrap();
        query_path.pos_vec.clear();
        query_path.pos_vec.clone_from(&path.pos_vec);
    }
}


type Vector<T> = Array<T, Ix1>;
struct System {
    pos_vec: Vector<Vec3>,
    vel_vec: Vector<Vec3>,
    consts_vec: Vector<Consts>,
}

impl System {
    fn new(bodies: &Vec<CelestialBodyCalcBody>) -> Self {
        let pos_vec = Array::from_iter(bodies.iter().map(|body| body.pos.translation));
        let vel_vec = Array::from_iter(bodies.iter().map(|body| body.celestial_body.vel));
        let consts_vec = Array::from_iter(bodies.iter().map(|body| body.into()));
        System {
            pos_vec,
            vel_vec,
            consts_vec,
        }
    }

    fn step(&mut self, dt: f32, time_passed: f32) -> (Vector<Vec3>, Vector<Vec3>) {
        let integrate_func = |_t: f32, vels: &Vector<Vec3>, poss: &Vector<Vec3>| -> Vector<Vec3> {

            let mut accs = Vector::from_elem(poss.raw_dim(), Vec3::ZERO);
            for (index1, (pos1, vel1, mut acc1, consts1)) in izip!(poss, vels, &mut accs, &self.consts_vec).enumerate() {
                for (index2, (pos2, vel2, consts2)) in izip!(poss, vels, &self.consts_vec).enumerate() {
                    if index1 == index2 {
                        continue;
                    }
                    interact_gravity(
                        &pos1, &vel1, &mut acc1, &consts1,
                        &pos2, &vel2, &consts2,
                    ); 
                }
            }
            accs   
        };

        
        let (new_pos, new_vel) = runge_kutta_4_nystorm(
            integrate_func, dt, time_passed, &self.pos_vec, &self.vel_vec
        );
        self.pos_vec = new_pos.clone();
        self.vel_vec = new_vel.clone();
        return (new_pos, new_vel);
    }
}

// TODO make this interface a trait or type
fn runge_kutta_4_nystorm<F, Num>(
    f: F, h: f32 /* dt */,  t0: f32, y0: &Num, dy0: &Num
) -> (Num, Num)
where 
    for<'a> Num: Add<Output=Num> + Add<&'a Num, Output=Num> + Add<f32, Output=Num> + Mul<f32, Output=Num>,
    for<'c> &'c Num: Add<Num, Output=Num> + Add<&'c Num, Output=Num> + Mul<f32, Output=Num>,
    F: for<'b> Fn(f32, &'b Num, &'b Num) -> Num, // f(t, x, x') = x'' or f(time, value, first-derivative) = second-derivative
{
    let k1 = f(t0, &dy0, &y0);

    let dy1 = dy0 + &k1 * (h * 0.5); 
    let y1 = y0 + ((dy0 + &dy1) * 0.5) * (h * 0.5);
    let k2 = f(t0 + h * 0.5, &dy1, &y1);

    let dy2 = dy0 + &k2 * (h * 0.5);
    let y2 = y0 + (dy0 + &dy2) * 0.5 * (h * 0.5); 
    let k3 = f(t0 + h * 0.5, &dy2, &y2);

    let dy3 = dy0 + &k3 * h;
    let y3 = y0 + (dy0 + &dy3) * (h * 0.5);
    let k4 = f(t0 + h, &dy3, &y3);

    let dy = dy0 + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (h / 6.0);
    let y = y0 + (dy1 + dy2 * 2.0 + dy3 * 2.0 + y3) * (h / 6.0);

    (y, dy)
}

#[allow(dead_code)]
fn runge_kutta_4<F, Num>(
    f: F, 
    h: f32, 
    x0: Num, 
    y0: Num
) -> Num
where
    Num: Mul<f32, Output = Num> + Add<Num, Output=Num> + Add<f32, Output=Num> + Copy,
    F: Fn(Num, Num) -> Num,
{
    let k1 = f(x0, y0);
    let k2 = f(x0 + h * 0.5, y0 + k1 * 0.5 * h);
    let k3 = f(x0 + 0.5 * h, y0 + k1 * 0.5 * h);
    let k4 = f(x0 + h, y0 + k3 * h);
    y0 + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (h / 6.0)
}

#[allow(dead_code)]
fn velocity_verlet(
    dt: f32,
    pos: Vec3,
    vel: Vec3,
    acc: Vec3,
    prev_acc: Vec3,
) -> (Vec3, Vec3) {
    let dt_sq = dt*dt;
    let new_pos = pos + vel * dt + acc * (dt_sq * 0.5);
    let new_vel = vel + (acc + prev_acc) * (dt * 0.5);
    return (new_pos, new_vel);
}
