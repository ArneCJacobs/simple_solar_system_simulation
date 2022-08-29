use bevy::{prelude::*, ecs::query::WorldQuery, time::FixedTimesteps};
use itertools::izip;
use crate::Settings;
use bevy_inspector_egui::Inspectable;
use core::ops::{Add, Mul};
use ndarray::{Ix1, Array, Axis};


const GRAVITY_CONSTANT: f32 = 0.03;
pub const DELTA_TIME: f64 = 0.001;


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
    pos1: &Vec3, vel1: &Vec3, acc1: &mut Vec3, consts1: &Consts,
    pos2: &Vec3, vel2: &Vec3, consts2: &Consts,
) {
    let delta = *pos2 - *pos1;
    let distance_sq: f32 = delta.length_squared();

    let f = GRAVITY_CONSTANT / distance_sq;
    let force_unit_mass = delta * f;
    *acc1 += force_unit_mass * consts2.mass;
    // cb2.acc -= force_unit_mass * consts1.mass;
    
}


impl<'a, T: CalcBody<'a>> StepCalcBody<'a> for T {
    fn interact(&'a mut self, other: &'a mut Self) {
        let (_, &mut pos1, cb1) = self.fields();
        let (_, &mut pos2, cb2) = other.fields();

        let delta = pos2 - pos1;
        let distance_sq: f32 = delta.length_squared();

        let f = GRAVITY_CONSTANT / distance_sq;
        let force_unit_mass = delta * f;
        cb1.acc += force_unit_mass * cb2.mass;
        cb2.acc -= force_unit_mass * cb1.mass;
    }

    fn integrate(&'a mut self, dt: f32, f: impl Integrator) {
        let (_, pos, mut cb) = self.fields();
        let (new_pos, new_vel) = f(
            dt, *pos, cb.vel, cb.acc, cb.prev_acc 
        );

        cb.prev_acc = cb.acc;
        *pos = new_pos;
        cb.acc = Vec3::ZERO;
        cb.vel = new_vel;
    }
} 

#[derive(WorldQuery)]
#[world_query(mutable)]
pub struct CelestialBodyQuery {
    entity: Entity,
    celestial_body: &'static mut CelestialBody,
    pos: &'static mut Transform, 
}
trait Integrator = Fn(f32, Vec3, Vec3, Vec3, Vec3) -> (Vec3, Vec3);
//
//

pub fn step_system(
    mut query: Query<CelestialBodyQuery>,
    settings: Res<Settings>,
) {
    if !settings.play {
        return;
    }
    let mut system: Vec<CelestialBodyCalcBody> = vec![]; 

    for body in &mut query {
        system.push(body.into());
    }

    let mut system_vector: Array<CelestialBodyCalcBody, Ix1> = Array::from_vec(system);

    let mut iter = query.iter_combinations_mut();
    while let Some([mut body1, mut body2]) = iter.fetch_next() {
        body1.interact(&mut body2);
    }

    let dt = DELTA_TIME as f32;
    // *time_passed += dt;
    // let closure = |f32, Vec3, Vec3, Vec3, Vec3| -> (Vec3, Vec3) { 
    //     runge_kutta_4_nystorm(f, h, t0, y0, dy0)
    // }
    for mut entry in &mut query {
        entry.integrate(dt, velocity_verlet);
    }
}
//
//
// pub fn integrate(
//     mut query: Query<CelestialBodyQuery>,
//     settings: Res<Settings>,
//     mut time_passed: Local<f32>,
// ) {
//     if !settings.play {
//         return;
//     }
// }

trait MultipleMut<T> {
    fn get_both_mut(&mut self, i: usize, j: usize) -> [&mut T; 2]; 
}

impl<T> MultipleMut<T> for Vec<T> {
    fn get_both_mut(&mut self, i: usize, j: usize) -> [&mut T; 2] {
        if i < j {
            let (head, tail) = self.split_at_mut(j);
            return [&mut head[i], &mut tail[0]];
        } else {
            let (head, tail) = self.split_at_mut(i);
            return [&mut tail[0], &mut head[j]];
        }
    }
}


use std::iter::zip;

pub fn estimate_paths(
    mut query: Query<CelestialBodyQuery>,
    mut paths_query: Query<&mut PredictedPath>,
    settings: Res<Settings>,
) {
    let mut system: Vec<CelestialBodyCalcBody> = vec![];
    let mut paths: Vec<PredictedPath> = vec![];

    for body in &mut query {
        system.push(body.into());
        paths.push(PredictedPath{ pos_vec: vec![]});
    }
    for i in 0..(settings.trail_length * settings.trail_interval) {
        for index1 in 0..system.len() {
            for index2 in 0..index1 {
                let [body1, body2] = system.get_both_mut(index1, index2);
                body1.interact(body2);
            }
        }

        for (body, path) in zip(&mut system, &mut paths) { 
            body.integrate(DELTA_TIME as f32, velocity_verlet);

            if (i % settings.trail_interval) == 0 {
                path.pos_vec.push(body.pos.translation);
            }
        }
    }

    for (body, path) in zip(&mut system, &paths) { 
        // path.pos_vec.clear();
        let mut query_path = paths_query.get_mut(body.entity).unwrap();
        query_path.pos_vec.clear();
        query_path.pos_vec.clone_from(&path.pos_vec);
    }
}

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

type Vector<T> = Array<T, Ix1>;
struct System {
    pos_vec: Vector<Vec3>,
    vel_vec: Vector<Vec3>,
    consts_vec: Vector<Consts>,
}

impl System {
    fn new(bodies: Vec<CelestialBodyCalcBody>) -> Self {
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
        let integrate_func = |_t: f32, poss: &Vector<Vec3>, vels: &Vector<Vec3>| -> Vector<Vec3> {
            let mut accs = Vector::from_elem(poss.raw_dim(), Vec3::ZERO);
            for (pos1, vel1, mut acc1, consts1) in izip!(poss, vels, &mut accs, &self.consts_vec) {
                for (pos2, vel2, consts2) in izip!(poss, vels, &self.consts_vec) {
                    interact_gravity(
                        &pos1, &vel1, &mut acc1, &consts1,
                        &pos2, &vel2, &consts2,
                    ); 
                }
            }
            accs   
        };

        runge_kutta_4_nystorm(
            integrate_func, dt, time_passed, &self.pos_vec, &self.vel_vec
        )
    }
}

fn test_integration() {
    let pos_vec: Vec<Vec3> = vec![Vec3::ZERO, Vec3::ZERO, Vec3::ZERO];
    let vector: Array<Vec3, Ix1> = Array::from_vec(pos_vec);
    let closure = |_dt: f32, poss: &Array<Vec3, Ix1>, _vels: &Array<Vec3, Ix1>| -> Array<Vec3, Ix1> { 
        Array::from_elem(poss.raw_dim(), Vec3::ZERO)
    }; 
    let dummy_pos = vector.clone();
    let dummy_vel = vector.clone();
    let dummy_dtime = 0.0;
    let (_new_pos, _new_vel) = runge_kutta_4_nystorm(
        closure,
        dummy_dtime, 
        0.0, 
        &dummy_pos, 
        &dummy_vel,
    );
}


#[allow(dead_code)]
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
