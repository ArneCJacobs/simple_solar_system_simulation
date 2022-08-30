use bevy::{prelude::*, ecs::query::WorldQuery};
use itertools::izip;
use crate::Settings;
use bevy_inspector_egui::Inspectable;
use core::ops::{Add, Mul};
use ndarray::{Ix1, Array};
use std::{iter::zip, f32::consts::PI};


const GRAVITY_CONSTANT: f32 = 0.01;
pub const DELTA_TIME: f64 = 0.0005;

pub fn calculate_circular_orbit_velocity(poss: &Vector<Vec3>, masses: &Vector<f32>) -> Vector<Vec3> {
    let mut baricenter = Vec3::ZERO; 
    for pos in poss {
        baricenter += *pos;
    }
    baricenter /= poss.len() as f32;
    let mut tau = Vector::from_elem(poss.raw_dim(), Vec3::ZERO);
    for (index1, (pos1, mass1, mut tau1)) in izip!(poss, masses, &mut tau).enumerate() {
        for (index2, (pos2, mass2)) in izip!(poss, masses).enumerate() {
            if index1 == index2 {
                continue;
            }
            let diff = *pos2 - *pos1;
            *tau1 += (mass2 / diff.length_squared()) * diff.normalize();
        }
    }
    println!("tau {:?}", tau);
    let dir = Vector::from_elem(poss.raw_dim(), baricenter) - poss;
    let r_sq = dir.mapv(|d| d.length_squared());
    let r = dir.mapv(|d| d.length());
    // TODO check if only one element in tau vector is nonzero, otherwise panic
    let other_mass = &r_sq * tau.map(|v| v.length());
    println!("other_mass: {:?}", other_mass);
    let vels_magnintude = ((masses + other_mass) / &r) * GRAVITY_CONSTANT;
    let vels = dir.mapv(|delta| delta.normalize());
    let mut vels = vels * &vels_magnintude;
    let rotation = Quat::from_rotation_z(PI / 2.0);    
    vels.mapv_inplace(|vel| rotation * vel);

    return vels;
}


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


fn interact_gravity(
    pos1: &Vec3, mass1: f32, acc1: &mut Vec3,
    pos2: &Vec3, mass2: f32,
) {
    // let delta = *pos2 - *pos1;
    // let distance_sq: f32 = delta.length_squared();
    // //
    // let f = GRAVITY_CONSTANT / distance_sq;
    // let force_unit_mass = delta * f;
    // *acc1 += force_unit_mass * mass2;
    // cb2.acc -= force_unit_mass * consts1.mass;
    let delta = *pos2 - *pos1;
    let length_sq = delta.length_squared();
    let force_dir = delta.normalize(); // TODO should be normalized
    let force = force_dir * GRAVITY_CONSTANT * (mass1 * mass2) / length_sq;
    *acc1 += force / mass1;
}

#[derive(WorldQuery)]
#[world_query(mutable)]
pub struct CelestialBodyQuery {
    entity: Entity,
    celestial_body: &'static mut CelestialBody,
    pos: &'static mut Transform, 
}

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


pub type Vector<T> = Array<T, Ix1>;
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
            for (index1, (pos1, _vel1, mut acc1, consts1)) in izip!(poss, vels, &mut accs, &self.consts_vec).enumerate() {
                for (index2, (pos2, _vel2, consts2)) in izip!(poss, vels, &self.consts_vec).enumerate() {
                    if index1 == index2 {
                        continue;
                    }
                    interact_gravity(
                        &pos1, consts1.mass, &mut acc1,
                        &pos2, consts2.mass,
                    ); 
                }
            }
            accs   
        };

        
        let (new_pos, new_vel) = velocity_verlet(
            integrate_func, dt, time_passed, &self.pos_vec, &self.vel_vec
        );
        self.pos_vec = new_pos.clone();
        self.vel_vec = new_vel.clone();
        return (new_pos, new_vel);
    }
}

trait Vectorspace<Num> = Sized
where
    for<'a> Num: Add<f32, Output=Num> + Mul<f32, Output=Num> + Add<Output=Num> + Add<&'a Num, Output=Num>, 
    for<'b> &'b Num: Add<Num, Output=Num> + Add<&'b Num, Output=Num> + Mul<f32, Output=Num>;

trait Integrator<Num> = for<'a> Fn(f32, &'a Num, &'a Num) -> Num;  

// TODO make this interface a trait or type
fn runge_kutta_4_nystorm<F, Num>(
    f: F, h: f32 /* dt */,  t0: f32, y0: &Num, dy0: &Num
) -> (Num, Num)
where 
    Num: Vectorspace<Num>,
    F: Integrator<Num>, // f(t, x', x) = x'' or f(time, first-derivative, value) = second-derivative
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

// #[allow(dead_code)]
// fn runge_kutta_4<F, Num>(
//     f: F, 
//     h: f32, 
//     x0: Num, 
//     y0: Num
// ) -> Num
// where
//     Num: Mul<f32, Output = Num> + Add<Num, Output=Num> + Add<f32, Output=Num> + Copy,
//     F: Fn(Num, Num) -> Num,
// {
//     let k1 = f(x0, y0);
//     let k2 = f(x0 + h * 0.5, y0 + k1 * 0.5 * h);
//     let k3 = f(x0 + 0.5 * h, y0 + k1 * 0.5 * h);
//     let k4 = f(x0 + h, y0 + k3 * h);
//     y0 + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (h / 6.0)
// }

fn velocity_verlet<F, Num>(
    f: F, h: f32 /* dt */,  t0: f32, y0: &Num, dy0: &Num
) -> (Num, Num)
where 
    Num: Vectorspace<Num>,
    F: Integrator<Num>, // f(t, x', x) = x'' or f(time, first-derivative, value) = second-derivative
{
    let ddy0 = f(t0, &dy0, y0);
    let dy1_2 = dy0 + ddy0 * (h * 0.5); // v(t + 1/2 dt)
    let y = y0 + &dy1_2 * h;
    // let dy_approx = y 
    let ddy = f(t0 + h, &dy1_2, &y); // probably an approximation of dy should be given here instead of dy1_2 
    let dy = dy1_2 + ddy * (h * 0.5);

    (y, dy)
}

// #[allow(dead_code)]
// fn velocity_verlet(
//     dt: f32,
//     pos: Vec3,
//     vel: Vec3,
//     acc: Vec3,
//     prev_acc: Vec3,
// ) -> (Vec3, Vec3) {
//     let dt_sq = dt*dt;
//     let new_pos = pos + vel * dt + acc * (dt_sq * 0.5);
//     let new_vel = vel + (acc + prev_acc) * (dt * 0.5);
//     return (new_pos, new_vel);
// }
