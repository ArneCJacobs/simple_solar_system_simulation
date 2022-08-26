use bevy::{prelude::*, ecs::query::WorldQuery};
use crate::Settings;
use core::ops::{Mul, Add};

const GRAVITY_CONSTANT: f32 = 0.03;
pub const DELTA_TIME: f64 = 0.005;


#[derive(Reflect, Component)]
pub struct Star;

#[derive(Component, Default, Clone)]
pub struct PredictedPath {
    pub pos_vec: Vec<Vec3>
}

#[derive(Reflect, Component, Default, Debug, Clone)]
pub struct CelestialBody {
    pub mass: f32,
    pub vel: Vec3, 
    pub acc: Vec3, 
    pub prev_acc: Vec3,
}
//
// impl CelestialBody {
//     pub fn interact(&mut self, other: &mut Self) {
//         todo!(); 
//     }
// }


#[derive(WorldQuery)]
#[world_query(mutable)]
pub struct CelestialBodyQuery {
    entity: Entity,
    celestial_body: &'static mut CelestialBody,
    pos: &'static mut Transform, 
}

impl Clone for CelestialBodyQueryItem<'_> {
    fn clone(&self) -> Self {
        CelestialBodyQueryItem {
            entity: self.entity.clone(),
            celestial_body: self.celestial_body.clone(),
            pos: self.pos.clone()
        }
    }
}

trait Integrator = Fn(f32, Vec3, Vec3, Vec3, Vec3) -> (Vec3, Vec3);

impl<'w> CelestialBodyQueryItem<'w> {
    fn interact(&mut self, other: &mut Self) {
        let (new_acc_self, new_acc_other) = newtonian_gravity(
            &self.mass, &self.pos.translation, &self.acc, 
            &other.mass, &other.pos.translation, &other.acc, 
        );
        self.acc = new_acc_self;
        other.acc = new_acc_other;
    }

    fn integrate(&mut self, dt: f32, f: impl Integrator) {
        let (new_pos, new_vel) = f(
            dt, self.pos.translation, self.vel, self.acc, self.prev_acc 
        );

        self.prev_acc = self.acc;
        self.pos.translation = new_pos;
        self.acc = Vec3::ZERO;
        self.vel = new_vel;
    }
}

use std::ops::{Deref, DerefMut};


impl<'w> Deref for CelestialBodyQueryItem<'w> {
    type Target = CelestialBody;

    fn deref(&self) -> &Self::Target {
        &self.celestial_body
    }
}

impl<'w> DerefMut for CelestialBodyQueryItem<'w> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.celestial_body
    }
}



pub fn interact_bodies(
    mut query: Query<CelestialBodyQuery>,
    settings: Res<Settings>,
) {
    if !settings.play {
        return;
    }

    let mut iter = query.iter_combinations_mut();
    while let Some([mut body1, mut body2]) = iter.fetch_next() {
        body1.interact(&mut body2);
    }
}


pub fn integrate(
    mut query: Query<CelestialBodyQuery>,
    settings: Res<Settings>,
) {
    if !settings.play {
        return;
    }
    let dt = DELTA_TIME as f32;
    for mut entry in &mut query {
        entry.integrate(dt, velocity_verlet);
    }
}

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
    let mut system: Vec<CelestialBodyQueryItem> = vec![];
    let mut paths: Vec<PredictedPath> = vec![];

    for body in &mut query {
        system.push(body.clone());
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

fn newtonian_gravity(
    m1: &f32, &pos1: &Vec3, acc1: &Vec3,
    m2: &f32, &pos2: &Vec3, acc2: &Vec3,
) -> (Vec3, Vec3){
    let delta = pos2 - pos1;
    let distance_sq: f32 = delta.length_squared();
    let force_dir = delta.normalize();
    let force = force_dir * GRAVITY_CONSTANT * (*m1) * (*m2) / distance_sq;

    let new_acc1 = *acc1 + force * (*m2) / (*m1);
    let new_acc2 = *acc2 - force * (*m1) / (*m1);
    (new_acc1, new_acc2)
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
#[allow(dead_code)]
fn runge_kutta_4_nystorm<F, Num>
(
    f: F,
    h: f32,
    t0: Num,
    y0: Num,
    dy0: Num,
) -> (Num, Num)
where 
    Num: Mul<f32, Output = Num> + Add<Num, Output=Num> + Add<f32, Output=Num> + Copy,
    F: Fn(Num, Num, Num) -> Num,
{
    let k1 = f(t0, dy0, y0);

    let dy1 = dy0 + k1 * (h * 0.5); 
    let y1 = y0 + ((dy0 + dy1) * 0.5) * (h * 0.5);
    let k2 = f(t0 + h * 0.5, dy1, y1);

    let dy2 = dy0 + dy0 * (h * 0.5);
    let y2 = y0 + (dy0 + dy2) * 0.5 * (h * 0.5); 
    let k3 = f(t0 + h * 0.5, dy2, y2);

    let dy3 = dy0 + k3 * h;
    let y3 = y0 + (dy0 + dy3) * (h * 0.5);
    let k4 = f(t0 + h, dy3, y3);

    let dy = dy0 + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (h / 6.0);
    let y = y0 + (dy1 + dy2 * 2.0 + dy3 * 2.0 + y3) * (h / 6.0);

    (dy, y)
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
