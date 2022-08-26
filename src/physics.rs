use bevy::prelude::*;
use serde::Deserialize;
use crate::Settings;
use core::ops::{Mul, Add};

const GRAVITY_CONSTANT: f32 = 0.03;
pub const DELTA_TIME: f64 = 0.005;

#[derive(Component, Default, Deserialize, Debug, Copy, Clone)]
pub struct Mass(pub f32);

#[derive(Reflect, Component, Default, Debug, Clone)]
pub struct Acceleration(pub Vec3);

#[derive(Reflect, Component, Default, Debug, Clone)]
pub struct Velocity(pub Vec3);

#[derive(Reflect, Component)]
pub struct Star;

#[derive(Component, Default, Clone)]
pub struct PredictedPath {
    pub pos_vec: Vec<Vec3>
}

#[derive(Reflect, Component, Default, Debug, Clone)]
pub struct PrevAcceleration(pub Vec3);

pub struct System {
    pub entity: Entity,
    pub mass: Mass,
    pub acc: Acceleration, 
    pub prev_acc: PrevAcceleration,
    pub pos: Vec3, 
    pub vel: Velocity, 
    pub path: PredictedPath,
}

#[derive(Component, Default)]
pub struct CelestialBody;

pub fn interact_bodies(
    mut query: Query<(Entity, &Mass, &GlobalTransform, &mut Acceleration)>,
    settings: Res<Settings>,
) {
    if !settings.play {
        return;
    }

    let mut system = Vec::new();
    for entity in &query {
        system.push(entity.0.clone());
    }

    // could be done more elegantly with iter_combinations_mut but this method is choosen to be as similar as possible to the estimate_paths function
    for index1 in 0..system.len() {
        for index2 in 0..index1 {
            // if index1 == index2 {
            //     continue;
            // }
            let s1 = query.get(system[index1]).unwrap();
            let s2 = query.get(system[index2]).unwrap();
            let mut acc1 = s1.3.clone();
            let mut acc2 = s2.3.clone();
            newtonian_gravity(
                &s1.1, &s1.2.translation(), &mut acc1,
                &s2.1, &s2.2.translation(), &mut acc2
            );
            query.get_mut(system[index1]).unwrap().3.0 = acc1.0;
            query.get_mut(system[index2]).unwrap().3.0 = acc2.0;
        }
    }


    // let mut iter = query.iter_combinations_mut();
    // while let Some(
    //     [
    //         (m1, transform1, mut acc1),
    //         (m2, transform2, mut acc2)
    //     ]) = iter.fetch_next() {
    //     newtonian_gravity(
    //         &m1, &transform1.translation(), &mut acc1,
    //         &m2, &transform2.translation(), &mut acc2
    //     );
    // }
}

fn newtonian_gravity(
    Mass(m1): &Mass, &pos1: &Vec3, acc1: &mut Acceleration,
    Mass(m2): &Mass, &pos2: &Vec3, acc2: &mut Acceleration,
) {
    let delta = pos2 - pos1;
    let distance_sq: f32 = delta.length_squared();

    let f = GRAVITY_CONSTANT / distance_sq;
    let force_unit_mass = delta * f;

    acc1.0 += force_unit_mass * (*m2);
    acc2.0 -= force_unit_mass * (*m1);

}

pub fn integrate(
    mut query: Query<(&mut Acceleration, &mut PrevAcceleration, &mut Transform, &mut Velocity)>,
    settings: Res<Settings>,
) {
    if !settings.play {
        return;
    }
    let dt_sq = (DELTA_TIME * DELTA_TIME) as f32;
    let dt = DELTA_TIME as f32;
    for(mut acceleration, mut prev_acceleration, mut transform, mut velocity) in &mut query {
        // let new_pos = transform.translation + velocity.0 * dt + acceleration.0 * (dt_sq * 0.5);
        // let new_acc = Vec3::ZERO;
        // let new_vel = velocity.0 + (acceleration.0 + prev_acceleration.0) * (dt * 0.5);

        let (new_pos, new_vel, new_acc) = velocity_verlet(
            dt,
            dt_sq,
            acceleration.0,
            prev_acceleration.0,
            transform.translation,
            velocity.0
        );

        prev_acceleration.0 = acceleration.0;
        transform.translation = new_pos;
        acceleration.0 = new_acc;
        velocity.0 = new_vel;
    }
}

pub fn estimate_paths(
    mut query: Query<(Entity, &Mass, &Acceleration, &PrevAcceleration, &Transform, &Velocity, &mut PredictedPath)>,
    settings: Res<Settings>,
) {
    let mut system: Vec<System> = vec![];
    for(entity, mass, acceleration, prev_acceleration, transform, velocity, mut path) in &mut query {
        path.pos_vec.clear();

        system.push(System {
            entity,
            mass: mass.clone(),
            pos: transform.translation.clone(),
            vel: velocity.clone(),
            acc: acceleration.clone(),
            prev_acc: prev_acceleration.clone(),
            path: PredictedPath{ pos_vec: vec![] },
        });
    }

    for i in 0..(settings.trail_length * settings.trail_interval) {
        for index1 in 0..system.len() {
            for index2 in 0..index1 {
                // if index1 == index2 {
                //     continue;
                // }
                let s1 = &system[index1];
                let s2 = &system[index2];
                let mut acc1 = s1.acc.clone();
                let mut acc2 = s2.acc.clone();
                newtonian_gravity(
                    &s1.mass, &s1.pos, &mut acc1,
                    &s2.mass, &s2.pos, &mut acc2
                );
                system.get_mut(index1).unwrap().acc = acc1;
                system.get_mut(index2).unwrap().acc = acc2;
            }
        }

        // update pos, vel

        let dt = DELTA_TIME as f32;
        let dt_sq = (DELTA_TIME * DELTA_TIME) as f32;
        for s in &mut system {
            let (new_pos, new_vel, new_acc) = velocity_verlet(
                dt,
                dt_sq,
                s.acc.0,
                s.prev_acc.0,
                s.pos,
                s.vel.0
            );
            s.prev_acc.0 = s.acc.0;
            s.pos = new_pos;
            s.acc.0 = new_acc;
            s.vel.0 = new_vel;


            if (i % settings.trail_interval) == 0 {
                s.path.pos_vec.push(new_pos);
            }
        }
    }

    for s in &system {
        let (_, _, _, _, _, _, mut path) = query.get_mut(s.entity).unwrap();
        path.pos_vec.clone_from(&s.path.pos_vec);
    }

}

fn velocity_verlet(
    dt: f32,
    dt_sq: f32,
    acceleration: Vec3,
    prev_acceleration: Vec3,
    pos: Vec3,
    velocity: Vec3,
) -> (Vec3, Vec3, Vec3) {
    let new_pos = pos + velocity * dt + acceleration * (dt_sq * 0.5);
    let new_vel = velocity + (acceleration + prev_acceleration) * (dt * 0.5);
    let new_acc = Vec3::ZERO;

    return (new_pos, new_vel, new_acc);
}

fn runge_kutta_4_nystttrom<F, Num, Fl>(
    f: F,
    dt: f32,
    acceleration: Num,
    prev_acceleration: Num,
    pos: Num,
    velocity: Num,
) -> f32
where 
    Num: Mul<f32> + Add<Num>,
    F: Fn(Num, Num, Num) -> Num
{
    todo!();    
}

#[allow(dead_code)]
fn runge_kutta_4<F, Num>(f: F, h: f32, x0: Num, y0: Num) -> Num
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
