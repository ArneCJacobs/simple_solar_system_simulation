use crate::integration::{SecondOrderFunction, Vectorspace};

use super::simplectic_integrator_constructor::simplectic_integrator_constructor;

#[allow(dead_code)]
pub fn velocity_verlet<F, Num>(
    f: F, h: f32 /* dt */,  t0: f32, y0: &Num, dy0: &Num
) -> (Num, Num)
where 
    Num: Vectorspace<Num>,
    F: SecondOrderFunction<Num>, // f(t, x', x) = x'' or f(time, first-derivative, value) = second-derivative
{
    let ddy0 = f(t0, dy0, y0);
    let dy1_2 = dy0 + ddy0 * (h * 0.5); // v(t + 1/2 dt)
    let y = y0 + &dy1_2 * h;
    // let dy_approx = y 
    let ddy = f(t0 + h, &dy1_2, &y); // probably an approximation of dy should be given here instead of dy1_2 
    let dy = dy1_2 + ddy * (h * 0.5);

    (y, dy)
}

// https://en.wikipedia.org/wiki/Symplectic_integrator#Examples
#[allow(dead_code)]
pub fn velocity_verlet_generic<F, Num>(
    f: F, h: f32 /* dt */,  t0: f32, y0: &Num, dy0: &Num
) -> (Num, Num)
where 
    Num: Vectorspace<Num>,
    F: SecondOrderFunction<Num>, // f(t, x', x) = x'' or f(time, first-derivative, value) = second-derivative
{
    simplectic_integrator_constructor([0., 1.], [0.5, 0.5])(f, h, t0, y0, dy0)
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
