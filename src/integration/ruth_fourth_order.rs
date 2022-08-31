use crate::integration::{SecondOrderFunction, Vectorspace};
use super::simplectic_integrator_constructor::simplectic_integrator_constructor;

pub fn ruth_fourth_order<F, Num>(
    f: F, h: f32 /* dt */,  t0: f32, y0: &Num, dy0: &Num
) -> (Num, Num)
where 
    Num: Vectorspace<Num>,
    F: SecondOrderFunction<Num>, // f(t, x', x) = x'' or f(time, first-derivative, value) = second-derivative
{
    let c1 = 1.0 / (2.0 * (2.0 - 2f32.powf(1./3.)));
    let c2 = (1. - 2.0f32.powf(1./3.)) / (2. * (2. - 2f32.powf(1./3.)));
    let c3 = c2;
    let c4 = c1;

    let d1 = 1. / (2. - 2f32.powf(1./3.));
    let d3 = d1;
    let d2 = - (2f32.powf(1./3.)) / (2. - 2f32.powf(1./3.));
    let d4 = 0.;
    let func = simplectic_integrator_constructor(
        [c1, c2, c3, c4], 
        [d1, d2, d3, d4]
    );

    func(f, h, t0, y0, dy0)
}
