use crate::integration::{Integrator, SecondOrderFunction, Vectorspace};
use std::iter::zip;

#[inline(always)]
pub fn simplectic_integrator_constructor<F, Num, const N: usize>(
    ci: [f32; N], di: [f32; N] //TODO make these generic floats 
) -> impl Integrator<F, Num>
    where 
        F: SecondOrderFunction<Num>,
        Num: Vectorspace<Num> 
{
    move |f: F, h: f32,  t0: f32, y0: &Num, dy0: &Num| -> (Num, Num) {
        let mut y = y0 +  dy0 * (ci[0] * h);
        let ddy0 = f(t0, dy0, &y);
        let mut dy = dy0 + ddy0 * (di[0] * h);
        for (c, d) in zip(&ci, &di).skip(1) {
            y = y + &dy * (c * h); 
            let ddy = f(t0 + h * c, &dy, &y);
            dy = dy + ddy * (h * d); 
        }
        (y, dy)
    }
}
