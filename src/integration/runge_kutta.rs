use crate::integration::{SecondOrderFunction, Vectorspace};

#[allow(dead_code)]
fn runge_kutta_4_nystorm<F, Num>(
    f: F, h: f32 /* dt */,  t0: f32, y0: &Num, dy0: &Num
) -> (Num, Num)
where 
    Num: Vectorspace<Num>,
    F: SecondOrderFunction<Num>, // f(t, x', x) = x'' or f(time, first-derivative, value) = second-derivative
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

