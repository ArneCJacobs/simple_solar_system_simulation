use core::ops::{Add, Mul};

pub mod runge_kutta;
pub mod velocity_verlet;
pub mod simplectic_integrator_constructor;
pub mod ruth_fourth_order;


pub trait Vectorspace<Num> = Sized
where
    for<'a> Num: Add<f32, Output=Num> + Mul<f32, Output=Num> + Add<Output=Num> + Add<&'a Num, Output=Num>, 
    for<'b> &'b Num: Add<Num, Output=Num> + Add<&'b Num, Output=Num> + Mul<f32, Output=Num>;

pub trait SecondOrderFunction<Num> = for<'a> Fn(f32, &'a Num, &'a Num) -> Num;  
pub trait Integrator<F, Num> = Fn(F, f32, f32, &Num,  &Num) -> (Num, Num)
    where
        Num: Vectorspace<Num>,
        F: SecondOrderFunction<Num>;

