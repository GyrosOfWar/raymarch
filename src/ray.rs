use nalgebra::*;

pub type Point = Pnt3<f32>;
pub type Vector = Vec3<f32>;

pub struct Ray {
    pub origin: Point,
    pub direction: Vector
}

impl Ray {
    pub fn new(ro: Point, rd: Vector) -> Ray {
        Ray {
            origin: ro,
            direction: rd
        }
    }

    pub fn eval(&self, t: f32) -> Point {
        self.origin + self.direction * t
    }
}
