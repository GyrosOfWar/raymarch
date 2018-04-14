use nalgebra::*;

pub type Point = Point3<f32>;
pub type Vector = Vector3<f32>;

#[derive(Clone, Copy, Debug)]
pub struct Ray {
    pub origin: Point,
    pub direction: Vector,
}

impl Ray {
    #[inline]
    pub fn new(ro: Point, rd: Vector) -> Ray {
        Ray {
            origin: ro,
            direction: rd,
        }
    }

    #[inline]
    pub fn eval(&self, t: f32) -> Point {
        self.origin + self.direction * t
    }
}
