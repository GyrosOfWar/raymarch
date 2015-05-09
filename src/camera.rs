#[allow(unused, dead_code)]

use nalgebra::*;
use ray::{Ray, Point, Vector};
use std::fmt;

pub trait Camera: fmt::Debug {
    fn ray(&self, u: f32, v: f32) -> Ray;
}

#[derive(Debug, Copy, Clone)]
pub struct OrthographicCamera {
    pub eye: Point,
    pub right: Vector,
    pub up: Vector
}

impl OrthographicCamera {
    pub fn new(eye: Point, right: Vector, up: Vector) -> OrthographicCamera {
        OrthographicCamera {
            eye: eye,
            right: right,
            up: up
        }
    }
}

impl Camera for OrthographicCamera {
    #[inline]
    fn ray(&self, u: f32, v: f32) -> Ray {        
        let ro = self.eye + self.right * u + self.up * v;
        let rd = self.right.cross(&self.up).normalize();
        Ray::new(ro, rd)
    }
}
