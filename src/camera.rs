use nalgebra::*;
use ray::{Ray, Point, Vector};
use std::fmt;
use rand::{Rng, thread_rng, ThreadRng};

pub trait Camera: fmt::Debug {
    fn samples(&mut self, x: u32, y: u32) -> Vec<Ray>;
}

#[derive(Debug)]
pub struct OrthographicCamera {
    eye: Point,
    right: Vector,
    up: Vector,
    x_res: f32,
    y_res: f32,
    sampler: Box<Sampler>
}

impl OrthographicCamera {
    pub fn new(eye: Point, right: Vector, up: Vector, x_res: f32, y_res: f32, sampler: Box<Sampler>) -> OrthographicCamera {
        OrthographicCamera {
            eye: eye,
            right: right,
            up: up,
            x_res: x_res,
            y_res: y_res, 
            sampler: sampler
        }
    }
}

impl Camera for OrthographicCamera {
    #[inline]
    fn samples(&mut self, x: u32, y: u32) -> Vec<Ray> {
        self.sampler.get_samples(x, y)
          .iter()
          .map(|&(u, v)| {
	        let (u, v) = pixel_to_ndc(x, y, self.x_res, self.y_res);
	        let ro = self.eye + self.right * u + self.up * v;
	        let rd = self.right.cross(&self.up).normalize();
	        Ray::new(ro, rd)
        }).collect()
    }
}

#[inline]
fn pixel_to_ndc(x: u32, y: u32, x_res: f32, y_res: f32) -> (f32, f32) {
	let u = x as f32 * 2.0 / x_res - 1.0;
	let v = x as f32 * 2.0 / y_res - 1.0;
	(u, v)
}

pub type Sample = (f32, f32);

// TODO change to using iterators
pub trait Sampler: fmt::Debug {
    fn get_samples(&mut self, px: u32, py: u32) -> Vec<Sample>;
}

#[derive(Debug)]
pub struct SimpleSampler {
    pub x_res: f32,
    pub y_res: f32
}

impl Sampler for SimpleSampler {
    fn get_samples(&mut self, px: u32, py: u32) -> Vec<Sample> {
        vec![pixel_to_ndc(px, py, self.x_res, self.y_res)]
    }
}

pub struct StratifiedSampler {
    rng: ThreadRng,
    x_res: f32,
    y_res: f32,
    samples_per_pixel: usize
}

impl StratifiedSampler {
    pub fn new(x_res: f32, y_res: f32, spp: usize) -> StratifiedSampler {
        StratifiedSampler {
            rng: thread_rng(),
            x_res: x_res,
            y_res: y_res,
            samples_per_pixel: spp
        }
    }
}

impl Sampler for StratifiedSampler {
    fn get_samples(&mut self, px: u32, py: u32) -> Vec<Sample> {
        vec![]
    }
}

impl fmt::Debug for StratifiedSampler {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "x_res: {}, y_res: {}, samples_per_pixel: {}", 
            self.x_res, self.y_res, self.samples_per_pixel)
    }
}