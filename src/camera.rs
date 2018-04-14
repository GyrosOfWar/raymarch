use rand::{thread_rng, Rng, ThreadRng};
use ray::{Point, Ray, Vector};
use std::fmt;

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
    sampler: Box<Sampler>,
}

impl OrthographicCamera {
    pub fn new(
        eye: Point,
        right: Vector,
        up: Vector,
        x_res: f32,
        y_res: f32,
        sampler: Box<Sampler>,
    ) -> OrthographicCamera {
        OrthographicCamera {
            eye: eye,
            right: right,
            up: up,
            x_res: x_res,
            y_res: y_res,
            sampler: sampler,
        }
    }
}

impl Camera for OrthographicCamera {
    #[inline]
    fn samples(&mut self, x: u32, y: u32) -> Vec<Ray> {
        self.sampler
            .get_samples(x, y)
            .iter()
            .map(|&(u, v)| {
                //println!("Sampling at ({}, {})", u, v);
                let ro = self.eye + self.right * u + self.up * v;
                let rd = self.right.cross(&self.up).normalize();
                Ray::new(ro, rd)
            })
            .collect()
    }
}

#[inline]
fn pixel_to_ndc(x: f32, y: f32, x_res: f32, y_res: f32) -> (f32, f32) {
    let u = x * 2.0 / x_res - 1.0;
    let v = y * 2.0 / y_res - 1.0;
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
    pub y_res: f32,
}

impl Sampler for SimpleSampler {
    fn get_samples(&mut self, px: u32, py: u32) -> Vec<Sample> {
        vec![pixel_to_ndc(px as f32, py as f32, self.x_res, self.y_res)]
    }
}

pub struct StratifiedSampler {
    rng: ThreadRng,
    x_res: f32,
    y_res: f32,
    samples_per_pixel: usize,
    samples_per_side: usize,
    stride: f32,
}

impl StratifiedSampler {
    pub fn new(x_res: f32, y_res: f32, spp: usize) -> StratifiedSampler {
        if (spp as f32).sqrt() % 1.0 != 0.0 {
            panic!("Invalid sample count per pixel");
        }

        let samples_per_side = (spp as f32).sqrt() as usize;
        StratifiedSampler {
            rng: thread_rng(),
            x_res: x_res,
            y_res: y_res,
            samples_per_pixel: spp,
            samples_per_side: samples_per_side,
            stride: 1.0 / samples_per_side as f32,
        }
    }
}

impl Sampler for StratifiedSampler {
    fn get_samples(&mut self, px: u32, py: u32) -> Vec<Sample> {
        // Split up into n sub-regions
        // For each sub-region, select a sample within [x - 0.5, x + 0.5] X [y - 0.5, y + 0.5]
        let mut samples = Vec::new();
        let px = px as usize;
        let py = py as usize;
        for i in 0..self.samples_per_side {
            for j in 0..self.samples_per_side {
                let x = px as f32 + (i as f32 * self.stride);
                let y = py as f32 + (j as f32 * self.stride);
                let x_max = x + self.stride;
                let y_max = y + self.stride;

                let sx = self.rng.gen_range(x, x_max);
                let sy = self.rng.gen_range(y, y_max);
                samples.push(pixel_to_ndc(sx, sy, self.x_res, self.y_res));
            }
        }

        samples
    }
}

impl fmt::Debug for StratifiedSampler {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "x_res: {}, y_res: {}, samples_per_pixel: {}",
            self.x_res, self.y_res, self.samples_per_pixel
        )
    }
}
