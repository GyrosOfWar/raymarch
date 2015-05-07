//extern crate "nalgebra" as na;
extern crate nalgebra;
extern crate image;

use image::{ImageBuffer, Rgb, Pixel};
use nalgebra::*;
use std::path::Path;

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

pub type Point = Pnt3<f32>;
pub type Vector = Vec3<f32>;

pub const MAX_STEPS: usize = 100;
pub const EPSILON: f32 = 0.00001;

pub struct RayMarcher {
    image: ImageBuffer<Rgb<u8>, Vec<u8>>,
    x_size: usize,
    y_size: usize,
    camera: Camera,
    avg_iters: f32,
    distance_estimator: Vec<DistanceEstimator>
}

impl RayMarcher {
    pub fn new(x_size: usize, y_size: usize, camera: Camera, distance_estimator: Vec<DistanceEstimator>) -> RayMarcher {
        RayMarcher {
            image: ImageBuffer::new(x_size as u32, y_size as u32),
            x_size: x_size,
            y_size: y_size,
            camera: camera,
            avg_iters: 0.0,
            distance_estimator: distance_estimator
        }
    }

    pub fn render(&mut self) {
        let x_res = self.x_size as f32;
        let y_res = self.y_size as f32;
        let mut iter_counts: Vec<usize> = Vec::new();
        for (x, y, pixel) in self.image.enumerate_pixels_mut() {
            let u = (x as f32) * 2.0 / x_res - 1.0;
            let v = (y as f32) * 2.0 / y_res - 1.0;
            let ray = self.camera.ray(u, v);
            let count = RayMarcher::march_ray(ray, pixel, &self.distance_estimator);
            iter_counts.push(count);
        }
        let sum: usize = iter_counts.iter().fold(0, |sum, next| sum + next);
        self.avg_iters = sum as f32 / iter_counts.len() as f32;
    }

    #[inline]
    fn eval_distance(current: Point, dist: &Vec<DistanceEstimator>) -> f32 {
        let values = dist.iter().map(|d| d.eval(current));
        let mut d = std::f32::INFINITY;
        for i in values {
            if i < d {
                d = i;
            }
        }
	d
    }

    #[inline]
    fn march_ray(ray: Ray, pixel: &mut Rgb<u8>, dist: &Vec<DistanceEstimator>) -> usize {
        let mut t = 0.0;
        for step in 0..MAX_STEPS {
	    let current = ray.eval(t);
            let d = RayMarcher::eval_distance(current, dist);
	    if d < EPSILON {
                let brightness = 255 - (((step as f32) / (MAX_STEPS as f32)) * 256.0) as u8;
                *pixel = Rgb { data: [brightness, brightness, brightness] };
	        return step;
	    }
	    t += d;
        }
        MAX_STEPS
    }
}

pub struct Camera {
    eye: Point,
    right: Vector,
    up: Vector
}

impl Camera {
    pub fn zero() -> Camera {
        Camera {
            eye: Pnt3::new(0.0, 0.0, -1.0),
            right: Vec3::new(1.0, 0.0, 0.0),
            up: Vec3::new(0.0, 1.0, 0.0)
        }
    }

    pub fn new(eye: Point, right: Vector, up: Vector) -> Camera {
        Camera {
            eye: eye,
            right: right,
            up: up
        }
    }

    #[inline]
    pub fn ray(&self, u: f32, v: f32) -> Ray {
        // let ro = self.eye.add_v(&self.right.mul_s(u).add_v(&self.up.mul_s(v)));
        // let rd = self.right.cross(&self.up).normalize();
        let ro = self.eye + self.right * u + self.up * v;
        let rd = self.right.cross(&self.up).normalize();
        Ray::new(ro, rd)
    }
}

pub type EstimatorFunc = Box<Fn(Point, &[f32]) -> f32>;

pub struct DistanceEstimator {
    params: Vec<f32>,
    func: EstimatorFunc
}

impl DistanceEstimator {
    pub fn new(params: Vec<f32>, f: EstimatorFunc) -> DistanceEstimator {
        DistanceEstimator {
            params: params,
            func: f
        }
    }

    pub fn sphere_estimator(radius: f32) -> DistanceEstimator {
        DistanceEstimator {
            params: vec![radius],
            func: Box::new(|p, params| p.to_vec().norm() - params[0])
        }
    }

    pub fn cube_estimator(dimensions: Point) -> DistanceEstimator {
        DistanceEstimator {
            params: vec![dimensions.x, dimensions.y, dimensions.z],
            func: Box::new(move |p, params| {
                let d = Pnt3::new(p.x.abs(), p.y.abs(), p.z.abs()) - dimensions;
                d.y.max(d.z).max(d.x).min(0.0) + comp_max(d, 0.0).norm()
            })
        }
    }

    #[inline]
    pub fn eval(&self, p: Point) -> f32 {
        (self.func)(p, &self.params)
    }

    #[inline]
    pub fn calc_normal(&self, p: Point, hx: f32, hy: f32, hz: f32) -> Vector {
        Vec3::new(
            self.eval(p + hx) - self.eval(p - hx),
            self.eval(p + hy) - self.eval(p - hy),
            self.eval(p + hz) - self.eval(p - hz))
    }
    
    pub fn repeat(self, c: Point) -> DistanceEstimator {
        DistanceEstimator {
            params: self.params.clone(),
            func: Box::new(move |p, params| {
                let q = Pnt3::new(
                    (p.x % c.x) - c.x * -0.5,
                    (p.y % c.y) - c.y * -0.5,
                    (p.z % c.z) - c.z * -0.5);
                (self.func)(q, params)
            })
        }
    }

    pub fn intersect(self, other: DistanceEstimator) -> DistanceEstimator {
        let mut params = Vec::new();
        push_all(&mut params, &self.params);
        push_all(&mut params, &other.params);
        let len = self.params.len();
        DistanceEstimator {
            params: params,
            func: Box::new(move |p, params| {
                let first = (self.func)(p, &params[..len]);
                let second = (other.func)(p, &params[len..]);
                first.max(second)
            })
        }
    }
}

fn push_all<T: Clone>(dest: &mut Vec<T>, src: &[T]) {
    for elem in src {
        dest.push(elem.clone());
    }
}

fn comp_max(p: Vector, t: f32) -> Vector {
    Vec3::new(p.x.max(t), p.y.max(t), p.z.max(t))
}

fn main() {
    //let sphere = DistanceEstimator::sphere_estimator(0.1).repeat(Pnt3::new(0.4, 1.0, 0.4));
    let cube = DistanceEstimator::cube_estimator(Pnt3::new(0.1, 0.1, 0.1)).repeat(Pnt3::new(0.6, 1.0, 0.6));
   // let intersect = sphere.intersect(cube);
    let camera = Camera {
        eye: Pnt3::new(-1.0, 1.0, -1.0),
        right: Vec3::new(1.0, 0.0, 0.0),
        up: Vec3::new(0.0, 0.1, 0.9)
    };
    let mut renderer = RayMarcher::new(800, 800, camera, vec![cube]);
    renderer.render();
    renderer.image.save(Path::new("image.png")).unwrap();
    println!("Average iterations per pixel: {}", renderer.avg_iters);
}
