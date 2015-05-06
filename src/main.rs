extern crate cgmath;
extern crate image;

use cgmath::{Point3, Vector3, Point, Vector, EuclideanVector};
use image::{ImageBuffer, Rgb, Pixel};

use std::path::Path;

pub type Ray = cgmath::Ray3<f32>;
pub const MAX_STEPS: usize = 64;
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
    pub fn new(x_size: usize, y_size: usize, distance_estimator: Vec<DistanceEstimator>) -> RayMarcher {
        RayMarcher {
            image: ImageBuffer::new(x_size as u32, y_size as u32),
            x_size: x_size,
            y_size: y_size,
            camera: Camera::zero(),
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
    fn march_ray(ray: Ray, pixel: &mut Rgb<u8>, dist: &Vec<DistanceEstimator>) -> usize {
        let mut t = 0.0;
        for step in 0..MAX_STEPS {
	    let current = ray.origin.add_v(&ray.direction.mul_s(t));
	    let values = dist.iter().map(|d| d.eval(current));
            let mut d = std::f32::INFINITY;
            for i in values {
                if i < d {
                    d = i;
                }
            }
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
    eye: Point3<f32>,
    right: Vector3<f32>,
    up: Vector3<f32>
}

impl Camera {
    pub fn zero() -> Camera {
        Camera {
            eye: Point3::new(0.0, 0.0, -4.0),
            right: Vector3::new(1.0, 0.0, 0.0),
            up: Vector3::new(0.0, 1.0, 0.0)
        }
    }

    #[inline]
    pub fn ray(&self, u: f32, v: f32) -> Ray {
        let ro = self.eye.add_v(&self.right.mul_s(u).add_v(&self.up.mul_s(v)));
        let rd = self.right.cross(&self.up).normalize();

        cgmath::Ray::new(ro, rd)
    }
}

pub fn repeat<F>(p: Point3<f32>, dist: F, params: &Vec<f32>) -> f32 where F: Fn(Point3<f32>, &Vec<f32>) -> f32 {
    let x = p.x % 1.0 - 0.5;
    let z = p.z % 1.0 - 0.5;
    let q = Point3::new(x, p.y, z);
    dist(q, params)
}

pub type EstimatorFunc = Box<Fn(Point3<f32>, &Vec<f32>) -> f32>;

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
            func: Box::new(|p, params| p.to_vec().length() - params[0])
        }
    }

    pub fn eval(&self, p: Point3<f32>) -> f32 {
        (self.func)(p, &self.params)
    }

    pub fn repeat_estimator(e: DistanceEstimator, c: Point3<f32>) -> DistanceEstimator {
        DistanceEstimator {
            params: e.params.clone(),
            func: Box::new(move |p, params| {
                // let q = p.rem_v(c) - c.mul_s(0.5);
                let q = Point3::new(
                    p.x % c.x - c.x * -0.5,
                    p.y % c.y - c.y * -0.5,
                    p.z % c.z - c.z * -0.5
                    );
                (e.func)(q, params)
            })
        }
    }
}

fn main() {
    let sphere = DistanceEstimator::sphere_estimator(0.1);
    let mut renderer = RayMarcher::new(800, 800, vec![
        DistanceEstimator::repeat_estimator(sphere, Point3::new(1.0, 1.0, 1.0))]);
    renderer.render();
    renderer.image.save(Path::new("image.png")).unwrap();
    println!("Average iterations per pixel: {}", renderer.avg_iters);
}
