extern crate image;
extern crate nalgebra;
extern crate rand;

// TODO port to webasm?

use camera::*;
use distance_estimator::*;
use image::{ImageBuffer, Rgb};
use light::*;
use nalgebra::*;
use ray::{Point, Ray, Vector};
use scene::Scene;
use std::path::Path;

mod camera;
mod distance_estimator;
mod light;
mod ray;
mod scene;

pub const MAX_STEPS: usize = 64;
pub const EPSILON: f32 = 0.00001;

pub struct RayMarcher {
    x_size: usize,
    y_size: usize,
    scene: Scene,
}

impl RayMarcher {
    pub fn new(x_size: usize, y_size: usize, scene: Scene) -> RayMarcher {
        RayMarcher {
            x_size,
            y_size,
            scene,
        }
    }

    pub fn render(&mut self) -> ImageBuffer<Rgb<f32>, Vec<f32>> {
        let mut image = ImageBuffer::new(self.x_size as u32, self.y_size as u32);
        for (x, y, pixel) in image.enumerate_pixels_mut() {
            let rays = self.scene.camera.samples(x, y);
            let samples: Vec<_> = rays.into_iter().map(|r| self.march_ray(r)).collect();
            let mut color = Vector3::new(0.0, 0.0, 0.0);
            let sample_count = samples.len() as f32;
            for (count, s) in samples {
                if count < MAX_STEPS {
                    color += s / sample_count;
                }
            }
            let c = Rgb {
                data: [color.x, color.y, color.z],
            };
            *pixel = c;
        }
        image
    }

    #[inline]
    fn shade_pixel(&self, p: Point, normal: Vector) -> Color {
        let mut color = Vector3::new(0.0, 0.0, 0.0);
        for light in &self.scene.lights {
            let light_dir = light.calc_direction(p);
            let light_intensity = light.calc_intensity(p);
            let dot = normal.dot(&light_dir).max(0.0);
            color += light_intensity * dot;
        }
        color
    }

    #[inline]
    fn background_color(&self) -> Color {
        Vector3::new(0.0, 0.0, 0.0)
    }

    #[inline]
    fn march_ray(&self, ray: Ray) -> (usize, Color) {
        let mut t = 0.0;
        for step in 0..MAX_STEPS {
            let current = ray.eval(t);
            let d = self.scene.distance_f.eval(current);
            if d < EPSILON {
                let h = Vector::new(0.05, 0.05, 0.05);
                let color = self.shade_pixel(current, self.scene.distance_f.normal(current, h));
                return (step, color);
            }
            t += d;
        }
        (MAX_STEPS, self.background_color())
    }
}

fn to_u8(channel: f32) -> u8 {
    (clamp(channel, 0.0, 1.0) * 255.0) as u8
}

fn main() {
    //while p < p_max {
    let sampler = StratifiedSampler::new(800.0, 800.0, 9);
    let camera = OrthographicCamera::new(
        Point3::new(0.0, 0.0, -1.0),
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        800.0,
        800.0,
        Box::new(sampler),
    );
    let sphere = DistanceEstimator::sphere_estimator(0.2).repeat(Point3::new(0.4, 1.0, 0.4));
    //let cube = DistanceEstimator::cube_estimator(Point3::new(0.5, 0.5, 0.5)); //.repeat(Point3::new(0.6, 1.0, 0.6));
    //let intersect = cube.intersect(sphere);
    //let min = DistanceEstimator::min_estimator(vec![sphere, cube]);
    let lights = vec![Light::new(
        Point3::new(-6.0, -5.0, -1.0),
        Vector3::new(1.0, 1.0, 1.0),
        0.5,
    )];
    //Light::new(Point3::new(-5.0, -5.0, 1.0), Vector3::new(1.0, 1.0, 1.0), 0.7)];
    let scene = Scene::new(lights, sphere, Box::new(camera));
    let mut renderer = RayMarcher::new(800, 800, scene);
    let result = renderer.render();
    let w = result.width();
    let h = result.height();
    let pixels: Vec<_> = result.into_raw();
    let bytes: Vec<_> = pixels.into_iter().map(to_u8).collect();
    let image: ImageBuffer<Rgb<u8>, Vec<u8>> = ImageBuffer::from_raw(w, h, bytes).unwrap();
    image.save(Path::new("image_0.png")).unwrap();
}
