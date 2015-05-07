extern crate nalgebra;
extern crate image;

use image::{ImageBuffer, Rgb, Pixel};
use nalgebra::*;
use std::path::Path;
use camera::{Camera, OrthographicCamera};
use ray::{Ray, Point, Vector};
use distance_estimator::*;
use light::*;

mod camera;
mod ray;
mod distance_estimator;
mod light;

pub const MAX_STEPS: usize = 64;
pub const EPSILON: f32 = 0.00001;

pub struct RayMarcher {
    x_size: usize,
    y_size: usize,
    camera: Box<Camera>,
    avg_iters: f32,
    distance_estimator: DistanceEstimator,
    lights: Vec<Light>
}

impl RayMarcher {
    pub fn new(x_size: usize, y_size: usize, camera: Box<Camera>, distance_estimator: DistanceEstimator,
               lights: Vec<Light>) -> RayMarcher {
        RayMarcher {
            x_size: x_size,
            y_size: y_size,
            camera: camera,
            avg_iters: 0.0,
            distance_estimator: distance_estimator,
            lights: lights
        }
    }

    pub fn render(&mut self) -> ImageBuffer<Rgb<f32>, Vec<f32>> {
        // let sample_pattern = [(-0.5, -0.5),
        //                       (0.5, -0.5),
        //                       (-0.5, 0.5),
        //                       (0.5, 0.5)];
        let sample_pattern = [(0.0, 0.0)];
        let sample_count = sample_pattern.len() as f32;
        
        let mut image = ImageBuffer::new(self.x_size as u32, self.y_size as u32);
        let x_res = self.x_size as f32;
        let y_res = self.y_size as f32;
        let mut iter_sum = 0;
        
        for (x, y, pixel) in image.enumerate_pixels_mut() {
            let xx = x as f32;
            let yy = y as f32;

            let samples = sample_pattern
                .iter()
                .map(|&(x_off, y_off)| {
                    let u = (xx + x_off) * 2.0 / x_res - 1.0;
                    let v = (yy + y_off) * 2.0 / y_res - 1.0;
                    let ray = self.camera.ray(u, v);
                    self.march_ray(ray).1
                });
            let mut color = [0.0, 0.0, 0.0];
            for sample in samples {
                color[0] += sample.data[0] / sample_count;
                color[1] += sample.data[1] / sample_count;
                color[2] += sample.data[2] / sample_count;
            }
            let c = Rgb { data: color };
            *pixel = c;
            //iter_sum += count;
        }
        let n = x_res * y_res;
        self.avg_iters = iter_sum as f32 / n;

        return image;
    }

    #[inline]
    fn shade_pixel(&self, p: Point, normal: Vector) -> Color {
        let mut color = [0.0, 0.0, 0.0];
        for light in self.lights.iter() {
            let light_dir = (light.position - p).normalize();
            let dot = normal.dot(&light_dir);
            let light = light.color.map(|c| c * dot); 
            color[0] += light.data[0];
            color[1] += light.data[1];
            color[2] += light.data[2];
        }
        
        Rgb { data: color }
    }

    #[inline]
    fn background_color(&self) -> Color {
        Rgb { data: [0.0, 0.0, 0.0] }
    }

    #[inline]
    fn march_ray(&self, ray: Ray) -> (usize, Color) {
        let mut t = 0.0;
        for step in 0..MAX_STEPS {
	    let current = ray.eval(t);
            let d = self.distance_estimator.eval(current);
	    if d < EPSILON {
                let h = 0.05;
                let color = self.shade_pixel(current, self.distance_estimator.normal(current, h, h, h));
	        return (step, color);
	    }
	    t += d;
        }
        (MAX_STEPS, self.background_color())
    }
}

fn to_u8(channel: f32) -> u8 {
    (channel * 256.0) as u8
}

fn main() {
    let sphere = DistanceEstimator::sphere_estimator(0.8); //.repeat(Pnt3::new(0.4, 1.0, 0.4));
    //let cube = DistanceEstimator::cube_estimator(Pnt3::new(0.1, 0.1, 0.1)).repeat(Pnt3::new(0.6, 1.0, 0.6));
    // let intersect = sphere.intersect(cube);
    //let min = DistanceEstimator::min_estimator(vec![sphere, cube]);
    let camera = OrthographicCamera {
        eye: Pnt3::new(0.0, 0.0, -1.0),
        right: Vec3::new(1.0, 0.0, 0.0),
        up: Vec3::new(0.0, 1.0, 0.0)
    };
    let lights = vec![
        Light {
            position: Pnt3::new(1.0, -1.0, 1.0),
            color: Rgb { data: [1.0, 1.0, 1.0] }
        }, Light {
            position: Pnt3::new(5.0, 1.0, 1.0),
            color: Rgb { data: [1.0, 1.0, 1.0] }
        }];
    
    let mut renderer = RayMarcher::new(800, 800, Box::new(camera), sphere, lights);
    let result = renderer.render();
    let w = result.width();
    let h = result.height();
    let pixels: Vec<_> = result.into_raw();
    let bytes: Vec<_> = pixels.into_iter().map(|f| to_u8(f)).collect();
    let image: ImageBuffer<Rgb<u8>, Vec<u8>> = ImageBuffer::from_raw(w, h, bytes).unwrap();
    image.save(Path::new("image.png")).unwrap();

    println!("Average iterations per pixel: {}", renderer.avg_iters);
}
