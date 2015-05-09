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
mod scene;

pub const MAX_STEPS: usize = 150;
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
         let sample_pattern = [(-0.25, -0.25),
                               (0.25, -0.25),
                               (-0.25, 0.25),
                               (0.25, 0.25)];
     //   let sample_pattern = [(0.0, 0.0)];
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
          	  		let xf = xx + x_off;
            		let yf = yy + y_off;
                    let u = xf * 2.0 / x_res - 1.0;
                    let v = yf * 2.0 / y_res - 1.0;
                    let ray = self.camera.ray(u, v);
                    let (cnt, sample) = self.march_ray(ray);
					iter_sum += cnt;
					sample
        		});
            let mut color = Vec3::new(0.0, 0.0, 0.0);
            for sample in samples {
				color = color + (sample / sample_count);
            }
            
            let c = Rgb { data: *color.as_array() };
            *pixel = c;
        }
        let n = x_res * y_res * sample_count;
        self.avg_iters = iter_sum as f32 / n;

        return image;
    }

    #[inline]
    fn shade_pixel(&self, p: Point, normal: Vector) -> Color {
        let mut color = Vec3::new(0.0, 0.0, 0.0);
        for light in self.lights.iter() {
            let light_dir = light.calc_direction(p);
            let light_intensity = light.calc_intensity(p);
            let dot = normal.dot(&light_dir).max(0.0);
            color = color + (light_intensity * dot);
        }
        color
    }
    

    #[inline]
    fn background_color(&self) -> Color {
        Vec3::new(0.0, 0.0, 0.0)
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
    (clamp(channel, 0.0, 1.0) * 255.0) as u8
}

fn main() { 
	let mut p = 0.1;
	let increment = 0.1;
	let p_max = 1.0;
	let mut i = 0;
	
	while p < p_max {
        let camera = OrthographicCamera {
        	eye: Pnt3::new(0.0, 0.0, -1.0),
        	right: Vec3::new(p, 1.0 - p, 0.0),
        	up: Vec3::new(0.0, 1.0, 0.0)
    	};
        //let sphere = DistanceEstimator::sphere_estimator(p); // .repeat(Pnt3::new(0.4, 1.0, 0.4));
	    let cube = DistanceEstimator::cube_estimator(Pnt3::new(0.5, 0.5, 0.5)); //.repeat(Pnt3::new(0.6, 1.0, 0.6));
	    //let intersect = cube.intersect(sphere);
	    //let min = DistanceEstimator::min_estimator(vec![sphere, cube]);
		let lights = vec![Light::new(Pnt3::new(-5.0, -5.0, -1.0), Vec3::new(1.0, 1.0, 1.0), 0.7)];
						  //Light::new(Pnt3::new(-5.0, -5.0, 1.0), Vec3::new(1.0, 1.0, 1.0), 0.7)];
	   	
	    let mut renderer = RayMarcher::new(800, 800, Box::new(camera.clone()), cube, lights);
	    let result = renderer.render();
	    let w = result.width();
	    let h = result.height();
	    let pixels: Vec<_> = result.into_raw();
	    let bytes: Vec<_> = pixels.into_iter().map(|f| to_u8(f)).collect();
	    let image: ImageBuffer<Rgb<u8>, Vec<u8>> = ImageBuffer::from_raw(w, h, bytes).unwrap();
	    image.save(Path::new(&format!("image_{}.png", i))).unwrap();
	    i += 1;
	    p += increment;
	}
    //println!("Average iterations per pixel: {}", renderer.avg_iters);
}
