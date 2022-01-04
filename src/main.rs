use crate::file::read_file;
use crate::ray::{Point, Ray, Vector};
use crate::scene::Scene;
use color_eyre::Report;
use image::Pixel;
use image::{ImageBuffer, Rgb};
use light::Color;
use std::path::Path;
use std::time::Instant;

mod camera;
mod distance_estimator;
mod file;
mod light;
mod ray;
mod scene;

pub const MAX_STEPS: usize = 64;
pub const EPSILON: f32 = 0.00001;

pub struct RayMarcher {
    x_size: u32,
    y_size: u32,
    scene: Scene,
}

impl RayMarcher {
    pub fn new(x_size: u32, y_size: u32, scene: Scene) -> RayMarcher {
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
            let mut color = Vector::new(0.0, 0.0, 0.0);
            let sample_count = samples.len() as f32;
            for (count, s) in samples {
                if count < MAX_STEPS {
                    color += s / sample_count;
                }
            }
            *pixel = Rgb::from_channels(color.x, color.y, color.z, 0.0);
        }
        image
    }

    #[inline]
    fn shade_pixel(&self, p: Point, normal: Vector) -> Color {
        let mut color = Vector::new(0.0, 0.0, 0.0);
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
        Vector::new(0.0, 0.0, 0.0)
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
    (nalgebra::clamp(channel, 0.0, 1.0) * 255.0) as u8
}

fn main() -> Result<(), Report> {
    color_eyre::install()?;
    let file_name = std::env::args().nth(1).unwrap_or("scene.json".into());
    let (scene, x, y) = read_file(&file_name)?;
    let mut renderer = RayMarcher::new(x, y, scene);

    let start = Instant::now();
    let result = renderer.render();
    let elapsed = start.elapsed();
    println!("took {} ms to render", elapsed.as_millis());

    let w = result.width();
    let h = result.height();
    let pixels: Vec<_> = result.into_raw();
    let bytes: Vec<_> = pixels.into_iter().map(to_u8).collect();
    let image: ImageBuffer<Rgb<u8>, Vec<u8>> = ImageBuffer::from_raw(w, h, bytes).unwrap();
    image.save(Path::new("image_0.png")).unwrap();

    Ok(())
}
