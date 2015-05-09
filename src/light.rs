use ray::*;
use image::Rgb;
use nalgebra::*;

pub type Color = Vec3<f32>;

#[derive(Debug)]
pub struct Light {
    position: Point,
    color: Color,
    intensity: f32,
    constant_attenuation: f32
}

impl Light {
	pub fn new(position: Point, color: Color, intensity: f32) -> Light {
		Light {
			position: position,
			color: color,
			intensity: intensity,
			constant_attenuation: 1.0
		}
	}
	
	pub fn calc_intensity(&self, surface_point: Point) -> Color {
		let distance = (self.position - surface_point).norm();
		let factor = distance * self.intensity * 1.0 / self.constant_attenuation;
		
		self.color * factor
	}
	
	pub fn calc_direction(&self, surface_point: Point) -> Vector {
		(self.position - surface_point).normalize()
	}
	
}