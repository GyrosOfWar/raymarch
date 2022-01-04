use crate::ray::{Point, Vector};
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

pub type Color = Vector3<f32>;

#[derive(Serialize, Deserialize, Debug)]
#[serde(rename_all = "camelCase")]
pub struct Light {
    position: Point,
    color: Color,
    intensity: f32,
    constant_attenuation: f32,
}

impl Light {
    pub fn new(position: Point, color: Color, intensity: f32) -> Light {
        Light {
            constant_attenuation: 1.0,
            position,
            color,
            intensity,
        }
    }

    #[inline]
    pub fn calc_intensity(&self, surface_point: Point) -> Color {
        let distance = (self.position - surface_point).norm();
        let factor = distance * self.intensity * (1.0 / self.constant_attenuation);

        self.color * factor
    }

    #[inline]
    pub fn calc_direction(&self, surface_point: Point) -> Vector {
        (self.position - surface_point).normalize()
    }
}
