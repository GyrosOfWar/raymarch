use ray::*;
use image::Rgb;

pub type Color = Rgb<f32>;

pub struct Light {
    pub position: Point,
    pub color: Color
}
