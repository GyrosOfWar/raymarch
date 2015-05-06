extern crate cgmath;

use cgmath::{Point3, Vector3, Point, Vector};

pub type Ray = cgmath::Ray3<f32>;

pub const MAX_STEPS: usize = 500;

pub const EPSILON: f32 = 0.00001;

pub fn ray_march<F, G>(ray: Ray, scene_distance: F, plot_point: G) -> bool
		where F: Fn(Point3<f32>) -> f32,
		      G: Fn(f32, usize)  -> () {
	let mut t = 0.0;
	for i in 0..MAX_STEPS {
		let current = ray.origin.add_v(&ray.direction.mul_s(t));
		let d = scene_distance(current);
		if d < EPSILON {
			plot_point(d, i);
			return true;
		}
		t += d;
	}

	false
}

#[test]
fn it_works() {
}
