use light::Light;
use distance_estimator::DistanceEstimator;
use camera::Camera;

#[derive(Debug)]
pub struct Scene {
	lights: Vec<Light>, 
	distance_f: DistanceEstimator,
	camera: Box<Camera>
}

impl Scene {
	pub fn new(lights: Vec<Light>, distance_estimator: DistanceEstimator, camera: Box<Camera>) -> Scene {
		Scene {
			lights: lights,
			distance_f: distance_estimator,
			camera: camera
		}
	}
	
}