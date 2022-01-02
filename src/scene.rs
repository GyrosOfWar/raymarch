use crate::camera::{Camera};
use crate::distance_estimator::DistanceEstimator;
use crate::light::Light;

pub struct Scene {
    pub lights: Vec<Light>,
    pub distance_f: DistanceEstimator,
    pub camera: Box<dyn Camera>,
}

impl Scene {
    pub fn new(
        lights: Vec<Light>,
        distance_estimator: DistanceEstimator,
        camera: Box<dyn Camera>,
    ) -> Scene {
        Scene {
            distance_f: distance_estimator,
            lights,
            camera,
        }
    }
}
