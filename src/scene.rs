use camera::Camera;
use distance_estimator::DistanceEstimator;
use light::Light;

pub struct Scene {
    pub lights: Vec<Light>,
    pub distance_f: DistanceEstimator,
    pub camera: Box<Camera>,
}

impl Scene {
    pub fn new(
        lights: Vec<Light>,
        distance_estimator: DistanceEstimator,
        camera: Box<Camera>,
    ) -> Scene {
        Scene {
            distance_f: distance_estimator,
            lights,
            camera,
        }
    }
}
