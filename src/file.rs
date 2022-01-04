use std::{fs, path::Path};

use color_eyre::Report;
use serde::{Deserialize, Serialize};

use crate::camera::{OrthographicCamera, Sampler, SimpleSampler, StratifiedSampler};
use crate::distance_estimator::DistanceEstimator;
use crate::scene::Scene;
use crate::{
    light::Light,
    ray::{Point, Vector},
};

#[derive(Debug, Deserialize, Serialize)]
pub struct Camera {
    pub eye: Point,
    pub right: Vector,
    pub up: Vector,
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(tag = "type")]
#[serde(rename_all = "camelCase")]
pub enum Object {
    Cube { dimensions: Point },
    Sphere { radius: f32 },
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(rename_all = "camelCase")]
pub enum SamplerType {
    Simple,
    Stratified,
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct File {
    pub camera: Camera,
    pub lights: Vec<Light>,
    pub objects: Vec<Object>,
    pub size: (u32, u32),
    pub sampler_type: SamplerType,
}

impl Into<Scene> for File {
    fn into(self) -> Scene {
        let (x_size, y_size) = self.size;
        let sampler: Box<dyn Sampler> = match self.sampler_type {
            SamplerType::Simple => Box::new(SimpleSampler {
                x_res: x_size as f32,
                y_res: y_size as f32,
            }),
            SamplerType::Stratified => {
                Box::new(StratifiedSampler::new(x_size as f32, y_size as f32, 8))
            }
        };

        let camera =
            OrthographicCamera::new(self.camera.eye, self.camera.right, self.camera.up, sampler);

        let objects: Vec<_> = self
            .objects
            .into_iter()
            .map(|obj| match obj {
                Object::Cube { dimensions } => DistanceEstimator::cube_estimator(dimensions),
                Object::Sphere { radius } => DistanceEstimator::sphere_estimator(radius),
            })
            .collect();

        Scene {
            lights: self.lights,
            distance_f: DistanceEstimator::min_estimator(objects),
            camera: Box::new(camera),
        }
    }
}

pub fn read_file(path: impl AsRef<Path>) -> Result<(Scene, u32, u32), Report> {
    let mut reader = fs::File::open(path)?;
    let file: File = serde_json::from_reader(&mut reader)?;
    let (x, y) = file.size;
    let scene: Scene = file.into();
    Ok((scene, x, y))
}
