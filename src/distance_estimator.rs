#[allow(unused, dead_code)]

use ray::*;
use nalgebra::*;
use std::f32;
use std::fmt;

pub type EstimatorFunc = Box<Fn(Point, &[f32]) -> f32>;

pub struct DistanceEstimator {
    params: Vec<f32>,
    func: EstimatorFunc,
    name: String
}

impl fmt::Debug for DistanceEstimator {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{} with parameters {:?}", self.name, self.params)
    }
}

impl DistanceEstimator {
    pub fn sphere_estimator(radius: f32) -> DistanceEstimator {
        DistanceEstimator {
            params: vec![radius],
            func: Box::new(|p, params| p.to_vec().norm() - params[0]),
            name: "Sphere".to_string()
        }
    }

    pub fn cube_estimator(dimensions: Point) -> DistanceEstimator {
        DistanceEstimator {
            params: vec![dimensions.x, dimensions.y, dimensions.z],
            func: Box::new(move |p, _| {
                let d = Pnt3::new(p.x.abs(), p.y.abs(), p.z.abs()) - dimensions;
                d.y.max(d.z).max(d.x).min(0.0) + comp_max(d, 0.0).norm()
            }),
            name: "Cube".to_string()
        }
    }
    // Compose together a vector of estimators by taking the minimum of each of them
    pub fn min_estimator(estimators: Vec<DistanceEstimator>) -> DistanceEstimator {
        let names: Vec<_> = estimators.iter().map(|e| e.name.clone()).collect();
        let name = format!("Minimum of {:?}", names);
        DistanceEstimator {
            params: Vec::new(),
            func: Box::new(move |p, _| {
                let values = estimators.iter().map(|e| e.eval(p));
                let mut min = f32::INFINITY;
                for v in values {
                    if v < min {
                        min = v;
                    }
                }
                min
            }),
            name: name
        }
    }
    
    #[inline]
    pub fn eval(&self, p: Point) -> f32 {
        (self.func)(p, &self.params)
    }

    #[inline]
    pub fn normal(&self, p: Point, hx: f32, hy: f32, hz: f32) -> Vector {
        Vec3::new(
            self.eval(p + hx) - self.eval(p - hx),
            self.eval(p + hy) - self.eval(p - hy),
            self.eval(p + hz) - self.eval(p - hz))
    }
    
    pub fn repeat(self, c: Point) -> DistanceEstimator {
        let name = format!("Repeat of {}", self.name.clone());
        DistanceEstimator {
            params: self.params.clone(),
            func: Box::new(move |p, params| {
                let q = Pnt3::new(
                    (p.x % c.x) - c.x * -0.5,
                    (p.y % c.y) - c.y * -0.5,
                    (p.z % c.z) - c.z * -0.5);
                (self.func)(q, params)
            }),
            name: name
        }
    }

    pub fn intersect(self, other: DistanceEstimator) -> DistanceEstimator {
        let mut params = Vec::new();
        push_all(&mut params, &self.params);
        push_all(&mut params, &other.params);
        let len = self.params.len();
        let name = format!("Intersection of {} and {}", self.name.clone(), other.name.clone());

        DistanceEstimator {
            params: params,
            func: Box::new(move |p, params| {
                let first = (self.func)(p, &params[..len]);
                let second = (other.func)(p, &params[len..]);
                first.max(second)
            }),
            name: name
        }
    }

}

fn push_all<T: Clone>(dest: &mut Vec<T>, src: &[T]) {
    for elem in src {
        dest.push(elem.clone());
    }
}

fn comp_max(p: Vector, t: f32) -> Vector {
    Vec3::new(p.x.max(t), p.y.max(t), p.z.max(t))
}
