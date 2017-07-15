
extern crate ord_subset;
extern crate alga;
extern crate ncollide as nc;
extern crate nalgebra as na;

use ord_subset::*;
use alga::linear::EuclideanSpace;
use na::{Point3, Vector3};
use nc::query::RayCast;
use na::geometry::Isometry3;

pub use na::zero;

pub type U = f64;
pub type P3 = Point3<U>;
pub type V3 = Vector3<U>;
pub type Ray = nc::query::Ray3<U>;

pub struct Scene {
    objects: Vec<Box<Object + Sync>>,
}

impl Scene {
    pub fn new<I>(iter: I) -> Self
    where
        I: IntoIterator<Item = Box<Object + Sync>>,
    {
        Scene { objects: iter.into_iter().collect() }
    }

    pub fn cast_ray(&self, ray: &Ray) -> Option<Intersection> {
        self.objects
            .iter()
            .filter_map(|obj| obj.intersection(ray))
            .ord_subset_min_by_key(|i| i.dist)
    }
}

pub struct Intersection {
    // location: P3,
    pub dist: U,
}

pub trait Object {
    fn intersection(&self, ray: &Ray) -> Option<Intersection>;
}

pub struct Sphere {
    shape: nc::shape::Ball<U>,
    pos: P3,
}

impl Sphere {
    pub fn new(pos: P3, radius: U) -> Self {
        Sphere {
            shape: nc::shape::Ball::new(radius),
            pos,
        }
    }
}

impl Object for Sphere {
    fn intersection(&self, ray: &Ray) -> Option<Intersection> {
        let m = Isometry3::new(self.pos.coordinates(), na::zero());
        self.shape.toi_with_ray(&m, ray, true).map(|toi| {
            Intersection { dist: toi }
        })
    }
}
