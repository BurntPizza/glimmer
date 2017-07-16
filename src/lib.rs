
extern crate image;
extern crate ord_subset;
extern crate alga;
extern crate ncollide as nc;
extern crate nalgebra as na;

use image::Rgb;
use ord_subset::*;
use alga::linear::EuclideanSpace;
use na::{Point3, Vector3};
use na::geometry::Isometry3;
use nc::query::RayCast;

use std::any::Any;

pub type U = f32;
pub type P3 = Point3<U>;
pub type V3 = Vector3<U>;
pub type Ray = nc::query::Ray3<U>;
pub type Color = Rgb<U>;

pub struct Scene {
    lights: Vec<PointLight>,
    spheres: Vec<Sphere>,
}

impl Scene {
    pub fn new<I>(iter: I) -> Self
    where
        I: IntoIterator<Item = Box<Any>>,
    {
        let mut spheres = vec![];
        let mut lights = vec![];

        for obj in iter {
            if obj.is::<Sphere>() {
                spheres.push(*obj.downcast().unwrap());
                continue;
            }
            if obj.is::<PointLight>() {
                lights.push(*obj.downcast().unwrap());
                continue;
            }
            panic!();
        }

        Scene { spheres, lights }
    }

    pub fn cast_ray(&self, ray: &Ray) -> Option<Intersection> {
        self.spheres
            .iter()
            .filter_map(|obj| obj.intersection(ray))
            .ord_subset_min_by_key(|i| i.dist)
    }

    pub fn lights(&self) -> &[PointLight] {
        &*self.lights
    }
}

pub struct PointLight {
    pub pos: P3,
    pub color: Color,
}

impl PointLight {
    pub fn new(pos: P3, color: Color) -> Self {
        PointLight { pos, color }
    }
}

pub struct Intersection<'a> {
    pub obj: &'a Object,
    pub pos: P3,
    pub dist: U,
    pub norm: V3,
}

pub trait Object {
    fn intersection(&self, ray: &Ray) -> Option<Intersection>;
    fn material(&self) -> &Material;
}

pub struct Sphere {
    pos: P3,
    shape: nc::shape::Ball<U>,
    material: Material,
}

#[derive(Copy, Clone)]
pub struct Material {
    pub color: Color,
    pub diffuse_coef: U,
}

impl Material {
    pub fn new(color: Color, diffuse_coef: U) -> Self {
        Material {
            color,
            diffuse_coef,
        }
    }
}

impl Sphere {
    pub fn new(pos: P3, radius: U, material: Material) -> Self {
        Sphere {
            pos,
            shape: nc::shape::Ball::new(radius),
            material,
        }
    }
}

impl Object for Sphere {
    fn intersection(&self, ray: &Ray) -> Option<Intersection> {
        let m = Isometry3::new(self.pos.coordinates(), na::zero());
        self.shape.toi_and_normal_with_ray(&m, ray, true).map(|i| {
            let pos = ray.origin + ray.dir * i.toi;
            Intersection {
                pos,
                dist: i.toi,
                obj: self as &Object,
                norm: i.normal,
            }
        })
    }

    fn material(&self) -> &Material {
        &self.material
    }
}
