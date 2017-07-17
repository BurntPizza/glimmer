
extern crate image;
extern crate ord_subset;
extern crate alga;
extern crate ncollide as nc;
extern crate nalgebra as na;

use std::sync::Arc;

use image::Rgb;
use ord_subset::*;
use alga::linear::EuclideanSpace;
use na::{Point2, Point3, Vector2, Vector3};
use nc::query::RayCast;

pub type U = f32;
pub type P2 = Point2<U>;
pub type P3 = Point3<U>;
pub type V2 = Vector2<U>;
pub type V3 = Vector3<U>;
pub type Ray = nc::query::Ray3<U>;
pub type Color = Rgb<U>;

pub struct Scene {
    lights: Vec<PointLight>,
    spheres: Vec<Sphere>,
}

impl Scene {
    pub fn new() -> Self {
        Scene {
            spheres: vec![],
            lights: vec![],
        }
    }

    pub fn add(&mut self, obj: Sphere) {
        self.spheres.push(obj);
    }

    pub fn add_light(&mut self, light: PointLight) {
        self.lights.push(light);
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

pub enum ObjRef<'a> {
    Sphere(&'a Sphere),
}

pub struct Intersection<'a> {
    pub info: &'a ObjInfo,
    pub pos: P3,
    pub dist: U,
    pub norm: V3,
    pub uv: P2,
}

pub trait Object {
    fn intersection(&self, ray: &Ray) -> Option<Intersection>;
    fn info(&self) -> &ObjInfo;
}

pub struct ObjInfo {
    pub material: Material,
}

// Solid color | Procedural texture
// reflectivity
// transparency
// index of refraction
// emissivity

#[derive(Copy, Clone)]
pub struct Material {
    pub texture: Texture,
    pub diffuse_coef: U,
    pub reflectivity: U,
}


impl ObjInfo {
    pub fn new(material: Material) -> Self {
        Self { material }
    }
}

#[derive(Copy, Clone)]
pub enum Texture {
    Solid(Color),
    Procedural(fn(P2) -> Color),
}

impl Texture {
    pub fn color(&self, p: P2) -> Color {
        match *self {
            Texture::Solid(c) => c,
            Texture::Procedural(f) => f(p),
        }
    }
}

pub const XOR_TEXTURE: Texture = Texture::Procedural(xor_texture_impl);

pub fn solid_texture(c: Color) -> Texture {
    Texture::Solid(c)
}

fn xor_texture_impl(p: P2) -> Color {
    let s = 128.0;
    let p = p * s;
    let x = p[0] as u16;
    let y = p[1] as u16;
    let c = (x ^ y) as U / s;
    Color { data: [c; 3] }
}

pub struct Sphere {
    pos: P3,
    shape: nc::shape::Ball<U>,
    info: Arc<ObjInfo>,
}
impl Material {
    pub fn new(texture: Texture, diffuse_coef: U, reflectivity: U) -> Self {
        Material {
            texture,
            diffuse_coef,
            reflectivity,
        }
    }
}

impl Sphere {
    pub fn new(pos: P3, radius: U, info: Arc<ObjInfo>) -> Self {
        Sphere {
            pos,
            shape: nc::shape::Ball::new(radius),
            info,
        }
    }
}

impl Object for Sphere {
    fn intersection(&self, ray: &Ray) -> Option<Intersection> {
        let m = na::Translation3::from_vector(self.pos.coordinates());
        self.shape
            .toi_and_normal_and_uv_with_ray(&m, ray, true)
            .map(|i| {
                let pos = ray.origin + ray.dir * i.toi;
                Intersection {
                    pos,
                    dist: i.toi,
                    info: self.info(),
                    norm: i.normal,
                    uv: i.uvs.unwrap(),
                }
            })
    }

    fn info(&self) -> &ObjInfo {
        &self.info
    }
}


impl<'a> Object for ObjRef<'a> {
    fn intersection(&self, ray: &Ray) -> Option<Intersection> {
        match *self {
            ObjRef::Sphere(ref s) => s.intersection(ray),
        }
    }

    fn info(&self) -> &ObjInfo {
        match *self {
            ObjRef::Sphere(ref s) => s.info(),
        }
    }
}
