
extern crate image;
extern crate ord_subset;
extern crate alga;
extern crate nalgebra as na;

use std::sync::Arc;

use image::Rgb;
use ord_subset::*;
use alga::linear::EuclideanSpace;
use na::{Point2, Point3, Vector2, Vector3};

pub type U = f32;
pub type P2 = Point2<U>;
pub type P3 = Point3<U>;
pub type V2 = Vector2<U>;
pub type V3 = Vector3<U>;
pub type Color = Rgb<U>;

pub struct Scene {
    lights: Vec<PointLight>,
    spheres: Vec<Sphere>,
    planes: Vec<Plane>,
}

impl Scene {
    pub fn new() -> Self {
        Scene {
            spheres: vec![],
            lights: vec![],
            planes: vec![],
        }
    }

    pub fn add_sphere(&mut self, obj: Sphere) {
        self.spheres.push(obj);
    }

    pub fn add_plane(&mut self, obj: Plane) {
        self.planes.push(obj);
    }

    pub fn add_light(&mut self, light: PointLight) {
        self.lights.push(light);
    }

    pub fn cast_ray(&self, ray: &Ray) -> Option<Intersection> {
        self.spheres
            .iter()
            .filter_map(|obj| obj.intersection(ray))
            .chain(self.planes.iter().filter_map(|obj| obj.intersection(ray)))
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

impl Material {
    pub fn new(texture: Texture, diffuse_coef: U, reflectivity: U) -> Self {
        Material {
            texture,
            diffuse_coef,
            reflectivity,
        }
    }
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
pub const NOISE_TEXTURE: Texture = Texture::Procedural(noise_texture_impl);

pub fn solid_texture(c: Color) -> Texture {
    Texture::Solid(c)
}

fn xor_texture_impl(p: P2) -> Color {
    let x = (p[0] * 255.0) as u8;
    let y = (p[1] * 255.0) as u8;
    let c = (x ^ y) as U / 255.0;
    Color { data: [c; 3] }
}

fn noise_texture_impl(p: P2) -> Color {
    let s = 16.0;
    let x = (p[0] * s) as u32;
    let y = (p[1] * s) as u32;

    let mut seed = x * 3 ^ y * 7;
    seed *= 2364275237;
    seed ^= seed << 13;
    seed ^= seed >> 17;
    seed ^= seed << 5;

    let c = seed as u8 as U / 255.0;
    Color { data: [c; 3] }
}

pub struct Ray {
    pub origin: P3,
    pub dir: V3,
}

impl Ray {
    pub fn new(origin: P3, dir: V3) -> Self {
        Self { origin, dir }
    }
}

pub struct Sphere {
    pos: P3,
    radius: U,
    info: Arc<ObjInfo>,
}

impl Sphere {
    pub fn new(pos: P3, radius: U, info: Arc<ObjInfo>) -> Self {
        Sphere {
            pos,
            radius,
            info,
        }
    }
}

pub struct Plane {
    center: P3,
    normal: V3,
    info: Arc<ObjInfo>,
}

impl Plane {
    pub fn new(center: P3, normal: V3, info: Arc<ObjInfo>) -> Self {
        Self {
            center,
            normal,
            info,
        }
    }
}

impl Object for Sphere {
    fn intersection(&self, ray: &Ray) -> Option<Intersection> {
        let radius = self.radius;
        let p = ray.origin - self.pos;
        let p_d = na::dot(&p, &ray.dir);
        if p_d > 0.0 {
            return None;
        }
        let r_squared = radius * radius;
        let a = p - p_d * ray.dir;
        let a_squared = na::dot(&a, &a);
        if a_squared > r_squared {
            return None;
        }
        let h = (r_squared - a_squared).sqrt();
        let i = a - h * ray.dir;
        let intersection = self.pos + i;
        let normal = i / radius;

        let pi = std::f32::consts::PI;
        let rpi = 1.0 / pi;
        let hp = 1.0 / (pi * 2.0);
        let u = 0.5 + normal[2].atan2(normal[0]) * hp;
        let v = 0.5 - normal[1].asin() * rpi;

        return Some(Intersection {
            uv: P2::new(u, v),
            pos: intersection,
            norm: normal,
            info: self.info(),
            dist: na::distance(&ray.origin, &intersection),
        });
    }

    fn info(&self) -> &ObjInfo {
        &self.info
    }
}

impl Object for Plane {
    fn intersection(&self, ray: &Ray) -> Option<Intersection> {
        let denom = na::dot(&ray.dir, &self.normal);
        if denom != 0.0 {
            let d = na::dot(&(self.center - ray.origin), &self.normal) / denom;
            if d >= 0.0 {
                let pos = ray.origin + ray.dir * d;
                let pc = pos.coordinates();
                let v0 = V3::new(1.0, 0.0, 0.0);
                let v1 = self.normal.cross(&v0);
                let v2 = self.normal.cross(&v1);
                let u = na::dot(&v1, &pc) * (1.0 / 8.0);
                let v = na::dot(&v2, &pc) * (1.0 / 8.0);
                let uv = P2::new(u, v);
                let it = Intersection {
                    norm: self.normal,
                    pos,
                    dist: d,
                    info: self.info(),
                    uv,
                };
                return Some(it);
            }
        }
        // parallel (considering contained in plane as not an intersection)
        return None;
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
