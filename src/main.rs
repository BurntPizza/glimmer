
extern crate image;
extern crate glimmer;
extern crate rayon;

use std::cmp::*;
use image::{Rgb, RgbImage};
use rayon::prelude::*;
use glimmer::*;

const PARALLEL: bool = true;

fn main() {
    let w = 1920;
    let h = 1080;

    let a = Sphere::new(P3::new(-1.5, 0.0, 1.0), 1.0);
    let b = Sphere::new(P3::new(0.0, 0.0, 3.0), 1.0);
    let c = Sphere::new(P3::new(1.5, 0.0, 5.0), 1.0);
    let d = Sphere::new(P3::new(3.0, 0.0, 7.0), 1.0);

    let scene = Scene::new(vec![a, b, c, d].into_iter().map(|o| {
        Box::new(o) as Box<Object + Sync>
    }));

    let mut image = RgbImage::new(w, h);

    render(&scene, &mut image);

    image.save("/tmp/glim.png").unwrap();
}

fn render(scene: &Scene, image: &mut RgbImage) {
    let w = image.width();
    let h = image.height();
    let t = min(w, h) as U;

    if PARALLEL {
        (0..h).into_par_iter().for_each(|y| for x in 0..w {
            process(x, y, &image, &scene, w, h, t);
        });
    } else {
        for y in 0..h {
            for x in 0..w {
                process(x, y, &image, &scene, w, h, t);
            }
        }
    }
}

#[inline]
fn process(x: u32, y: u32, image: &RgbImage, scene: &Scene, w: u32, h: u32, t: U) {
    let xt = (x as U - w as U * 0.5) / t;
    let yt = (y as U - h as U * 0.5) / t;
    let ray = Ray::new(P3::new(0.0, 0.0, 0.0), V3::new(xt, yt, 1.0).normalize());
    let d = scene.cast_ray(&ray).map_or(
        0,
        |it| (255.0 - it.dist * 10.0) as u8,
    );
    let p = Rgb { data: [d, d, d] };

    // can't share image mutably between threads, so here's a hacked put_pixel:
    // safe since all (x, y)s are never repeated
    unsafe {
        *(image.get_pixel(x, y) as *const _ as *mut _) = p;
    }
}
