
extern crate nalgebra as na;
extern crate rayon;
extern crate image;
extern crate glimmer;

use std::any::Any;

use na::*;
use rayon::prelude::*;
use image::{Rgb, RgbImage, Pixel};
use glimmer::*;

fn main() {
    let w = 1280;
    let h = 720;

    let c1 = Rgb { data: [0.7, 0.0, 0.0] };
    let c2 = Rgb { data: [0.2, 0.6, 0.2] };
    let c3 = Rgb { data: [0.9, 0.9, 0.9] };

    let m1 = Material::new(c1, 0.8);
    let m2 = Material::new(c2, 0.8);
    let m3 = Material::new(c3, 0.8);

    let a = Sphere::new(P3::new(-1.5, 0.0, 1.0), 1.0, m1);
    let b = Sphere::new(P3::new(0.0, 0.0, 3.0), 1.0, m2);
    let c = Sphere::new(P3::new(1.5, 0.0, 5.0), 1.0, m3);
    let d = Sphere::new(P3::new(3.0, 0.0, 7.0), 1.0, m1);

    let l1 = PointLight::new(P3::new(-2.0, -2.0, -1.0), c3);
    let l2 = PointLight::new(P3::new(0.0, 5.0, -2.0), c3.map(|c| c * 0.5));

    let scene = Scene::new(vec![x(a), x(b), x(c), x(d), x(l1), x(l2)]);

    let mut image = RgbImage::new(w, h);

    render(&scene, &mut image);

    image.save("/tmp/glim.png").unwrap();
}

fn render(scene: &Scene, image: &mut RgbImage) {
    let w = image.width();
    let h = image.height();
    let t = min(w, h) as U;

    (0..h).into_par_iter().for_each(|y| for x in 0..w {
        let xt = (x as U - w as U * 0.5) / t;
        let yt = (y as U - h as U * 0.5) / t;
        let ray = Ray::new(P3::new(0.0, 0.0, 0.0), V3::new(xt, yt, 1.0).normalize());

        let c = scene.cast_ray(&ray).map_or(Rgb { data: [0, 0, 0] }, |it| {
            // ambient
            let ambient = 0.05;
            let mut diffuse = [0.0; 3];
            let m = it.obj.material();

            for light in scene.lights() {
                let lv = (light.pos - it.pos).normalize();
                let ldist2 = distance_squared(&light.pos, &it.pos);
                // lv * 0.0001 is a hack to avoid this shadow ray colliiding with the current object
                let lray = Ray::new(it.pos + lv * 0.0001, lv);

                if let Some(lit) = scene.cast_ray(&lray) {
                    if lit.dist * lit.dist <= ldist2 {
                        continue;
                    }
                }

                let ldot = dot(&lv, &it.norm);

                for i in 0..3 {
                    diffuse[i] += ldot * m.color[i] * light.color[i];
                }
            }

            let mut data = [0; 3];

            for i in 0..3 {
                let c = ((m.color[i] * ambient + diffuse[i]) * 255.0) as u32;
                data[i] = min(c, 255) as u8;
            }

            Rgb { data }
        });

        // can't share image mutably between threads, so here's a hacked put_pixel:
        // safe since all (x, y)s are never repeated
        unsafe {
            *(image.get_pixel(x, y) as *const _ as *mut _) = c;
        }
    })
}

fn x<T: 'static>(t: T) -> Box<Any> {
    Box::new(t) as Box<Any>
}
