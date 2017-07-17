
extern crate nalgebra as na;
extern crate rayon;
extern crate image;
extern crate glimmer;

use std::sync::Arc;

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


    let m1 = Material::new(solid_texture(c1), 0.8, 0.0);
    let m2 = Material::new(solid_texture(c2), 0.8, 0.0);
    let m3 = Material::new(XOR_TEXTURE, 0.8, 0.0);
    let m4 = Material::new(solid_texture(c3), 0.2, 0.8);

    let info1 = Arc::new(ObjInfo::new(m1));
    let info2 = Arc::new(ObjInfo::new(m2));
    let info3 = Arc::new(ObjInfo::new(m3));
    let info4 = Arc::new(ObjInfo::new(m4));

    let a = Sphere::new(P3::new(-1.5, 0.0, 1.0), 1.0, info1.clone());
    let b = Sphere::new(P3::new(0.0, 0.0, 3.0), 1.0, info2.clone());
    let c = Sphere::new(P3::new(1.5, 0.0, 5.0), 1.0, info3.clone());
    let d = Sphere::new(P3::new(3.0, 0.0, 3.0), 1.0, info4.clone());

    let l1 = PointLight::new(P3::new(-2.0, -2.0, -1.0), c3);
    let l2 = PointLight::new(P3::new(0.0, 5.0, -2.0), c3.map(|c| c * 1.0));

    let mut scene = Scene::new();
    scene.add(a);
    scene.add(b);
    scene.add(c);
    scene.add(d);
    scene.add_light(l1);
    scene.add_light(l2);

    let infos = [info1, info2, info3, info4];
    let n = 8;
    let r = 1.0;
    for i in 0..n {
        let x = ((i * 3562215327) ^ 463557371) + 3;
        let a = i as U / n as U + 0.03;
        scene.add(Sphere::new(
            P3::new(
                (a * 3.14).cos() * r,
                0.5,
                3.0 - (a * 3.14).sin() * r,
            ),
            0.1,
            infos[x % infos.len()].clone(),
        ));
    }

    let mut image = RgbImage::from_fn(w, h, |_, _| Rgb { data: [0; 3] });

    render(&scene, &mut image);

    image.save("/tmp/glim.png").unwrap();
}

fn render(scene: &Scene, image: &mut RgbImage) {
    let w = image.width();
    let h = image.height();
    let t = min(w, h) as U;

    (0..h).into_par_iter().for_each(|y| {
        for x in 0..w {
            let xt = (x as U - w as U * 0.5) / t;
            let yt = (y as U - h as U * 0.5) / t;
            let ray = Ray::new(P3::new(0.0, 0.0, 0.0), V3::new(xt, yt, 1.0).normalize());

            let c = cast(&scene, &ray, 0);
            let mut data = [0; 3];

            for i in 0..3 {
                data[i] = min((c[i] as U * 255.0) as u8, 255);
            }

            // can't share image mutably between threads, so here's a hacked put_pixel:
            // safe since all (x, y)s are never repeated
            unsafe {
                *(image.get_pixel(x, y) as *const _ as *mut _) = Rgb { data };
            }
        }
    })
}

fn cast(scene: &Scene, ray: &Ray, depth: u8) -> Color {
    scene.cast_ray(&ray).map_or(Rgb { data: [0.0; 3] }, |it| {
        let ambient = 0.05;
        let mut diffuse = [0.0; 3];
        let m = it.info.material;
        let diffuse_coef = m.diffuse_coef;
        let tcolor = m.texture.color(it.uv);

        for light in scene.lights() {
            let lv = (light.pos - it.pos).normalize();
            let ldist2 = distance_squared(&light.pos, &it.pos);
            // lv * 0.0001 is a hack to avoid shadow ray colliding with the current object
            let lray = Ray::new(it.pos + lv * 0.0001, lv);

            if let Some(lit) = scene.cast_ray(&lray) {
                if lit.dist * lit.dist <= ldist2 {
                    continue;
                }
            }

            let ldot = dot(&lv, &it.norm);

            for i in 0..3 {
                diffuse[i] += ldot * tcolor[i] * diffuse_coef * light.color[i];
            }
        }

        if depth < 3 && m.reflectivity != 0.0 {
            let v = ray.origin - it.pos;
            let r = (2.0 * dot(&it.norm, &v) * it.norm - v).normalize();
            let ray = Ray::new(it.pos + r * 0.0001, r);
            let c = cast(scene, &ray, depth + 1);

            for i in 0..3 {
                diffuse[i] += c[i] * m.reflectivity;
            }
        }

        let mut data = [0.0; 3];

        for i in 0..3 {
            data[i] = tcolor[i] * ambient + diffuse[i];
        }

        Rgb { data }
    })
}
