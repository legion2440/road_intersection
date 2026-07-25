//! Pre-rendered car sprites, one small texture per immutable route.

use crate::drawing::{filled_box, filled_circle, rounded_box};
use crate::geometry::{CAR_LEN, CAR_W};
use sdl2::pixels::{Color, PixelFormatEnum};
use sdl2::render::{BlendMode, Canvas, Texture, TextureCreator};
use sdl2::video::{Window, WindowContext};

pub fn route_color(route: usize) -> Color {
    match route {
        0 => Color::RGB(143, 134, 230),
        1 => Color::RGB(224, 163, 92),
        _ => Color::RGB(79, 201, 180),
    }
}

fn route_light(route: usize) -> Color {
    match route {
        0 => Color::RGB(182, 174, 244),
        1 => Color::RGB(242, 196, 137),
        _ => Color::RGB(130, 226, 209),
    }
}

/// The sprite faces +x, matching the heading used by `copy_ex`.
pub fn build_cars<'a>(
    canvas: &mut Canvas<Window>,
    texture_creator: &'a TextureCreator<WindowContext>,
) -> Vec<Texture<'a>> {
    let (width, height) = (CAR_LEN as u32, CAR_W as u32);
    let mut textures = Vec::new();

    for route in 0..3 {
        let mut texture = texture_creator
            .create_texture_target(PixelFormatEnum::RGBA8888, width, height)
            .unwrap();
        texture.set_blend_mode(BlendMode::Blend);
        canvas
            .with_texture_canvas(&mut texture, |target| {
                target.set_blend_mode(BlendMode::Blend);
                target.set_draw_color(Color::RGBA(0, 0, 0, 0));
                target.clear();
                let (width, height) = (width as i16, height as i16);
                rounded_box(target, 0, 0, width - 1, height / 2, 4, route_light(route));
                rounded_box(
                    target,
                    0,
                    height / 2,
                    width - 1,
                    height - 1,
                    4,
                    route_color(route),
                );
                rounded_box(target, 0, 0, width - 1, height - 1, 4, route_color(route));
                rounded_box(target, 1, 1, width - 2, height / 2, 4, route_light(route));
                filled_box(
                    target,
                    width - 13,
                    3,
                    width - 5,
                    height - 4,
                    Color::RGB(11, 13, 20),
                );
                filled_circle(target, width - 2, 3, 1, Color::RGB(255, 248, 220));
                filled_circle(target, width - 2, height - 3, 1, Color::RGB(255, 248, 220));
            })
            .unwrap();
        textures.push(texture);
    }
    textures
}
