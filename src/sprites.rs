//! Padded BMP car textures and BMP traffic-light sprites.

use crate::drawing::{filled_box, filled_circle};
use crate::geometry::{CAR_LEN, CAR_W};
use sdl2::hint::Hint;
use sdl2::pixels::{Color, PixelFormatEnum};
use sdl2::rect::Rect;
use sdl2::render::{BlendMode, Canvas, Texture, TextureCreator};
use sdl2::rwops::RWops;
use sdl2::surface::Surface;
use sdl2::video::{Window, WindowContext};

pub const CAR_FRAME_W: u32 = 30;
pub const CAR_FRAME_H: u32 = 17;
pub const CAR_ROUTES: u32 = 3;
pub const CAR_PAD: u32 = 2;
pub const CAR_TEXTURE_W: u32 = CAR_LEN as u32 + 2 * CAR_PAD;
pub const CAR_TEXTURE_H: u32 = CAR_W as u32 + 2 * CAR_PAD;
pub const TRAFFIC_LIGHT_FRAME_W: u32 = 17;
pub const TRAFFIC_LIGHT_FRAME_H: u32 = 35;
pub const TRAFFIC_LIGHT_FRAMES: u32 = 2;

const CHROMA_KEY: Color = Color::RGB(255, 0, 255);
const CARS_BMP: &[u8] = include_bytes!("../assets/cars.bmp");
const TRAFFIC_LIGHTS_BMP: &[u8] = include_bytes!("../assets/traffic_lights.bmp");

pub struct SpriteSheets<'a> {
    pub cars: Vec<Texture<'a>>,
    pub traffic_lights: Texture<'a>,
}

impl<'a> SpriteSheets<'a> {
    pub fn load(
        canvas: &mut Canvas<Window>,
        texture_creator: &'a TextureCreator<WindowContext>,
    ) -> Result<Self, String> {
        Ok(Self {
            cars: build_cars(canvas, texture_creator)?,
            traffic_lights: build_traffic_lights(texture_creator)?,
        })
    }
}

pub fn build_cars<'a>(
    canvas: &mut Canvas<Window>,
    texture_creator: &'a TextureCreator<WindowContext>,
) -> Result<Vec<Texture<'a>>, String> {
    // sdl2 0.34 applies this hint to textures when they are created.
    if !sdl2::hint::set_with_priority("SDL_RENDER_SCALE_QUALITY", "0", &Hint::Override) {
        return Err("failed to enable nearest-neighbour texture sampling".to_string());
    }

    let source = load_sheet(
        texture_creator,
        CARS_BMP,
        "cars.bmp",
        CAR_FRAME_W,
        CAR_FRAME_H * CAR_ROUTES,
        true,
    )?;
    let mut cars = Vec::with_capacity(CAR_ROUTES as usize);

    for route in 0..CAR_ROUTES {
        let mut texture = texture_creator
            .create_texture_target(None, CAR_TEXTURE_W, CAR_TEXTURE_H)
            .map_err(|error| {
                format!("failed to create padded car texture for route {route}: {error}")
            })?;
        texture.set_blend_mode(BlendMode::Blend);

        let source_rect = Rect::new(0, (route * CAR_FRAME_H) as i32, CAR_FRAME_W, CAR_FRAME_H);
        let destination_rect = Rect::new(CAR_PAD as i32, CAR_PAD as i32, CAR_FRAME_W, CAR_FRAME_H);
        let mut draw_error = None;
        canvas
            .with_texture_canvas(&mut texture, |target| {
                target.set_blend_mode(BlendMode::None);
                target.set_draw_color(Color::RGBA(0, 0, 0, 0));
                target.clear();
                target.set_blend_mode(BlendMode::Blend);

                if let Err(error) = target.copy(&source, Some(source_rect), Some(destination_rect))
                {
                    draw_error = Some(error);
                    return;
                }

                let x0 = CAR_PAD as i16;
                let y0 = CAR_PAD as i16;
                let x1 = x0 + CAR_FRAME_W as i16 - 1;
                let y1 = y0 + CAR_FRAME_H as i16 - 1;

                filled_box(
                    target,
                    x0 + 19,
                    y0 + 3,
                    x0 + 22,
                    y0 + 12,
                    Color::RGB(11, 13, 20),
                );
                filled_box(
                    target,
                    x0 + 3,
                    y0 + 3,
                    x0 + 4,
                    y0 + 11,
                    Color::RGB(11, 13, 20),
                );

                for (x, color) in [
                    (x1 - 1, Color::RGB(255, 246, 214)),
                    (x0 + 1, Color::RGB(200, 60, 60)),
                ] {
                    filled_circle(target, x, y0 + 2, 1, color);
                    filled_circle(target, x, y1 - 2, 1, color);
                }
            })
            .map_err(|error| {
                format!("failed to draw padded car texture for route {route}: {error}")
            })?;
        if let Some(error) = draw_error {
            return Err(format!(
                "failed to copy car sprite for route {route}: {error}"
            ));
        }
        cars.push(texture);
    }

    Ok(cars)
}

fn build_traffic_lights<'a>(
    texture_creator: &'a TextureCreator<WindowContext>,
) -> Result<Texture<'a>, String> {
    load_sheet(
        texture_creator,
        TRAFFIC_LIGHTS_BMP,
        "traffic_lights.bmp",
        TRAFFIC_LIGHT_FRAME_W * TRAFFIC_LIGHT_FRAMES,
        TRAFFIC_LIGHT_FRAME_H,
        false,
    )
}

fn load_sheet<'a>(
    texture_creator: &'a TextureCreator<WindowContext>,
    bytes: &[u8],
    name: &str,
    expected_width: u32,
    expected_height: u32,
    clean_chroma_fringe: bool,
) -> Result<Texture<'a>, String> {
    let mut source = RWops::from_bytes(bytes)
        .map_err(|error| format!("failed to open embedded {name}: {error}"))?;
    let surface = Surface::load_bmp_rw(&mut source)
        .map_err(|error| format!("failed to load embedded sprite sheet {name}: {error}"))?;
    if surface.width() != expected_width || surface.height() != expected_height {
        return Err(format!(
            "invalid sprite sheet {name}: expected {expected_width}x{expected_height}, got {}x{}",
            surface.width(),
            surface.height()
        ));
    }
    let mut surface = if clean_chroma_fringe {
        surface
            .convert_format(PixelFormatEnum::RGBA32)
            .map_err(|error| format!("failed to convert sprite sheet {name}: {error}"))?
    } else {
        surface
    };
    if clean_chroma_fringe {
        remove_chroma_fringe(&mut surface);
    }
    surface
        .set_color_key(true, CHROMA_KEY)
        .map_err(|error| format!("failed to set color key for {name}: {error}"))?;
    let mut texture = texture_creator
        .create_texture_from_surface(&surface)
        .map_err(|error| format!("failed to create texture from {name}: {error}"))?;
    texture.set_blend_mode(BlendMode::Blend);
    Ok(texture)
}

fn remove_chroma_fringe(surface: &mut Surface<'_>) {
    let width = surface.width() as usize;
    let height = surface.height() as usize;
    let pitch = surface.pitch() as usize;
    surface.with_lock_mut(|pixels| {
        for y in 0..height {
            for x in 0..width {
                let offset = y * pitch + x * 4;
                let [red, green, blue, _alpha] = pixels[offset..offset + 4] else {
                    unreachable!("RGBA32 pixels always have four channels");
                };
                if is_chroma_fringe(red, green, blue) {
                    pixels[offset..offset + 4].copy_from_slice(&[255, 0, 255, 255]);
                }
            }
        }
    });
}

fn is_chroma_fringe(red: u8, green: u8, blue: u8) -> bool {
    red > 180 && green < 64 && blue > 140
}

pub fn traffic_light_frame(green: bool) -> Rect {
    Rect::new(
        i32::from(green) * TRAFFIC_LIGHT_FRAME_W as i32,
        0,
        TRAFFIC_LIGHT_FRAME_W,
        TRAFFIC_LIGHT_FRAME_H,
    )
}

pub fn route_color(route: usize) -> Color {
    match route {
        0 => Color::RGB(153, 113, 228),
        1 => Color::RGB(240, 163, 58),
        _ => Color::RGB(68, 199, 179),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn car_sheet_contains_one_frame_per_route() {
        let mut source = RWops::from_bytes(CARS_BMP).unwrap();
        let surface = Surface::load_bmp_rw(&mut source).unwrap();
        assert_eq!(surface.width(), CAR_FRAME_W);
        assert_eq!(surface.height(), CAR_FRAME_H * CAR_ROUTES);
    }

    #[test]
    fn padded_texture_keeps_the_original_car_size() {
        assert_eq!(CAR_TEXTURE_W - 2 * CAR_PAD, CAR_FRAME_W);
        assert_eq!(CAR_TEXTURE_H - 2 * CAR_PAD, CAR_FRAME_H);
    }

    #[test]
    fn fringe_cleanup_preserves_route_colors_and_lamps() {
        for color in [
            route_color(0),
            route_color(1),
            route_color(2),
            Color::RGB(255, 246, 214),
            Color::RGB(200, 60, 60),
        ] {
            assert!(!is_chroma_fringe(color.r, color.g, color.b));
        }
        assert!(is_chroma_fringe(255, 0, 255));
        assert!(is_chroma_fringe(196, 17, 206));
    }
}
