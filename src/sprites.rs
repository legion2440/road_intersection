//! BMP sprite-sheet loading and source-frame selection.

use sdl2::pixels::Color;
use sdl2::rect::Rect;
use sdl2::render::{BlendMode, Texture, TextureCreator};
use sdl2::surface::Surface;
use sdl2::video::WindowContext;
use std::path::Path;

pub const CAR_FRAME_W: u32 = 30;
pub const CAR_FRAME_H: u32 = 17;
pub const CAR_FRAMES: u32 = 2;
pub const CAR_ROUTES: u32 = 3;
pub const TRAFFIC_LIGHT_FRAME_W: u32 = 17;
pub const TRAFFIC_LIGHT_FRAME_H: u32 = 35;
pub const TRAFFIC_LIGHT_FRAMES: u32 = 2;

const CAR_ANIMATION_TICKS: u64 = 10;
const CHROMA_KEY: Color = Color::RGB(255, 0, 255);
const CARS_PATH: &str = "assets/cars.bmp";
const TRAFFIC_LIGHTS_PATH: &str = "assets/traffic_lights.bmp";

pub struct SpriteSheets<'a> {
    pub cars: Texture<'a>,
    pub traffic_lights: Texture<'a>,
}

impl<'a> SpriteSheets<'a> {
    pub fn load(texture_creator: &'a TextureCreator<WindowContext>) -> Result<Self, String> {
        Ok(Self {
            cars: build_cars(texture_creator)?,
            traffic_lights: build_traffic_lights(texture_creator)?,
        })
    }
}

pub fn build_cars<'a>(
    texture_creator: &'a TextureCreator<WindowContext>,
) -> Result<Texture<'a>, String> {
    load_sheet(
        texture_creator,
        CARS_PATH,
        CAR_FRAME_W * CAR_FRAMES,
        CAR_FRAME_H * CAR_ROUTES,
    )
}

fn build_traffic_lights<'a>(
    texture_creator: &'a TextureCreator<WindowContext>,
) -> Result<Texture<'a>, String> {
    load_sheet(
        texture_creator,
        TRAFFIC_LIGHTS_PATH,
        TRAFFIC_LIGHT_FRAME_W * TRAFFIC_LIGHT_FRAMES,
        TRAFFIC_LIGHT_FRAME_H,
    )
}

fn load_sheet<'a>(
    texture_creator: &'a TextureCreator<WindowContext>,
    path: &str,
    expected_width: u32,
    expected_height: u32,
) -> Result<Texture<'a>, String> {
    let mut surface = Surface::load_bmp(Path::new(path))
        .map_err(|error| format!("failed to load sprite sheet {path}: {error}"))?;
    if surface.width() != expected_width || surface.height() != expected_height {
        return Err(format!(
            "invalid sprite sheet {path}: expected {expected_width}x{expected_height}, got {}x{}",
            surface.width(),
            surface.height()
        ));
    }
    surface
        .set_color_key(true, CHROMA_KEY)
        .map_err(|error| format!("failed to set color key for {path}: {error}"))?;
    let mut texture = texture_creator
        .create_texture_from_surface(&surface)
        .map_err(|error| format!("failed to create texture from {path}: {error}"))?;
    texture.set_blend_mode(BlendMode::Blend);
    Ok(texture)
}

pub fn car_frame(route: usize, visual_tick: u64) -> Rect {
    let frame = (visual_tick / CAR_ANIMATION_TICKS) % CAR_FRAMES as u64;
    Rect::new(
        frame as i32 * CAR_FRAME_W as i32,
        route as i32 * CAR_FRAME_H as i32,
        CAR_FRAME_W,
        CAR_FRAME_H,
    )
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
        0 => Color::RGB(143, 134, 230),
        1 => Color::RGB(224, 163, 92),
        _ => Color::RGB(79, 201, 180),
    }
}
