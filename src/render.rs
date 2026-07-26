//! Passive rendering of the immutable state produced by `Sim::step`.

use crate::drawing::{
    filled_box, filled_circle, filled_polygon, line, rounded_box, rounded_rectangle, thick_line,
};
use crate::geometry::*;
use crate::lights::Phase;
use crate::sprites::{
    car_frame, route_color, traffic_light_frame, SpriteSheets, TRAFFIC_LIGHT_FRAME_H,
    TRAFFIC_LIGHT_FRAME_W,
};
use crate::vehicle::Sim;
use fontdue::Font;
use sdl2::pixels::{Color, PixelFormatEnum};
use sdl2::rect::Rect;
use sdl2::render::{BlendMode, Canvas, TextureCreator};
use sdl2::video::{Window, WindowContext};
use std::fs;
use std::path::Path;

const GROUND: Color = Color::RGB(18, 19, 29);
const BLOCK: Color = Color::RGB(26, 28, 40);
const BLOCK_EDGE: Color = Color::RGB(63, 66, 77);
const ASPHALT: Color = Color::RGB(27, 29, 41);
const EDGE: Color = Color::RGB(89, 93, 108);
const DASH: Color = Color::RGB(147, 151, 171);
const STOP: Color = Color::RGB(230, 233, 245);
const RED_ON: Color = Color::RGB(255, 85, 96);
const GREEN_ON: Color = Color::RGB(57, 217, 138);
const TEXT: Color = Color::RGB(233, 233, 237);
const MUTED: Color = Color::RGB(147, 151, 171);
const FONT_SIZE: f32 = 13.0;

pub struct UiFont(Font);

impl UiFont {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, String> {
        let bytes = fs::read(path).map_err(|error| error.to_string())?;
        Font::from_bytes(bytes, fontdue::FontSettings::default())
            .map(UiFont)
            .map_err(|error| error.to_string())
    }
}

fn rot_k(point: (f64, f64), turns: usize) -> (f64, f64) {
    let mut point = point;
    for _ in 0..turns {
        point = (CX - (point.1 - CY), CY + (point.0 - CX));
    }
    point
}

fn poly_rect(canvas: &mut Canvas<Window>, rect: [(f64, f64); 4], turns: usize, color: Color) {
    let points: Vec<_> = rect
        .iter()
        .map(|point| {
            let point = rot_k(*point, turns);
            (point.0 as i16, point.1 as i16)
        })
        .collect();
    filled_polygon(canvas, &points, color);
}

pub fn draw(
    canvas: &mut Canvas<Window>,
    texture_creator: &TextureCreator<WindowContext>,
    sim: &Sim,
    sprites: &SpriteSheets,
    font: Option<&UiFont>,
    visual_tick: u64,
) -> Result<(), String> {
    canvas.set_blend_mode(BlendMode::Blend);
    canvas.set_draw_color(GROUND);
    canvas.clear();

    let blocks = [
        (0.0, 0.0, CX - LANE, CY - LANE),
        (CX + LANE, 0.0, W as f64, CY - LANE),
        (0.0, CY + LANE, CX - LANE, H as f64),
        (CX + LANE, CY + LANE, W as f64, H as f64),
    ];
    for (x0, y0, x1, y1) in blocks {
        rounded_box(
            canvas,
            (x0 + 6.0) as i16,
            (y0 + 6.0) as i16,
            (x1 - 6.0) as i16,
            (y1 - 6.0) as i16,
            10,
            BLOCK,
        );
        rounded_rectangle(
            canvas,
            (x0 + 6.0) as i16,
            (y0 + 6.0) as i16,
            (x1 - 6.0) as i16,
            (y1 - 6.0) as i16,
            10,
            BLOCK_EDGE,
        );
    }

    filled_box(
        canvas,
        (CX - LANE) as i16,
        0,
        (CX + LANE) as i16,
        H as i16,
        ASPHALT,
    );
    filled_box(
        canvas,
        0,
        (CY - LANE) as i16,
        W as i16,
        (CY + LANE) as i16,
        ASPHALT,
    );

    for &x in &[CX - LANE, CX + LANE] {
        line(canvas, x as i16, 0, x as i16, (CY - LANE) as i16, EDGE);
        line(
            canvas,
            x as i16,
            (CY + LANE) as i16,
            x as i16,
            H as i16,
            EDGE,
        );
    }
    for &y in &[CY - LANE, CY + LANE] {
        line(canvas, 0, y as i16, (CX - LANE) as i16, y as i16, EDGE);
        line(
            canvas,
            (CX + LANE) as i16,
            y as i16,
            W as i16,
            y as i16,
            EDGE,
        );
    }

    draw_dashes(canvas, CX, 0.0, CX, CY - LANE);
    draw_dashes(canvas, CX, CY + LANE, CX, H as f64);
    draw_dashes(canvas, 0.0, CY, CX - LANE, CY);
    draw_dashes(canvas, CX + LANE, CY, W as f64, CY);

    for origin in 0..4 {
        let bar_width = (2.0 * LANE) / 8.0;
        for stripe in 0..8 {
            let x = -LANE + stripe as f64 * bar_width + 2.0;
            poly_rect(
                canvas,
                [
                    (x, -LANE - 18.0),
                    (x + bar_width - 4.0, -LANE - 18.0),
                    (x + bar_width - 4.0, -LANE - 8.0),
                    (x, -LANE - 8.0),
                ]
                .map(|point| (CX + point.0, CY + point.1)),
                origin,
                Color::RGBA(207, 211, 229, 36),
            );
        }
        poly_rect(
            canvas,
            [
                (-LANE, -LANE - 6.0),
                (0.0, -LANE - 6.0),
                (0.0, -LANE - 1.0),
                (-LANE, -LANE - 1.0),
            ]
            .map(|point| (CX + point.0, CY + point.1)),
            origin,
            STOP,
        );

        let green = sim.lights.is_green(origin);
        let (housing_x, housing_y) = (-LANE - 18.0, -LANE - 2.0);
        let center = rot_k((CX + housing_x, CY + housing_y), origin);
        let destination = Rect::from_center(
            (center.0 as i32, center.1 as i32),
            TRAFFIC_LIGHT_FRAME_W,
            TRAFFIC_LIGHT_FRAME_H,
        );
        canvas
            .copy_ex(
                &sprites.traffic_lights,
                Some(traffic_light_frame(green)),
                Some(destination),
                origin as f64 * 90.0,
                None,
                false,
                false,
            )
            .map_err(|error| format!("failed to draw traffic-light sprite: {error}"))?;
    }

    for vehicle in &sim.vehicles {
        let destination = Rect::from_center(
            (vehicle.position.0 as i32, vehicle.position.1 as i32),
            CAR_LEN as u32,
            CAR_W as u32,
        );
        canvas
            .copy_ex(
                &sprites.cars,
                Some(car_frame(vehicle.route, visual_tick)),
                Some(destination),
                vehicle.angle.to_degrees(),
                None,
                false,
                false,
            )
            .map_err(|error| format!("failed to draw car sprite: {error}"))?;
    }

    if let Some(font) = font {
        draw_hud(canvas, texture_creator, sim, font);
    }
    canvas.present();
    Ok(())
}

fn draw_dashes(canvas: &mut Canvas<Window>, x0: f64, y0: f64, x1: f64, y1: f64) {
    let (dx, dy) = (x1 - x0, y1 - y0);
    let length = dx.hypot(dy);
    let (unit_x, unit_y) = (dx / length, dy / length);
    let mut progress = 0.0;
    while progress < length {
        let end = (progress + 14.0).min(length);
        thick_line(
            canvas,
            (x0 + unit_x * progress) as i16,
            (y0 + unit_y * progress) as i16,
            (x0 + unit_x * end) as i16,
            (y0 + unit_y * end) as i16,
            2,
            DASH,
        );
        progress += 26.0;
    }
}

fn text(
    canvas: &mut Canvas<Window>,
    texture_creator: &TextureCreator<WindowContext>,
    font: &UiFont,
    value: &str,
    x: i32,
    y: i32,
    color: Color,
) {
    if value.is_empty() {
        return;
    }

    let line_metrics = match font.0.horizontal_line_metrics(FONT_SIZE) {
        Some(metrics) => metrics,
        None => return,
    };
    let baseline = line_metrics.ascent.ceil() as i32;
    let height = line_metrics.new_line_size.ceil().max(1.0) as usize;
    let mut pen_x = 0.0;
    let mut glyphs = Vec::new();

    for character in value.chars() {
        let (metrics, bitmap) = font.0.rasterize(character, FONT_SIZE);
        let advance_width = metrics.advance_width;
        glyphs.push((pen_x, metrics, bitmap));
        pen_x += advance_width;
    }

    let width = pen_x.ceil().max(1.0) as usize;
    let mut pixels = vec![0u8; width * height * 4];
    for (pen_x, metrics, bitmap) in glyphs {
        let glyph_x = pen_x.round() as i32 + metrics.xmin;
        let glyph_y = baseline - metrics.height as i32 - metrics.ymin;
        for row in 0..metrics.height {
            for column in 0..metrics.width {
                let destination_x = glyph_x + column as i32;
                let destination_y = glyph_y + row as i32;
                if destination_x < 0
                    || destination_y < 0
                    || destination_x >= width as i32
                    || destination_y >= height as i32
                {
                    continue;
                }
                let alpha = bitmap[row * metrics.width + column];
                let offset = (destination_y as usize * width + destination_x as usize) * 4;
                pixels[offset] = color.r;
                pixels[offset + 1] = color.g;
                pixels[offset + 2] = color.b;
                pixels[offset + 3] = ((alpha as u16 * color.a as u16) / 255) as u8;
            }
        }
    }

    if let Ok(mut texture) = texture_creator.create_texture_streaming(
        PixelFormatEnum::RGBA32,
        width as u32,
        height as u32,
    ) {
        texture.set_blend_mode(BlendMode::Blend);
        if texture.update(None, &pixels, width * 4).is_ok() {
            let _ = canvas.copy(
                &texture,
                None,
                Some(Rect::new(x, y, width as u32, height as u32)),
            );
        }
    }
}

fn draw_hud(
    canvas: &mut Canvas<Window>,
    texture_creator: &TextureCreator<WindowContext>,
    sim: &Sim,
    font: &UiFont,
) {
    rounded_box(canvas, 14, 14, 290, 238, 8, Color::RGBA(20, 22, 30, 220));
    rounded_rectangle(canvas, 14, 14, 290, 238, 8, BLOCK_EDGE);

    let signal = match sim.lights.phase {
        Phase::Clearing => "All-red - clearing".to_string(),
        Phase::Green => format!(
            "Green: {}",
            ["North", "East", "South", "West"][sim.lights.green_dir]
        ),
    };
    text(
        canvas,
        texture_creator,
        font,
        "ROAD INTERSECTION",
        26,
        24,
        MUTED,
    );
    text(canvas, texture_creator, font, &signal, 26, 44, TEXT);

    let queues = sim.queue_lengths();
    let lane_capacity = sim.capacity();
    let names = ["N down", "E left", "S up", "W right"];
    for origin in 0..4 {
        let y = 78 + origin as i32 * 22;
        filled_circle(
            canvas,
            34,
            (y + 8) as i16,
            5,
            if sim.lights.is_green(origin) {
                GREEN_ON
            } else {
                RED_ON
            },
        );
        text(canvas, texture_creator, font, names[origin], 46, y, TEXT);

        let full_width = 150.0;
        let filled_width =
            (queues[origin] as f64 / lane_capacity as f64 * full_width).min(full_width) as i16;
        filled_box(
            canvas,
            112,
            y as i16,
            (112.0 + full_width) as i16,
            (y + 12) as i16,
            Color::RGB(35, 37, 50),
        );
        if filled_width > 0 {
            filled_box(
                canvas,
                112,
                y as i16,
                112 + filled_width,
                (y + 12) as i16,
                if queues[origin] >= lane_capacity {
                    RED_ON
                } else {
                    route_color(0)
                },
            );
        }
    }

    let stats = format!(
        "spawned {}   passed {}   on road {}",
        sim.spawned,
        sim.passed,
        sim.vehicles.len()
    );
    text(canvas, texture_creator, font, &stats, 26, 174, MUTED);
    text(
        canvas,
        texture_creator,
        font,
        "arrows spawn - r random - esc quit",
        26,
        192,
        MUTED,
    );
    text(canvas, texture_creator, font, "Routes:", 26, 214, MUTED);
    for (route, label, x) in [(0, "straight", 82), (1, "left", 166), (2, "right", 226)] {
        filled_box(canvas, x, 217, x + 8, 225, route_color(route));
        text(
            canvas,
            texture_creator,
            font,
            label,
            x as i32 + 13,
            214,
            TEXT,
        );
    }
}
