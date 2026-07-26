//! Passive rendering of the immutable state produced by `Sim::step`.

use crate::drawing::{
    filled_box, filled_circle, filled_polygon, line, rounded_box, rounded_rectangle, thick_line,
};
use crate::geometry::*;
use crate::lights::Phase;
use crate::sprites::{
    route_color, traffic_light_frame, SpriteSheets, CAR_TEXTURE_H, CAR_TEXTURE_W,
    TRAFFIC_LIGHT_FRAME_H, TRAFFIC_LIGHT_FRAME_W,
};
use crate::vehicle::{Sim, Vehicle};
use fontdue::Font;
use sdl2::pixels::{Color, PixelFormatEnum};
use sdl2::rect::{Point, Rect};
use sdl2::render::{BlendMode, Canvas, TextureCreator};
use sdl2::video::{Window, WindowContext};

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
const ACCENT: Color = Color::RGB(145, 132, 217);
const PANEL_BG: Color = Color::RGB(20, 21, 31);
const CARD_BG: Color = Color::RGB(24, 26, 36);
const BODY_FONT_SIZE: f32 = 13.0;
const SMALL_FONT_SIZE: f32 = 11.0;
const TITLE_FONT_SIZE: f32 = 22.0;
const SIGNAL_FONT_SIZE: f32 = 19.0;
const STAT_FONT_SIZE: f32 = 22.0;
const PANEL_CONTENT_X: i32 = W as i32 + 20;
const PANEL_CONTENT_WIDTH: i32 = PANEL_W as i32 - 40;
const CONTROL_TOP_Y: i32 = 540;
const CONTROL_ROW_GAP: i32 = 10;
const CONTROL_BUTTON_HEIGHT: i32 = 46;
const CONTROL_COLUMN_GAP: i32 = 10;
const CONTROL_COLUMN_WIDTH: i32 = (PANEL_CONTENT_WIDTH - 2 * CONTROL_COLUMN_GAP) / 3;
const CONTROL_BOTTOM_Y: i32 = CONTROL_TOP_Y + 3 * (CONTROL_BUTTON_HEIGHT + CONTROL_ROW_GAP);
const CONTROL_BOTTOM_HEIGHT: i32 = 44;
const TURN_SIGNAL_HALF_PERIOD_TICKS: u64 = FIXED_HZ as u64 / 3;
const TURN_SIGNAL_COLOR: Color = Color::RGB(255, 166, 48);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PanelAction {
    Spawn(usize),
    SpawnRandom,
    TogglePause,
    Reset,
}

#[derive(Clone, Copy, Debug)]
pub struct ViewState {
    pub visual_tick: u64,
    pub paused: bool,
    pub rejected_origin_mask: u8,
}

pub struct UiFont(Font);

impl UiFont {
    pub fn embedded() -> Result<Self, String> {
        Font::from_bytes(
            include_bytes!("../assets/font.ttf").as_slice(),
            fontdue::FontSettings::default(),
        )
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
    font: &UiFont,
    view: ViewState,
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
                    (x, CROSSWALK_START - CY),
                    (x + bar_width - 4.0, CROSSWALK_START - CY),
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
            base_stop_line_rect(),
            origin,
            if view.rejected_origin_mask & (1 << origin) != 0 {
                RED_ON
            } else {
                STOP
            },
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
            .copy(
                &sprites.traffic_lights,
                Some(traffic_light_frame(green)),
                Some(destination),
            )
            .map_err(|error| format!("failed to draw traffic-light sprite: {error}"))?;
    }

    if matches!(sim.lights.phase, Phase::Clearing) {
        draw_clearing_outline(canvas);
    }

    for vehicle in &sim.vehicles {
        let destination = Rect::from_center(
            (
                vehicle.position.0.round() as i32,
                vehicle.position.1.round() as i32,
            ),
            CAR_TEXTURE_W,
            CAR_TEXTURE_H,
        );
        canvas
            .copy_ex(
                &sprites.cars[vehicle.route],
                None,
                Some(destination),
                vehicle.angle.to_degrees(),
                None,
                false,
                false,
            )
            .map_err(|error| format!("failed to draw car sprite: {error}"))?;
        if turn_signal_visible(
            &sim.paths[vehicle.origin][vehicle.route],
            vehicle.route,
            vehicle.progress,
            view.visual_tick,
        ) {
            draw_turn_signal(canvas, vehicle);
        }
    }

    draw_panel(canvas, texture_creator, sim, font, view.paused);
    canvas.present();
    Ok(())
}

fn base_stop_line_rect() -> [(f64, f64); 4] {
    let half_thickness = STOP_LINE_THICKNESS / 2.0;
    [
        (CX - LANE, STOP_LINE_COORD - half_thickness),
        (CX, STOP_LINE_COORD - half_thickness),
        (CX, STOP_LINE_COORD + half_thickness),
        (CX - LANE, STOP_LINE_COORD + half_thickness),
    ]
}

fn draw_clearing_outline(canvas: &mut Canvas<Window>) {
    let color = Color::RGBA(145, 132, 217, 150);
    let left = (CX - LANE) as i16;
    let right = (CX + LANE) as i16;
    let top = (CY - LANE) as i16;
    let bottom = (CY + LANE) as i16;
    thick_line(canvas, left, top, right, top, 2, color);
    thick_line(canvas, right, top, right, bottom, 2, color);
    thick_line(canvas, right, bottom, left, bottom, 2, color);
    thick_line(canvas, left, bottom, left, top, 2, color);
}

fn turn_signal_visible(path: &Path, route: usize, progress: f64, visual_tick: u64) -> bool {
    turn_signal_active(path, route, progress)
        && (visual_tick / TURN_SIGNAL_HALF_PERIOD_TICKS).is_multiple_of(2)
}

fn turn_signal_active(path: &Path, route: usize, progress: f64) -> bool {
    if !matches!(route, 1 | 2) {
        return false;
    }

    let signal_start = path.stop_progress / 2.0;
    let curve_end = path.cum[path.cum.len() - 2];
    let signal_end = (curve_end + CAR_LEN / 2.0).min(path.len);
    progress >= signal_start && progress < signal_end
}

fn draw_turn_signal(canvas: &mut Canvas<Window>, vehicle: &Vehicle) {
    let forward = (vehicle.angle.cos(), vehicle.angle.sin());
    let side = (-forward.1, forward.0);
    let longitudinal_offset = CAR_LEN / 2.0 - 1.5;
    let lateral_offset = if vehicle.route == 1 {
        -(CAR_W / 2.0 - 2.5)
    } else {
        CAR_W / 2.0 - 2.5
    };

    for longitudinal in [-longitudinal_offset, longitudinal_offset] {
        let x = vehicle.position.0 + forward.0 * longitudinal + side.0 * lateral_offset;
        let y = vehicle.position.1 + forward.1 * longitudinal + side.1 * lateral_offset;
        filled_circle(
            canvas,
            x.round() as i16,
            y.round() as i16,
            2,
            TURN_SIGNAL_COLOR,
        );
    }
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
    text_sized(
        canvas,
        texture_creator,
        font,
        value,
        x,
        y,
        BODY_FONT_SIZE,
        color,
    );
}

#[allow(clippy::too_many_arguments)]
fn text_sized(
    canvas: &mut Canvas<Window>,
    texture_creator: &TextureCreator<WindowContext>,
    font: &UiFont,
    value: &str,
    x: i32,
    y: i32,
    size: f32,
    color: Color,
) {
    if value.is_empty() {
        return;
    }

    let line_metrics = match font.0.horizontal_line_metrics(size) {
        Some(metrics) => metrics,
        None => return,
    };
    let baseline = line_metrics.ascent.ceil() as i32;
    let height = line_metrics.new_line_size.ceil().max(1.0) as usize;
    let mut pen_x = 0.0;
    let mut glyphs = Vec::new();

    for character in value.chars() {
        let (metrics, bitmap) = font.0.rasterize(character, size);
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

fn text_width(font: &UiFont, value: &str, size: f32) -> i32 {
    value
        .chars()
        .map(|character| font.0.metrics(character, size).advance_width)
        .sum::<f32>()
        .ceil() as i32
}

#[allow(clippy::too_many_arguments)]
fn text_centered(
    canvas: &mut Canvas<Window>,
    texture_creator: &TextureCreator<WindowContext>,
    font: &UiFont,
    value: &str,
    center_x: i32,
    y: i32,
    size: f32,
    color: Color,
) {
    text_sized(
        canvas,
        texture_creator,
        font,
        value,
        center_x - text_width(font, value, size) / 2,
        y,
        size,
        color,
    );
}

fn card(canvas: &mut Canvas<Window>, x: i32, y: i32, width: i32, height: i32) {
    rounded_box(
        canvas,
        x as i16,
        y as i16,
        (x + width) as i16,
        (y + height) as i16,
        8,
        CARD_BG,
    );
    rounded_rectangle(
        canvas,
        x as i16,
        y as i16,
        (x + width) as i16,
        (y + height) as i16,
        8,
        BLOCK_EDGE,
    );
}

fn draw_arrow(
    canvas: &mut Canvas<Window>,
    center_x: i16,
    center_y: i16,
    direction_x: i16,
    direction_y: i16,
    color: Color,
) {
    let start_x = center_x - direction_x * 5;
    let start_y = center_y - direction_y * 5;
    let end_x = center_x + direction_x * 5;
    let end_y = center_y + direction_y * 5;
    thick_line(canvas, start_x, start_y, end_x, end_y, 2, color);

    let base_x = end_x - direction_x * 4;
    let base_y = end_y - direction_y * 4;
    let perpendicular_x = -direction_y * 3;
    let perpendicular_y = direction_x * 3;
    line(
        canvas,
        end_x,
        end_y,
        base_x + perpendicular_x,
        base_y + perpendicular_y,
        color,
    );
    line(
        canvas,
        end_x,
        end_y,
        base_x - perpendicular_x,
        base_y - perpendicular_y,
        color,
    );
}

fn control_rects() -> [(PanelAction, Rect); 7] {
    let left_x = PANEL_CONTENT_X;
    let center_x = left_x + CONTROL_COLUMN_WIDTH + CONTROL_COLUMN_GAP;
    let right_x = center_x + CONTROL_COLUMN_WIDTH + CONTROL_COLUMN_GAP;
    let row_step = CONTROL_BUTTON_HEIGHT + CONTROL_ROW_GAP;
    let bottom_width = (PANEL_CONTENT_WIDTH - CONTROL_COLUMN_GAP) / 2;

    [
        (
            PanelAction::Spawn(2),
            Rect::new(
                center_x,
                CONTROL_TOP_Y,
                CONTROL_COLUMN_WIDTH as u32,
                CONTROL_BUTTON_HEIGHT as u32,
            ),
        ),
        (
            PanelAction::Spawn(1),
            Rect::new(
                left_x,
                CONTROL_TOP_Y + row_step,
                CONTROL_COLUMN_WIDTH as u32,
                CONTROL_BUTTON_HEIGHT as u32,
            ),
        ),
        (
            PanelAction::SpawnRandom,
            Rect::new(
                center_x,
                CONTROL_TOP_Y + row_step,
                CONTROL_COLUMN_WIDTH as u32,
                CONTROL_BUTTON_HEIGHT as u32,
            ),
        ),
        (
            PanelAction::Spawn(3),
            Rect::new(
                right_x,
                CONTROL_TOP_Y + row_step,
                CONTROL_COLUMN_WIDTH as u32,
                CONTROL_BUTTON_HEIGHT as u32,
            ),
        ),
        (
            PanelAction::Spawn(0),
            Rect::new(
                center_x,
                CONTROL_TOP_Y + 2 * row_step,
                CONTROL_COLUMN_WIDTH as u32,
                CONTROL_BUTTON_HEIGHT as u32,
            ),
        ),
        (
            PanelAction::TogglePause,
            Rect::new(
                left_x,
                CONTROL_BOTTOM_Y,
                bottom_width as u32,
                CONTROL_BOTTOM_HEIGHT as u32,
            ),
        ),
        (
            PanelAction::Reset,
            Rect::new(
                left_x + bottom_width + CONTROL_COLUMN_GAP,
                CONTROL_BOTTOM_Y,
                bottom_width as u32,
                CONTROL_BOTTOM_HEIGHT as u32,
            ),
        ),
    ]
}

pub fn panel_action_at(x: i32, y: i32) -> Option<PanelAction> {
    control_rects()
        .into_iter()
        .find_map(|(action, rect)| rect.contains_point(Point::new(x, y)).then_some(action))
}

fn draw_button_background(canvas: &mut Canvas<Window>, rect: Rect, accent: bool) {
    let x2 = rect.x() + rect.width() as i32;
    let y2 = rect.y() + rect.height() as i32;
    rounded_box(
        canvas,
        rect.x() as i16,
        rect.y() as i16,
        x2 as i16,
        y2 as i16,
        8,
        CARD_BG,
    );
    rounded_rectangle(
        canvas,
        rect.x() as i16,
        rect.y() as i16,
        x2 as i16,
        y2 as i16,
        8,
        if accent { ACCENT } else { BLOCK_EDGE },
    );
}

#[allow(clippy::too_many_arguments)]
fn draw_direction_button(
    canvas: &mut Canvas<Window>,
    texture_creator: &TextureCreator<WindowContext>,
    font: &UiFont,
    rect: Rect,
    direction_x: i16,
    direction_y: i16,
    key_label: &str,
) {
    draw_button_background(canvas, rect, false);
    let center_x = rect.x() + rect.width() as i32 / 2;
    let center_y = rect.y() + rect.height() as i32 / 2;
    draw_arrow(
        canvas,
        (center_x - 10) as i16,
        center_y as i16,
        direction_x,
        direction_y,
        TEXT,
    );
    text_sized(
        canvas,
        texture_creator,
        font,
        key_label,
        center_x + 10,
        center_y - 7,
        SMALL_FONT_SIZE,
        MUTED,
    );
}

fn draw_text_button(
    canvas: &mut Canvas<Window>,
    texture_creator: &TextureCreator<WindowContext>,
    font: &UiFont,
    rect: Rect,
    label: &str,
    accent: bool,
) {
    draw_button_background(canvas, rect, accent);
    text_centered(
        canvas,
        texture_creator,
        font,
        label,
        rect.x() + rect.width() as i32 / 2,
        rect.y() + 13,
        BODY_FONT_SIZE,
        if accent { ACCENT } else { TEXT },
    );
}

fn draw_panel(
    canvas: &mut Canvas<Window>,
    texture_creator: &TextureCreator<WindowContext>,
    sim: &Sim,
    font: &UiFont,
    paused: bool,
) {
    let panel_x = W as i32;
    let content_x = PANEL_CONTENT_X;
    let content_width = PANEL_CONTENT_WIDTH;
    filled_box(
        canvas,
        panel_x as i16,
        0,
        (WIN_W - 1) as i16,
        (H - 1) as i16,
        PANEL_BG,
    );
    line(
        canvas,
        panel_x as i16,
        0,
        panel_x as i16,
        H as i16,
        BLOCK_EDGE,
    );

    text_sized(
        canvas,
        texture_creator,
        font,
        "Road Intersection",
        content_x,
        18,
        TITLE_FONT_SIZE,
        TEXT,
    );
    text_sized(
        canvas,
        texture_creator,
        font,
        "SDL2 traffic simulation",
        content_x,
        48,
        SMALL_FONT_SIZE,
        MUTED,
    );

    let active_y = 78;
    card(canvas, content_x, active_y, content_width, 94);
    text_sized(
        canvas,
        texture_creator,
        font,
        "ACTIVE SIGNAL",
        content_x + 14,
        active_y + 12,
        SMALL_FONT_SIZE,
        ACCENT,
    );
    let (signal_color, signal_label) = match sim.lights.phase {
        Phase::Clearing => (RED_ON, "All-red - clearing".to_string()),
        Phase::Green => (
            GREEN_ON,
            format!(
                "From {}",
                ["North", "East", "South", "West"][sim.lights.green_dir]
            ),
        ),
    };
    filled_circle(
        canvas,
        (content_x + 20) as i16,
        (active_y + 46) as i16,
        6,
        signal_color,
    );
    text_sized(
        canvas,
        texture_creator,
        font,
        &signal_label,
        content_x + 36,
        active_y + 34,
        SIGNAL_FONT_SIZE,
        TEXT,
    );
    text_sized(
        canvas,
        texture_creator,
        font,
        "One approach moves at a time",
        content_x + 14,
        active_y + 66,
        SMALL_FONT_SIZE,
        MUTED,
    );
    text_sized(
        canvas,
        texture_creator,
        font,
        "All-red clearance keeps the box collision-free",
        content_x + 14,
        active_y + 79,
        SMALL_FONT_SIZE,
        MUTED,
    );

    let queues = sim.queue_lengths();
    let lane_capacity = sim.capacity();
    let table_y = 188;
    let table_height = 148;
    card(canvas, content_x, table_y, content_width, table_height);
    let signal_x = content_x + 150;
    let queue_x = content_x + 220;
    let capacity_x = content_x + 276;
    for (label, x) in [
        ("APPROACH", content_x + 14),
        ("SIGNAL", signal_x),
        ("QUEUE", queue_x),
        ("CAP.", capacity_x),
    ] {
        text_sized(
            canvas,
            texture_creator,
            font,
            label,
            x,
            table_y + 12,
            SMALL_FONT_SIZE,
            MUTED,
        );
    }

    let names = ["North", "East", "South", "West"];
    let directions = [(0, 1), (-1, 0), (0, -1), (1, 0)];
    for origin in 0..4 {
        let row_y = table_y + 40 + origin as i32 * 26;
        if origin > 0 {
            line(
                canvas,
                (content_x + 14) as i16,
                (row_y - 7) as i16,
                (content_x + content_width - 14) as i16,
                (row_y - 7) as i16,
                Color::RGB(45, 47, 60),
            );
        }
        text(
            canvas,
            texture_creator,
            font,
            names[origin],
            content_x + 14,
            row_y,
            TEXT,
        );
        draw_arrow(
            canvas,
            (content_x + 72) as i16,
            (row_y + 8) as i16,
            directions[origin].0,
            directions[origin].1,
            MUTED,
        );
        filled_circle(
            canvas,
            (signal_x + 10) as i16,
            (row_y + 8) as i16,
            5,
            if sim.lights.is_green(origin) {
                GREEN_ON
            } else {
                RED_ON
            },
        );
        text(
            canvas,
            texture_creator,
            font,
            &queues[origin].to_string(),
            queue_x + 8,
            row_y,
            if queues[origin] >= lane_capacity {
                RED_ON
            } else {
                TEXT
            },
        );
        text(
            canvas,
            texture_creator,
            font,
            &lane_capacity.to_string(),
            capacity_x + 8,
            row_y,
            MUTED,
        );
    }

    let legend_y = 352;
    card(canvas, content_x, legend_y, content_width, 60);
    text_sized(
        canvas,
        texture_creator,
        font,
        "ROUTE COLOUR CODE",
        content_x + 14,
        legend_y + 10,
        SMALL_FONT_SIZE,
        ACCENT,
    );
    for (route, label, x) in [
        (0, "Straight", content_x + 14),
        (1, "Left turn", content_x + 106),
        (2, "Right turn", content_x + 210),
    ] {
        rounded_box(
            canvas,
            x as i16,
            (legend_y + 36) as i16,
            (x + 16) as i16,
            (legend_y + 46) as i16,
            3,
            route_color(route),
        );
        text_sized(
            canvas,
            texture_creator,
            font,
            label,
            x + 22,
            legend_y + 32,
            SMALL_FONT_SIZE,
            TEXT,
        );
    }

    let stats_y = 428;
    let stats_gap = 10;
    let stat_width = (content_width - 2 * stats_gap) / 3;
    for (index, (value, label, color)) in [
        (sim.spawned.to_string(), "Spawned", TEXT),
        (sim.passed.to_string(), "Cleared", ACCENT),
        (sim.vehicles.len().to_string(), "On road", TEXT),
    ]
    .into_iter()
    .enumerate()
    {
        let x = content_x + index as i32 * (stat_width + stats_gap);
        card(canvas, x, stats_y, stat_width, 70);
        text_centered(
            canvas,
            texture_creator,
            font,
            &value,
            x + stat_width / 2,
            stats_y + 12,
            STAT_FONT_SIZE,
            color,
        );
        text_centered(
            canvas,
            texture_creator,
            font,
            label,
            x + stat_width / 2,
            stats_y + 44,
            SMALL_FONT_SIZE,
            MUTED,
        );
    }

    let controls_y = 518;
    text_sized(
        canvas,
        texture_creator,
        font,
        "SPAWN A VEHICLE",
        content_x + 14,
        controls_y + 12,
        SMALL_FONT_SIZE,
        ACCENT,
    );

    let rejected_label = format!("Rejected: {}", sim.rejected);
    text_sized(
        canvas,
        texture_creator,
        font,
        &rejected_label,
        content_x + content_width - text_width(font, &rejected_label, SMALL_FONT_SIZE),
        controls_y + 12,
        SMALL_FONT_SIZE,
        MUTED,
    );

    for (action, rect) in control_rects() {
        match action {
            PanelAction::Spawn(2) => {
                draw_direction_button(canvas, texture_creator, font, rect, 0, -1, "From S")
            }
            PanelAction::Spawn(1) => {
                draw_direction_button(canvas, texture_creator, font, rect, -1, 0, "From E")
            }
            PanelAction::Spawn(3) => {
                draw_direction_button(canvas, texture_creator, font, rect, 1, 0, "From W")
            }
            PanelAction::Spawn(0) => {
                draw_direction_button(canvas, texture_creator, font, rect, 0, 1, "From N")
            }
            PanelAction::SpawnRandom => {
                draw_text_button(canvas, texture_creator, font, rect, "R", true)
            }
            PanelAction::TogglePause => draw_text_button(
                canvas,
                texture_creator,
                font,
                rect,
                if paused { "Resume" } else { "Pause" },
                paused,
            ),
            PanelAction::Reset => {
                draw_text_button(canvas, texture_creator, font, rect, "Reset", false)
            }
            PanelAction::Spawn(_) => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn panel_buttons_map_to_their_actions() {
        for (action, rect) in control_rects() {
            assert_eq!(
                panel_action_at(
                    rect.x() + rect.width() as i32 / 2,
                    rect.y() + rect.height() as i32 / 2
                ),
                Some(action)
            );
        }
    }

    #[test]
    fn scene_click_is_not_a_panel_action() {
        assert_eq!(panel_action_at((W / 2) as i32, (H / 2) as i32), None);
    }

    #[test]
    fn stop_line_precedes_the_crosswalk() {
        let rect = base_stop_line_rect();
        let far_edge = rect
            .iter()
            .map(|point| point.1)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(far_edge < CROSSWALK_START);
    }

    #[test]
    fn indicators_start_mid_approach_and_end_after_the_turn() {
        let paths = build_paths();
        for route in [1, 2] {
            let path = &paths[0][route];
            let signal_start = path.stop_progress / 2.0;
            let signal_end = (path.cum[path.cum.len() - 2] + CAR_LEN / 2.0).min(path.len);

            assert!(!turn_signal_active(path, route, signal_start - 0.01));
            assert!(turn_signal_active(path, route, signal_start));
            assert!(turn_signal_active(path, route, signal_end - 0.01));
            assert!(!turn_signal_active(path, route, signal_end));

            assert!(turn_signal_visible(path, route, signal_start, 0));
            assert!(!turn_signal_visible(
                path,
                route,
                signal_start,
                TURN_SIGNAL_HALF_PERIOD_TICKS
            ));
            assert!(turn_signal_visible(
                path,
                route,
                signal_start,
                2 * TURN_SIGNAL_HALF_PERIOD_TICKS
            ));
        }

        let straight = &paths[0][0];
        assert!(!turn_signal_active(
            straight,
            0,
            straight.stop_progress / 2.0
        ));
    }
}
