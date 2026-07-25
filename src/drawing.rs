//! Small core-SDL drawing helpers used to keep the project self-contained.

use sdl2::pixels::Color;
use sdl2::rect::{Point, Rect};
use sdl2::render::{Canvas, RenderTarget};

pub fn filled_box<T: RenderTarget>(
    canvas: &mut Canvas<T>,
    x0: i16,
    y0: i16,
    x1: i16,
    y1: i16,
    color: Color,
) {
    if x1 < x0 || y1 < y0 {
        return;
    }
    canvas.set_draw_color(color);
    let _ = canvas.fill_rect(Rect::new(
        x0 as i32,
        y0 as i32,
        (x1 - x0 + 1) as u32,
        (y1 - y0 + 1) as u32,
    ));
}

pub fn line<T: RenderTarget>(
    canvas: &mut Canvas<T>,
    x0: i16,
    y0: i16,
    x1: i16,
    y1: i16,
    color: Color,
) {
    canvas.set_draw_color(color);
    let _ = canvas.draw_line(
        Point::new(x0 as i32, y0 as i32),
        Point::new(x1 as i32, y1 as i32),
    );
}

pub fn thick_line<T: RenderTarget>(
    canvas: &mut Canvas<T>,
    x0: i16,
    y0: i16,
    x1: i16,
    y1: i16,
    width: u8,
    color: Color,
) {
    let dx = (x1 - x0) as f64;
    let dy = (y1 - y0) as f64;
    let length = dx.hypot(dy).max(1.0);
    let nx = -dy / length;
    let ny = dx / length;
    let half = width as i32 / 2;
    for offset in -half..=(width as i32 - half - 1) {
        let offset_x = (nx * offset as f64).round() as i16;
        let offset_y = (ny * offset as f64).round() as i16;
        line(
            canvas,
            x0 + offset_x,
            y0 + offset_y,
            x1 + offset_x,
            y1 + offset_y,
            color,
        );
    }
}

pub fn filled_circle<T: RenderTarget>(
    canvas: &mut Canvas<T>,
    center_x: i16,
    center_y: i16,
    radius: i16,
    color: Color,
) {
    canvas.set_draw_color(color);
    let radius_squared = (radius as i32).pow(2);
    for y in -radius..=radius {
        let half_width = (radius_squared - (y as i32).pow(2)).max(0) as f64;
        let half_width = half_width.sqrt().floor() as i16;
        let _ = canvas.draw_line(
            Point::new((center_x - half_width) as i32, (center_y + y) as i32),
            Point::new((center_x + half_width) as i32, (center_y + y) as i32),
        );
    }
}

pub fn rounded_box<T: RenderTarget>(
    canvas: &mut Canvas<T>,
    x0: i16,
    y0: i16,
    x1: i16,
    y1: i16,
    radius: i16,
    color: Color,
) {
    if x1 < x0 || y1 < y0 {
        return;
    }
    let radius = radius.max(0).min((x1 - x0) / 2).min((y1 - y0) / 2);
    canvas.set_draw_color(color);

    for y in y0..=y1 {
        let relative = if y < y0 + radius {
            y0 + radius - y
        } else if y > y1 - radius {
            y - (y1 - radius)
        } else {
            0
        };
        let inset = if relative == 0 {
            0
        } else {
            let inside = ((radius as i32).pow(2) - (relative as i32).pow(2)).max(0);
            radius - (inside as f64).sqrt().floor() as i16
        };
        let _ = canvas.draw_line(
            Point::new((x0 + inset) as i32, y as i32),
            Point::new((x1 - inset) as i32, y as i32),
        );
    }
}

pub fn rounded_rectangle<T: RenderTarget>(
    canvas: &mut Canvas<T>,
    x0: i16,
    y0: i16,
    x1: i16,
    y1: i16,
    radius: i16,
    color: Color,
) {
    let radius = radius.max(0).min((x1 - x0) / 2).min((y1 - y0) / 2);
    line(canvas, x0 + radius, y0, x1 - radius, y0, color);
    line(canvas, x0 + radius, y1, x1 - radius, y1, color);
    line(canvas, x0, y0 + radius, x0, y1 - radius, color);
    line(canvas, x1, y0 + radius, x1, y1 - radius, color);

    canvas.set_draw_color(color);
    for degrees in 0..=90 {
        let angle = (degrees as f64).to_radians();
        let dx = (radius as f64 * angle.cos()).round() as i16;
        let dy = (radius as f64 * angle.sin()).round() as i16;
        let points = [
            Point::new((x1 - radius + dx) as i32, (y0 + radius - dy) as i32),
            Point::new((x0 + radius - dx) as i32, (y0 + radius - dy) as i32),
            Point::new((x1 - radius + dx) as i32, (y1 - radius + dy) as i32),
            Point::new((x0 + radius - dx) as i32, (y1 - radius + dy) as i32),
        ];
        let _ = canvas.draw_points(points.as_slice());
    }
}

pub fn filled_polygon<T: RenderTarget>(
    canvas: &mut Canvas<T>,
    points: &[(i16, i16)],
    color: Color,
) {
    if points.len() < 3 {
        return;
    }
    let min_y = points.iter().map(|point| point.1).min().unwrap();
    let max_y = points.iter().map(|point| point.1).max().unwrap();
    canvas.set_draw_color(color);

    for y in min_y..=max_y {
        let mut intersections = Vec::new();
        for index in 0..points.len() {
            let (x0, y0) = points[index];
            let (x1, y1) = points[(index + 1) % points.len()];
            if (y0 <= y && y1 > y) || (y1 <= y && y0 > y) {
                let x = x0 as f64 + (y - y0) as f64 * (x1 - x0) as f64 / (y1 - y0) as f64;
                intersections.push(x.round() as i16);
            }
        }
        intersections.sort_unstable();
        for pair in intersections.chunks_exact(2) {
            let _ = canvas.draw_line(
                Point::new(pair[0] as i32, y as i32),
                Point::new(pair[1] as i32, y as i32),
            );
        }
    }
}
