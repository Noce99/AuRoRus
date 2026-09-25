//! The [`Drawing`] topic: what an executor wants shown on a map view, as a
//! list of [`Shape`]s in the world frame.
//!
//! Drawings are the only thing a map view (e.g. `web_gui`) renders on its
//! canvas. Every executor that wants something on the map claims its own
//! drawing topic via [`crate::Captain::claim_drawing`] - named
//! [`DRAW_TOPIC_PREFIX`] followed by the executor's name, e.g.
//! `draw/SimulatedVehicle` - and publishes a fresh [`Drawing`] whenever what
//! it wants shown changes. A viewer finds every such topic by its prefix, so
//! a new sensor or algorithm shows up on the map without the viewer knowing
//! anything about it: the viewer only knows how to draw each [`Shape`] kind.
//!
//! A drawing's shapes are grouped into named [`DrawingElement`]s - e.g. the map
//! server's "Map", "Race line" and "Start/finish line" - which a viewer
//! lets the user show or hide one by one.

use std::sync::Arc;
use std::time::Duration;

/// Prefix every drawing topic's name starts with - see
/// [`crate::Captain::claim_drawing`].
pub const DRAW_TOPIC_PREFIX: &str = "draw/";

/// Everything one executor currently wants drawn, replacing whatever it
/// published before. Coordinates are in the same world frame (meters) as
/// [`crate::environment::MapInfo`].
///
/// Build one with [`Drawing::element`], which keeps
/// [`shapes`](Self::shapes) and [`elements`](Self::elements) in step.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Drawing {
    pub shapes: Vec<Shape>,
    /// Names [`shapes`](Self::shapes) in consecutive runs: the first
    /// element covers the first `shape_count` shapes, the next one the
    /// following ones, and so on. A viewer shows or hides the shapes of
    /// every element with the same name together.
    pub elements: Vec<DrawingElement>,
    /// How old this drawing may get before a viewer starts fading it out,
    /// in milliseconds - `None` for a drawing that's only republished when
    /// it changes (e.g. the map), which must never fade.
    ///
    /// Only the writer knows how often it publishes, so only it can tell
    /// "stopped publishing" apart from "nothing changed". Pick a value
    /// comfortably above both the writer's own period and a viewer's poll
    /// period (tens of milliseconds) - see [`Drawing::DEFAULT_STALE_AFTER`].
    pub stale_after_ms: Option<u32>,
    /// Paint order between drawings: lower values are painted first, i.e.
    /// underneath. Drawings with equal values are painted in topic-name
    /// order.
    pub z_index: i32,
}

impl Drawing {
    /// A sensible [`stale_after_ms`](Self::stale_after_ms) floor for a
    /// periodically republished drawing.
    pub const DEFAULT_STALE_AFTER: Duration = Duration::from_millis(500);

    /// Appends `shapes` as one element called `name` - kept even when
    /// `shapes` is empty (e.g. no loop closed yet), so the element stays
    /// listed in a viewer rather than coming and going.
    pub fn element(
        mut self,
        name: impl Into<String>,
        shapes: impl IntoIterator<Item = Shape>,
    ) -> Self {
        let before = self.shapes.len();
        self.shapes.extend(shapes);
        self.elements.push(DrawingElement {
            name: name.into(),
            shape_count: u32::try_from(self.shapes.len() - before).unwrap_or(u32::MAX),
        });
        self
    }

    /// Sets [`stale_after_ms`](Self::stale_after_ms).
    pub fn stale_after(mut self, after: Duration) -> Self {
        self.stale_after_ms = Some(u32::try_from(after.as_millis()).unwrap_or(u32::MAX));
        self
    }

    /// Sets [`z_index`](Self::z_index).
    pub fn z_index(mut self, z_index: i32) -> Self {
        self.z_index = z_index;
        self
    }
}

/// A named run of a [`Drawing`]'s shapes - see [`Drawing::elements`].
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct DrawingElement {
    pub name: String,
    pub shape_count: u32,
}

/// An RGBA color, `a = 255` being fully opaque.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: u8,
}

impl Color {
    pub const RED: Self = Self::rgb(0xff, 0x3b, 0x3b);
    pub const AMBER: Self = Self::rgb(0xff, 0xb0, 0x20);
    pub const GREEN: Self = Self::rgb(0x3b, 0xd1, 0x6f);
    pub const BLUE: Self = Self::rgb(0x3b, 0x8e, 0xff);
    pub const WHITE: Self = Self::rgb(0xff, 0xff, 0xff);
    pub const BLACK: Self = Self::rgb(0x10, 0x14, 0x18);
    pub const PURPLE: Self = Self::rgb(0x80, 0x00, 0xff);

    /// A fully opaque color.
    pub const fn rgb(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b, a: 255 }
    }

    /// This color with its alpha replaced.
    pub const fn with_alpha(self, a: u8) -> Self {
        Self { a, ..self }
    }
}

/// One thing to draw. All positions and lengths are in world meters, except
/// the `_px` fields, which are screen pixels - so that e.g. a point or a
/// line stays visible at any zoom level. Angles are in radians, measured
/// like [`crate::topics::VehicleStatus::heading_rad`].
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Shape {
    /// A vehicle, drawn as a body with four wheels, the front two turned
    /// by `steering_rad`.
    Vehicle {
        /// Reference point of the vehicle (e.g. its center of gravity).
        x_m: f64,
        y_m: f64,
        heading_rad: f64,
        /// Signed speed along `heading_rad` - lets a viewer dead-reckon the
        /// vehicle forward between two samples, so it moves smoothly at
        /// display rate instead of stepping once per sample.
        speed_mps: f64,
        /// Front-wheel steering angle, same convention as
        /// [`crate::topics::VescCommand::servo_position_rad`].
        steering_rad: f64,
        /// Body length and width.
        length_m: f64,
        width_m: f64,
        /// Distance from (`x_m`, `y_m`) forward to the front axle, and
        /// backward to the rear axle.
        front_axle_m: f64,
        rear_axle_m: f64,
        color: Color,
    },
    /// A set of dots of a fixed on-screen size, e.g. LIDAR hits.
    Points {
        points: Vec<[f32; 2]>,
        radius_px: f32,
        color: Color,
    },
    /// Connected line segments through `points`, joined back to the first
    /// point if `closed`.
    Polyline {
        points: Vec<[f32; 2]>,
        closed: bool,
        width_px: f32,
        color: Color,
    },
    /// A circle, filled or outlined.
    Circle {
        x_m: f64,
        y_m: f64,
        radius_m: f64,
        filled: bool,
        color: Color,
    },
    /// An arc of the circle centered on (`x_m`, `y_m`), outlined from
    /// direction `start_rad` counterclockwise (increasing angle) to
    /// `end_rad`. A span of `2 * pi` or more draws the full circle.
    CircularArc {
        x_m: f64,
        y_m: f64,
        radius_m: f64,
        start_rad: f64,
        end_rad: f64,
        width_px: f32,
        color: Color,
    },
    /// A circular sector ("pie slice"): the region bounded by the two radii
    /// at `start_rad` and `end_rad` and the [`Shape::CircularArc`] between
    /// them, with the same conventions - filled or outlined.
    CircularSector {
        x_m: f64,
        y_m: f64,
        radius_m: f64,
        start_rad: f64,
        end_rad: f64,
        filled: bool,
        color: Color,
    },
    /// A rectangle centered on (`x_m`, `y_m`), `length_m` along
    /// `heading_rad` and `width_m` across it, filled or outlined. A square
    /// is one with equal sides.
    Rect {
        x_m: f64,
        y_m: f64,
        length_m: f64,
        width_m: f64,
        heading_rad: f64,
        filled: bool,
        color: Color,
    },
    /// A text label anchored (centered) at (`x_m`, `y_m`).
    Text {
        x_m: f64,
        y_m: f64,
        text: String,
        size_px: f32,
        color: Color,
    },
    /// A grayscale image: one byte per pixel, row-major, `0` drawn darkest
    /// and `255` lightest - e.g. an occupancy map (`255` drivable). Pixel
    /// `(0, 0)`'s corner sits at (`origin_x_m`, `origin_y_m`), and rows grow
    /// along +y.
    ///
    /// `pixels` is behind an [`Arc`] so a writer that already holds the
    /// buffer (e.g. [`crate::topics::SelectedMap::pixels`]) can share it
    /// rather than copy it, and so every read of the topic stays cheap.
    /// Viewers may cache the decoded image for as long as the drawing
    /// holding it is unchanged, so put a raster in a drawing that's rarely
    /// republished.
    Raster {
        origin_x_m: f64,
        origin_y_m: f64,
        resolution_m_per_px: f64,
        width_px: u32,
        height_px: u32,
        pixels: Arc<[u8]>,
    },
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Drawings are recorded by the debug recorder like any other topic, so
    /// every shape kind has to survive a bincode round trip.
    #[test]
    fn every_shape_survives_a_bincode_round_trip() {
        let drawing = Drawing::default()
            .element(
                "Everything",
                [
                    Shape::Vehicle {
                        x_m: 1.0,
                        y_m: 2.0,
                        heading_rad: 0.5,
                        speed_mps: -1.0,
                        steering_rad: 0.1,
                        length_m: 0.45,
                        width_m: 0.25,
                        front_axle_m: 0.16,
                        rear_axle_m: 0.16,
                        color: Color::AMBER,
                    },
                    Shape::Points {
                        points: vec![[1.0, 2.0]],
                        radius_px: 2.5,
                        color: Color::RED,
                    },
                    Shape::Polyline {
                        points: vec![[0.0, 0.0], [1.0, 1.0]],
                        closed: true,
                        width_px: 2.0,
                        color: Color::BLUE,
                    },
                    Shape::Circle {
                        x_m: 0.0,
                        y_m: 0.0,
                        radius_m: 1.0,
                        filled: false,
                        color: Color::GREEN,
                    },
                    Shape::CircularArc {
                        x_m: 0.0,
                        y_m: 0.0,
                        radius_m: 1.0,
                        start_rad: -0.5,
                        end_rad: 0.5,
                        width_px: 2.0,
                        color: Color::GREEN,
                    },
                    Shape::CircularSector {
                        x_m: 0.0,
                        y_m: 0.0,
                        radius_m: 1.0,
                        start_rad: -0.5,
                        end_rad: 0.5,
                        filled: true,
                        color: Color::GREEN.with_alpha(64),
                    },
                    Shape::Rect {
                        x_m: 0.0,
                        y_m: 0.0,
                        length_m: 1.0,
                        width_m: 1.0,
                        heading_rad: 0.0,
                        filled: true,
                        color: Color::WHITE,
                    },
                    Shape::Text {
                        x_m: 0.0,
                        y_m: 0.0,
                        text: "hi".into(),
                        size_px: 12.0,
                        color: Color::BLACK.with_alpha(128),
                    },
                    Shape::Raster {
                        origin_x_m: 0.0,
                        origin_y_m: 0.0,
                        resolution_m_per_px: 0.05,
                        width_px: 2,
                        height_px: 1,
                        pixels: vec![0u8, 255].into(),
                    },
                ],
            )
            .element("Nothing", [])
            .stale_after(Drawing::DEFAULT_STALE_AFTER)
            .z_index(-3);

        let encoded = bincode::serde::encode_to_vec(&drawing, bincode::config::standard()).unwrap();
        let (decoded, _): (Drawing, _) =
            bincode::serde::decode_from_slice(&encoded, bincode::config::standard()).unwrap();

        assert_eq!(decoded, drawing);
    }

    #[test]
    fn element_names_consecutive_runs_of_shapes() {
        let dot = |x| Shape::Circle {
            x_m: x,
            y_m: 0.0,
            radius_m: 1.0,
            filled: true,
            color: Color::RED,
        };

        let drawing = Drawing::default()
            .element("Two", [dot(0.0), dot(1.0)])
            .element("None", [])
            .element("One", [dot(2.0)]);

        assert_eq!(drawing.shapes, vec![dot(0.0), dot(1.0), dot(2.0)]);
        let runs: Vec<_> = drawing
            .elements
            .iter()
            .map(|e| (e.name.as_str(), e.shape_count))
            .collect();
        assert_eq!(runs, vec![("Two", 2), ("None", 0), ("One", 1)]);
    }

    #[test]
    fn shapes_serialize_to_json_keyed_by_their_snake_case_kind() {
        let shape = Shape::Circle {
            x_m: 1.0,
            y_m: 2.0,
            radius_m: 3.0,
            filled: true,
            color: Color::RED,
        };

        let json = serde_json::to_value(&shape).unwrap();

        assert_eq!(json["circle"]["radius_m"], 3.0);
        assert_eq!(json["circle"]["color"]["r"], 0xff);
    }
}
