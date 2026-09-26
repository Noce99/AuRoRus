//! [`RaceLinePublisher`]: publishes one fixed race line on one vehicle's
//! race line topic - e.g. the line an opponent follows, which is picked once
//! when the opponent is added, unlike the ego vehicle's, which
//! [`crate::sensors::MapServer`] switches as the driver picks another.

use crate::topics::{Color, Drawing, SelectedRaceLine, Shape, VehicleTopics};
use crate::{Captain, Executor, Ticker};
use std::any::Any;

/// How often [`RaceLinePublisher`] checks whether it should stop, in Hz - it
/// has nothing else to do once its line is published.
const IDLE_RATE_HZ: f64 = 10.0;

/// Publishes `line` on its vehicle's [`VehicleTopics::race_line`] once, and
/// draws it - thin, in `color` - until stopped.
pub struct RaceLinePublisher {
    id: u16,
    name: String,
    vehicle: VehicleTopics,
    line: SelectedRaceLine,
    color: Color,
}

impl RaceLinePublisher {
    pub fn new(
        name: impl Into<String>,
        vehicle: VehicleTopics,
        line: SelectedRaceLine,
        color: Color,
    ) -> Self {
        Self {
            id: 0,
            name: name.into(),
            vehicle,
            line,
            color,
        }
    }
}

/// `line`, drawn as a thin closed polyline in `color`.
fn drawing(line: &SelectedRaceLine, color: Color) -> Drawing {
    let points = line
        .points
        .iter()
        .map(|point| [point.x as f32, point.y as f32])
        .collect();
    Drawing::default()
        .element(
            "Race line",
            [Shape::Polyline {
                points,
                closed: true,
                width_px: 1.5,
                color: color.with_alpha(160),
            }],
            true,
        )
        // Above the map's own race line drawing, below the vehicles.
        .z_index(-90)
}

impl Executor for RaceLinePublisher {
    fn init(&mut self, id: u16) {
        self.id = id;
    }

    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<SelectedRaceLine>(
            &self.vehicle.race_line(),
            self.id,
            SelectedRaceLine::default,
        );
        captain.claim_drawing(self.id);
    }

    fn run(&mut self, captain: &Captain) {
        captain
            .drawing(self.id)
            .write(self.id, drawing(&self.line, self.color))
            .expect("lost writer authorization for the race line's drawing topic");
        captain
            .topic::<SelectedRaceLine>(&self.vehicle.race_line())
            .write(self.id, self.line.clone())
            .expect("lost writer authorization for the race_line topic");

        let mut ticker = Ticker::new(IDLE_RATE_HZ);
        while captain.is_running(self.id) {
            ticker.wait();
        }
    }

    fn name(&self) -> String {
        self.name.clone()
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn fresh(&self) -> Box<dyn Executor> {
        Box::new(Self::new(
            self.name.clone(),
            self.vehicle.clone(),
            self.line.clone(),
            self.color,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::environment::SpeedPoint;

    #[test]
    fn the_line_is_drawn_closed_in_its_color() {
        let line = SelectedRaceLine {
            points: vec![
                SpeedPoint {
                    x: 0.0,
                    y: 0.0,
                    speed_mps: 1.0,
                },
                SpeedPoint {
                    x: 1.0,
                    y: 0.0,
                    speed_mps: 1.0,
                },
                SpeedPoint {
                    x: 1.0,
                    y: 1.0,
                    speed_mps: 1.0,
                },
            ],
            ..SelectedRaceLine::default()
        };
        let drawing = drawing(&line, Color::BLUE);
        assert_eq!(
            drawing.shapes,
            vec![Shape::Polyline {
                points: vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
                closed: true,
                width_px: 1.5,
                color: Color::BLUE.with_alpha(160),
            }]
        );
    }
}
