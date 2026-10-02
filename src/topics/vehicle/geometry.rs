//! The [`VehicleGeometry`] topic: the vehicle's size, published by whatever
//! drives it - [`crate::actuators::Vesc`] from the car's calibration,
//! [`crate::simulation::SimulatedVehicle`] from the one it simulates (see
//! [`crate::calibration::CarCalibration`]) - so every algorithm uses the one
//! measured wheelbase and width instead of keeping its own copy.

/// Name of the topic [`VehicleGeometry`] is published on.
pub const VEHICLE_GEOMETRY_TOPIC_NAME: &str = "vehicle_geometry";

/// The vehicle's size - see [`crate::calibration::Geometry`]. Its reference
/// point (every pose's, and [`crate::topics::VehicleStatus`]'s) is its center of
/// gravity, which the body is drawn centered on.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct VehicleGeometry {
    /// Between the front and rear axles, in meters.
    pub wheelbase_m: f64,
    /// From the rear axle forward to the reference point, in meters.
    pub rear_axle_to_cg_m: f64,
    /// Between the left and right wheels' centers, in meters.
    pub track_width_m: f64,
    /// The body's overall length and width, in meters.
    pub body_length_m: f64,
    pub body_width_m: f64,
}

impl VehicleGeometry {
    /// From the reference point forward to the front axle, in meters.
    pub fn lf_m(&self) -> f64 {
        self.wheelbase_m - self.rear_axle_to_cg_m
    }

    /// From the reference point back to the rear axle, in meters.
    pub fn lr_m(&self) -> f64 {
        self.rear_axle_to_cg_m
    }
}

impl Default for VehicleGeometry {
    /// The template car's (`config/car_template.toml`): a roughly
    /// 1/10-scale RC car.
    fn default() -> Self {
        crate::calibration::CarCalibration::template("template").vehicle_geometry()
    }
}
