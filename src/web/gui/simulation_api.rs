//! The Simulation panel's API: the parameters the simulated sensors
//! ([`crate::simulation::SimulatedImu`], [`crate::simulation::SimulatedLidar`])
//! run with (their `simulated_*_status` topics), and the endpoints tuning
//! them live (`simulated_*_parameters`) and saving or reloading their config
//! files - per sensor, the same shape as the Detector panel's, see
//! [`super::detector_api`].

use super::live_api::{loaded, stamped_json};
use crate::simulation;
use crate::topics::{
    AlgorithmParameter, SIMULATED_IMU_PARAMETERS_TOPIC_NAME, SIMULATED_IMU_STATUS_TOPIC_NAME,
    SIMULATED_LIDAR_PARAMETERS_TOPIC_NAME, SIMULATED_LIDAR_STATUS_TOPIC_NAME,
    SimulatedSensorParameters, SimulatedSensorStatus,
};
use crate::web::{bad_request, error_response, json_response, read_json};
use crate::{Captain, WriteMeta};
use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Mutex;
use tiny_http::{Request, ResponseBox};

/// The values a sensor's config file holds for its parameters, by name, and
/// the file's path.
type SavedValues = Result<(BTreeMap<String, f64>, PathBuf), String>;

/// One simulated sensor the panel tunes: its topics, and its config file.
pub struct Sensor {
    /// For error messages, e.g. `"IMU"`.
    name: &'static str,
    status_topic: &'static str,
    parameters_topic: &'static str,
    save: fn(&[AlgorithmParameter]) -> Result<PathBuf, String>,
    saved: fn(&[AlgorithmParameter]) -> SavedValues,
}

pub const IMU: Sensor = Sensor {
    name: "IMU",
    status_topic: SIMULATED_IMU_STATUS_TOPIC_NAME,
    parameters_topic: SIMULATED_IMU_PARAMETERS_TOPIC_NAME,
    save: simulation::save_imu_parameters,
    saved: simulation::saved_imu_values,
};

pub const LIDAR: Sensor = Sensor {
    name: "lidar",
    status_topic: SIMULATED_LIDAR_STATUS_TOPIC_NAME,
    parameters_topic: SIMULATED_LIDAR_PARAMETERS_TOPIC_NAME,
    save: simulation::save_lidar_parameters,
    saved: simulation::saved_lidar_values,
};

/// The body of [`simulation`]: every sensor's status, `None` for one that
/// doesn't run in this binary (e.g. on the real car).
#[derive(serde::Serialize)]
struct SimulationBody {
    imu: Option<SimulatedSensorStatus>,
    lidar: Option<SimulatedSensorStatus>,
}

/// `GET /api/simulation` - every simulated sensor's status (the parameter
/// values it runs with), as a `StampedBody` stamped by the IMU's.
pub fn simulation(captain: &Captain) -> ResponseBox {
    let status = |sensor: &Sensor| captain.try_topic::<SimulatedSensorStatus>(sensor.status_topic);
    let imu = status(&IMU);
    let meta = imu
        .as_ref()
        .map_or_else(WriteMeta::default, |topic| topic.meta());
    let body = SimulationBody {
        imu: imu.map(|topic| topic.read().into_value()),
        lidar: status(&LIDAR).map(|topic| topic.read().into_value()),
    };
    stamped_json(&body, meta)
}

/// `sensor`'s tunable parameters with the values it currently runs with -
/// or a `400` response if it reports none.
fn sensor_parameters(
    captain: &Captain,
    sensor: &Sensor,
) -> Result<Vec<AlgorithmParameter>, ResponseBox> {
    let parameters = captain
        .try_topic::<SimulatedSensorStatus>(sensor.status_topic)
        .map(|topic| topic.read().into_value().parameters)
        .unwrap_or_default();
    if parameters.is_empty() {
        return Err(bad_request(&format!(
            "no simulated {} is running",
            sensor.name
        )));
    }
    Ok(parameters)
}

/// Serializes the read-modify-writes of the `simulated_*_parameters` topics
/// across `WebGui`'s worker threads, as the detector's own lock does.
static SIMULATED_SENSOR_PARAMETERS_LOCK: Mutex<()> = Mutex::new(());

/// Read-modify-writes `sensor`'s parameters topic with `change`, under
/// [`SIMULATED_SENSOR_PARAMETERS_LOCK`].
fn write_sensor_parameters(
    captain: &Captain,
    writer_id: u16,
    sensor: &Sensor,
    change: impl FnOnce(&mut SimulatedSensorParameters),
) {
    let _guard = SIMULATED_SENSOR_PARAMETERS_LOCK
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    let topic = captain.topic::<SimulatedSensorParameters>(sensor.parameters_topic);
    let mut wanted = topic.read().into_value();
    change(&mut wanted);
    topic.write(writer_id, wanted).unwrap_or_else(|_| {
        panic!(
            "lost writer authorization for the {} topic",
            sensor.parameters_topic
        )
    });
}

#[derive(serde::Deserialize)]
struct SetSensorParameterBody {
    name: String,
    value: f64,
}

/// `POST /api/simulated_<sensor>_parameter` - body `{"name": "...", "value":
/// ...}` - sets one parameter's wanted value on `sensor`'s parameters topic,
/// for it to apply before its next reading; it reports the value it actually
/// runs with on its status topic.
pub fn set_sensor_parameter(
    request: &mut Request,
    captain: &Captain,
    writer_id: u16,
    sensor: &Sensor,
) -> ResponseBox {
    let body: SetSensorParameterBody = match read_json(request) {
        Ok(body) => body,
        Err(response) => return response,
    };
    if !body.value.is_finite() {
        return bad_request("value must be a finite number");
    }
    let parameters = match sensor_parameters(captain, sensor) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    if !parameters
        .iter()
        .any(|parameter| parameter.name == body.name)
    {
        return bad_request(&format!(
            "the simulated {} has no tunable parameter {:?}",
            sensor.name, body.name
        ));
    }
    write_sensor_parameters(captain, writer_id, sensor, |wanted| {
        wanted.values.insert(body.name, body.value);
    });
    json_response(&(), 200)
}

/// `POST /api/simulated_<sensor>_parameters_save` - writes the parameter
/// values `sensor` currently runs with into its config file, keeping the
/// file's comments. Responds with the file's path.
pub fn save_sensor_parameters(captain: &Captain, sensor: &Sensor) -> ResponseBox {
    let parameters = match sensor_parameters(captain, sensor) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    match (sensor.save)(&parameters) {
        Ok(path) => loaded(&path),
        Err(err) => error_response(500, &err),
    }
}

/// `POST /api/simulated_<sensor>_parameters_load` - asks `sensor` to run
/// with the values its config file holds again, undoing any unsaved tuning,
/// by writing them to its parameters topic like a slider would. Responds
/// with the file's path.
pub fn load_sensor_parameters(captain: &Captain, writer_id: u16, sensor: &Sensor) -> ResponseBox {
    let parameters = match sensor_parameters(captain, sensor) {
        Ok(parameters) => parameters,
        Err(response) => return response,
    };
    let (values, path) = match (sensor.saved)(&parameters) {
        Ok(saved) => saved,
        Err(err) => return error_response(500, &err),
    };
    write_sensor_parameters(captain, writer_id, sensor, |wanted| {
        wanted.values.extend(values)
    });
    loaded(&path)
}
