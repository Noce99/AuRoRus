use aurorus::{Executor, Runner};
use std::thread;
use std::time::Duration;

mod lidar;
mod reader;
mod topics;

const WRITER_HZ: f64 = 50.0;
const READER_RATES_HZ: [f64; 3] = [30.0, 60.0, 150.0];

fn main() {
    let mut runner = Runner::new();
    runner.activate_verbose();

    let lidar = lidar::Lidar::new(WRITER_HZ, String::from("Lidar"));
    let reader_1 = reader::Reader::new(READER_RATES_HZ[0], format!("Reader {}Hz", READER_RATES_HZ[0]));
    let reader_2 = reader::Reader::new(READER_RATES_HZ[1], format!("Reader {}Hz", READER_RATES_HZ[1]));
    let reader_3 = reader::Reader::new(READER_RATES_HZ[2], format!("Reader {}Hz", READER_RATES_HZ[2]));

    let lidar_id = runner.add_executor(lidar.boxed());
    let reader_1_id = runner.add_executor(reader_1.boxed());
    runner.add_executor(reader_2.boxed());
    runner.add_executor(reader_3.boxed());
    runner.run_all();

    thread::sleep(Duration::from_millis(500));

    // Swap reader_1 out for a fresh Reader instance under the same id, without
    // disturbing the lidar or the other two readers.
    let new_reader_1 =
        reader::Reader::new(READER_RATES_HZ[0], format!("New Reader {}Hz", READER_RATES_HZ[0]));
    runner
        .switch_executor(reader_1_id, new_reader_1.boxed())
        .expect("reader_1 should still be running");

    thread::sleep(Duration::from_millis(500));

    let new_lidar = lidar::Lidar::new(WRITER_HZ, String::from("New Lidar"));
    runner
        .switch_executor(lidar_id, new_lidar.boxed())
        .expect("lidar should still be running");

    thread::sleep(Duration::from_millis(500));
    runner.stop();

    runner.join_all();
}
