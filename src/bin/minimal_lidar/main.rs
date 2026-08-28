use crate::topics::{SCAN_TOPIC_NAME, Scan};
use efficient_data_sharing::{Executor, Runner};
use std::thread;
use std::time::Duration;

mod lidar;
mod reader;
mod topics;

// Same rates used by the lidar_benchmark binary, minus the 300 Hz reader (there are
// only 3 readers here).
const WRITER_HZ: f64 = 50.0;
const READER_RATES_HZ: [f64; 3] = [30.0, 60.0, 150.0];

fn main() {
    println!("Executor frequencies:");
    println!("  Lidar (writer) - {WRITER_HZ} Hz");
    for (i, hz) in READER_RATES_HZ.iter().enumerate() {
        println!("  Reader {} - {hz} Hz", i + 1);
    }
    println!();

    let mut runner = Runner::new();
    runner.register_topic::<Scan>(SCAN_TOPIC_NAME, Scan::new());

    let lidar = lidar::Lidar::new(WRITER_HZ);
    let reader_1 = reader::Reader::new(READER_RATES_HZ[0]);
    let reader_2 = reader::Reader::new(READER_RATES_HZ[1]);
    let reader_3 = reader::Reader::new(READER_RATES_HZ[2]);

    let lidar_id = runner.add_executor(lidar.boxed());
    let reader_1_id = runner.add_executor(reader_1.boxed());
    runner.add_executor(reader_2.boxed());
    runner.add_executor(reader_3.boxed());
    runner.run_all();

    thread::sleep(Duration::from_millis(500));

    // Swap reader_1 out for a fresh Reader instance under the same id, without
    // disturbing the lidar or the other two readers.
    println!("Switching executor [{}]...", reader_1_id);
    runner
        .switch_executor(reader_1_id, reader::Reader::new(READER_RATES_HZ[0]).boxed())
        .expect("reader_1 should still be running");

    thread::sleep(Duration::from_millis(500));

    println!("Switching executor [{}]...", lidar_id);
    runner
        .switch_executor(lidar_id, lidar::Lidar::new(WRITER_HZ).boxed())
        .expect("lidar should still be running");

    thread::sleep(Duration::from_millis(500));
    runner.stop();

    runner.join_all();
}
