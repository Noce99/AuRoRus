use efficient_data_sharing::{ExecutorHandler, TopicHandler};
use std::sync::{Arc};
use std::thread;
use std::time::{Duration};
use crate::topics::{Scan, SCAN_TOPIC_NAME};

mod topics;
mod lidar;
mod reader;

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

    let mut topics = TopicHandler::new();
    topics.register_topic::<Scan>(SCAN_TOPIC_NAME, Scan::new());
    let topics = Arc::new(topics);

    let lidar = lidar::Lidar{id:0, rate_hz: WRITER_HZ};
    let reader_1 = reader::Reader{id:0, rate_hz: READER_RATES_HZ[0]};
    let reader_2 = reader::Reader{id:0, rate_hz: READER_RATES_HZ[1]};
    let reader_3 = reader::Reader{id:0, rate_hz: READER_RATES_HZ[2]};

    let mut handler = ExecutorHandler::new(topics.clone());
    let lidar_id = handler.add_executor(Box::new(lidar));
    let reader_1_id = handler.add_executor(Box::new(reader_1));
    handler.add_executor(Box::new(reader_2));
    handler.add_executor(Box::new(reader_3));
    handler.run_all();

    thread::sleep(Duration::from_millis(500));

    // Swap reader_1 out for a fresh Reader instance under the same id, without
    // disturbing the lidar or the other two readers.
    println!("Switching executor [{}]...", reader_1_id);
    handler
        .switch_executor(reader_1_id, Box::new(reader::Reader { id: 0, rate_hz: READER_RATES_HZ[0] }))
        .expect("reader_1 should still be running");

    thread::sleep(Duration::from_millis(500));

    println!("Switching executor [{}]...", lidar_id);
    handler
        .switch_executor(lidar_id, Box::new(lidar::Lidar{id:0, rate_hz: WRITER_HZ}))
        .expect("lidar should still be running");

    thread::sleep(Duration::from_millis(500));
    topics.stop();

    handler.join_all();
}