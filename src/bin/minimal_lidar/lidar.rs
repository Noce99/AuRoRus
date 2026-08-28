use crate::topics::{SCAN_TOPIC_NAME, Scan};
use efficient_data_sharing::{Executor, Topic, TopicHandler};
use std::any::Any;
use std::thread;
use std::time::Duration;

pub struct Lidar {
    pub id: u8,
    pub rate_hz: f64,
}

impl Lidar {
    pub fn new(rate_hz: f64) -> Self {
        Self { id: 0, rate_hz }
    }
}

impl Executor for Lidar {
    fn init(&mut self, id: u8) {
        self.id = id
    }
    fn run(&mut self, topics: &TopicHandler) {
        let scan_topic = topics.topic::<Scan>(SCAN_TOPIC_NAME);
        scan_topic
            .set_writer(self.id)
            .expect("scan topic already has a different writer");

        println!("Lidar Started!");

        let interval = Duration::from_secs_f64(1.0 / self.rate_hz);
        while topics.is_running(self.id) {
            scan_topic
                .write(self.id, Scan::new())
                .expect("lost writer authorization for lidar_scan topic");
            thread::sleep(interval);
        }

        println!("Lidar Finished!");
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
}
