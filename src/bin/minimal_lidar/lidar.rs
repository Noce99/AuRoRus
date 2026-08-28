use crate::topics::{SCAN_TOPIC_NAME, Scan};
use aurorus::{Captain, Executor, Topic};
use std::any::Any;
use std::thread;
use std::time::Duration;

pub struct Lidar {
    pub id: u8,
    pub rate_hz: f64,
    pub name: String,
}

impl Lidar {
    pub fn new(rate_hz: f64, name: String) -> Self {
        Self { id: 0, rate_hz, name }
    }
}

impl Executor for Lidar {
    fn init(&mut self, id: u8) {
        self.id = id
    }
    fn run(&mut self, captain: &Captain) {
        let scan_topic = captain.claim_writer::<Scan>(SCAN_TOPIC_NAME, self.id);

        println!("{} Started!", self.name);

        let interval = Duration::from_secs_f64(1.0 / self.rate_hz);
        while captain.is_running(self.id) {
            scan_topic
                .write(self.id, Scan::new())
                .expect("lost writer authorization for lidar_scan topic");
            thread::sleep(interval);
        }

        println!("{} Finished!", self.name);
    }
    fn name(&self) -> String {
        self.name.clone()
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
}
