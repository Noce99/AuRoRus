use crate::topics::{SCAN_TOPIC_NAME, Scan};
use aurorus::{Captain, Executor, Ticker};
use std::any::Any;

pub struct Lidar {
    pub id: u8,
    pub rate_hz: f64,
    pub name: String,
}

impl Lidar {
    pub fn new(rate_hz: f64, name: String) -> Self {
        Self {
            id: 0,
            rate_hz,
            name,
        }
    }
}

impl Executor for Lidar {
    fn init(&mut self, id: u8) {
        self.id = id
    }
    fn claim_writing_topics(&mut self, captain: &Captain) {
        captain.claim_writer::<Scan>(SCAN_TOPIC_NAME, self.id, Scan::new);
    }
    fn run(&mut self, captain: &Captain) {
        let scan_topic = captain.topic::<Scan>(SCAN_TOPIC_NAME);

        println!("{} Started!", self.name);

        let mut ticker = Ticker::new(self.rate_hz);
        while captain.is_running(self.id) {
            scan_topic
                .write(self.id, Scan::new())
                .expect("lost writer authorization for lidar_scan topic");
            ticker.wait();
        }

        println!("{} Finished!", self.name);
    }
    fn name(&self) -> String {
        self.name.clone()
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn fresh(&self) -> Box<dyn Executor> {
        Box::new(Lidar::new(self.rate_hz, self.name.clone()))
    }
}
