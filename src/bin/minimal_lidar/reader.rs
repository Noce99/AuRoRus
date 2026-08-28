use crate::topics::{SCAN_TOPIC_NAME, Scan};
use efficient_data_sharing::{Executor, Topic, TopicHandler};

use std::any::Any;
use std::thread;
use std::time::Duration;

pub struct Reader {
    pub id: u8,
    pub rate_hz: f64,
}

impl Reader {
    pub fn new(rate_hz: f64) -> Self {
        Self { id: 0, rate_hz }
    }
}

impl Executor for Reader {
    fn init(&mut self, id: u8) {
        self.id = id
    }
    fn run(&mut self, topics: &TopicHandler) {
        let scan_topic = topics.topic::<Scan>(SCAN_TOPIC_NAME);

        println!("Executor [{}] Started!", self.id);

        let interval = Duration::from_secs_f64(1.0 / self.rate_hz);
        while topics.is_running(self.id) {
            let _scan = scan_topic.read();
            // Just sleeping but the work should go there
            thread::sleep(interval);
        }

        println!("Executor [{}] Finished!", self.id);
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
}
