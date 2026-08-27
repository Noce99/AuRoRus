use efficient_data_sharing::{Executor, Topic, TopicHandler};
use crate::topics::{Scan, SCAN_TOPIC_NAME};
use std::any::Any;
use std::thread;
use std::time::Duration;

pub struct Lidar{
    pub id: u8,
    pub rate_hz: f64,
}

impl Executor for Lidar{
    fn init(&mut self, id: u8){
        self.id = id
    }
    fn run(&mut self, topics: &TopicHandler) {
        let scan_topic = topics.topic::<Scan>(SCAN_TOPIC_NAME);
        // A WriterAlreadySet error is fine if it's this executor's own id reclaiming
        // the slot (e.g. after being restarted by ExecutorHandler::switch_executor) -
        // anything else means a genuinely different writer beat us to it.
        if scan_topic.set_writer(self.id).is_err() && scan_topic.writer() != Some(self.id) {
            panic!("scan topic already has a different writer");
        }

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