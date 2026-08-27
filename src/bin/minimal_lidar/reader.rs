use efficient_data_sharing::{Executor, Topic, TopicHandler};
use crate::topics::{Scan, SCAN_TOPIC_NAME};

use std::time::{Duration, SystemTime, UNIX_EPOCH};
use std::any::Any;
use std::thread;

pub struct Reader{
    pub id: u8,
    pub rate_hz: f64,
}

impl Executor for Reader{
    fn init(&mut self, id: u8){
        self.id = id
    }
    fn run(&mut self, topics: &TopicHandler) {
        let scan_topic = topics.topic::<Scan>(SCAN_TOPIC_NAME);

        println!("Executor [{}] Started!", self.id);

        let mut readings: u32 = 0;
        let mut mean_age: f32 = 0.;
        let mut m2_age: f32 = 0.; // sum of squared deviations from the running mean, a la Welford
        let mut min_age: f32 = f32::MAX;
        let mut max_age: f32 = f32::MIN;
        let interval = Duration::from_secs_f64(1.0 / self.rate_hz);
        while topics.is_running(self.id) {
            let scan = scan_topic.read();
            let now = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_micros();
            let age = if now >= scan.timestamp {
                (now - scan.timestamp) as f32
            } else {
                println!("WTF? Executor [{}] is reading a negative time diff? [{} us]", self.id, now as i128 - scan.timestamp as i128);
                0.0
            };
            readings += 1;
            let delta = age - mean_age;
            mean_age += delta / readings as f32;
            m2_age += delta * (age - mean_age);
            min_age = min_age.min(age);
            max_age = max_age.max(age);
            thread::sleep(interval);
        }

        if readings > 0 {
            let sigma_age = if readings > 1 { (m2_age / (readings - 1) as f32).sqrt() } else { 0. };
            println!(
                "Executor [{}] Finished [{} readings] - age: {:.1} +- {:.1} us (min {:.1} us, max {:.1} us)!",
                self.id, readings, mean_age, sigma_age, min_age, max_age
            );
        } else {
            println!("Executor [{}] Finished [0 readings]!", self.id);
        }
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
}