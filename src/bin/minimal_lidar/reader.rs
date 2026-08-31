use crate::topics::{SCAN_TOPIC_NAME, Scan};
use aurorus::{Captain, Executor};

use std::any::Any;
use std::thread;
use std::time::Duration;

pub struct Reader {
    pub id: u8,
    pub rate_hz: f64,
    pub name: String,
}

impl Reader {
    pub fn new(rate_hz: f64, name: String) -> Self {
        Self { id: 0, rate_hz, name}
    }
}

impl Executor for Reader {
    fn init(&mut self, id: u8) {
        self.id = id
    }
    fn run(&mut self, captain: &Captain) {
        let scan_topic = captain.topic::<Scan>(SCAN_TOPIC_NAME);

        println!("{} [id = {}] Started!", self.name, self.id);

        let interval = Duration::from_secs_f64(1.0 / self.rate_hz);
        while captain.is_running(self.id) {
            let _scan = scan_topic.read();
            // Just sleeping but the work should go there
            thread::sleep(interval);
        }

        println!("{} [id = {}] Finished!", self.name, self.id);
    }
    fn name(&self) -> String {
        self.name.clone()
    }
    fn as_any(&self) -> &dyn Any {
        self
    }
    fn fresh(&self) -> Box<dyn Executor> {
        Box::new(Reader::new(self.rate_hz, self.name.clone()))
    }
}
