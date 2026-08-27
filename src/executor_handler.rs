//! Owns every [`Executor`] and runs them all in parallel against a shared
//! [`TopicHandler`].

use crate::executor::Executor;
use crate::topic_handler::TopicHandler;
use std::sync::Arc;
use std::thread;

/// A collection of executors sharing one [`TopicHandler`], started together.
///
/// Register every executor with [`add_executor`](Self::add_executor), then call
/// [`run_all`](Self::run_all) once everything is ready. Each executor runs on its own
/// thread until the `TopicHandler` it was given reports
/// [`is_running`](TopicHandler::is_running) as `false`.
pub struct ExecutorHandler {
    topics: Arc<TopicHandler>,
    executors: Vec<(u8, Box<dyn Executor>)>,
}

impl ExecutorHandler {
    /// Creates a handler that will run its executors against `topics`.
    pub fn new(topics: Arc<TopicHandler>) -> Self {
        Self {
            topics,
            executors: Vec::new(),
        }
    }

    /// Registers `executor`, assigning it a unique id (in registration order,
    /// starting at 0). The id is passed to the executor's [`Executor::init`]
    /// once it starts running, and returned here in case the caller needs it.
    pub fn add_executor(&mut self, executor: Box<dyn Executor>) -> u8 {
        let id = self.executors.len() as u8;
        self.executors.push((id, executor));
        id
    }

    /// Spawns one thread per registered executor, all at once: each calls
    /// [`Executor::init`] then [`Executor::run`]. Every thread hands its executor
    /// back through its join handle once `run` returns, so the caller can inspect
    /// final state (e.g. downcast via [`Executor::as_any`] to read out results)
    /// after joining.
    pub fn run_all(self) -> Vec<thread::JoinHandle<Box<dyn Executor>>> {
        let topics = self.topics;
        self.executors
            .into_iter()
            .map(|(id, mut executor)| {
                let topics = topics.clone();
                thread::Builder::new()
                    .name(format!("executor-{id}"))
                    .spawn(move || {
                        executor.init(id);
                        executor.run(&topics);
                        executor
                    })
                    .expect("failed to spawn executor thread")
            })
            .collect()
    }
}
