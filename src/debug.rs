use alloc::string::String;

use defmt::println;
use embassy_time::Instant;

pub struct StopWatch {
    name: String,
    create_ts: Instant,
}

impl StopWatch {
    pub fn new(name: String) -> Self {
        Self {
            name,
            create_ts: Instant::now(),
        }
    }
}

impl Drop for StopWatch {
    fn drop(&mut self) {
        let elapsed = Instant::now() - self.create_ts;
        println!("{} in {}ms", self.name.as_str(), elapsed.as_millis());
    }
}
