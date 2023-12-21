use embassy_rp::gpio;
use embassy_time::{Duration, Timer};

pub trait TimedToggle {
    #[allow(async_fn_in_trait)]
    async fn toggle_for(&mut self, hold_time: Duration);
}

impl<P: gpio::Pin> TimedToggle for gpio::Output<'static, P> {
    async fn toggle_for(&mut self, hold_time: Duration) {
        self.toggle();
        Timer::after(hold_time).await;
        self.toggle();
    }
}
