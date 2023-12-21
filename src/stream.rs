use core::ops::Deref;

use embassy_sync::{blocking_mutex::raw::RawMutex, pubsub::PubSubChannel};
use futures::prelude::{stream, Stream};

pub fn channel_to_stream<
    M: RawMutex,
    T: Clone,
    const CAP: usize,
    const SUBS: usize,
    const PUBS: usize,
>(
    channel: &'static PubSubChannel<M, T, CAP, SUBS, PUBS>,
) -> impl Stream<Item = T> {
    stream::unfold(channel.subscriber().unwrap(), |mut sub| async {
        // TODO(shyndman): Maybe add optional logging on lagged streams?
        Some((sub.next_message_pure().await, sub))
    })
}
