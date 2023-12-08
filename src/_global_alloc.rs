use talc::*;

// note: miri thinks this violates stacked borrows.
// this only occurs if #[global_allocator] is used.
// use the allocator API if you want nice things.

static mut ARENA: [u8; 156_000] = [0; 156_000];

#[global_allocator]
static ALLOCATOR: Talck<spin::Mutex<()>, ClaimOnOom> =
    Talc::new(unsafe { ClaimOnOom::new(Span::from_array(&mut ARENA)) }).lock();
