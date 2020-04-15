//! This library acts as a small extension to the `nannou_laser` library, providing a small set of
//! SMODE-specific items as they are required during development of the plugin.
//!
//! We use this library within our C++ SMODE plugin by compiling it to a static library, generating
//! a C header using `cbindgen` and linking to the static library using the traditional SMODE
//! approach. Please see the `README.md` for a detailed guide.
//!
//! This library re-exports the `nannou_laser` library in its entirety.

#[doc(inline)]
pub use nannou_laser::*;
use std::sync::mpsc;

/// The maximum number of enqueued frame messages allowed to avoid leaking.
pub const QUEUE_LIMIT: usize = 8;

/// Stored by the thread responsible for "push"ing frames to the device.
///
/// The queue is bounded to avoid leaking in the case that the device callback is stopped while the
/// pushing thread continues.
#[repr(C)]
pub struct FrameSender {
    inner: *mut FrameSenderInner,
}

/// Used within the device callback to retrieve the most recent frame msg that has been pushed.
#[repr(C)]
pub struct FrameReceiver {
    inner: *mut FrameReceiverInner,
}

/// A fully prepared frame containing a list of sequences to render during the frame render
/// callback.
#[repr(C)]
pub struct FrameMsg {
    inner: *mut FrameMsgInner,
}

/// Indicator for distinguishing between whether a sequence is of points or lines.
#[repr(C)]
pub enum SequenceType {
    Points,
    Lines,
}

struct FrameSenderInner {
    tx: mpsc::SyncSender<FrameMsg>,
}

struct FrameReceiverInner {
    rx: mpsc::Receiver<FrameMsg>,
}

struct FrameMsgInner {
    sequences: Vec<Sequence>,
}

/// A single sequence of points, representing either lines or points.
struct Sequence {
    ty: SequenceType,
    points: Vec<Point>,
}

/// Create a new queue for safely passing prepared frames across threads.
#[no_mangle]
pub unsafe extern "C" fn frame_queue_new(f_tx: *mut FrameSender, f_rx: *mut FrameReceiver) {
    let (tx, rx) = mpsc::sync_channel(QUEUE_LIMIT);
    let tx = Box::new(FrameSenderInner { tx });
    let rx = Box::new(FrameReceiverInner { rx });
    *f_tx = FrameSender {
        inner: Box::into_raw(tx),
    };
    *f_rx = FrameReceiver {
        inner: Box::into_raw(rx),
    };
}

/// Create a new empty frame that we may begin to prepare.
#[no_mangle]
pub unsafe extern "C" fn frame_msg_new(frame_msg: *mut FrameMsg) {
    let inner_boxed = Box::new(FrameMsgInner { sequences: vec![] });
    let inner = Box::into_raw(inner_boxed);
    *frame_msg = FrameMsg { inner };
}

/// Add the given sequence to the frame message.
///
/// This function copies the given points into a new new slice owned by the `FrameMsg`.
#[no_mangle]
pub unsafe extern "C" fn frame_msg_add_sequence(
    frame_msg: *mut FrameMsg,
    ty: SequenceType,
    points: *const Point,
    len: usize,
) {
    let frame_msg: &mut FrameMsg = &mut *frame_msg;
    let sequence = sequence_new(ty, points, len);
    (*frame_msg.inner).sequences.push(sequence);
}

/// Send the given frame over the channel.
///
/// Takes ownership over the given `frame_msg`.
///
/// Returns `true` if the message sent successfully. Returns `false` if the channel has been closed
/// or if the number of queued messages exceeds `QUEUE_LIMIT`.
#[no_mangle]
pub unsafe extern "C" fn send_frame_msg(frame_tx: *const FrameSender, frame_msg: FrameMsg) -> bool {
    let frame_tx: &FrameSender = &*frame_tx;

    let res = (*frame_tx.inner).tx.try_send(frame_msg);
    match res {
        Ok(_) => true,
        Err(mpsc::TrySendError::Full(frame_msg))
        | Err(mpsc::TrySendError::Disconnected(frame_msg)) => {
            frame_msg_drop(frame_msg);
            false
        }
    }
}

/// Receive the most recent frame msg if there is one waiting.
///
/// All other pending frame messages will be dropped.
///
/// Returns `true` if a message was received, `false` if no message was received.
#[no_mangle]
pub unsafe extern "C" fn recv_frame_msg(
    frame_rx: *const FrameReceiver,
    frame_msg: *mut FrameMsg,
) -> bool {
    let frame_rx: &FrameReceiver = &*frame_rx;
    let mut last_msg = None;
    for msg in (*frame_rx.inner).rx.try_iter() {
        // If we have another frame, drop the last one.
        if let Some(msg) = last_msg.take() {
            frame_msg_drop(msg);
        }
        last_msg = Some(msg);
    }
    match last_msg {
        None => false,
        Some(msg) => {
            *frame_msg = msg;
            true
        }
    }
}

/// Extend the contents of the given `Frame` with the sequences contained within the given
/// `FrameMsg`.
#[no_mangle]
pub unsafe extern "C" fn extend_frame_with_msg(frame: *mut ffi::Frame, frame_msg: *const FrameMsg) {
    let frame_msg: &FrameMsg = &*frame_msg;
    if frame_msg.inner != std::ptr::null_mut() {
        let msg_inner: &FrameMsgInner = &*frame_msg.inner;
        for sequence in &msg_inner.sequences {
            let points = sequence.points.as_ptr();
            let len = sequence.points.len();
            match sequence.ty {
                SequenceType::Points => ffi::frame_add_points(frame, points, len),
                SequenceType::Lines => ffi::frame_add_lines(frame, points, len),
            }
        }
    }
}

/// Take ownership over the given `FrameSender` and free its resources.
#[no_mangle]
pub unsafe extern "C" fn frame_sender_drop(tx: FrameSender) {
    if tx.inner != std::ptr::null_mut() {
        Box::from_raw(tx.inner);
    }
}

/// Take ownership over the given `FrameReceiver` and free its resources.
#[no_mangle]
pub unsafe extern "C" fn frame_receiver_drop(rx: FrameReceiver) {
    if rx.inner != std::ptr::null_mut() {
        Box::from_raw(rx.inner);
    }
}

/// Take ownership over the given `FrameMsg` and free its resources.
#[no_mangle]
pub unsafe extern "C" fn frame_msg_drop(frame_msg: FrameMsg) {
    if frame_msg.inner != std::ptr::null_mut() {
        Box::from_raw(frame_msg.inner);
    }
}

unsafe fn sequence_new(ty: SequenceType, points: *const Point, len: usize) -> Sequence {
    let slice = std::slice::from_raw_parts(points, len);
    let points = slice.to_vec();
    Sequence { ty, points }
}
