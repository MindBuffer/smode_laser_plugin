#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <new>

namespace smode {
namespace laser {

/// For the blank ab: `[a, a.blanked(), b.blanked(), (0..delay).map(|_| b.blanked())]`.
static const uint32_t BLANK_MIN_POINTS = 3;

/// The DAC sends UDP broadcast messages on port 7654.
///
/// This does not appeared to be documented in the protocol, but was found within the
/// `github.com/j4cbo/j4cDAC` repository.
static const uint16_t BROADCAST_PORT = 7654;

/// Communication with the DAC happens over TCP on port 7765.
static const uint16_t COMMUNICATION_PORT = 7765;

/// The default rate at which the DAC will yield frames of points.
static const uint32_t DEFAULT_FRAME_HZ = 60;

/// The default rate at which the DAC should request points per second.
static const uint32_t DEFAULT_POINT_HZ = 10000;

/// The maximum number of enqueued frame messages allowed to avoid leaking.
static const uintptr_t QUEUE_LIMIT = 8;

enum class IpAddrVersion {
  V4,
  V6,
};

enum class Result {
  Success = 0,
  DetectDacFailed,
  BuildStreamFailed,
  DetectDacsAsyncFailed,
};

/// Indicator for distinguishing between whether a sequence is of points or lines.
enum class SequenceType {
  Points,
  Lines,
};

struct ApiInner;

struct BufferInner;

struct DetectDacsAsyncInner;

struct FrameInner;

struct FrameMsgInner;

struct FrameReceiverInner;

struct FrameSenderInner;

struct FrameStreamInner;

struct RawStreamInner;

/// Allows for detecting and enumerating laser DACs on a network and establishing new streams of
/// communication with them.
struct Api {
  ApiInner *inner;
};

/// A handle to a non-blocking DAC detection thread.
struct DetectDacsAsync {
  DetectDacsAsyncInner *inner;
};

/// Periodically, and as part of ACK packets, the DAC sends its current playback status to the
/// host.
struct DacStatus {
  /// This remains undocumented in the protocol.
  ///
  /// The original implementation source simply sets this to `0`.
  uint8_t protocol;
  /// The current state of the "light engine" state machine.
  uint8_t light_engine_state;
  /// The current state of the "playback" state machine.
  uint8_t playback_state;
  /// The currently-selected data source:
  ///
  /// - `0`: Network streaming (the protocol implemented in this library).
  /// - `1`: ILDA playback from SD card.
  /// - `2`: Internal abstract generator.
  uint8_t source;
  /// If the light engine is `Ready`, this will be `0`.
  ///
  /// Otherwise, bits will be set as follows:
  ///
  /// - `0`: Emergency stop occurred due to E-Stop packet or invalid command.
  /// - `1`: Emergency stop occurred due to E-Stop input to projector.
  /// - `2`: Emergency stop input to projector is currently active.
  /// - `3`: Emergency stop occurred due to over-temperature condition.
  /// - `4`: Over-temperature condition is currently active.
  /// - `5`: Emergency stop occurred due to loss of ethernet link.
  ///
  /// All remaining are reserved for future use.
  uint16_t light_engine_flags;
  /// These flags may be non-zero during normal operation.
  ///
  /// Bits are defined as follows:
  ///
  /// - `0`: **Shutter state**. `0` is closed, `1` is open.
  /// - `1`: **Underflow**. `1` if the last stream ended with underflow rather than a `Stop`
  ///   command. This is reset to `0` by the `Prepare` command.
  /// - `2`: **E-Stop**. `1` if the last stream ended because the E-Stop state was entered. Reset
  ///   to zero by the `Prepare` command.
  uint16_t playback_flags;
  /// This field is undocumented within the protocol reference.
  ///
  /// By looking at the source code of the original implementation, this seems to represent the
  /// state of the current `source`.
  ///
  /// If `source` is set to `1` for ILDA playback from SD card, the following flags are defined:
  ///
  /// - `0`: `ILDA_PLAYER_PLAYING`.
  /// - `1`: `ILDA_PLAYER_REPEAT`.
  ///
  /// If `source` is set to `2` for the internal abstract generator, the flags are defined as
  /// follows:
  ///
  /// - `0`: `ABSTRACT_PLAYING`.
  uint16_t source_flags;
  /// The number of points currently buffered.
  uint16_t buffer_fullness;
  /// If in the `Prepared` or `Playing` playback states, this is the number of points per
  /// second for which the DAC is configured.
  ///
  /// If in the `Idle` playback state, this will be `0`.
  uint32_t point_rate;
  /// If in the `Playing` playback state, this is the number of points that the DAC has actually
  /// emitted since it started playing.
  ///
  /// If in the `Prepared` or `Idle` playback states, this will be `0`.
  uint32_t point_count;
};
/// The light engine is ready.
static const uint8_t DacStatus_LIGHT_ENGINE_READY = 0;
/// In the case where the DAC is also used for thermal control of laser apparatus, this is the
/// state that is entered after power-up.
static const uint8_t DacStatus_LIGHT_ENGINE_WARMUP = 1;
/// Lasers are off but thermal control is still active.
static const uint8_t DacStatus_LIGHT_ENGINE_COOLDOWN = 2;
/// An emergency stop has been triggered, either by an E-stop input on the DAC, an E-stop
/// command over the network, or a fault such as over-temperature.
static const uint8_t DacStatus_LIGHT_ENGINE_EMERGENCY_STOP = 3;
/// The default state:
///
/// - No points may be added to the buffer.
/// - No output is generated.
/// - All analog outputs are at 0v.
/// - The shutter is controlled by the data source.
static const uint8_t DacStatus_PLAYBACK_IDLE = 0;
/// The buffer will accept points.
///
/// The output is the same as the `Idle` state
static const uint8_t DacStatus_PLAYBACK_PREPARED = 1;
/// Points are being sent to the output.
static const uint8_t DacStatus_PLAYBACK_PLAYING = 2;
/// Network streaming (the protocol implemented in this library).
static const uint8_t DacStatus_SOURCE_NETWORK_STREAMING = 0;
/// ILDA playback from SD card.
static const uint8_t DacStatus_SOURCE_ILDA_PLAYBACK_SD = 1;
/// Internal abstract generator.
static const uint8_t DacStatus_SOURCE_INTERNAL_ABSTRACT_GENERATOR = 2;

/// Regardless of the data source being used, each DAC broadcasts a status/ID datagram over UDP to
/// its local network's broadcast address once per second.
struct DacBroadcast {
  /// The unique hardware identifier for the DAC.
  uint8_t mac_address[6];
  /// This field is undocumented in the official protocol but seems to represent a version number
  /// for the hardware in use by the DAC.
  uint16_t hw_revision;
  /// This field is undocumented in the official protocol but seems to represent the version of
  /// the protocol implementation. As of writing this, this is hardcoded as `2` in the original
  /// source.
  uint16_t sw_revision;
  /// The DAC's maximum buffer capacity for storing points that are yet to be converted to
  /// output.
  ///
  /// As of writing this, this is hardcoded to `1800` in the original DAC source code.
  uint16_t buffer_capacity;
  /// The DAC's maximum point rate.
  ///
  /// As of writing this, this is hardcoded to `100_000` in the original DAC source code.
  uint32_t max_point_rate;
  /// The current status of the DAC.
  DacStatus dac_status;
};

struct IpAddr {
  IpAddrVersion version;
  /// 4 bytes used for `V4`, 16 bytes used for `V6`.
  unsigned char bytes[16];
};

struct SocketAddr {
  IpAddr ip;
  unsigned short port;
};

struct DacEtherDream {
  DacBroadcast broadcast;
  SocketAddr source_addr;
};

union DetectedDacKind {
  DacEtherDream ether_dream;
};

struct DetectedDac {
  DetectedDacKind kind;
};

struct Frame {
  FrameInner *inner;
};

/// A fully prepared frame containing a list of sequences to render during the frame render
/// callback.
struct FrameMsg {
  FrameMsgInner *inner;
};

/// A position in 2D space represented by x and y coordinates.
using Position = float[2];

/// Red, green and blue channels of a single colour.
using Rgb = float[3];

/// The point type used within the laser frame stream API.
///
/// The point represents the location to which the scanner should point and the colour that the
/// scanner should be at this point.
///
/// If two consecutive points have two different colours, the `color` values will be linearly
/// interpolated.
struct Point {
  /// The position of the point. `-1` represents the minimum value along the axis and `1`
  /// represents the maximum.
  Position position;
  /// The color of the point.
  Rgb color;
  /// The minimum number of extra times this point should be drawn.
  ///
  /// `0` is the default used for drawing sequences of smooth line segments.
  ///
  /// Values greater than `0` are useful for accenting individual points.
  uint32_t weight;
};
/// The default weight for points used to draw lines.
static const uint32_t Point_DEFAULT_LINE_POINT_WEIGHT = 0;

/// Stored by the thread responsible for "push"ing frames to the device.
///
/// The queue is bounded to avoid leaking in the case that the device callback is stopped while the
/// pushing thread continues.
struct FrameSender {
  FrameSenderInner *inner;
};

/// Used within the device callback to retrieve the most recent frame msg that has been pushed.
struct FrameReceiver {
  FrameReceiverInner *inner;
};

struct StreamConfig {
  const DetectedDac *detected_dac;
  unsigned int point_hz;
  unsigned int latency_points;
};

/// Configuration options for eulerian circuit interpolation.
struct InterpolationConfig {
  /// The minimum distance the interpolator can travel along an edge before a new point is
  /// required.
  float distance_per_point;
  /// The number of points to insert at the end of a blank to account for light modulator delay.
  uint32_t blank_delay_points;
  /// The amount of delay to add based on the angle of the corner in radians.
  float radians_per_point;
};
/// The default distance the interpolator can travel before a new point is required.
static const float InterpolationConfig_DEFAULT_DISTANCE_PER_POINT = 0.1;
/// The default number of points inserted for the end of each blank segment.
static const uint32_t InterpolationConfig_DEFAULT_BLANK_DELAY_POINTS = 10;
/// The default radians per point of delay to reduce corner inertia.
static const float InterpolationConfig_DEFAULT_RADIANS_PER_POINT = 0.6;

struct FrameStreamConfig {
  StreamConfig stream_conf;
  uint32_t frame_hz;
  InterpolationConfig interpolation_conf;
};

struct FrameStream {
  FrameStreamInner *inner;
};

/// Cast to `extern fn(*mut raw::c_void, *mut Frame)` internally.
using FrameRenderCallback = void(*)(void*, Frame*);

struct Buffer {
  BufferInner *inner;
};

/// Cast to `extern fn(*mut raw::c_void, *mut Buffer)` internally.
using RawRenderCallback = void(*)(void*, Buffer*);

struct RawStream {
  RawStreamInner *inner;
};

extern "C" {

/// Must be called in order to correctly clean up the API resources.
void api_drop(Api api);

/// Used for retrieving the last error that occurred from the API.
const char *api_last_error(const Api *api);

/// Given some uninitialized pointer to an `Api` struct, fill it with a new Api instance.
void api_new(Api *api);

/// Retrieve a list of the currently available DACs.
void available_dacs(DetectDacsAsync *detect_dacs_async, DetectedDac **first_dac, unsigned int *len);

/// Block the current thread until a new DAC is detected and return it.
Result detect_dac(Api *api, DetectedDac *detected_dac);

/// Given some uninitialised pointer to a `DetectDacsAsync` struct, fill it with a new instance.
///
/// If the given `timeout_secs` is `0`, DAC detection will never timeout and detected DACs that no
/// longer broadcast will remain accessible in the device map.
Result detect_dacs_async(Api *api, float timeout_secs, DetectDacsAsync *detect_dacs);

/// Must be called in order to correctly clean up the `DetectDacsAsync` resources.
void detect_dacs_async_drop(DetectDacsAsync detect);

/// Used for retrieving the last error that occurred from the API.
const char *detect_dacs_async_last_error(const DetectDacsAsync *detect);

/// Extend the contents of the given `Frame` with the sequences contained within the given
/// `FrameMsg`.
void extend_frame_with_msg(Frame *frame, const FrameMsg *frame_msg);

/// Add a sequence of consecutive lines.
///
/// If some points already exist in the frame, this method will create a blank segment between the
/// previous point and the first point before appending this sequence.
void frame_add_lines(Frame *frame, const Point *points, uintptr_t len);

/// Add a sequence of consecutive points separated by blank space.
///
/// If some points already exist in the frame, this method will create a blank segment between the
/// previous point and the first point before appending this sequence.
void frame_add_points(Frame *frame, const Point *points, uintptr_t len);

uint32_t frame_hz(const Frame *frame);

uint32_t frame_latency_points(const Frame *frame);

/// Add the given sequence to the frame message.
///
/// This function copies the given points into a new new slice owned by the `FrameMsg`.
void frame_msg_add_sequence(FrameMsg *frame_msg,
                            SequenceType ty,
                            const Point *points,
                            uintptr_t len);

/// Take ownership over the given `FrameMsg` and free its resources.
void frame_msg_drop(FrameMsg frame_msg);

/// Create a new empty frame that we may begin to prepare.
void frame_msg_new(FrameMsg *frame_msg);

uint32_t frame_point_hz(const Frame *frame);

/// Create a new queue for safely passing prepared frames across threads.
void frame_queue_new(FrameSender *f_tx, FrameReceiver *f_rx);

/// Take ownership over the given `FrameReceiver` and free its resources.
void frame_receiver_drop(FrameReceiver rx);

/// Take ownership over the given `FrameSender` and free its resources.
void frame_sender_drop(FrameSender tx);

/// Initialise the given frame stream configuration with default values.
void frame_stream_config_default(FrameStreamConfig *conf);

/// Must be called in order to correctly clean up the frame stream.
void frame_stream_drop(FrameStream stream);

/// Spawn a new frame rendering stream.
Result new_frame_stream(Api *api,
                        FrameStream *stream,
                        const FrameStreamConfig *config,
                        void *callback_data,
                        FrameRenderCallback frame_render_callback,
                        RawRenderCallback process_raw_callback);

/// Spawn a new frame rendering stream.
Result new_raw_stream(Api *api,
                      RawStream *stream,
                      const StreamConfig *config,
                      void *callback_data,
                      RawRenderCallback process_raw_callback);

uint32_t points_per_frame(const Frame *frame);

/// Must be called in order to correctly clean up the raw stream.
void raw_stream_drop(RawStream stream);

/// Receive the most recent frame msg if there is one waiting.
///
/// All other pending frame messages will be dropped.
///
/// Returns `true` if a message was received, `false` if no message was received.
bool recv_frame_msg(const FrameReceiver *frame_rx, FrameMsg *frame_msg);

/// Send the given frame over the channel.
///
/// Takes ownership over the given `frame_msg`.
///
/// Returns `true` if the message sent successfully. Returns `false` if the channel has been closed
/// or if the number of queued messages exceeds `QUEUE_LIMIT`.
bool send_frame_msg(const FrameSender *frame_tx, FrameMsg frame_msg);

/// Initialise the given raw stream configuration with default values.
void stream_config_default(StreamConfig *conf);

} // extern "C"

} // namespace laser
} // namespace smode
