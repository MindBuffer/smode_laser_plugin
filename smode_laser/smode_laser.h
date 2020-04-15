#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <new>

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
  CloseStreamFailed,
  NullPointer,
};

/// Indicator for distinguishing between whether a sequence is of points or lines.
enum class SequenceType {
  Points,
  Lines,
};

enum class StreamErrorKind {
  EtherDreamFailedToDetectDacs,
  EtherDreamFailedToConnectStream,
  EtherDreamFailedToPrepareStream,
  EtherDreamFailedToBeginStream,
  EtherDreamFailedToSubmitData,
  EtherDreamFailedToSubmitPointRate,
  EtherDreamFailedToStopStream,
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

struct StreamErrorActionInner;

struct StreamErrorInner;

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

/// An Ether Dream DAC that was detected on the network.
struct DacEtherDream {
  DacBroadcast broadcast;
  SocketAddr source_addr;
};

/// A union for distinguishing between the kind of LASER DAC that was detected. Currently, only
/// EtherDream is supported, however this will gain more variants as more protocols are added (e.g.
/// AVB).
union DetectedDacKind {
  DacEtherDream ether_dream;
};

/// Represents a DAC that has been detected on the network along with any information collected
/// about the DAC in the detection process.
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

/// A handle to a stream that requests frames of LASER data from the user.
///
/// Each "frame" has an optimisation pass applied that optimises the path for inertia, minimal
/// blanking, point de-duplication and segment order.
struct FrameStream {
  FrameStreamInner *inner;
};

/// A set of stream configuration parameters applied to the initialisation of both `Raw` and
/// `Frame` streams.
struct StreamConfig {
  /// A valid pointer to a `DetectedDac` that should be targeted.
  const DetectedDac *detected_dac;
  /// The rate at which the DAC should process points per second.
  ///
  /// This value should be no greater than the detected DAC's `max_point_hz`.
  unsigned int point_hz;
  /// The maximum latency specified as a number of points.
  ///
  /// Each time the laser indicates its "fullness", the raw stream will request enough points
  /// from the render function to fill the DAC buffer up to `latency_points`.
  ///
  /// This value should be no greaterthan the DAC's `buffer_capacity`.
  unsigned int latency_points;
  /// The timeout duration of the stream in seconds.
  ///
  /// A negative value indicates that the stream should never timeout. This is the default case.
  float tcp_timeout_secs;
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

/// A set of stream configuration parameters unique to `Frame` streams.
struct FrameStreamConfig {
  StreamConfig stream_conf;
  /// The rate at which the stream will attempt to present images via the DAC. This value is used
  /// in combination with the DAC's `point_hz` in order to determine how many points should be
  /// used to draw each frame. E.g.
  ///
  /// ```ignore
  /// let points_per_frame = point_hz / frame_hz;
  /// ```
  ///
  /// This is simply used as a minimum value. E.g. if some very simple geometry is submitted, this
  /// allows the DAC to spend more time creating the path for the image. However, if complex geometry
  /// is submitted that would require more than the ideal `points_per_frame`, the DAC may not be able
  /// to achieve the desired `frame_hz` when drawing the path while also taking the
  /// `distance_per_point` and `radians_per_point` into consideration.
  uint32_t frame_hz;
  /// Configuration options for eulerian circuit interpolation.
  InterpolationConfig interpolation_conf;
};

using FrameRenderCallback = void(*)(void*, Frame*);

struct Buffer {
  BufferInner *inner;
};

using RawRenderCallback = void(*)(void*, Buffer*);

struct StreamError {
  const StreamErrorInner *inner;
};

struct StreamErrorAction {
  StreamErrorActionInner *inner;
};

using StreamErrorCallback = void(*)(void*, const StreamError*, StreamErrorAction*);

/// A handle to a raw LASER stream that requests the exact number of points that the DAC is
/// awaiting in each call to the user's callback.
struct RawStream {
  RawStreamInner *inner;
};

/// An owned instance of a raw C string.
struct RawString {
  char *inner;
};

extern "C" {

/// Must be called in order to correctly clean up the API resources.
void api_drop(Api api);

/// Used for retrieving the last error that occurred from the API.
const char *api_last_error(const Api *api);

/// Given some uninitialized pointer to an `Api` struct, fill it with a new Api instance.
void api_new(Api *api);

/// Retrieve a list of the currently available DACs.
///
/// Calling this function should never block, and simply provide the list of DACs that have
/// broadcast their availability within the last specified DAC timeout duration.
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

/// Retrieve the current `frame_hz` at the time of rendering this `Frame`.
uint32_t frame_hz(const Frame *frame);

/// Retrieve the current `latency_points` at the time of rendering this `Frame`.
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

/// Retrieve the current `point_hz` at the time of rendering this `Frame`.
uint32_t frame_point_hz(const Frame *frame);

/// Create a new queue for safely passing prepared frames across threads.
void frame_queue_new(FrameSender *f_tx, FrameReceiver *f_rx);

/// Take ownership over the given `FrameReceiver` and free its resources.
void frame_receiver_drop(FrameReceiver rx);

/// Take ownership over the given `FrameSender` and free its resources.
void frame_sender_drop(FrameSender tx);

/// Close the TCP communication thread and wait for the thread to join.
///
/// This consumes and drops the `Stream`, returning the result produced by joining the thread.
///
/// This method will block until the associated thread has been joined.
Result frame_stream_close(Api *api, FrameStream stream);

/// Initialise the given frame stream configuration with default values.
void frame_stream_config_default(FrameStreamConfig *conf);

/// Must be called in order to correctly clean up the frame stream.
void frame_stream_drop(FrameStream stream);

/// Returns whether or not the communication thread has closed.
///
/// A stream may be closed if an error has occurred and the stream error callback indicated to
/// close the thread. A stream might also be closed if another `close` was called on another handle
/// to the stream.
///
/// In this case, the `Stream` should be closed or dropped and a new one should be created to
/// replace it.
bool frame_stream_is_closed(const FrameStream *stream);

/// Update the `blank_delay_points` field of the interpolation configuration. This represents the
/// number of points to insert at the end of a blank to account for light modulator delay.
///
/// The value will be updated on the laser thread prior to requesting the next frame.
///
/// Returns `true` on success or `false` if the communication channel was closed.
bool frame_stream_set_blank_delay_points(const FrameStream *stream, uint32_t points);

/// Update the `distance_per_point` field of the interpolation configuration used within the
/// optimisation pass for frames. This represents the minimum distance the interpolator can travel
/// along an edge before a new point is required.
///
/// The value will be updated on the laser thread prior to requesting the next frame.
///
/// Returns `true` on success or `false` if the communication channel was closed.
bool frame_stream_set_distance_per_point(const FrameStream *stream, float distance_per_point);

/// Update the rate at which the stream will attempt to present images via the DAC. This value is
/// used in combination with the DAC's `point_hz` in order to determine how many points should be
/// used to draw each frame. E.g.
///
/// ```ignore
/// let points_per_frame = point_hz / frame_hz;
/// ```
///
/// This is simply used as a minimum value. E.g. if some very simple geometry is submitted, this
/// allows the DAC to spend more time creating the path for the image. However, if complex geometry
/// is submitted that would require more than the ideal `points_per_frame`, the DAC may not be able
/// to achieve the desired `frame_hz` when drawing the path while also taking the
/// `distance_per_point` and `radians_per_point` into consideration.
///
/// The value will be updated on the laser thread prior to requesting the next frame.
///
/// Returns `true` on success or `false` if the communication channel was closed.
bool frame_stream_set_frame_hz(const FrameStream *stream, uint32_t frame_hz);

/// The maximum latency specified as a number of points.
///
/// Each time the laser indicates its "fullness", the raw stream will request enough points
/// from the render function to fill the DAC buffer up to `latency_points`.
///
/// This value should be no greaterthan the DAC's `buffer_capacity`.
///
/// Returns `true` on success or `false` if the communication channel was closed.
bool frame_stream_set_latency_points(const FrameStream *stream, uint32_t points);

/// Update the rate at which the DAC should process points per second.
///
/// This value should be no greater than the detected DAC's `max_point_hz`.
///
/// By default this value is `stream::DEFAULT_POINT_HZ`.
///
/// Returns `true` on success or `false` if the communication channel was closed.
bool frame_stream_set_point_hz(const FrameStream *stream, uint32_t point_hz);

/// Update the `radians_per_point` field of the interpolation configuration. This represents the
/// amount of delay to add based on the angle of the corner in radians.
///
/// The value will be updated on the laser thread prior to requesting the next frame.
///
/// Returns `true` on success or `false` if the communication channel was closed.
bool frame_stream_set_radians_per_point(const FrameStream *stream, float radians_per_point);

/// Spawn a new frame rendering stream.
///
/// The `frame_render_callback` is called each time the stream is ready for a new `Frame` of laser
/// points. Each "frame" has an optimisation pass applied that optimises the path for inertia,
/// minimal blanking, point de-duplication and segment order.
///
/// The `process_raw_callback` allows for optionally processing the raw points before submission to
/// the DAC. This might be useful for:
///
/// - applying post-processing effects onto the optimised, interpolated points.
/// - monitoring the raw points resulting from the optimisation and interpolation processes.
/// - tuning brightness of colours based on safety zones.
///
/// The given function will get called right before submission of the optimised, interpolated
/// buffer.
Result new_frame_stream(Api *api,
                        FrameStream *stream,
                        const FrameStreamConfig *config,
                        void *callback_data,
                        FrameRenderCallback frame_render_callback,
                        RawRenderCallback process_raw_callback,
                        StreamErrorCallback stream_error_callback);

/// Spawn a new frame rendering stream.
///
/// A raw LASER stream requests the exact number of points that the DAC is awaiting in each call to
/// the user's `process_raw_callback`. Keep in mind that no optimisation passes are applied. When
/// using a raw stream, this is the responsibility of the user.
Result new_raw_stream(Api *api,
                      RawStream *stream,
                      const StreamConfig *config,
                      void *callback_data,
                      RawRenderCallback process_raw_callback,
                      StreamErrorCallback stream_error_callback);

/// Retrieve the current ideal `points_per_frame` at the time of rendering this `Frame`.
uint32_t points_per_frame(const Frame *frame);

/// Close the TCP communication thread and wait for the thread to join.
///
/// This consumes and drops the `Stream`, returning the result produced by joining the thread.
///
/// This method will block until the associated thread has been joined.
Result raw_stream_close(Api *api, RawStream stream);

/// Must be called in order to correctly clean up the raw stream.
void raw_stream_drop(RawStream stream);

/// Returns whether or not the communication thread has closed.
///
/// A stream may be closed if an error has occurred and the stream error callback indicated to
/// close the thread. A stream might also be closed if another `close` was called on another handle
/// to the stream.
///
/// In this case, the `Stream` should be closed or dropped and a new one should be created to
/// replace it.
bool raw_stream_is_closed(const RawStream *stream);

/// The maximum latency specified as a number of points.
///
/// Each time the laser indicates its "fullness", the raw stream will request enough points
/// from the render function to fill the DAC buffer up to `latency_points`.
///
/// This value should be no greaterthan the DAC's `buffer_capacity`.
///
/// Returns `true` on success or `false` if the communication channel was closed.
bool raw_stream_set_latency_points(const RawStream *stream, uint32_t points);

/// Update the rate at which the DAC should process points per second.
///
/// This value should be no greater than the detected DAC's `max_point_hz`.
///
/// By default this value is `stream::DEFAULT_POINT_HZ`.
///
/// Returns `true` on success or `false` if the communication channel was closed.
bool raw_stream_set_point_hz(const RawStream *stream, uint32_t point_hz);

/// Must be called in order to correctly clean up a raw string.
void raw_string_drop(RawString msg);

/// Returns the pointer to the beginning of the C string for reading.
const char *raw_string_ref(const RawString *msg);

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

/// Set the error action to close the TCP communication thread.
void stream_error_action_set_close_thread(StreamErrorAction *action);

/// Set the error action to reattempt the TCP stream connection.
///
/// This action attempts to reconnect to the specified DAC in the case that one was provided, or
/// any DAC in the case that `None` was provided.
void stream_error_action_set_reattempt_connect(StreamErrorAction *action);

/// Set the error action to redetect the DAC.
///
/// This action attempts to re-detect the same DAC in the case that one was specified, or any DAC in the
/// case that `None` was provided.
///
/// This can be useful in the case where the DAC has dropped from the network and may have
/// re-appeared broadcasting from a different IP address.
void stream_error_action_set_redetect_dacs(StreamErrorAction *action,
                                           float timeout_secs);

/// Retrieve the number of attempts from the stream error.
///
/// If the error is `EtherDreamFailedToConnectStream`, this refers to the consecutive number of
/// failed attempts to establish a TCP connection with the DAC.
///
/// If the error is `EtherDreamFailedToDetectDac`, this refers to the consecutive number of failed
/// attempts to detect the requested DAC.
uint32_t stream_error_attempts(const StreamError *err);

/// Retrieve the kind of the stream error.
StreamErrorKind stream_error_kind(const StreamError *err);

/// Allocate a new C string containing the error message.
RawString stream_error_message(const StreamError *err);

} // extern "C"

} // namespace laser
