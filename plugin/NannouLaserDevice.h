/* -------------------------------------- . ---------------------------------- .
| Filename : NannouLaserDevice.h          |                                    |
| Author   : MindBuffer                   |                                    |
| Started  : 17/04/2020 13:05             |                                    |
` --------------------------------------- . --------------------------------- */

#ifndef SMODE_NANNOU_LASER_DEVICE_H_
# define SMODE_NANNOU_LASER_DEVICE_H_

#include "LaserDevice.h"
#include "../smode_laser/smode_laser.h"

namespace smode
{
class NannouLaserDevice : public LaserDevice
{
public:
  NannouLaserDevice(const DeviceIdentifier& identifier, laser::Api* _api, laser::DetectedDac _dac)
    : LaserDevice(identifier), dacPointsPerSecond(10000), latencyPoints(166), targetFps(60), blankDelayPoints(10), distancePerPoint(0.1), anglePerPoint(0.6), laser_api(_api), dac(_dac), callback_data(std::make_shared<CallbackData>())
  {
    dacPointsPerSecond.setParent(this);
    latencyPoints.setParent(this);
    targetFps.setParent(this);
    distancePerPoint.setParent(this);
    blankDelayPoints.setParent(this);
    anglePerPoint.setParent(this);
  }
  NannouLaserDevice() {}

  bool hasVariableConstraints() const override
    {return true;}

  void variableChangedCallback(Object* variable, Object* changedObject) override
  {      
    if (!isRenderingServiceCurrent())
      return;

    if (isInitialized()) {
      if (variable == &dacPointsPerSecond) {
        uint32_t dpps = juce::jlimit((uint32_t)1000, (uint32_t)dac.kind.ether_dream.broadcast.max_point_rate, (uint32_t)dacPointsPerSecond);
        laser::frame_stream_set_point_hz(&frame_stream, dpps);
      }
      else if (variable == &latencyPoints) {
        uint32_t lp = juce::jlimit((uint32_t)10, (uint32_t)dac.kind.ether_dream.broadcast.buffer_capacity, (uint32_t)latencyPoints);
        laser::frame_stream_set_latency_points(&frame_stream, lp);
      }
      else if (variable == &targetFps) {
        laser::frame_stream_set_frame_hz(&frame_stream, (uint32_t)targetFps);
      }
      else if (variable == &distancePerPoint) {
        laser::frame_stream_set_distance_per_point(&frame_stream, (uint32_t)distancePerPoint);
      }
      else if (variable == &blankDelayPoints) {
        laser::frame_stream_set_blank_delay_points(&frame_stream, (uint32_t)blankDelayPoints);
      }
      else if (variable == &anglePerPoint) {
        laser::frame_stream_set_radians_per_point(&frame_stream, (float)anglePerPoint);
      }
    }

    if (isApplyingVariableConstraints()) {
      if (variable == &dacPointsPerSecond) {
        VariableConstraintsScope _(*this);
        uint32_t runtimeMax = dac.kind.ether_dream.broadcast.max_point_rate;
        if ((uint32_t)dacPointsPerSecond > runtimeMax)
          dacPointsPerSecond.set(runtimeMax);
      }
      else if (variable == &latencyPoints) {
        VariableConstraintsScope _(*this);
        uint32_t runtimeMax = dac.kind.ether_dream.broadcast.buffer_capacity;
        if ((uint32_t)latencyPoints > runtimeMax)
          latencyPoints.set(runtimeMax);
      }
    }

    BaseClass::variableChangedCallback(variable, changedObject);
  }

  bool initializeDevice() override {
    BaseClass::initializeDevice();

    // Prepare the callback data and frame msg queue.
    laser::frame_msg_new(&callback_data->msg);
    laser::frame_queue_new(&frame_tx, &callback_data->frame_rx);
    callback_data->laser_device = this;

    // Initialise the stream with default configuration.
    laser::FrameStreamConfig config = {};
    laser::frame_stream_config_default(&config);
    config.stream_conf.tcp_timeout_secs = TCP_TIMEOUT_SECS;
    config.stream_conf.detected_dac = &dac;
    config.interpolation_conf.blank_delay_points = (uint32_t)blankDelayPoints;
    config.interpolation_conf.distance_per_point = (uint32_t)distancePerPoint;
    config.interpolation_conf.radians_per_point = (float)anglePerPoint;
    config.frame_hz = (uint32_t)targetFps;
    config.stream_conf.latency_points = (uint32_t)latencyPoints;
    config.stream_conf.point_hz = (uint32_t)dacPointsPerSecond;

    // Data to be shared with the frame render callback.
    // For now, just share the frame receiver.
    CallbackData* cb_data_ptr = callback_data.get();
    void* callback_data = cb_data_ptr;

    // Spawn the stream.
    laser::Result res = laser::new_frame_stream(
      laser_api,
      &frame_stream,
      &config,
      callback_data,
      NannouLaserDevice::frameRenderCallback,
      NannouLaserDevice::processRawCallback,
      NannouLaserDevice::streamErrorCallback
    );

    if (res != laser::Result::Success) {
      const char* err = laser::api_last_error(laser_api);
      String failureReason = "Failed to spawn laser frame stream: " + String(err);
      DBG(failureReason);
      return false;
    }

    return true;
  }

  void deinitializeDevice() override {
    // Clean up the frame resources and ensure the thread is joined.
    laser::frame_stream_drop(frame_stream);
    // Now that the laser thread is stopped, we can clean up our channel and callback data.
    laser::frame_sender_drop(frame_tx);
    laser::frame_receiver_drop(callback_data->frame_rx);
    laser::frame_msg_drop(callback_data->msg);
    BaseClass::deinitializeDevice();
  }

  void update(const FrameInformation& frame) override {
      updateFrame();
  }

  void addLineSequence(const std::vector<Point>& new_points) override {
    if (!frame_points.empty() && !new_points.empty()) {
        Point a = frame_points.back();
        Point b = new_points.front();
        a.color = glm::vec3(0.f);
        b.color = glm::vec3(0.f);
        frame_points.push_back(a);
        frame_points.push_back(b);
    }
    frame_points.insert(frame_points.end(), new_points.begin(), new_points.end());
  }

  // Send the current `frame_points` as a `FrameMsg` to the DAC callback.
  void updateFrame() {
    // We always want to send a msg, even if there were no points.
    // This is because the render callback always emits the last frame received.
    laser::FrameMsg msg;
    laser::frame_msg_new(&msg);
    // Write points if we're not muted, we're not in an error state and we actually have some.
    if (!frame_points.empty() && !mute && !getStatus().isError()) {
      // Layout *must* match or we will get very strange behaviour.
      jassert(sizeof(Point) == sizeof(laser::Point));
      const laser::Point* points = reinterpret_cast<const laser::Point*>(&frame_points[0]);
      laser::SequenceType ty = laser::SequenceType::Lines;
      laser::frame_msg_add_sequence(&msg, ty, points, frame_points.size());
    }
    laser::send_frame_msg(&frame_tx, msg);
    // Clear the frame points, ready to collect from the renderers before next update.
    frame_points.clear();
  }

  OIL_OBJECT(NannouLaserDevice);

private:
  // Data shared between the stream thread and the GUI rendering thread.
  struct CallbackData {
    laser::FrameReceiver frame_rx;
    laser::FrameMsg msg;
    Device* laser_device;
  };

  // Called by the DAC stream thread when ready for a new frame of points.
  static void frameRenderCallback(void* data, laser::Frame* frame) {
    CallbackData* cb_data = (CallbackData*)data;
    // Ensure the device status is in the success state.
    if (cb_data->laser_device->getStatus().getState() != SuccessState::success) {
        SuccessStatus status = SuccessStatus();
        status.setSuccess();
        cb_data->laser_device->setStatusFromOtherThread(status);
    }
    // Update our current frame data.
    laser::FrameMsg msg = cb_data->msg;
    if (laser::recv_frame_msg(&cb_data->frame_rx, &cb_data->msg)) {
      laser::frame_msg_drop(msg);
      msg = cb_data->msg;
    }
    // Write the data to the frame.
    laser::extend_frame_with_msg(frame, &cb_data->msg);
  }

  // Called by the DAC stream thread after interpolation and optimisations have
  // been applied, right before sending the points to the DAC. This is normally
  // used if some post-processing must be applied on the points, but normally
  // this can be done prior to submitting points to the frame render callback.
  static void processRawCallback(void* data, laser::Buffer* buffer) {
    // Nothing to be done.
  }

  static String streamErrorToString(const laser::StreamError* err) {
      laser::RawString rs = laser::stream_error_message(err);
      String message = String(laser::raw_string_ref(&rs));
      laser::raw_string_drop(rs);
      return message;
  }

  // Called by the DAC stream thread when some error occurs.
  static void streamErrorCallback(void* data, const laser::StreamError* err, laser::StreamErrorAction* action) {
    // Update the status if the message would have changed.
    CallbackData* cb_data = (CallbackData*)data;
    SuccessStatus status = cb_data->laser_device->getStatus();
    String msg = NannouLaserDevice::streamErrorToString(err);
    if (status.getMessage() != msg) {
        status.setError(msg);
        cb_data->laser_device->setStatusFromOtherThread(status);
    }
    // Handle the error.
    switch (laser::stream_error_kind(err)) {
      // If communication dropped out, re-attempt to connect to the TCP stream.
      case laser::StreamErrorKind::EtherDreamFailedToPrepareStream:
      case laser::StreamErrorKind::EtherDreamFailedToBeginStream:
      case laser::StreamErrorKind::EtherDreamFailedToSubmitData:
      case laser::StreamErrorKind::EtherDreamFailedToSubmitPointRate:
        laser::stream_error_action_set_reattempt_connect(action);
        break;
      // Attempt re-connection 5 times before attempting to re-detect the DAC.
      case laser::StreamErrorKind::EtherDreamFailedToConnectStream:
        if (laser::stream_error_attempts(err) % TCP_RECONNECT_ATTEMPTS != 0) {
          laser::stream_error_action_set_reattempt_connect(action);
        } else {
          float timeout_secs = TCP_TIMEOUT_SECS;
          laser::stream_error_action_set_redetect_dacs(action, timeout_secs);
        }
        break;
      // If we failed to detect the DAC, try again.
      // Note: If an ether dream DAC is broadcasting, it does so once per second.
      case laser::StreamErrorKind::EtherDreamFailedToDetectDacs: {
        float timeout_secs = TCP_TIMEOUT_SECS;
        laser::stream_error_action_set_redetect_dacs(action, timeout_secs);
        break;
      }
      // The default action is to close the thread.
      default:
        break;
    }
  }

  // The TCP timeout describes how long the stream should wait to connect, read
  // or write to or from the TCP stream before emitting an error via the
  // `streamErrorCallback`. The current `streamErrorCallback` implementation
  // will continuously attempt to re-connect to the TCP stream or re-detect the
  // DAC in the case that a timeout occurs.
  static const uint32_t TCP_TIMEOUT_SECS = 2;
  // The number of times to attempt re-connecting to the TCP stream in the case
  // that it drops out. Once this number is exceeded, the `streamErrorCallback`
  // implementation will then attempt to re-detect the DAC.
  static const uint32_t TCP_RECONNECT_ATTEMPTS = 5;

  // A pointer to the laser API instance.
  // Is valid between `initializeFactory` and `deinitializeFactory`.
  laser::Api* laser_api;
  // The detected DAC associated with this Device instance.
  laser::DetectedDac dac;

  // A handle to the DAC stream thread.
  // Valid between `initializeDevice` and `deinitializeDevice`.
  laser::FrameStream frame_stream;
  // The sending half of the thread-safe, bounded, non-blocking queue for
  // sending `FrameMsg`s.
  laser::FrameSender frame_tx;
  // Shared with the laser callback.
  std::shared_ptr<CallbackData> callback_data;
  // The buffer used for collecting points from each of the geometry renderers.
  std::vector<Point> frame_points;

  PositiveInteger dacPointsPerSecond;
  PositiveInteger latencyPoints;
  PositiveInteger targetFps;
  Percentage distancePerPoint;
  PositiveInteger blankDelayPoints;
  PositiveAngle anglePerPoint; // This is actually in radians but displayed to the user in degrees

  typedef LaserDevice BaseClass;
};
}; // namespace smode

#endif // !SMODE_NANNOU_LASER_DEVICE_H_
