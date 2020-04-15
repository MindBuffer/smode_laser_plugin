/* -------------------------------------- . ---------------------------------- .
| Filename : LaserDevice.h                |                                    |
| Author   : MindBuffer                   |                                    |
| Started  : 05/04/2020 07:16             |                                    |
` --------------------------------------- . --------------------------------- */

#ifndef SMODE_LASER_DEVICE_H_
# define SMODE_LASER_DEVICE_H_

#include "LaserLibrary.h"
#include "../smode_laser/smode_laser.h"

namespace smode
{
  struct CallbackData {
    laser::FrameReceiver frame_rx;
    laser::FrameMsg msg;
    Device* laser_device;
  };

  void frameRenderCallback(void* data, laser::Frame* frame) {
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

  void processRawCallback(void* data, laser::Buffer* buffer) {
    // Nothing to be done.
  }

  String streamErrorToString(const laser::StreamError* err) {
      laser::RawString rs = laser::stream_error_message(err);
      String message = String(laser::raw_string_ref(&rs));
      laser::raw_string_drop(rs);
      return message;
  }

  void streamErrorCallback(void* data, const laser::StreamError* err, laser::StreamErrorAction* action) {
    // Update the status if the message would have changed.
    CallbackData* cb_data = (CallbackData*)data;
    SuccessStatus status = cb_data->laser_device->getStatus();
    String msg = streamErrorToString(err);
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
        if (laser::stream_error_attempts(err) % 5 != 0) {
          laser::stream_error_action_set_reattempt_connect(action);
        } else {
          float timeout_secs = 1.1;
          laser::stream_error_action_set_redetect_dacs(action, timeout_secs);
        }
        break;

      // If we failed to detect the DAC, try again.
      // Note: If an ether dream DAC is broadcasting, it does so once per second.
      case laser::StreamErrorKind::EtherDreamFailedToDetectDacs: {
        float timeout_secs = 1.1;
        laser::stream_error_action_set_redetect_dacs(action, timeout_secs);
        break;
      }

      // The default action is to close the thread.
      default:
        break;
    }
  }

  class LaserDevice : public ControlDevice
  {
  public:
    LaserDevice(const DeviceIdentifier& identifier, laser::DetectedDac _dac)
      : ControlDevice(identifier), dacPointsPerSecond(10000), latencyPoints(166), targetFps(60), blankDelayPoints(10), distancePerPoint(0.1), anglePerPoint(0.6), dac(_dac),
      callback_data(std::make_shared<CallbackData>())
    {
      dacPointsPerSecond.setParent(this);
      latencyPoints.setParent(this);
      targetFps.setParent(this);
      distancePerPoint.setParent(this);
      blankDelayPoints.setParent(this);
      anglePerPoint.setParent(this);
    }
    LaserDevice() {}

    // Object
    bool hasVariableConstraints() const override
      {return true;}

    void variableChangedCallback(Object* variable, Object* changedObject) override
    {      
      if (isInitialized() && isRenderingServiceCurrent()) {
        if (variable == &dacPointsPerSecond) {
          laser::frame_stream_set_point_hz(&frame_stream, (uint32_t)dacPointsPerSecond);
        }
        else if (variable == &latencyPoints) {
          laser::frame_stream_set_latency_points(&frame_stream, (uint32_t)latencyPoints);
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

      if (!isApplyingVariableConstraints()) {
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
      config.stream_conf.tcp_timeout_secs = 1.2;
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
        frameRenderCallback,
        processRawCallback,
        streamErrorCallback
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

    struct Point
    {
      glm::vec2 position; // in range [-1, 1]
      glm::vec3 color;
      uint32_t weight; // 0 for smooth line segments, > 0 for accenting individual points
    };

    // Creates a new `Frame` from the given points and makes it available to the render callback.
    void updateFrame(const std::vector<Point>& smode_points) {
      // We always want to send a msg, even if its empty.
      // This is because the render callback always emits the last frame received.
      laser::FrameMsg msg;
      laser::frame_msg_new(&msg);
      // Only write the points if we're not muted, we're not in an error state and we actually have some.
      if (!smode_points.empty() && !mute && !getStatus().isError()) {
        // Layout *must* match or we will get very strange behaviour.
        jassert(sizeof(Point) == sizeof(laser::Point));
        const laser::Point* points = reinterpret_cast<const laser::Point*>(&smode_points[0]);
        laser::SequenceType ty = laser::SequenceType::Lines;
        laser::frame_msg_add_sequence(&msg, ty, points, smode_points.size());
      }
      laser::send_frame_msg(&frame_tx, msg);
    }

    // A pointer to the laser API instance.
    // Is valid between `initializeFactory` and `deinitializeFactory`.
    laser::Api* laser_api;
    // The detected DAC associated with this Device instance.
    laser::DetectedDac dac;
    laser::FrameStream frame_stream;
    laser::FrameSender frame_tx;
    // Shared with the laser callback.
    std::shared_ptr<CallbackData> callback_data;

    OIL_OBJECT(LaserDevice);

  private:
    typedef ControlDevice BaseClass;

    PositiveInteger dacPointsPerSecond;
    PositiveInteger latencyPoints;
    PositiveInteger targetFps;
    Percentage distancePerPoint;
    PositiveInteger blankDelayPoints;
    PositiveAngle anglePerPoint; // This is actually in radians but displayed to the user in degrees
  };


}; /* namespace smode */

#endif // !SMODE_LASER_DEVICE_H_
