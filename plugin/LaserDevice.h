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
    smode::laser::FrameReceiver frame_rx;
    smode::laser::FrameMsg msg;
  };

  void frameRenderCallback(void* data, smode::laser::Frame* frame) {
    CallbackData* cb_data = (CallbackData*)data;
    smode::laser::FrameMsg msg = cb_data->msg;
    if (smode::laser::recv_frame_msg(&cb_data->frame_rx, &cb_data->msg)) {
      smode::laser::frame_msg_drop(msg);
      msg = cb_data->msg;
    }
    smode::laser::extend_frame_with_msg(frame, &cb_data->msg);
  }

  void processRawCallback(void* data, smode::laser::Buffer* buffer) {
    // Nothing to be done.
  }

  class LaserDevice : public ControlDevice
  {
  public:
    LaserDevice(const DeviceIdentifier& identifier)
      : ControlDevice(identifier), dacPointsPerSecond(10000), latency(166), targetFps(60), distancePerPoint(0.1), blankDelayPoints(10), radiansPerPoint(0.6),
      callback_data(std::make_shared<CallbackData>())
    {
      dacPointsPerSecond.setParent(this);
      latency.setParent(this);
      targetFps.setParent(this);
      distancePerPoint.setParent(this);
      blankDelayPoints.setParent(this);
      radiansPerPoint.setParent(this);
    }
    LaserDevice() {}

    
    bool initializeDevice() override {
      ControlDevice::initializeDevice();

      // Prepare the callback data and frame msg queue.
      smode::laser::frame_msg_new(&callback_data->msg);
      smode::laser::frame_queue_new(&frame_tx, &callback_data->frame_rx);

      // Initialise the stream with default configuration.
      smode::laser::FrameStreamConfig config = {};
      smode::laser::frame_stream_config_default(&config);
      config.stream_conf.detected_dac = &dac;

      // Data to be shared with the frame render callback.
      // For now, just share the frame receiver.
      CallbackData* cb_data_ptr = callback_data.get();
      void* callback_data = cb_data_ptr;

      // Spawn the stream.
      smode::laser::Result res = smode::laser::new_frame_stream(
        laser_api,
        &frame_stream,
        &config,
        callback_data,
        frameRenderCallback,
        processRawCallback
      );

      if (res != smode::laser::Result::Success) {
        const char* err = smode::laser::api_last_error(laser_api);
        String failureReason = "Failed to spawn laser frame stream: " + String(err);
        DBG(failureReason);
        return false;
      }

      return true;
    }

    void deinitializeDevice() override {
      // Clean up the frame resources and ensure the thread is joined.
      smode::laser::frame_stream_drop(frame_stream);
      // Now that the laser thread is stopped, we can clean up our channel and callback data.
      smode::laser::frame_sender_drop(frame_tx);
      smode::laser::frame_receiver_drop(callback_data->frame_rx);
      smode::laser::frame_msg_drop(callback_data->msg);
      ControlDevice::deinitializeDevice();
    }

    struct Point
    {
      glm::vec2 position; // in range [-1, 1]
      glm::vec3 color;
      uint32_t weight; // 0 for smooth line segments, > 0 for accenting individual points
    };

    void addPoints(const std::vector<Point>& smode_points) {
      if (smode_points.empty()) {
        return;
      }
      std::vector<smode::laser::Point> points;
      for (auto p : smode_points) {
        smode::laser::Point point = {};
        point.position[0] = p.position.x;
        point.position[1] = p.position.y;
        point.color[0] = p.color.r;
        point.color[1] = p.color.g;
        point.color[2] = p.color.b;
        point.weight = p.weight;
        points.push_back(point);
      }
      smode::laser::FrameMsg msg;
      smode::laser::frame_msg_new(&msg);
      // TODO: Retrieve the type from somewhere the user can control.
      smode::laser::SequenceType ty = smode::laser::SequenceType::Lines;
      smode::laser::frame_msg_add_sequence(&msg, ty, &points[0], points.size());
      smode::laser::send_frame_msg(&frame_tx, msg);
    }

    const PositiveInteger& getDacPointsPerSecond() const
    {
      return dacPointsPerSecond;
    }

    PositiveInteger& getDacPointsPerSecond()
    {
      return dacPointsPerSecond;
    }

    const PositiveInteger& getLatency() const
    {
      return latency;
    }

    PositiveInteger& getLatency()
    {
      return latency;
    }

    const PositiveInteger& getTargetFps() const
    {
      return targetFps;
    }

    PositiveInteger& getTargetFps()
    {
      return targetFps;
    }

    const Real& getDistancePerPoint() const
    {
      return distancePerPoint;
    }

    Real& getDistancePerPoint()
    {
      return distancePerPoint;
    }

    const PositiveInteger& getBlankDelayPoints() const
    {
      return blankDelayPoints;
    }

    PositiveInteger& getBlankDelayPoints()
    {
      return blankDelayPoints;
    }

    const Real& getRadiansPerPoint() const
    {
      return radiansPerPoint;
    }

    Real& getRadiansPerPoint()
    {
      return radiansPerPoint;
    }

    // A pointer to the laser API instance.
    // Is valid between `initializeFactory` and `deinitializeFactory`.
    smode::laser::Api* laser_api;
    // The detected DAC associated with this Device instance.
    smode::laser::DetectedDac dac;
    smode::laser::FrameStream frame_stream;
    smode::laser::FrameSender frame_tx;
    // Shared with the laser callback.
    std::shared_ptr<CallbackData> callback_data;

    OIL_OBJECT(LaserDevice);

  private:
    PositiveInteger dacPointsPerSecond;
    PositiveInteger latency;
    PositiveInteger targetFps;
    Real distancePerPoint;
    PositiveInteger blankDelayPoints;
    Real radiansPerPoint;
  };


}; /* namespace smode */

#endif // !SMODE_LASER_DEVICE_H_