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
  void frameRenderCallback(void* data, smode::laser::Frame* frame) {
    DBG("frameRenderCallback");
  }

  void processRawCallback(void* data, smode::laser::Buffer* buffer) {
    // Nothing to be done.
  }

  class LaserDevice : public ControlDevice
  {
  public:
    LaserDevice(const DeviceIdentifier& identifier)
      : ControlDevice(identifier), dacPointsPerSecond(10000), latency(166), targetFps(60), distancePerPoint(0.1), blankDelayPoints(10), radiansPerPoint(0.6)
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
      DBG("INIT CONTROLDEVICE");
      ControlDevice::initializeDevice();

      // Initialise the stream with default configuration.
      smode::laser::FrameStreamConfig config = {};
      smode::laser::frame_stream_config_default(&config);
      config.stream_conf.detected_dac = &dac;

      // Data to be shared with the frame render callback.
      // TODO
      void* callback_data = nullptr;

      DBG("SPAWNING DA STREAM");

      // Spawn the stream.
      smode::laser::Result res = smode::laser::new_frame_stream(
        laser_api,
        &frame_stream,
        &config,
        &callback_data,
        frameRenderCallback,
        processRawCallback
      );

      if (res != smode::laser::Result::Success) {
        const char* err = smode::laser::api_last_error(laser_api);
        String failureReason = "Failed to spawn laser frame stream: " + String(err);
        DBG(failureReason);
        return false;
      }

      DBG("DID IT");
      return true;
    }

    void deinitializeDevice() override {
      DBG("DE INIT! CUNT");
      smode::laser::frame_stream_drop(frame_stream);
      ControlDevice::deinitializeDevice();
    }

    struct Point
    {
      glm::vec2 position; // in range [-1, 1]
      glm::vec3 color;
      uint32_t weight; // 0 for smooth line segments, > 0 for accenting individual points
    };

    void addPoints(const std::vector<Point>& points) {
      // TODO
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