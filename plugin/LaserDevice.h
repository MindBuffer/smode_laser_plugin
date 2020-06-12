/* -------------------------------------- . ---------------------------------- .
| Filename : LaserDevice.h                |                                    |
| Author   : MindBuffer                   |                                    |
| Started  : 05/04/2020 07:16             |                                    |
` --------------------------------------- . --------------------------------- */

#ifndef SMODE_LASER_DEVICE_H_
# define SMODE_LASER_DEVICE_H_

#include "LaserLibrary.h"

namespace smode
{
class LaserDevice : public ControlDevice
{
public:
  LaserDevice(const DeviceIdentifier& identifier)
    : ControlDevice(identifier) {}
  LaserDevice() {} // default constructible for proxy

  struct Point
  {
    glm::vec2 position; // in range [-1, 1]
    glm::vec3 color;
    uint32_t weight; // 0 for smooth line segments, > 0 for accenting individual points
  };

  // Concatenate the given points representing a line sequence onto the points
  // to be presented for this frame.
  //
  // This method will be called once for each `LaserGeometryRenderer` targeting
  // the `LaserDevice` per `update`. It is the responsibility of the
  // implementor to ensure that a blank line is inserted between each added
  // line sequence. The accumulated points should be flushed to the LASER DAC
  // and cleared once per `update`.
  virtual void addLineSequence(const std::vector<Point>& new_points) = 0;

  // ControlDevice
  void update(const FrameInformation& frame) override
  {
    BaseClass::update(frame);
    ioChannel.getConsumed().set(1);
  }

  OIL_ABSTRACT_OBJECT(LaserDevice);

private:
  typedef ControlDevice BaseClass;
};

}; /* namespace smode */

#endif // !SMODE_LASER_DEVICE_H_
