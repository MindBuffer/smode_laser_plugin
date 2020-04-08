/* -------------------------------------- . ---------------------------------- .
| Filename : LaserDevice.h                |                                    |
| Author   : MindBuffer                   |                                    |
| Started  : 05/04/2020 07:16             |                                    |
` --------------------------------------- . --------------------------------- */

#ifndef SMODE_LASER_DEVICE_H_
# define SMODE_LASER_DEVICE_H_

#include "LaserLibrary.h"
//#include "../smode_laser/smode_laser.h"

namespace smode
{
  class LaserDevice : public ControlDevice
  {
  public:
    LaserDevice(const LaserDeviceIdentifier& identifier)
      : ControlDevice(identifier) {}
    LaserDevice() {}

    struct Point
    {
      glm::vec2 position; // in range [-1, 1]
      glm::vec3 color;
      uint32_t weight; // 0 for smooth line segments, > 0 for accenting individual points
    };

    void addPoints(const std::vector<Point>& points) {
      // TODO
    }

    OIL_OBJECT(LaserDevice);
  };


}; /* namespace smode */

#endif // !SMODE_LASER_DEVICE_H_