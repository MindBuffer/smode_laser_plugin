/* -------------------------------------- . ---------------------------------- .
| Filename : LaserDeviceSelector.h        |                                    |
| Author   : Francis Maes                 |                                    |
| Started  : 07/04/2020 08:06             |                                    |
` --------------------------------------- . --------------------------------- */

#ifndef SMODE_LASER_DEVICE_SELECTOR_H_
# define SMODE_LASER_DEVICE_SELECTOR_H_

#include "LaserLibrary.h"

namespace smode
{
class LaserDeviceSelector : public TypedDeviceSelector<LaserDevice>
{
public:
  LaserDeviceSelector() {}

  bool acceptDeviceClass(Class* deviceClass) const override
  {
    return deviceClass->inheritsFrom(oil::getClass("LaserDevice"));
  }

  OIL_ABSTRACT_OBJECT(LaserDeviceSelector);
};

}; /* namespace smode */

#endif // !SMODE_LASER_DEVICE_SELECTOR_H_