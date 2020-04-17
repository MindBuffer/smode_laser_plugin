/* -------------------------------------- . ---------------------------------- .
| Filename : NannouLaserDeviceIdentifier.h|                                    |
| Author   : MindBuffer                   |                                    |
| Started  : 05/04/2020 07:16             |                                    |
` --------------------------------------- . --------------------------------- */

#ifndef SMODE_NANNOU_LASER_DEVICE_IDENTIFIER_H_
# define SMODE_NANNOU_LASER_DEVICE_IDENTIFIER_H_

#include "LaserLibrary.h"

namespace smode
{
class NannouLaserDeviceIdentifier : public StringDeviceIdentifier
{
public:
NannouLaserDeviceIdentifier(const String& factory, const String& value)
  : StringDeviceIdentifier(factory, value) {}
NannouLaserDeviceIdentifier() {}

OIL_OBJECT(NannouLaserDeviceIdentifier);
};
}; /* namespace smode */

#endif // !SMODE_NANNOU_LASER_DEVICE_IDENTIFIER_H_
