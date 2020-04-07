#ifndef SMODE_LASER_DEVICE_IDENTIFIER_H_
# define SMODE_LASER_DEVICE_IDENTIFIER_H_

#include "LaserLibrary.h"

namespace smode
{
	class LaserDeviceIdentifier : public StringDeviceIdentifier
	{
	public:
	LaserDeviceIdentifier(const String& factory, const String& value)
		: StringDeviceIdentifier(factory, value) {}
	LaserDeviceIdentifier() {}

	OIL_OBJECT(LaserDeviceIdentifier);

	};

}; /* namespace smode */

#endif // !SMODE_LASER_DEVICE_IDENTIFIER_H_