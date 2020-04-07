#ifndef SMODE_LASER_DEVICE_H_
# define SMODE_LASER_DEVICE_H_

#include "LaserLibrary.h"
//#include "../smode_laser/smode_laser.h"

namespace smode
{
	
	class LaserDevice : public Device
	{
	public:
	LaserDevice() {}

	OIL_OBJECT(LaserDevice);
	};

}; /* namespace smode */

#endif // !SMODE_LASER_DEVICE_H_