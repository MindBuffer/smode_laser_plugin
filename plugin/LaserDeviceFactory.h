/* -------------------------------------- . ---------------------------------- .
| Filename : LaserDeviceFactory.h         |   							       |
| Author   : MindBuffer                   |                                    |
| Started  : 05/04/2020 07:16             |                                    |
` --------------------------------------- . --------------------------------- */

#ifndef SMODE_LASER_DEVICE_FACTORY_H_
# define SMODE_LASER_DEVICE_FACTORY_H_

#include "LaserLibrary.h"
#include "LaserDeviceIdentifier.h"
#include "LaserDevice.h"
#include "../smode_laser/smode_laser.h"

namespace smode
{
	String macAddressToString(uint8_t mac_address[6])
	{
		// TODO: Should produce a nicer mac address string.
		String s;
		for (int i = 0; i < 6; i++) {
			 s += String(mac_address[i]);
		}
		return s;
	}

	class LaserDeviceFactory : public DeviceFactory
	{
	public:
		LaserDeviceFactory() {}

		String getName() const override
		{
			return "Laser";
		}

		bool matchName(const String& factoryName) const  override // retrocompatibility
		{
			return factoryName == getName();
		}

		bool isEnumerable() const override
		{
			return true;
		}

		bool initializeFactory(Engine& engine, String& failureReason) override
		{
			float dac_timeout_secs = 1.5;
			smode::laser::api_new(&api);
			smode::laser::Result res = smode::laser::detect_dacs_async(&api, dac_timeout_secs, &dac_detector);
			if (res != smode::laser::Result::Success) {
				const char* err = smode::laser::api_last_error(&api);
				failureReason = "Failed to spawn DAC detection thread: " + String(err);
			}
			return true;
		}

		void configurationApplied() override
		{
			DBG("configurationApplied");
		}

		void deinitializeFactory() override
		{
			smode::laser::detect_dacs_async_drop(dac_detector);
			smode::laser::api_drop(api);
		}

		bool enumerateDevices(Engine& engine, std::vector<DeviceIdentifier* >& res, String& failureReason) override
		{
			// Collect the `DetectedDacs` from the asynchronous detector.
			smode::laser::DetectedDac* dacs = nullptr;
			unsigned int dac_count = 0;
			smode::laser::available_dacs(&dac_detector, &dacs, &dac_count);
			detected_dacs.clear();
			if (dac_count > 0) {
				detected_dacs.assign(dacs, dacs + dac_count);
			}

			// Convert the detected DACs into `LaserDeviceIdentifier`s that SMODE can work with.
			for (auto dac : detected_dacs) {
				String mac_string = macAddressToString(dac.kind.ether_dream.broadcast.mac_address);
				res.push_back(new LaserDeviceIdentifier(getName(), mac_string));
			}
			return true;
		}

		Device* createDevice(const DeviceIdentifier& identifier, GraphicsContext& graphics) const override
		{
			DBG("createDevice");
			for (auto dac : detected_dacs) {
				String mac_string = macAddressToString(dac.kind.ether_dream.broadcast.mac_address);
				if (mac_string == identifier.getFriendlyName()) {
					// TODO.
					return new LaserDevice();
				}
			}
			return nullptr;
		}

		OIL_OBJECT(LaserDeviceFactory);

	private:
		smode::laser::Api api;
		smode::laser::DetectDacsAsync dac_detector;
		std::vector<smode::laser::DetectedDac> detected_dacs;
	};
}; /* namespace smode */

#endif // !SMODE_LASER_DEVICE_FACTORY_H_