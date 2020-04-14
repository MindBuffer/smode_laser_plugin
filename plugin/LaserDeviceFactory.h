/* -------------------------------------- . ---------------------------------- .
| Filename : LaserDeviceFactory.h         |                                    |
| Author   : MindBuffer                   |                                    |
| Started  : 05/04/2020 07:16             |                                    |
` --------------------------------------- . --------------------------------- */

#ifndef SMODE_LASER_DEVICE_FACTORY_H_
# define SMODE_LASER_DEVICE_FACTORY_H_

#include "LaserLibrary.h"
#include "LaserDeviceIdentifier.h"
#include "LaserDevice.h"

namespace smode
{
  String macAddressToString(uint8_t mac_address[6])
  {
    String s;
    s += String::toHexString(mac_address[0]);
    for (int i = 1; i < 6; i++) {
      s += ":";
      s += String::toHexString(mac_address[i]);
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
      laser::api_new(&api);
      laser::Result res = laser::detect_dacs_async(&api, dac_timeout_secs, &dac_detector);
      if (res != laser::Result::Success) {
        const char* err = laser::api_last_error(&api);
        failureReason = "Failed to spawn DAC detection thread: " + String(err);
        return false;
      }

      if (!BaseClass::initializeFactory(engine, failureReason)) {
        return false;
      }

      return true;
    }

    void configurationApplied() override
    {
    }

    void deinitializeFactory() override
    {
      laser::detect_dacs_async_drop(dac_detector);
      laser::api_drop(api);
      BaseClass::deinitializeFactory();
    }

    bool enumerateDevices(Engine& engine, std::vector<DeviceIdentifier* >& res, String& failureReason) override
    {
      // Collect the `DetectedDacs` from the asynchronous detector.
      laser::DetectedDac* dacs = nullptr;
      unsigned int dac_count = 0;
      laser::available_dacs(&dac_detector, &dacs, &dac_count);
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
      for (auto dac : detected_dacs) {
        String mac_string = macAddressToString(dac.kind.ether_dream.broadcast.mac_address);

        if (mac_string == identifier.getFriendlyName()) {
            LaserDevice* device = new LaserDevice(identifier, dac);
            device->laser_api = &api;
            return device;
        }
      }
      return nullptr;
    }

    OIL_OBJECT(LaserDeviceFactory);

  private:
    mutable laser::Api api;
    laser::DetectDacsAsync dac_detector;
    std::vector<laser::DetectedDac> detected_dacs;
    typedef DeviceFactory BaseClass;
  };
}; /* namespace smode */

#endif // !SMODE_LASER_DEVICE_FACTORY_H_
