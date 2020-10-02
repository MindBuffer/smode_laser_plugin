/* -------------------------------------- . ---------------------------------- .
| Filename : NannouLaserDeviceFactory.h   |                                    |
| Author   : MindBuffer                   |                                    |
| Started  : 05/04/2020 07:16             |                                    |
` --------------------------------------- . --------------------------------- */

#ifndef SMODE_NANNOU_LASER_DEVICE_FACTORY_H_
# define SMODE_NANNOU_LASER_DEVICE_FACTORY_H_

#include "LaserLibrary.h"
#include "NannouLaserDeviceIdentifier.h"
#include "NannouLaserDevice.h"

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

class NannouLaserDeviceFactory : public DeviceFactory
{
public:
  NannouLaserDeviceFactory() {}

  String getName() const override
    {return "Laser";}

  bool matchName(const String& factoryName) const  override // retrocompatibility
    {return factoryName == getName();}

  bool isEnumerable() const override
    {return true;}

  bool initializeFactory(Engine& engine, String& failureReason) override
  {
    float dac_timeout_secs = 1.5;
    laser::api_new(&api);
    laser::Result res = laser::detect_dacs_async(&api, dac_timeout_secs, &dac_detector);
    if (res != laser::Result::Success) {
      const char* err = laser::api_last_error(&api);
      failureReason = "Failed to spawn DAC detection thread: " + String(err);
      dac_detector = { 0, };
      return false;
    }

    return BaseClass::initializeFactory(engine, failureReason);
  }

  void configurationApplied() override
    {}

  void deinitializeFactory() override
  {
    laser::detect_dacs_async_drop(dac_detector);
    dac_detector = {0,};
    laser::api_drop(api);
    BaseClass::deinitializeFactory();
  }

  bool enumerateDevices(Engine& engine, std::vector<DeviceIdentifier* >& res, String& failureReason) override
  {
    // Collect the `DetectedDacs` from the asynchronous detector.
    laser::DetectedDac* dacs = nullptr;
    unsigned int dac_count = 0;
    if (dac_detector.inner) // check if init has failed to avoid available_dacs crash (can happens when twice instance of Smode run in same computer)
      laser::available_dacs(&dac_detector, &dacs, &dac_count);
    detected_dacs.clear();
    if (dac_count > 0)
      detected_dacs.assign(dacs, dacs + dac_count);

    // Convert the detected DACs into `NannouLaserDeviceIdentifier`s that SMODE can work with.
    for (auto dac : detected_dacs) {
      String mac_string = macAddressToString(dac.kind.ether_dream.broadcast.mac_address);
      res.push_back(new NannouLaserDeviceIdentifier(getName(), mac_string));
    }
    return true;
  }

  Device* createDevice(const DeviceIdentifier& identifier, GraphicsContext& graphics) const override
  {
    laser::DetectedDac* detectedDac = nullptr;
    for (auto dac : detected_dacs)
    {
      String mac_string = macAddressToString(dac.kind.ether_dream.broadcast.mac_address);
      if (mac_string == identifier.getFriendlyName())
        detectedDac = &dac;
    }
    return new NannouLaserDevice(identifier, &api, detectedDac);
  }

  OIL_OBJECT(NannouLaserDeviceFactory);

private:
  mutable laser::Api api;
  laser::DetectDacsAsync dac_detector = { 0, };
  std::vector<laser::DetectedDac> detected_dacs;
  typedef DeviceFactory BaseClass;
};
}; /* namespace smode */

#endif // !SMODE_NANNOU_LASER_DEVICE_FACTORY_H_
