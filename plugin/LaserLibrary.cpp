/*-----------------------------.---------------------------------------------.
| Filename: xxxLibrary.h/cpp   | Oil Generated Library:                      |
| Author  : LibraryGenerator   | plugin                                      |
| Started : Automatic          | Delete this file if you have include issues |
` ---------------------------- . ------------------------------------------ */

#include "LaserLibrary.h"
#include "Laser.h" // automatic
#include "LaserDevice.h" // automatic
#include "LaserDeviceFactory.h" // automatic
#include "LaserDeviceIdentifier.h" // automatic

namespace smode
{

class LaserDeviceFactoryClass : public oil::CppClass
{
public:
  oil::Class* declareBaseClass() const override
    {return getClassRelativeToThis("DeviceFactory");}

  bool isAbstract() const override
    {return false;}

  Object* createObjectImpl() const override
    {return new LaserDeviceFactory();}


  JUCE_LEAK_DETECTOR(LaserDeviceFactoryClass);
};

oil::Class* LaserDeviceFactory::staticClass = nullptr;

class LaserDeviceIdentifierClass : public oil::CppClass
{
public:
  oil::Class* declareBaseClass() const override
    {return getClassRelativeToThis("StringDeviceIdentifier");}

  bool isAbstract() const override
    {return false;}

  Object* createObjectImpl() const override
    {return new LaserDeviceIdentifier();}


  JUCE_LEAK_DETECTOR(LaserDeviceIdentifierClass);
};

oil::Class* LaserDeviceIdentifier::staticClass = nullptr;

class LaserDeviceClass : public oil::CppClass
{
public:
  oil::Class* declareBaseClass() const override
    {return getClassRelativeToThis("ControlDevice");}

  bool isAbstract() const override
    {return false;}

  Object* createObjectImpl() const override
    {return new LaserDevice();}


  JUCE_LEAK_DETECTOR(LaserDeviceClass);
};

oil::Class* LaserDevice::staticClass = nullptr;


class pluginLibrary : public smode::PluginLibrary
{
public:
  pluginLibrary()
  {
    addClass("LaserDeviceFactory", LaserDeviceFactory::staticClass = new LaserDeviceFactoryClass());
    addClass("LaserDeviceIdentifier", LaserDeviceIdentifier::staticClass = new LaserDeviceIdentifierClass());
    addClass("LaserDevice", LaserDevice::staticClass = new LaserDeviceClass());
  }

  ~pluginLibrary() override
  {
    removeClass(LaserDevice::staticClass);
    LaserDevice::staticClass = nullptr;
    removeClass(LaserDeviceIdentifier::staticClass);
    LaserDeviceIdentifier::staticClass = nullptr;
    removeClass(LaserDeviceFactory::staticClass);
    LaserDeviceFactory::staticClass = nullptr;
  }

  String getName() const override
    {return "plugin";}
  String getDescription(const String& language) const override
  {
    if (language == "us-en")
      return "Smode Laser plugin."
    ;
    return String();
  }

  JUCE_LEAK_DETECTOR(pluginLibrary);
};

oil::Class* getClassPointerByLookup(const LaserDeviceFactory*) {return LaserDeviceFactory::staticClass;}
oil::Class* getClassPointerByLookup(const LaserDeviceIdentifier*) {return LaserDeviceIdentifier::staticClass;}
oil::Class* getClassPointerByLookup(const LaserDevice*) {return LaserDevice::staticClass;}

}; /* namespace smode */

smode::PluginLibrary* smode::createpluginLibrary()
  {return new smode::pluginLibrary();}

#ifndef SMODE_STATIC_LIBS
extern "C" {

OIL_EXPORT smode::PluginLibrary* createPluginLibrary()
  {return new smode::pluginLibrary();}


OIL_EXPORT const char* getPluginEdition()
  {return SMODE_EDITION_NAME;}


} /* extern C */
#endif // !SMODE_STATIC_LIBS
