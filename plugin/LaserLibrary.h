/*-----------------------------.---------------------------------------------.
| Filename: xxxLibrary.h/cpp   | Oil Generated Library:                      |
| Author  : LibraryGenerator   | plugin                                      |
| Started : Automatic          | Delete this file if you have include issues |
` ---------------------------- . ------------------------------------------ */

#ifndef SMODE_PLUGIN_LIBRARY_H_
# define SMODE_PLUGIN_LIBRARY_H_

# include <SmodeSDK/SmodeSDK.h>

namespace smode
{

class LaserDeviceFactory;
class LaserDeviceIdentifier;
class LaserDevice;
class PluginLibrary;

extern oil::Class* getClassPointerByLookup(const LaserDeviceFactory* );
extern oil::Class* getClassPointerByLookup(const LaserDeviceIdentifier* );
extern oil::Class* getClassPointerByLookup(const LaserDevice* );


smode::PluginLibrary* createpluginLibrary();

}; /* namespace smode */

#endif // !SMODE_PLUGIN_LIBRARY_H_
