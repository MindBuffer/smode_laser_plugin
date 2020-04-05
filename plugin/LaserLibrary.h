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

class Laser;
class PluginLibrary;

extern oil::Class* getClassPointerByLookup(const Laser* );


smode::PluginLibrary* createpluginLibrary();

}; /* namespace smode */

#endif // !SMODE_PLUGIN_LIBRARY_H_
