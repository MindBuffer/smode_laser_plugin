/*-----------------------------.---------------------------------------------.
| Filename: xxxLibrary.h/cpp   | Oil Generated Library:                      |
| Author  : LibraryGenerator   | plugin                                      |
| Started : Automatic          | Delete this file if you have include issues |
` ---------------------------- . ------------------------------------------ */

#include "LaserLibrary.h"
#include "Laser.h" // automatic

namespace smode
{

class LaserClass : public oil::CppClass
{
public:
  oil::Class* declareBaseClass() const override
    {return getClassRelativeToThis("Tool");}

  bool isAbstract() const override
    {return false;}

  Object* createObjectImpl() const override
    {return new Laser();}

  void declareMemberVariables(ExecutionContext& context) override
  {
    StateMap onLaserParameters;
    onLaserParameters["default"] = State::createFromString(context, "true", "Declaration of Laser::on::default");
    addMemberVariable(context, "Boolean", "on", false, onLaserParameters);
    addMemberVariable(context, "WeakPointer(Layer)", "target", false);
  }
  const Object& getMemberVariable(const Object& __obj__, size_t __variableIndex__) const override
  {
    const Laser& __object__ = (const Laser& )__obj__;
    const size_t __baseClassNumMemberVariables__ = getBaseClassNumMemberVariables();
    if (__variableIndex__ < __baseClassNumMemberVariables__)
      return getBaseClass()->getMemberVariable(__object__, __variableIndex__);
    __variableIndex__ -= __baseClassNumMemberVariables__;

    switch (__variableIndex__) // if you have compilation error bellow, this is probably because you miss to add OIL_OBJECT macro to your class.
    {
      case 0: return __object__.on;
      case 1: return __object__.target;
      default: jassertfalse; return __object__;
    };
  }

  JUCE_LEAK_DETECTOR(LaserClass);
};

oil::Class* Laser::staticClass = nullptr;


class pluginLibrary : public smode::PluginLibrary
{
public:
  pluginLibrary()
  {
    addClass("Laser", Laser::staticClass = new LaserClass());
  }

  ~pluginLibrary() override
  {
    removeClass(Laser::staticClass);
    Laser::staticClass = nullptr;
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

oil::Class* getClassPointerByLookup(const Laser*) {return Laser::staticClass;}

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
