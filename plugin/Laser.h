#ifndef SMODE_LASER_H_
#define SMODE_LASER_H_

#include "../smode_laser/smode_laser.h"
//#include "MathFuncsStaticLib/MathFuncsLib.h"

namespace smode
{

class Laser : public Tool
{
public:
  Laser() {}

OIL_OBJECT(Laser);

private:
  Boolean on;
  WeakPointer<Layer> target;

public:
  // Object
  void variableChangedCallback(Object* variable, Object* changedObject) override
  {
    if ((variable == &on || variable == &target) && target)
      target->setActivation(on ? ActivationState::active : ActivationState::inactive);
      Tool::variableChangedCallback(variable, changedObject);

	  String dbg = String(rust_test_func());
	  DBG("rusty test val: " << dbg);

	// double a = 7.4;
	// int b = 99;

	// auto add = MathFuncs::MyMathFuncs::Add(a, b);
	
	
	// DBG("a + b = " << add);
	// DBG("a - b = " << MathFuncs::MyMathFuncs::Subtract(a, b));
	// DBG("a * b = " << MathFuncs::MyMathFuncs::Multiply(a, b));
	// DBG("a / b = " << MathFuncs::MyMathFuncs::Divide(a, b));
	
	//ExecutionContext::get().warning("hello world");

    //DBG("ddbbggg");
    //juce::Logger::OutputDebugString("hello world");
    //juce::Logger::OutputDebugString(std::to_string(Test::get_value()););
  }
};

}; /* namespace smode */

#endif // !SMODE_LASER_H_