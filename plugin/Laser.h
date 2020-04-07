/* -------------------------------------- . ---------------------------------- .
| Filename : FIXME.h                      |   Here is a template header        |
| Author   : FIXME                        |                                    |
| Started  : 03/03/2020 10:56             |                                    |
` --------------------------------------- . --------------------------------- */

#ifndef PLUGINS_LASER_DEVICE_H_
# define PLUGINS_LASER_DEVICE_H_

# include "../smode_laser/smode_laser.h"
# include "LaserLibrary.h"
//#include "MathFuncsStaticLib/MathFuncsLib.h"

namespace smode
{

// FIXME: This needs to be splitted in multiple files. 
// From Here: LaserDevice.h

class LaserDevice : public ControlDevice
{
public:
  LaserDevice() {}

  struct Point
  {
    glm::vec2 position; // in range [-1, 1]
    glm::vec3 color;
    uint32_t weight; // 0 for smooth line segments, > 0 for accenting individual points
  };

  virtual void addPoints(const std::vector<Point>& points) = 0;

  OIL_ABSTRACT_OBJECT(LaserDevice);
};

class LaserDeviceSelector : public TypedDeviceSelector<LaserDevice>
{
public:
  LaserDeviceSelector() {}

  OIL_ABSTRACT_OBJECT(LaserDeviceSelector);
};

// FIXME: Create file and implement EtherDreamLaserDevice.h

// FIXME: Extract into new file from Here: LaserGeometryRenderer.h

class LaserInputGeometry : public RenderedGeometry
{
public:
  LaserInputGeometry() : numLines(0) {}

  bool compute(GraphicsRenderer& renderer, const Geometry& inputGeometry)
  {
    numLines = inputGeometry.getNumLines();
    String failureReason;
    bool res = transformFeedbackGeometry(renderer, inputGeometry, 1, inputGeometry.getNumLines() * 2, geometryShader, failureReason);
    if (!res)
      numLines = 0;
    return res;
  }

  // Geometry
  size_t getNumPoints() const override
    {return 0;}

  size_t getNumLines() const override
    {return numLines;}

  size_t getNumTriangles() const override
    {return 0;}

  bool hasProperty(GeometryPropertyEnum p) const override
    {return p == GeometryProperty::lines || p == GeometryProperty::texCoords;}

  bool render(GraphicsRenderer& renderer, GeometryType::Enum geometryType, size_t numInstances) const override
  {
    if (geometryType == GeometryType::lines)
    {
      if (getNumLines() > 0)
      {
#  ifdef JUCE_DEBUG // Because getNumVerticeWritten(renderer.getGraphics()) flush opengl pipeline
        bool isExactValue = this->isNumPrimitivesExact(geometryType);
        const size_t verticesRequired = getNumLines() * 2;
        const size_t verticesWritten = getNumVerticeWritten(renderer.getGraphics());
        if (verticesWritten > verticesRequired || (isExactValue && verticesWritten != verticesRequired))
        {
          DBG(this->getClassName() + " fatal error : num lines registred : " + String(getNumLines()) + " num vertices required : " + String(verticesRequired) + " num vertices written : " + String(verticesWritten));
          jassertfalseOnce; // Your computation of numLines is incorrect please correct it following the spec in Geometry.h
        }
#  endif
        renderer.renderPrimitiveTransformFeedback(*this, transformFeedback, PrimitiveType::lines, numInstances);
      }
      return true;
    }
    return false;
  }

  bool downloadToRam(GraphicsContext& graphics, std::vector<LaserDevice::Point>& res, String& failureReason)
  {
    size_t numPoints = getNumVerticeWritten(graphics);
    if (!numPoints)
      return true;

    if (getBufferArray().getNumBuffers() == 0)
    {
      failureReason = "Invalid number of buffers";
      return false;
    }

    Buffer& buffer = *getBufferArray().getBuffer(0);
    jassert(buffer.getSizeInBytes() / sizeof(LaserDevice::Point) >= numPoints);
    res.resize(numPoints);
    const char* data = (const char*)buffer.readLock();
    memcpy(&res[0], data, numPoints * sizeof(LaserDevice::Point));
    buffer.unlockRead(data);
    return true;
  }

  OIL_OBJECT(LaserInputGeometry);

protected:
  LaserInputGeometryShader geometryShader;

  // RenderedGeometry
  void initializeInterleavedBufferLayer(BufferArray& bufferArray) override
  {
    bufferArray.appendInterleavedBufferLayout(BufferLayout(textureCoordinatesLocation, 2, BufferComponentType::float32));
    bufferArray.appendInterleavedBufferLayout(BufferLayout(colorsLocation, 3, BufferComponentType::float32));
    bufferArray.appendInterleavedBufferLayout(BufferLayout("weight", 1, BufferComponentType::uint32));
  }

  size_t numLines;

private:
  typedef RenderedGeometry BaseClass;
};

class LaserGeometryRenderer : public GeometryLayerUser
{
public:
  LaserGeometryRenderer() {}
  
  // GeometryLayerUser
  bool usesGeometryType(GeometryType::Enum type) const override
    {return type == GeometryType::lines || type == GeometryType::points;}
  
  void renderGeometry(GraphicsRenderer& renderer, LayerInterpreter& interpreter, const Geometry& geometry, double opacity) override
  {
    if (!inputGeometry.compute(renderer, geometry))
      return;

    String failureReason;
    if (!inputGeometry.downloadToRam(renderer.getGraphics(), points, failureReason))
      return;

    if (!points.size())
      return;

    LaserDevice* laserDevice = device.getDevice();
    if (!laserDevice)
      return;

    laserDevice->addPoints(points);
  }

  // Element
  int64_t computeVersion() const override
    {return (int64_t)getCurrentFrameNumber();}

  OIL_OBJECT(LaserGeometryRenderer);

protected:
  LaserDeviceSelector device;
  std::vector<LaserDevice::Point> points;

  LaserInputGeometry inputGeometry; // not introspected

private:
  typedef GeometryLayerUser BaseClass;
};

// FIXME: should not be required anymore soon
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

#endif // !PLUGINS_LASER_DEVICE_H_