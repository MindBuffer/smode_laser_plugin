/* -------------------------------------- . ---------------------------------- .
| Filename : LaserInputGeometry.h         |                                    |
| Author   : Francis Maes                 |                                    |
| Started  : 07/04/2020 08:06             |                                    |
` --------------------------------------- . --------------------------------- */

#ifndef SMODE_LASER_INPUT_GEOMETRY_H_
# define SMODE_LASER_INPUT_GEOMETRY_H_

#include "LaserLibrary.h"

namespace smode
{

class LaserInputGeometry : public RenderedGeometry
{
public:
  LaserInputGeometry() : numPoints(0), numLines(0) {}

  bool computePoints(GraphicsRenderer& renderer, const Geometry& inputGeometry, const GeometryMask& mask, float maskThreshold, size_t weight)
  {
    configure(pointsGeometryShader, mask, maskThreshold, weight);
    numLines = 0;
    numPoints = inputGeometry.getNumPoints();
    String failureReason;
    bool res = transformFeedbackGeometry(renderer, inputGeometry, 1, inputGeometry.getNumPoints(), pointsGeometryShader, failureReason);
    if (!res)
      numPoints = 0;
    return res;
  }

  bool computeLines(GraphicsRenderer& renderer, const Geometry& inputGeometry, const GeometryMask& mask, float maskThreshold, size_t weight)
  {
    configure(linesGeometryShader, mask, maskThreshold, weight);
    linesGeometryShader.getMask().set(&mask);
    pointsGeometryShader.getMaskTreshold.set(maskThreshold);
    pointsGeometryShader.getWeight().set(weight);
    numPoints = 0;
    numLines = inputGeometry.getNumLines();
    String failureReason;
    bool res = transformFeedbackGeometry(renderer, inputGeometry, 1, inputGeometry.getNumLines() * 2, linesGeometryShader, failureReason);
    if (!res)
      numLines = 0;
    return res;
  }

  // Geometry
  size_t getNumPoints() const override
    {return numPoints;}

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
  PointsLaserGeometryShader pointsGeometryShader;
  LinesLaserGeometryShader linesGeometryShader;

  // RenderedGeometry
  void initializeInterleavedBufferLayer(BufferArray& bufferArray) override
  {
    bufferArray.appendInterleavedBufferLayout(BufferLayout(textureCoordinatesLocation, 2, BufferComponentType::float32));
    bufferArray.appendInterleavedBufferLayout(BufferLayout(colorsLocation, 3, BufferComponentType::float32));
    bufferArray.appendInterleavedBufferLayout(BufferLayout("weight", 1, BufferComponentType::uint32));
  }

  size_t numPoints, numLines;

  void configure(LaserGeometryShader& geometryShader, const GeometryMask& mask, float maskThreshold, size_t weight)
  {
    geometryShader.getMask().set(&mask);
    geometryShader.getMaskTreshold.set(maskThreshold);
    geometryShader.getWeight().set(weight);
  }

private:
  typedef RenderedGeometry BaseClass;
};

}; /* namespace smode */

#endif // !SMODE_LASER_INPUT_GEOMETRY_H_