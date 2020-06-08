/* -------------------------------------- . ---------------------------------- .
| Filename : LaserGeometryRenderer.h      |                                    |
| Author   : Francis Maes                 |                                    |
| Started  : 07/04/2020 08:06             |                                    |
` --------------------------------------- . --------------------------------- */

#ifndef SMODE_LASER_GEOMETRY_RENDERER_H_
# define SMODE_LASER_GEOMETRY_RENDERER_H_

#include "LaserLibrary.h"
#include "LaserInputGeometry.h"
#include "LaserDevice.h"

namespace smode
{

class LaserGeometryRenderer : public GeometryLayerUser
{
public:
  LaserGeometryRenderer() {}

  // GeometryLayerUser
  bool usesGeometryType(GeometryType::Enum type) const override
    {return type == GeometryType::lines || type == GeometryType::points;}

  void renderGeometry(GraphicsRenderer& renderer, LayerInterpreter& interpreter, const Geometry& geometry, double opacity) override
  {
    LaserDevice* laserDevice = device.getDevice();
    if (!laserDevice)
      return;

    points.clear();
    bool ok;
    const GeometryMask& mask = maskCompositer.updateAndGetMask(geometry, masks);
    if (geometryType == LaserGeometryType::points)
      ok = inputGeometry.computePoints(renderer, geometry, mask, (float)maskThreshold, weight);
    else
      ok = inputGeometry.computeLines(renderer, geometry, mask, (float)maskThreshold, weight);
    if (ok)
    {
      String failureReason;
      if (inputGeometry.downloadToRam(renderer.getGraphics(), downloadedPoints, failureReason) && !downloadedPoints.empty())
      {
        if (geometryType == LaserGeometryType::points)
          computeFinalPoints(downloadedPoints, points);
        else
          computeFinalLines(downloadedPoints, points);
      }
    }
    laserDevice->addLineSequence(points);
  }

  // Element
  int64_t computeVersion() const override
    {return (int64_t)getCurrentFrameNumber();}

  OIL_OBJECT(LaserGeometryRenderer);

protected:
  LaserDeviceSelector device;
  LaserGeometryType geometryType;
  StrictlyPositiveInteger weight;
  Percentage maskThreshold;
  OwnedVector<GeometryMask> masks;

private:
  GeometryMaskCompositer maskCompositer;

private:
  typedef LaserDevice::Point Point;

  // not introspected:
  LaserInputGeometry inputGeometry;
  std::vector<Point> downloadedPoints;
  std::vector<Point> points;

  static bool isSamePosition(float a, float b)
    {return a == b;}

  static bool isSamePosition(const glm::vec2& a, const glm::vec2& b)
    {return isSamePosition(a.x, b.x) && isSamePosition(a.y, b.y);}

  static bool isSameColor(float a, float b)
    {return a == b;}

  static bool isSameColor(const glm::vec3& a, const glm::vec3& b)
    {return isSameColor(a.r, b.r) && isSameColor(a.g, b.g) && isSameColor(a.b, b.b);}

  static bool isSameColorAndPosition(const Point& a, const Point& b)
    {return isSamePosition(a.position, b.position) && isSameColor(a.color, b.color);}

  static Point withBlack(const Point& p)
    {Point res = p; res.color = glm::vec3(0.f); return res;}

  void computeFinalPoints(const std::vector<Point>& points, std::vector<Point>& res)
  {
    for (size_t i = 0; i < points.size(); ++i)
    {
      const Point& point = points[i];
      res.push_back(withBlack(point)); 
      res.push_back(point);
      res.push_back(point);
      res.push_back(withBlack(point));
    }
  }
  
  void computeFinalLines(const std::vector<Point>& points, std::vector<Point>& res)
  {
    jassert(points.size() % 2 == 0);
    if (points.size() < 2)
      return;
    res.push_back(points[0]);
    res.push_back(points[1]);
    for (size_t i = 2; i < points.size(); i += 2)
    {
      const Point& previousB = points[i - 1];
      const Point& currentA = points[i];
      const Point& currentB = points[i + 1];
      if (!isSamePosition(previousB.position, currentA.position))
      {
        res.push_back(withBlack(previousB));
        res.push_back(withBlack(currentA));
        res.push_back(currentA);
      }
      else if (!isSameColor(previousB.color, currentA.color))
        res.push_back(currentA);
      res.push_back(currentB);
    }
  }

private:
  typedef GeometryLayerUser BaseClass;
};

class LaserShapeRenderer : public ShapeRenderer
{
public:
  LaserShapeRenderer() {}

  struct AtomicShapeRendererCallback : public ShapeList::Callback
  {
    AtomicShapeRendererCallback() : graphicsRenderer(nullptr), renderer(nullptr), opacity(1.0) {}

    GraphicsRenderer* graphicsRenderer;
    LaserShapeRenderer* renderer;
    ShapeContext context;
    double opacity;

    bool processShape(const Shape& shape) override
    {
      context.getShapeLength().set(shape.getEdgeLength());
      renderer->renderShape(*graphicsRenderer, shape, context, opacity);
      context.gotoNextShape();
      return true;
    }
  };


  // ShapeRenderer
  void renderShapes(GraphicsRenderer& renderer, const ShapeList& shapes, const glm::mat4& transform, double opacity) override
  {
// FIXME: ugly copy-paste from AtomicShapeRenderer
    BaseClass::renderShapes(renderer, shapes, transform, opacity);

    const ShapeList& modifiedShapes = ShapeModifier::applyModifiers(shapes, modifiers);

    glm::vec2 canvasResolution = renderer.getTargetResolution();
    glm::mat4 currentWindowMatrix = smode::glmWindowInverseMatrix * renderer.getWindowMatrix();
    renderer.enterWindow(glm::inverse(currentWindowMatrix));
    glm::mat4 subTransform = currentWindowMatrix * (placement ? placement->getTransformMatrix() : glm::mat4(1.f)) * transform;

    AtomicShapeRendererCallback callback;
    ShapeContext& attributes = callback.context;
    attributes.getShapeIndex().set(0);
    attributes.getNumShapes().set(shapes.size());
    attributes.getShapeEdgePosition().set(0.0);
    attributes.getOverallLength().set(shapes.computeTotalEdgeLength(canvasResolution));
    attributes.getDistanceType().set(2);
    attributes.getMaxAbsDistance().set(100.0);

    callback.graphicsRenderer = &renderer;
    callback.renderer = this;
    callback.opacity = this->opacity * opacity;
    modifiedShapes.processShapes(canvasResolution, subTransform, callback);

    renderer.leaveWindow();
  }

  void renderShape(GraphicsRenderer& renderer, const Shape& shape, const ShapeContext& attributes, double opacity)
  {
    LaserDevice* laserDevice = device.getDevice();
    if (!laserDevice)
      return;

    glm::vec2 targetResolution(renderer.getTargetResolution());
    std::vector<Point> points;
    points.reserve(100);
    float edgeLengthPx = (float)shape.getEdgeLength();
    float step = juce::jmax(edgeLengthPx / 100.f, 1.f);
    glm::vec3 color((float)opacity);
    for (float k = 0.f; k <= edgeLengthPx; k += step)
    {
      glm::vec2 position, edgeDirection, shapeNormal;
      if (shape.getEdgeReferential(k, position, edgeDirection, shapeNormal))
      {
        Point point;
        point.color = color;
        point.position.x = 2.f * position.x / targetResolution.x - 1.f;
        point.position.y = 1.f - 2.f * position.y / targetResolution.y;
        point.weight = 1;
        points.push_back(point);
      }
    }
    if (shape.isClosed() && points.size())
      points.push_back(points[0]);
    laserDevice->addLineSequence(points);
  }

  // Element
  int64_t computeVersion() const override
    {return (int64_t)getCurrentFrameNumber();}

  OIL_OBJECT(LaserShapeRenderer);

protected:
  typedef LaserDevice::Point Point;

  LaserDeviceSelector device;

private:
  typedef ShapeRenderer BaseClass;
};

}; /* namespace smode */

#endif // !SMODE_LASER_GEOMETRY_RENDERER_H_
