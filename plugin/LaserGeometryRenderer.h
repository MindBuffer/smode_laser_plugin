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

  void variableChangedCallback(Object* variable, Object* changedObject) override
  {
    if (!isActive() && isRenderingServiceCurrent()) {
      LaserDevice* laserDevice = device.getDevice();
      if (laserDevice) {
        const std::vector<Point> empty;
        laserDevice->updateFrame(empty);
      }
    }
  }

  // GeometryLayerUser
  bool usesGeometryType(GeometryType::Enum type) const override
    {return type == GeometryType::lines || type == GeometryType::points;}

  void renderGeometry(GraphicsRenderer& renderer, LayerInterpreter& interpreter, const Geometry& geometry, double opacity) override
  {
    LaserDevice* laserDevice = device.getDevice();
    if (!laserDevice) {
      return;
    }

    // In the case that any of the following fails, we still want to submit at least an empty frame.
    // This is because the render callback always emits the last received frame.
    points.clear();
    if (inputGeometry.compute(renderer, geometry)) {
      String failureReason;
      if (inputGeometry.downloadToRam(renderer.getGraphics(), downloadedPoints, failureReason)) {
        if (!downloadedPoints.empty()) {
          computeFinalPoints(downloadedPoints, points);
        }
      }
    }
    laserDevice->updateFrame(points);
  }

  // Element
  int64_t computeVersion() const override
    {return (int64_t)getCurrentFrameNumber();}

  OIL_OBJECT(LaserGeometryRenderer);

protected:
  typedef LaserDevice::Point Point;

  LaserDeviceSelector device;
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
      }
      else if (!isSameColor(previousB.color, currentA.color))
        res.push_back(currentA);
      res.push_back(currentB);
    }
  }

  LaserInputGeometry inputGeometry; // not introspected

private:
  typedef GeometryLayerUser BaseClass;
};

}; /* namespace smode */

#endif // !SMODE_LASER_GEOMETRY_RENDERER_H_