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


}; /* namespace smode */

#endif // !SMODE_LASER_GEOMETRY_RENDERER_H_