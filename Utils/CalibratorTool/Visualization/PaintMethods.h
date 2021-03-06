/**
 * @file Controller/Visualization/PaintMethods.h
 * Declaration of class PaintMethods.
 *
 * @author <A href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</A>
 * @author Colin Graf
 */

#pragma once


#include "DebugDrawing.h"

class DebugDrawing;
class QPainter;
class QBrush;
class QPen;

/**
* @class PaintMethods
*
* Defines static methods to paint debug drawings to QPainters.
*/
class PaintMethods
{
private:
  static QBrush brush;
  static QBrush noBrush;
  static QPen pen;
  static QPen noPen;

public:
  /**
  * Paints a DebugDrawings to a QPainter.
  * @param painter The graphics context the DebugDrawing is painted to.
  * @param debugDrawing The DebugDrawing to paint.
  * @param baseTrans A basic transformation.
  */
  static void paintDebugDrawing(QPainter& painter, const DebugDrawing& debugDrawing, const QTransform& baseTrans);

  static void paintLine(const DebugDrawing::Line& element, QPainter& painter);
  static void paintPolygon(const DebugDrawing::Polygon& element, QPainter& painter);
  static void paintEllipse(const DebugDrawing::Ellipse& element, QPainter& painter);
  static void paintOrigin(const DebugDrawing::Origin& element, QPainter& painter, const QTransform& baseTrans);
  static void paintText(const DebugDrawing::Text& element, QPainter& painter);
  static void paintGridRGBA(const DebugDrawing::GridRGBA& element, QPainter& painter);
  static void paintGridMono(const DebugDrawing::GridMono& element, QPainter& painter);
  static void paintRectangle(const DebugDrawing::Rectangle& element, QPainter& painter);

  static void setPen(const DebugDrawing::Element& element, QPainter& painter);
  static void setBrush(const Drawings::FillStyle fillStyle, const ColorRGBA& fillColor, QPainter& painter);
};

/**
 * Returns the sign of a value.
 * \param a The value.
 * \return The sign of \c a.
 */
template <class V> inline V sgn(V a)
{
  return a < 0 ? V(-1) : a > 0 ? V(1) : 0;
}
