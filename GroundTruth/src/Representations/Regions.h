
#pragma once

#include "Representations/ColorModel/ColorModel.h"
#include "Tools/Math/Range.h"
#include "Tools/Math/Vector2.h"
#include "Tools/Streamable.h"
#include <opencv2/core/core.hpp>

class Regions : public Streamable
{
public:

  struct Rect {
    Vector2<int> center;
    Vector2<int> leftUpper;
    Vector2<int> rightBottom;
    ColorModel::Colors color;
    Rect(const Vector2<int>& center, const Vector2<int> &leftUpper, const Vector2<int>& rightBottom, const ColorModel::Colors& color) : center(center), leftUpper(leftUpper), rightBottom(rightBottom), color(color) {}

  };
  
  void draw() const;
  
  std::vector<Rect> regions;
};
