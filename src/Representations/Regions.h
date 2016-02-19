
#pragma once

#include "Tools/Range.h"
#include "Tools/Vector2.h"
#include "Representations/Image.h"
#include "Representations/ColorModel/ColorModel.h"
#include <cv.h>

class Regions {

public:
    using Color = ColorModel::Colors;
	struct Line {
		Vector2<int> left;
		Vector2<int> right;
        int depth;
        Color color;
        Line(Vector2<int> left, Vector2<int> right,int depth, Color color) : left(left), right(right), depth(depth), color(color) {}
        
        Vector2<int> getCenter()
        {
            return (left + right)/2.f;
        }
	};
    
    void draw(ImageBGR* image) const;
	
	std::vector<Line> regions;
};
