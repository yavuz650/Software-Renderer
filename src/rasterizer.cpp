#include "rasterizer.hpp"
#define WINDOW_HEIGHT 1024
#define WINDOW_WIDTH 1024
using namespace Eigen;

void Rasterizer::rasterize(std::vector<triangle> &triangles){
  for (int i = 0; i < triangles.size(); i++){
    std::array<Vector3f,3> v = triangles[i].getVertices();
    boundingBox bbox(Vector2f(WINDOW_WIDTH - 1, WINDOW_HEIGHT - 1), Vector2f(0, 0));
    for (int i = 0; i < 3; i++)
    {
      bbox.TR()(0) = std::max(bbox.TR()(0), v[i](0));
      bbox.TR()(1) = std::max(bbox.TR()(1), v[i](1));

      bbox.BL()(0) = std::min(bbox.BL()(0), v[i](0));
      bbox.BL()(1) = std::min(bbox.BL()(1), v[i](1));
    }

    Vector3f P;
    for (P(0) = bbox.BL()(0); P(0) <= bbox.TR()(0); P(0)++){
      for (P(1) = bbox.BL()(1); P(1) <= bbox.TR()(1); P(1)++){
        if(!(P(0) >=0 && P(0) <WINDOW_WIDTH && P(1) >= 0 && P(1) <WINDOW_HEIGHT))
          continue;
        if (triangles[i].isInsideTriangle(Vector2i(P(0), P(1)))){
          triangles[i].createFragment(Vector2f(P(0), P(1)));
        }
      }
    }
  }
}
