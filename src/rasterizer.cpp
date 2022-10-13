#include "rasterizer.hpp"
#define WINDOW_HEIGHT 1024
#define WINDOW_WIDTH 1024
using namespace Eigen;

void Rasterizer::rasterize(std::vector<Triangle> &triangles) {
  for (int i = 0; i < triangles.size(); i++) {
    std::array<Vertex, 3> vertices = triangles[i].getVertices();
    std::array<Vector4f, 3> v;
    v[0] = vertices[0].coords;
    v[1] = vertices[1].coords;
    v[2] = vertices[2].coords;
    boundingBox bbox(Vector2f(WINDOW_WIDTH - 1, WINDOW_HEIGHT - 1), Vector2f(0, 0));
    for (int i = 0; i < 3; i++) {
      bbox.TR()(0) = std::max(bbox.TR()(0), v[i](0));
      bbox.TR()(1) = std::max(bbox.TR()(1), v[i](1));

      bbox.BL()(0) = std::min(bbox.BL()(0), v[i](0));
      bbox.BL()(1) = std::min(bbox.BL()(1), v[i](1));
    }
    bbox.TR()(0) = floorf(bbox.TR()(0));
    bbox.TR()(1) = floorf(bbox.TR()(1));
    bbox.BL()(0) = floorf(bbox.BL()(0));
    bbox.BL()(1) = floorf(bbox.BL()(1));
    Vector3f P;
    for (P(0) = bbox.BL()(0); P(0) <= bbox.TR()(0); P(0)++) {
      for (P(1) = bbox.BL()(1); P(1) <= bbox.TR()(1); P(1)++) {
        if(!(P(0) >=0 && P(0) <WINDOW_WIDTH && P(1) >= 0 && P(1) <WINDOW_HEIGHT))
          continue;
        if (triangles[i].isInsideTriangle(P)) {
          triangles[i].createFragment(Vector2f(P(0), P(1)));
        }
      }
    }
#ifndef NDEBUG
  std::cout << "Generated " << triangles[i].getFragments().size() << " fragments\n";
#endif
  }
}
