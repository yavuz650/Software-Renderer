#ifndef RASTERIZER_HPP_
#define RASTERIZER_HPP_

#include <array>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <vector>

#include "triangle.hpp"
#include "utils.hpp"

using namespace Eigen;

class Rasterizer {
 public:
  void rasterize(std::vector<Triangle> &triangles);
};

#endif