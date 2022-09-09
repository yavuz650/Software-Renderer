#ifndef RASTERIZER_HPP_
#define RASTERIZER_HPP_

#include <vector>
#include <array>
#include <cmath>
#include <eigen3/Eigen/Core>
#include "triangle.hpp"
#include "utils.hpp"

using namespace Eigen;

class Rasterizer{
 public:
  void rasterize(std::vector<triangle> &triangles);
};

#endif