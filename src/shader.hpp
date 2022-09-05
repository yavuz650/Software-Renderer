#ifndef SHADER_HPP_
#define SHADER_HPP_

#include <vector>
#include <array>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "SDL2/SDL.h"
#include "triangle.hpp"
#include "tgaimage.hpp"
#include "utils.hpp"

using namespace Eigen;

class VertexShader{
 private:
  Vector3f lookDir;
  bool isBackface(triangle t);

 public:
  VertexShader();
  void shade(std::vector<triangle> &triangles,
             Matrix4f MVP,
             Matrix4f viewport,
             bool doBackfaceCulling=1);
};

class FragmentShader{
 public:
  void shade(std::vector<triangle> &triangles,
             ZBuffer &zbuffer,
             TGAImage &texture,
             SDL_Renderer *renderer,
             Vector3f lightDir);
};

#endif