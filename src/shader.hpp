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

class Shader{
 public:
  virtual void vertexShader(std::vector<triangle> &input) =0;
  virtual void fragmentShader(std::vector<triangle> &input, ZBuffer zbuffer,
                              SDL_Renderer *renderer) = 0;

  virtual ~Shader() {}
};

class HeadShader : public Shader{
 public:
  Matrix4f model, view, projection, viewport;
  TGAImage diffuseMap;
  TGAImage specularMap;
  SDL_Renderer *renderer;
  Vector3f lightDir;
  
  void vertexShader(std::vector<triangle> &input) override;
  void fragmentShader(std::vector<triangle> &input, ZBuffer zbuffer,
                      SDL_Renderer *renderer) override;
};

#endif