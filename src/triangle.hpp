#ifndef TRIANGLE_HPP_
#define TRIANGLE_HPP_

#include <vector>
#include <array>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "SDL2/SDL.h"

#include "utils.hpp"

using namespace Eigen;

class triangle{
 private:
  //Vertices
  std::array<Vector3f,3> v;
  //Texture coordinates
  std::array<Vector2f,3> uv;
  //Surface normal
  Vector3f normal;
  std::vector<Vector2f> fragments;
  Vector3f calculateNormal();
 public:
  Vector3f calculateNormal2();
  triangle();
  triangle(Vector3f v0, Vector3f v1, Vector3f v2);
  triangle(Vector3f v0, Vector3f v1, Vector3f v2,
           Vector2f uv0, Vector2f uv1, Vector2f uv2);
  triangle(std::array<Vector3f, 3> v_, std::array<Vector2f, 3> uv_);
  std::array<Vector3f,3> getVertices();
  std::array<Vector2f,3> getUv();
  void createFragment(Vector2f coords);
  std::vector<Vector2f>& getFragments();
  void transform(Matrix4f M, Matrix4f N, Matrix4f viewport);
  Vector3f barycentricCoords(Vector3f P);
  bool isInsideTriangle(Vector3f P);
  Vector3f getNormal();
};

#endif