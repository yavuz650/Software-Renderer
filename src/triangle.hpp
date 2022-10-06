#ifndef TRIANGLE_HPP_
#define TRIANGLE_HPP_

#include <vector>
#include <array>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "SDL2/SDL.h"

#include "utils.hpp"

using namespace Eigen;

class Vertex{
 public:
  //Vertex coordinates
  Vector4f coords;
  //Texture coordinates
  Vector2f uv;
  Vector3f normal;
  Vector3f fragPos;
  Vertex();
  Vertex(Vector3f coords_);
  Vertex(Vector3f coords_,
         Vector2f uv_);
  Vertex(Vector3f coords_,
         Vector2f uv_,
         Vector3f normal_);
};

class triangle{
 private:
  std::array<Vertex,3> v;
  std::vector<Vector2f> fragments;
  Vector3f surfaceNormal;
 public:
  Vector3f calculateNormal();
  triangle();
  triangle(Vertex v0, Vertex v1, Vertex v2);
  triangle(std::array<Vertex,3> v);
  std::array<Vertex,3> getVertices();
  //std::array<Vector2f,3> getUv();
  void createFragment(Vector2f coords);
  std::vector<Vector2f>& getFragments();
  void transform(Matrix4f model, Matrix4f view, Matrix4f projection, Matrix4f viewport);
  Vector3f barycentricCoords(Vector3f P);
  bool isInsideTriangle(Vector3f P);
  Vector3f getNormal();
};

#endif