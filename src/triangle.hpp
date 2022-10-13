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
  //All vertices must have coordinates, everything else is optional.
  Vector4f coords;
  //Optional attributes (normal vector, UV coordinates etc.)
  std::vector<Vector4f> vec4f;
  std::vector<Vector3f> vec3f;
  std::vector<Vector2f> vec2f;
};

class Triangle{
 private:
  std::array<Vertex,3> v;
  std::vector<Vector2f> fragments;
  Vector3f surfaceNormal;
 public:
  Vector3f calculateNormal();
  Triangle();
  Triangle(Vertex v0, Vertex v1, Vertex v2);
  Triangle(std::array<Vertex,3> v);
  std::array<Vertex,3> &getVertices();
  void createFragment(Vector2f coords);
  std::vector<Vector2f>& getFragments();
  Vector3f barycentricCoords(Vector3f P);
  bool isInsideTriangle(Vector3f P);
  Vector3f getNormal();
  bool isBackface();
};

#endif