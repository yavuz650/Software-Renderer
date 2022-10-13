#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <vector>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "SDL2/SDL.h"

#include "tgaimage.hpp"

using namespace Eigen;

Matrix4f viewport(int width, int height);
Vector3f barycentricCoords(Vector2i A, Vector2i B, Vector2i C, Vector2i P);
//Returns the view matrix
Matrix4f lookAt(Vector3f eye, Vector3f target, Vector3f up);
Matrix4f orthographic(float l, float r, float b, float t, float n, float f);
Matrix4f perspective(float l, float r, float b, float t, float n, float f);
Matrix4f translate(Matrix4f const &m, Vector3f v);

class boundingBox{
 private:
  Vector2f bottomLeft;
  Vector2f topRight;
 public:
  boundingBox(Vector2f bottomLeft_, Vector2f topRight_);
  Vector2f &BL();
  Vector2f &TR();
    // debug
  //  line(bboxTopLeft.x,bboxTopLeft.y,
  //      bboxBottomRight.x,bboxTopLeft.y, renderer, color);

  // line(bboxTopLeft.x,bboxTopLeft.y,
  //     bboxTopLeft.x,bboxBottomRight.y, renderer, color);

  // line(bboxBottomRight.x,bboxTopLeft.y,
  //     bboxBottomRight.x,bboxBottomRight.y, renderer, color);

  // line(bboxTopLeft.x,bboxBottomRight.y,
  //     bboxBottomRight.x,bboxBottomRight.y, renderer, color);
};

class DepthBuffer{
 private:
  int screenWidth;
  int screenHeight;
  std::vector<std::vector<float>> buffer;
 public:
  DepthBuffer(int screenWidth_, int screenHeight_);

  void visualize(SDL_Renderer *renderer, int screenWidth, int screenHeight);
  float get(int x, int y) const;
  void set(int x, int y, float f);
  void resetBuffer();
  void printBuffer();
};

void line(int x0, int y0, int x1, int y1, SDL_Renderer *renderer, TGAColor color);

#endif