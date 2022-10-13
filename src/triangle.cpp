#include "triangle.hpp"

Vector3f triangle::calculateNormal(){
  Vector3f v0(v[0].coords(0),v[0].coords(1),v[0].coords(2));
  Vector3f v1(v[1].coords(0),v[1].coords(1),v[1].coords(2));
  Vector3f v2(v[2].coords(0),v[2].coords(1),v[2].coords(2));
  return (v0-v2).cross(v1-v0);
}

Vector3f triangle::getNormal(){
  return surfaceNormal;
}

triangle::triangle() {}

triangle::triangle(Vertex v0, Vertex v1, Vertex v2){
  v[0] = v0;
  v[1] = v1;
  v[2] = v2;
  surfaceNormal = calculateNormal();
}

triangle::triangle(std::array<Vertex,3> v_){
  v[0] = v_[0];
  v[1] = v_[1];
  v[2] = v_[2];
  surfaceNormal = calculateNormal();
}

std::array<Vertex, 3>& triangle::getVertices() { return v; }

void triangle::createFragment(Vector2f coords){
  fragments.push_back(coords);
}

std::vector<Vector2f>& triangle::getFragments(){
  return fragments;
}

Vector3f triangle::barycentricCoords(Vector3f P)
{
  Vector3f a(v[0].coords(0),v[0].coords(1),v[0].coords(2));
  Vector3f b(v[1].coords(0),v[1].coords(1),v[1].coords(2));
  Vector3f c(v[2].coords(0),v[2].coords(1),v[2].coords(2));
  float alpha, beta, gamma;
  float x = P(0);
  float y = P(1);
  float term0 = a(1)-b(1); //y_a - y_b
  float term1 = b(0)-a(0); //x_b - x_a
  float term2 = a(0)*b(1) - b(0)*a(1); //x_a*y_b - x_b*y_a
  gamma = (term0*x + term1*y + term2) / (term0*c(0) + term1*c(1) + term2);

  term0 = a(1)-c(1); //y_a - y_c
  term1 = c(0)-a(0); //x_b - x_a
  term2 = a(0)*c(1) - c(0)*a(1); //x_a*y_c - x_c*y_a
  beta = (term0*x + term1*y + term2) / (term0*b(0) + term1*b(1) + term2);
  alpha = 1.0 - beta - gamma;

  return Vector3f(alpha, beta, gamma);
}

bool triangle::isInsideTriangle(Vector3f P)
{
  Vector3f u = barycentricCoords(P);
  return u(0) >= 0 &&
         1.0f >= u(1) && u(1) >= 0 &&
         1.0f >= u(2) && u(2) >= 0;
}

bool triangle::isBackface(){
  Vector3f normalVector = -calculateNormal();
  if(normalVector.dot(Vector3f(0,0,-1.0f)) < 0)
    return true;
  return false;
}  
