#include "triangle.hpp"

Vector3f triangle::calculateNormal(){
  return (v[2]-v[0]).cross((v[1]-v[0]));
}

Vector3f triangle::calculateNormal2(){
  return (v[1]-v[0]).cross((v[2]-v[1]));
}

triangle::triangle() {}

triangle::triangle(Vector3f v0, Vector3f v1, Vector3f v2) {
  v[0] = v0;
  v[1] = v1;
  v[2] = v2;
  normal = calculateNormal();
}

triangle::triangle(Vector3f v0, Vector3f v1, Vector3f v2,
                    Vector2f uv0, Vector2f uv1, Vector2f uv2) {
  v[0] = v0;
  v[1] = v1;
  v[2] = v2;
  uv[0] = uv0;
  uv[1] = uv1;
  uv[2] = uv2;
  normal = calculateNormal();
}

triangle::triangle(std::array<Vector3f, 3> v_, std::array<Vector2f, 3> uv_) {
  v[0] = v_[0];
  v[1] = v_[1];
  v[2] = v_[2];
  uv[0] = uv_[0];
  uv[1] = uv_[1];
  uv[2] = uv_[2];
  normal = calculateNormal();
}

std::array<Vector3f,3> triangle::getVertices(){
  return v;
}

std::array<Vector2f,3> triangle::getUv(){
  return uv;
}

void triangle::createFragment(Vector2f coords){
  fragments.push_back(coords);
}

std::vector<Vector2f>& triangle::getFragments(){
  return fragments;
}

void triangle::transform(Matrix4f M, Matrix4f N, Matrix4f viewport = Matrix4f::Identity()){
  Vector4f temp;
  for (int i = 0; i < 3; i++)
  {
    temp = Vector4f(v[i](0), v[i](1), v[i](2), 1.0f);
    temp = M*temp;
    v[i] = Vector3f(temp(0)/temp(3),temp(1)/temp(3),temp(2)/temp(3));
  }
  temp = Vector4f(normal(0),normal(1),normal(2),0);
  temp = N*temp;
  normal = Vector3f(temp(0),temp(1),temp(2));
  for (int i = 0; i < 3; i++)
  {
    temp = Vector4f(v[i](0), v[i](1), v[i](2), 1.0f);
    temp = viewport*temp;
    v[i] = Vector3f(temp(0)/temp(3),temp(1)/temp(3),temp(2)/temp(3));
  }
}

Vector3f triangle::barycentricCoords(Vector3f P)
{
  Vector3f a = v[0];
  Vector3f b = v[1];
  Vector3f c = v[2];
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

Vector3f triangle::getNormal(){
  return normal;
}
