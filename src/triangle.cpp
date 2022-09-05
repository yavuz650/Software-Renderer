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

bool triangle::isInsideTriangle(Vector2i P)
{
  Vector2i A = Vector2i(v[0](0), v[0](1));
  Vector2i B = Vector2i(v[1](0), v[1](1));
  Vector2i C = Vector2i(v[2](0), v[2](1));
  Vector2i AB = B - A;
  Vector2i AC = C - A;
  Vector2i PA = A - P;

  Vector3f u = (Vector3f(AB(0), AC(0), PA(0))).cross(Vector3f(AB(1), AC(1), PA(1)));

  return u(0) / u(2) >= 0. && u(1) / u(2) >= 0. && (u(0) / u(2) + u(1) / u(2)) <= 1.;
}

Vector3f triangle::getNormal(){
  return normal;
}
