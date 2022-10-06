#include "triangle.hpp"

Vertex::Vertex() {}
Vertex::Vertex(Vector3f coords_)
    : coords(Vector4f(coords_(0), coords_(1), coords_(2), 1)) {}
Vertex::Vertex(Vector3f coords_, Vector2f uv_)
    : coords(Vector4f(coords_(0), coords_(1), coords_(2), 1)), uv(uv_) {}
Vertex::Vertex(Vector3f coords_, Vector2f uv_, Vector3f normal_)
    : coords(Vector4f(coords_(0), coords_(1), coords_(2), 1)),
      uv(uv_),
      normal(normal_) {}

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

std::array<Vertex,3> triangle::getVertices(){
  return v;
}

void triangle::createFragment(Vector2f coords){
  fragments.push_back(coords);
}

std::vector<Vector2f>& triangle::getFragments(){
  return fragments;
}

void triangle::transform(Matrix4f model, Matrix4f view, Matrix4f projection,
                         Matrix4f viewport) {
  Vector4f temp;
  Matrix4f MVP = projection*view*model;
  Matrix4f MV = view*model;
  for (int i = 0; i < 3; i++)
  {
    Vertex vert(v[i]);
    v[i].coords = view*model*v[i].coords;
    v[i].fragPos = Vector3f(v[i].coords(0) / v[i].coords(3),
                            v[i].coords(1) / v[i].coords(3),
                            v[i].coords(2) / v[i].coords(3));

    v[i].coords = projection * v[i].coords;
    v[i].coords = viewport * v[i].coords;
    v[i].coords = Vector4f(v[i].coords(0) / v[i].coords(3),
                           v[i].coords(1) / v[i].coords(3),
                           v[i].coords(2) / v[i].coords(3), 1/v[i].coords(3));
    //Transform the normal vector
    temp = Vector4f(vert.normal(0),vert.normal(1),vert.normal(2),0);
    temp = MV.inverse().transpose()*temp;
    v[i].normal = -Vector3f(temp(0),temp(1),temp(2));
  }
  //Transform the surface normal vector
  temp = Vector4f(surfaceNormal(0),surfaceNormal(1),surfaceNormal(2),0);
  temp = MVP.inverse().transpose()*temp;
  surfaceNormal = Vector3f(temp(0),temp(1),temp(2));
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
