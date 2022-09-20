#include "triangle.hpp"

Vertex::Vertex() {}
Vertex::Vertex(Vector3f coords_) : coords(coords_) {}
Vertex::Vertex(Vector3f coords_,
               Vector2f uv_) : coords(coords_), uv(uv_) {}
Vertex::Vertex(Vector3f coords_,
               Vector2f uv_,
               Vector3f normal_) : coords(coords_), uv(uv_), normalInView(normal_) {}


Vector3f triangle::calculateNormal(){
  return (v[0].coords-v[2].coords).cross((v[1].coords-v[0].coords));
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

void triangle::transform(Matrix4f modelView, Matrix4f projectionViewport,
                         Matrix4f invModelView) {
  Vector4f temp;
  for (int i = 0; i < 3; i++)
  {
    Vertex vert(v[i]);
    //Transform the vertex
    temp = Vector4f(vert.coords(0), vert.coords(1), vert.coords(2), 1.0f);
    temp = modelView*temp;
    v[i].vertPosInView = Vector3f(temp(0)/temp(3),temp(1)/temp(3),temp(2)/temp(3));
    temp = projectionViewport*temp;
    v[i].coords = Vector3f(temp(0)/temp(3),temp(1)/temp(3),temp(2)/temp(3));
    //Transform the normal vector
    temp = Vector4f(vert.normalInView(0),vert.normalInView(1),vert.normalInView(2),0);
    temp = invModelView*temp;
    v[i].normalInView = Vector3f(temp(0),temp(1),temp(2));
  }
  //Transform the surface normal vector
  // temp = Vector4f(surfaceNormal(0),surfaceNormal(1),surfaceNormal(2),0);
  // temp = N*temp;
  // surfaceNormal = Vector3f(temp(0),temp(1),temp(2));
}

Vector3f triangle::barycentricCoords(Vector3f P)
{
  Vector3f a = v[0].coords;
  Vector3f b = v[1].coords;
  Vector3f c = v[2].coords;
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
