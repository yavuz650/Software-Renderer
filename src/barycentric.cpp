#include <iostream>
#include "triangle.hpp"

Vector3f bary_int(triangle T, Vector2i P)
{
  auto v = T.getVertices();
  Vector2i A = Vector2i(v[0](0), v[0](1));
  Vector2i B = Vector2i(v[1](0), v[1](1));
  Vector2i C = Vector2i(v[2](0), v[2](1));
  Vector2i AB = B - A;
  Vector2i AC = C - A;
  Vector2i PA = A - P;

  Vector3f u = (Vector3f(AB(0), AC(0), PA(0))).cross(Vector3f(AB(1), AC(1), PA(1)));

  return Vector3f(1-(u(0) / u(2)) - (u(1) / u(2)), u(0) / u(2), u(1) / u(2) );
  //return u(0) / u(2) >= 0. && u(1) / u(2) >= 0. && (u(0) / u(2) + u(1) / u(2)) <= 1.;
}

int main()
{

  // triangle T (Vector3f(438.134216, 425.602997, -2.588160),
  //             Vector3f(473.603943, 399.310272, -2.627651),
  //             Vector3f(446.371277, 451.419586, -2.639472));

  triangle T(Vector3f(1,0,0), Vector3f(4,0,0), Vector3f(1,3,0));


  //Vector3f P(465.134216, 405.310272, 0.000000);
  //Vector3f P(472.0f, 401.0f, 0.000000);
  //Vector3f P(438.134216, 425.602997, -2.588160);
  Vector3f P(1,2.999,0);

  Vector3f floating = T.barycentricCoords(P);
  if(T.isInsideTriangle(P))
    printf("Inside\n");
  Vector3f integer = bary_int(T,Vector2i(P(0),P(1)));

  printf("floating: %f %f %f\ninteger: %f %f %f\n",floating(0),floating(1),floating(2),
  integer(0),integer(1),integer(2));

}