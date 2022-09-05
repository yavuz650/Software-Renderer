#include "shader.hpp"
#define WINDOW_HEIGHT 1024
#define WINDOW_WIDTH 1024
bool VertexShader::isBackface(triangle t){
  Vector3f normalVector = t.getNormal();
  if(normalVector.dot(lookDir) < 0)
    return true;
  return false;
}

VertexShader::VertexShader() : lookDir(Vector3f(0,0,-1.0)) {}

void VertexShader::shade(std::vector<triangle> &triangles,
                         Matrix4f MVP,
                         Matrix4f viewport,
                         bool doBackfaceCulling){
  Matrix4f N = (MVP.inverse()).transpose();
  if(doBackfaceCulling){
    for (auto it=triangles.begin(); it!=triangles.end();)
    {
      it->transform(MVP,N,viewport);
      if(isBackface(*it))
        it=triangles.erase(it);
      else
        it++;
    }
  }
  else{
    //to be implemented, not really needed though
  }
}

void FragmentShader::shade(std::vector<triangle> &triangles,
                      ZBuffer &zbuffer,
                      TGAImage &texture,
                      SDL_Renderer *renderer,
                      Vector3f lightDir){
  for (int i = 0; i < triangles.size(); i++)
  {
    Vector3f normal = triangles[i].getNormal();
    normal.normalize();
    float intensity = normal.dot(lightDir);
    std::vector<Vector2f> fragments = triangles[i].getFragments();
    std::array<Vector3f,3> v = triangles[i].getVertices();
    std::array<Vector2f,3> uv = triangles[i].getUv();
    //something's fishy about the normal calculation
    normal = triangles[i].calculateNormal2();
    normal.normalize();
    for (int j = 0; j < fragments.size(); j++)
    {
      Vector3f P(fragments[j](0), fragments[j](1), 0);
      float A = normal(0) * (v[0](0) - P(0));
      float B = normal(1) * (v[0](1) - P(1));
      P(2) = A + B / normal(2) + v[0](2);
      if (P(2) > zbuffer.get(P(0),P(1)))
      {
        //determine texture coordinates
        Vector3f barycentricCoords_ = barycentricCoords(Vector2i(v[0](0),v[0](1)), Vector2i(v[1](0),v[1](1)), Vector2i(v[2](0),v[2](1)), Vector2i(P(0),P(1)));

        Vector2f interpolatedUv;
        interpolatedUv(0) = uv[0](0) * barycentricCoords_(0)
                        + uv[1](0) * barycentricCoords_(1)
                        + uv[2](0) * barycentricCoords_(2);

        interpolatedUv(1) = uv[0](1) * barycentricCoords_(0)
                        + uv[1](1) * barycentricCoords_(1)
                        + uv[2](1) * barycentricCoords_(2);

        TGAColor color = texture.get(interpolatedUv(0) * texture.width(),
                                    interpolatedUv(1) * texture.height());

        SDL_SetRenderDrawColor(renderer, intensity*color[2],
                                        intensity*color[1],
                                        intensity*color[0], 255);
        SDL_RenderDrawPoint(renderer, P(0), WINDOW_HEIGHT-P(1)-1);
        zbuffer.set(P(0),P(1),P(2));
      }
    }
  }
}
