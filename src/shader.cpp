#include "shader.hpp"
#define WINDOW_HEIGHT 1024
#define WINDOW_WIDTH 1024

bool VertexShader::isBackface(triangle t){
  Vector3f normalVector = t.getNormal().normalized();
  if(normalVector.dot(lookDir) < 0)
    return true;
  return false;
}

VertexShader::VertexShader() : lookDir(Vector3f(0,0,-1.0)) {}

void VertexShader::shade(std::vector<triangle> &triangles,
                         Matrix4f MVP,
                         Matrix4f viewport,
                         bool doBackfaceCulling){
#ifndef NDEBUG
  std::cout << "Vertex Shader...\n"
            << "  Initial # of triangles: " << triangles.size() << std::endl;
#endif
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
#ifndef NDEBUG
  std::cout << "  Final # of triangles: " << triangles.size() << std::endl;
#endif
}

void FragmentShader::shade(std::vector<triangle> &triangles,
                      ZBuffer &zbuffer,
                      TGAImage &texture,
                      SDL_Renderer *renderer,
                      Vector3f lightDir){
#ifndef NDEBUG
  std::cout << "Fragment Shader...\n"
            << "  Initial # of triangles: " << triangles.size() << std::endl;
#endif
  for (int i = 0; i < triangles.size(); i++)
  {
    //Vector3f normal = triangles[i].getNormal();
    //normal.normalize();
    //float intensity = normal.dot(lightDir);
    std::vector<Vector2f> fragments = triangles[i].getFragments();
    std::array<Vertex,3> vertices = triangles[i].getVertices();
    std::array<Vector3f,3> v;
    std::array<Vector2f,3> uv;
    std::array<Vector3f,3> n;
    v[0] = vertices[0].coords;
    v[1] = vertices[1].coords;
    v[2] = vertices[2].coords;
    uv[0] = vertices[0].uv;
    uv[1] = vertices[1].uv;
    uv[2] = vertices[2].uv;
    n[0] = vertices[0].normal;
    n[1] = vertices[1].normal;
    n[2] = vertices[2].normal;
    for (int j = 0; j < fragments.size(); j++)
    {
      Vector3f P(fragments[j](0), fragments[j](1), 0);
      Vector3f barycentricCoords_ = triangles[i].barycentricCoords(P);
      P(2) = v[0](2) * barycentricCoords_(0)
           + v[1](2) * barycentricCoords_(1)
           + v[2](2) * barycentricCoords_(2);
      if (P(2) > zbuffer.get(P(0),P(1)))
      {
        Vector3f normal;
        normal(0) = n[0](0) * barycentricCoords_(0) +
                    n[1](0) * barycentricCoords_(1) +
                    n[2](0) * barycentricCoords_(2);
        normal(1) = n[0](1) * barycentricCoords_(0) +
                    n[1](1) * barycentricCoords_(1) +
                    n[2](1) * barycentricCoords_(2);
        normal(2) = n[0](2) * barycentricCoords_(0) +
                    n[1](2) * barycentricCoords_(1) +
                    n[2](2) * barycentricCoords_(2);
        normal.normalize();
        float intensity = normal.dot(lightDir);
        if(intensity < 0.f)
          continue;
        //determine texture coordinates
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
#ifndef NDEBUG
  std::cout << "  Final # of triangles: " << triangles.size() << std::endl;
#endif
}
