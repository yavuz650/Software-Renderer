#include "shader.hpp"
#define WINDOW_HEIGHT 1024
#define WINDOW_WIDTH 1024

void HeadShader::vertexShader(std::vector<triangle> &input){
#ifndef NDEBUG
  std::cout << "Vertex Shader...\n"
            << "  Initial # of triangles: " << input.size() << std::endl;
#endif
  //if(doBackfaceCulling){
    for (auto it=input.begin(); it!=input.end();)
    {
      Vector4f temp;
      Matrix4f MVP = projection*view*model;
      Matrix4f MV = view*model;
      std::array<Vertex,3> &v = it->getVertices();
      for (int i = 0; i < 3; i++)
      {
        Vector4f &coords = v[i].coords;
        Vector3f &normal = v[i].vec3f[0];
        Vector3f fragPos;
        //Vertex vert(v[i]);
        coords = view*model*coords;
        fragPos = Vector3f(coords(0) / coords(3), coords(1) / coords(3),
                           coords(2) / coords(3));
        coords = projection * coords;
        coords = viewport * coords;
        coords = Vector4f(coords(0) / coords(3), coords(1) / coords(3),
                          coords(2) / coords(3), 1 / coords(3));
        
        //Transform the normal vector
        temp = Vector4f(normal(0),normal(1),normal(2),0);
        temp = MV.inverse().transpose()*temp;
        normal = -Vector3f(temp(0),temp(1),temp(2));
        v[i].vec3f.push_back(fragPos);
      }
      // //Transform the surface normal vector
      // temp = Vector4f(surfaceNormal(0),surfaceNormal(1),surfaceNormal(2),0);
      // temp = MVP.inverse().transpose()*temp;
      // surfaceNormal = Vector3f(temp(0),temp(1),temp(2));
      if(it->isBackface())
        it=input.erase(it);
      else
        it++;
    }
  //}
  //else{
    //to be implemented, not really needed though
  //}
#ifndef NDEBUG
  std::cout << "  Final # of triangles: " << input.size() << std::endl;
#endif
}

void HeadShader::fragmentShader(std::vector<triangle> &input, ZBuffer zbuffer,
                                SDL_Renderer *renderer) {
#ifndef NDEBUG
  std::cout << "Fragment Shader...\n"
            << "  Initial # of triangles: " << input.size() << std::endl;
#endif
  for (int i = 0; i < input.size(); i++)
  {
    //Vector3f normal = triangles[i].getNormal();
    //normal.normalize();
    //float intensity = normal.dot(lightDir);
    std::vector<Vector2f> fragments = input[i].getFragments();
    std::array<Vertex,3> vertices = input[i].getVertices();
    std::array<Vector4f,3> v;
    std::array<Vector2f,3> uv;
    std::array<Vector3f,3> n;
    std::array<Vector3f,3> fragPoses;
    v[0] = vertices[0].coords;
    v[1] = vertices[1].coords;
    v[2] = vertices[2].coords;
    uv[0] = vertices[0].vec2f[0];
    uv[1] = vertices[1].vec2f[0];
    uv[2] = vertices[2].vec2f[0];
    n[0] = vertices[0].vec3f[0];
    n[1] = vertices[1].vec3f[0];
    n[2] = vertices[2].vec3f[0];
    fragPoses[0] = vertices[0].vec3f[1];
    fragPoses[1] = vertices[1].vec3f[1];
    fragPoses[2] = vertices[2].vec3f[1];
    for (int j = 0; j < fragments.size(); j++)
    {
      Vector3f P(fragments[j](0), fragments[j](1), 0);
      Vector3f barycentricCoords_ = input[i].barycentricCoords(P);
      //Perspective correct barycentric coordinates
      float denom = barycentricCoords_(0) * v[0](3) +
                    barycentricCoords_(1) * v[1](3) +
                    barycentricCoords_(2) * v[2](3);
      barycentricCoords_(0) = barycentricCoords_(0)*v[0](3)/denom;
      barycentricCoords_(1) = barycentricCoords_(1)*v[1](3)/denom;
      barycentricCoords_(2) = barycentricCoords_(2)*v[2](3)/denom;
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
        Vector3f fragPos;
        fragPos(0) = fragPoses[0](0) * barycentricCoords_(0) +
                    fragPoses[1](0) * barycentricCoords_(1) +
                    fragPoses[2](0) * barycentricCoords_(2);
        fragPos(1) = fragPoses[0](1) * barycentricCoords_(0) +
                    fragPoses[1](1) * barycentricCoords_(1) +
                    fragPoses[2](1) * barycentricCoords_(2);
        fragPos(2) = fragPoses[0](2) * barycentricCoords_(0) +
                    fragPoses[1](2) * barycentricCoords_(1) +
                    fragPoses[2](2) * barycentricCoords_(2);
        float intensity = std::max<float>(normal.dot(lightDir),0.f);
        //determine texture coordinates
        Vector2f interpolatedUv;
        interpolatedUv(0) = uv[0](0) * barycentricCoords_(0)
                        + uv[1](0) * barycentricCoords_(1)
                        + uv[2](0) * barycentricCoords_(2);

        interpolatedUv(1) = uv[0](1) * barycentricCoords_(0)
                        + uv[1](1) * barycentricCoords_(1)
                        + uv[2](1) * barycentricCoords_(2);
        float exp = specularMap.get(interpolatedUv(0) * specularMap.width(),
                                    interpolatedUv(1) * specularMap.height())[0] / 1.f;
        Vector3f viewVec = -fragPos;
        Vector3f halfVec = (lightDir+fragPos).normalized();
        float specBase = std::max<float>(normal.dot(halfVec),0);
        float specular = pow(specBase,exp);
        TGAColor color = diffuseMap.get(interpolatedUv(0) * diffuseMap.width(),
                                    interpolatedUv(1) * diffuseMap.height());

        SDL_SetRenderDrawColor(
            renderer,
            std::min<float>((intensity + 0.6 * specular) * color[2], 255),
            std::min<float>((intensity + 0.6 * specular) * color[1], 255),
            std::min<float>((intensity + 0.6 * specular) * color[0], 255), 255);
        SDL_RenderDrawPoint(renderer, P(0), WINDOW_HEIGHT-P(1)-1);
        zbuffer.set(P(0),P(1),P(2));
      }
    }
  }
#ifndef NDEBUG
  std::cout << "  Final # of triangles: " << input.size() << std::endl;
#endif
}
