#include "tgaimage.hpp"
#include "model.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cassert>
#include "SDL2/SDL.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
const TGAColor blue = TGAColor(0, 0, 255, 255);

#define WINDOW_HEIGHT 1024
#define WINDOW_WIDTH 1024
Vector3f lightDirection(0, 0, -1);

#ifndef NDEBUG
  bool isOrthogonal(Vector3f A, Vector3f B){
    return fabs(A.dot(B)) < 0.00001;
  }
#endif

class boundingBox{
 private:
  Vector2f bottomLeft;
  Vector2f topRight;

 public:
  boundingBox(Vector2f bottomLeft_, Vector2f topRight_)
      : bottomLeft(bottomLeft_), topRight(topRight_) {}
  Vector2f &BL(){
    return bottomLeft;
  }
  Vector2f &TR(){
    return topRight;
  }
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

class ZBuffer{
 private:
  int screenWidth;
  int screenHeight;
  std::vector<std::vector<float>> buffer;
 public:
  ZBuffer(int screenWidth_, int screenHeight_)
      : screenWidth(screenWidth_),
        screenHeight(screenHeight_),
        buffer(screenWidth_, std::vector<float>(screenHeight_,
               -std::numeric_limits<float>::max())) {}

  void visualize(SDL_Renderer *renderer, int screenWidth, int screenHeight){
    for(int i=0; i<screenWidth; i++){
      for(int j=0; j<screenHeight; j++){
        SDL_SetRenderDrawColor(renderer, buffer[i][j]*10,
                                         buffer[i][j]*10,
                                         buffer[i][j]*10, 255);
        SDL_RenderDrawPoint(renderer, i, screenHeight-j);
      }
    }
  }
  float get(int x, int y) const{
    return buffer[x][y];
  }
  void set(int x, int y, float f){
    buffer[x][y] = f;
  }
  void resetBuffer(){
    for (int i = 0; i < screenWidth; i++)
    {
      std::fill(buffer[i].begin(), buffer[i].end(), -std::numeric_limits<float>::max());
    }
  }
  void printBuffer(){
    for(int i=0; i<screenWidth; i++){
      for(int j=0; j<screenHeight; j++){
        std::cout << buffer[i][j] << "\n";
      }
    }
  }
};

Matrix4f viewport(int width, int height){
  assert(width!=0 && height!=0);
  Matrix4f result = Matrix4f::Identity();
  result(0,0) = width/2;
  result(1,1) = height/2;
  result(0,3) = (width-1)/2;
  result(1,3) = (height-1)/2;
  return result;
}

Vector3f barycentricCoords(Vector2i A, Vector2i B, Vector2i C, Vector2i P)
{
  Vector2i AB = B - A;
  Vector2i AC = C - A;
  Vector2i PA = A - P;

  Vector3f u = Vector3f(AB(0), AC(0), PA(0)).cross(Vector3f(AB(1), AC(1), PA(1)));
  u = Vector3f(u(0) / u(2), u(1) / u(2), u(2) / u(2));
  return Vector3f(1-u(0)-u(1), u(0), u(1));
}


class triangle{
 private:
  //Vertices
  std::array<Vector3f,3> v;
  //Texture coordinates
  std::array<Vector2f,3> uv;
  //Surface normal
  Vector3f normal;
  std::vector<Vector2f> fragments;
  Vector3f calculateNormal(){
    return (v[2]-v[0]).cross((v[1]-v[0]));
  } 
 public:
  Vector3f calculateNormal2(){
    return (v[1]-v[0]).cross((v[2]-v[1]));
  } 
  triangle() {}
  triangle(Vector3f v0, Vector3f v1, Vector3f v2) {
    v[0] = v0;
    v[1] = v1;
    v[2] = v2;
    normal = calculateNormal();     
  }
  triangle(Vector3f v0, Vector3f v1, Vector3f v2,
           Vector2f uv0, Vector2f uv1, Vector2f uv2) {
    v[0] = v0;
    v[1] = v1;
    v[2] = v2;
    uv[0] = uv0;
    uv[1] = uv1;
    uv[2] = uv2;
    normal = calculateNormal();      
  }

  triangle(std::array<Vector3f, 3> v_, std::array<Vector2f, 3> uv_) {
    v[0] = v_[0];
    v[1] = v_[1];
    v[2] = v_[2];
    uv[0] = uv_[0];
    uv[1] = uv_[1];
    uv[2] = uv_[2];
    normal = calculateNormal();      
  }

  std::array<Vector3f,3> getVertices(){
    return v;
  }

  std::array<Vector2f,3> getUv(){
    return uv;
  }

  void createFragment(Vector2f coords){
    fragments.push_back(coords);
  }

  std::vector<Vector2f>& getFragments(){
    return fragments;
  }

  void transform(Matrix4f M, Matrix4f N, Matrix4f viewport = Matrix4f::Identity()){ 
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

  bool isInsideTriangle(Vector2i P)
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

  Vector3f getNormal(){
    return normal;
  }

  void draw(SDL_Renderer *renderer,
            std::array<Vector2f,3> uv, TGAImage &texture,
            ZBuffer &zBuffer, Matrix4f &projection,
            Matrix4f &view){

    // Matrix4f projection = orthographic(-1,1,-1,1,1,-1);
    // Matrix4f view = lookAt(Vector3f(-1,1,-3),Vector3f(0,0,0),Vector3f(0,1,0));
    Matrix4f vp = viewport(WINDOW_HEIGHT, WINDOW_WIDTH);
    //Matrix4f model = translate(Matrix4f::identity(4),Vector3f(0,0,-3.0));
    Matrix4f M = projection*view;
    Matrix4f N = (M.inverse()).transpose();
    Vector3f normalVector = (v[2]-v[1]).cross((v[2]-v[0]));

    Vector4f temp;
    for (int i = 0; i < 3; i++)
    {
      //printf("Before %f %f %f\n",v[i](0),v[i](1),v[i](2));
      temp = Vector4f(v[i](0), v[i](1), v[i](2), 1.0f);
      temp = vp*M*temp;
      v[i] = Vector3f(temp(0)/temp(3),temp(1)/temp(3),temp(2)/temp(3));
      //printf("After %f %f %f\n",v[i](0),v[i](1),v[i](2));
    }
    temp = N*Vector4f(normalVector(0),normalVector(1),normalVector(2),0.0);
    //normalVector = Vector3f(temp.x/temp.w,temp.y/temp.w,temp.z/temp.w);
    normalVector = Vector3f(temp(0),temp(1),temp(2));
    normalVector.normalize();
    float intensity = normalVector.dot(lightDirection);
    if(intensity < 0)
      return;

    boundingBox bbox(Vector2f(WINDOW_WIDTH - 1, WINDOW_HEIGHT - 1), Vector2f(0, 0));

    for (int i = 0; i < 3; i++)
    {
      bbox.TR()(0) = std::max(bbox.TR()(0), v[i](0));
      bbox.TR()(1) = std::max(bbox.TR()(1), v[i](1));

      bbox.BL()(0) = std::min(bbox.BL()(0), v[i](0));
      bbox.BL()(1) = std::min(bbox.BL()(1), v[i](1));
    }

    Vector3f P;
    Vector3f normal = (v[1] - v[0]).cross((v[2] - v[1]));
    normal.normalize();

    for (P(0) = bbox.BL()(0); P(0) <= bbox.TR()(0); P(0)++)
    {
      for (P(1) = bbox.BL()(1); P(1) <= bbox.TR()(1); P(1)++)
      {
        if(!(P(0) >=0 && P(0) <WINDOW_WIDTH && P(1) >= 0 && P(1) <WINDOW_HEIGHT))
          continue;
        if (isInsideTriangle(Vector2i(P(0), P(1))))
        {
          float A = normal(0) * (v[0](0) - P(0));
          float B = normal(1) * (v[0](1) - P(1));
          P(2) = A + B / normal(2) + v[0](2);
          if (P(2) > zBuffer.get(P(0),P(1)))
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
            SDL_SetRenderDrawColor(renderer, color[2]*intensity,
                                            color[1]*intensity,
                                            color[0]*intensity, 255);
            SDL_RenderDrawPoint(renderer, P(0), WINDOW_HEIGHT-P(1)-1);
            zBuffer.set(P(0),P(1),P(2));
            //std::cout << P(2) << "\n";
          }
        }
      }
    }
  }
};

class VertexShader{
 private:
  Vector3f lookDir;
  bool isBackface(triangle t){
    Vector3f normalVector = t.getNormal();
    if(normalVector.dot(lookDir) < 0)
      return true;
    return false;
  }

 public:
  VertexShader() : lookDir(Vector3f(0,0,-1.0)) {}

  void shade(std::vector<triangle> &triangles,
             Matrix4f MVP,
             Matrix4f viewport,
             bool doBackfaceCulling = 1){
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
};

class Rasterizer{
 public:
  void rasterize(std::vector<triangle> &triangles){
    for (int i = 0; i < triangles.size(); i++){
      std::array<Vector3f,3> v = triangles[i].getVertices();
      boundingBox bbox(Vector2f(WINDOW_WIDTH - 1, WINDOW_HEIGHT - 1), Vector2f(0, 0));
      for (int i = 0; i < 3; i++)
      {
        bbox.TR()(0) = std::max(bbox.TR()(0), v[i](0));
        bbox.TR()(1) = std::max(bbox.TR()(1), v[i](1));

        bbox.BL()(0) = std::min(bbox.BL()(0), v[i](0));
        bbox.BL()(1) = std::min(bbox.BL()(1), v[i](1));
      }

      Vector3f P;
      for (P(0) = bbox.BL()(0); P(0) <= bbox.TR()(0); P(0)++){
        for (P(1) = bbox.BL()(1); P(1) <= bbox.TR()(1); P(1)++){
          if(!(P(0) >=0 && P(0) <WINDOW_WIDTH && P(1) >= 0 && P(1) <WINDOW_HEIGHT))
            continue;
          if (triangles[i].isInsideTriangle(Vector2i(P(0), P(1)))){
            triangles[i].createFragment(Vector2f(P(0), P(1)));
          }
        }
      }
    }
  }
};

class FragmentShader{
 public:
  void shade(std::vector<triangle> &triangles, ZBuffer &zbuffer, TGAImage &texture,
             SDL_Renderer *renderer, Vector3f lightDir){
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
};

// template <class T>
// Vec2<T> worldToScreen(Vec2<T> v)
// {
//   return Vec2<T>((v.x + 1.) * WINDOW_WIDTH/2, (v.y + 1.) * WINDOW_HEIGHT/2);
// }
// template <class T>
// Vec3<T> worldToScreen(Vec3<T> v)
// {
//   return Vec3<T>((v.x + 1.) * WINDOW_WIDTH/2, (v.y + 1.) * WINDOW_HEIGHT/2, v.z);
// }

//Returns the view Matrix4f
Matrix4f lookAt(Vector3f eye, Vector3f target, Vector3f up){
  Vector3f z = (eye-target).normalized();
  Vector3f x = (up.cross(z)).normalized();
  Vector3f y = (z.cross(x)).normalized();

  Matrix4f view = Matrix4f::Identity();
  view(0,0) = x[0];
  view(0,1) = x[1];
  view(0,2) = x[2];
  view(1,0) = y[0];
  view(1,1) = y[1];
  view(1,2) = y[2];
  view(2,0) = z[0];
  view(2,1) = z[1];
  view(2,2) = z[2];
  view(0,3) = -(x.dot(eye));
  view(1,3) = -(y.dot(eye));
  view(2,3) = -(z.dot(eye));
  return view;
}

Matrix4f orthographic(float l, float r, float b, float t, float n, float f){
  //make sure the box is valid
  assert(r!=l && t!=b && n!=f);
  Matrix4f result = Matrix4f::Identity();
  result(0,0) = 2/(r-l);
  result(1,1) = 2/(t-b);
  result(2,2) = 2/(n-f);
  result(0,3) = -(r+l)/(r-l);
  result(1,3) = -(t+b)/(t-b);
  result(2,3) = -(n+f)/(n-f);
  return result;
}

Matrix4f perspective(float l, float r, float b, float t, float n, float f){
  //make sure the box is valid
  assert(r!=l && t!=b && n!=f);
  Matrix4f result = Matrix4f::Identity();
  result(0,0) = n;
  result(1,1) = n;
  result(2,2) = n+f;
  result(2,3) = -f*n;
  result(3,2) = 1;
  result(3,3) = 0;
  result = orthographic(l,r,b,t,n,f)*result;
  return result;
}

Matrix4f translate(Matrix4f const &m, Vector3f v)
{
  //only operate on 4x4 matrices
  //assert(m.ncols() == 4 && m.nrows() == 4);

  Matrix4f Result(m);
  Result(0,3) = m(0,0) * v(0) + m(0,1) * v(1) + m(0,2) * v(2) + m(0,3);
  Result(1,3) = m(1,0) * v(0) + m(1,1) * v(1) + m(1,2) * v(2) + m(1,3);
  Result(2,3) = m(2,0) * v(0) + m(2,1) * v(1) + m(2,2) * v(2) + m(2,3);
  Result(3,3) = m(3,3);
  return Result;
}

void line(int x0, int y0, int x1, int y1, SDL_Renderer *renderer, TGAColor color)
{
  bool steep = false;
  if (std::abs(x0 - x1) < std::abs(y0 - y1))
  {
    std::swap(x0, y0);
    std::swap(x1, y1);
    steep = true;
  }
  if (x0 > x1)
  {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }
  int dx = x1 - x0;
  int dy = y1 - y0;
  int derror2 = std::abs(dy) * 2;
  int error2 = 0;
  int y = y0;
  for (int x = x0; x <= x1; x++)
  {
    if (steep)
    {
      SDL_RenderDrawPoint(renderer, y, x);
    }
    else
    {
      SDL_RenderDrawPoint(renderer, x, y);
    }
    error2 += derror2;
    if (error2 > dx)
    {
      y += (y1 > y0 ? 1 : -1);
      error2 -= dx * 2;
    }
  }
}

int main(int argc, char **argv)
{
  Model *model = new Model("/home/yavuz/Desktop/Software-Renderer/obj/african_head.obj");
  SDL_Event event;
  SDL_Renderer *renderer;
  SDL_Window *window;
  SDL_Init(SDL_INIT_VIDEO);
  SDL_CreateWindowAndRenderer(WINDOW_WIDTH, WINDOW_HEIGHT, 0, &window, &renderer);
  SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
  SDL_RenderClear(renderer);
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

  ZBuffer zBuffer(WINDOW_WIDTH,WINDOW_HEIGHT);
  VertexShader vertShader;
  Rasterizer rasterizer;
  FragmentShader fragShader;
  TGAImage texture;
  texture.read_tga_file("/home/yavuz/Desktop/Software-Renderer/obj/african_head_diffuse.tga");
  texture.flip_vertically();
  std::vector<triangle> rawTriangles;
  for (int i = 0; i < model->nfaces(); i++)
  {
    std::vector<int> face = model->face(i);
    std::vector<int> textureIdx = model->textureIdx(i);
    rawTriangles.emplace_back(triangle(model->vert(face[0]),
                                       model->vert(face[1]),
                                       model->vert(face[2]),
                                       model->uv(textureIdx[0]),
                                       model->uv(textureIdx[1]),
                                       model->uv(textureIdx[2])));
  }

  float x,y;
  int angle;
  float r = 4.0;
  angle = 90;
  int delta = 2;
  while(1){
    x = cos(angle * M_PI / 180);
    y = sin(angle * M_PI / 180);
    angle+=delta%360;
    Matrix4f projection = orthographic(-1,1,-1,1,1,-1);
    Matrix4f view = lookAt(Vector3f(3*x,0,3*y),Vector3f(0,0,0),Vector3f(0,1,0));
    Matrix4f vp = viewport(WINDOW_HEIGHT, WINDOW_WIDTH);
    Matrix4f M = projection*view;
    std::vector<triangle> triangles(rawTriangles);
    SDL_SetRenderDrawColor(renderer,0,0,0,255);
    SDL_RenderClear(renderer);
    zBuffer.resetBuffer();
    vertShader.shade(triangles,M,vp);
    rasterizer.rasterize(triangles);
    fragShader.shade(triangles,zBuffer,texture,renderer,Vector3f(0,0,-1.0));
    //zBuffer.visualize(renderer,WINDOW_WIDTH,WINDOW_HEIGHT);
    // zBuffer.printBuffer();
    SDL_RenderPresent(renderer);
    if (SDL_PollEvent(&event)){
      if(event.type == SDL_QUIT || (event.type==SDL_WINDOWEVENT && event.window.
      event==SDL_WINDOWEVENT_CLOSE))
        break;
    }
  }

  std::cout << "Finished rendering\n";
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  delete model;
  return 0;
}
