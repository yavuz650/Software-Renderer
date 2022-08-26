#include "tgaimage.hpp"
#include "model.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include "SDL2/SDL.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
const TGAColor blue = TGAColor(0, 0, 255, 255);

#define WINDOW_HEIGHT 1024
#define WINDOW_WIDTH 1024
Vector3f lightDirection(0, 0, -1);

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
};

class zbuffer{
 private:
  int screenWidth;
  int screenHeight;
  std::vector<std::vector<float>> buffer;
 public:
  zbuffer(int screenWidth_, int screenHeight_)
      : screenWidth(screenWidth_),
        screenHeight(screenHeight_),
        buffer(screenWidth_, std::vector<float>(screenHeight_,
               -std::numeric_limits<float>::max())) {}

  void visualize(SDL_Renderer *renderer, int screenWidth, int screenHeight){
    for(int i=0; i<screenWidth; i++){
      for(int j=0; j<screenHeight; j++){
        SDL_SetRenderDrawColor(renderer, buffer[i][j]*100,
                                         buffer[i][j]*100,
                                         buffer[i][j]*100, 255);
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
  //z.normalize();
  Vector3f x = (up.cross(z)).normalized();
  //x.normalize();
  Vector3f y = (z.cross(x)).normalized();
  //y.normalize();

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

Matrix4f viewport(int width, int height){
  assert(width!=0 && height!=0);
  Matrix4f result = Matrix4f::Identity();
  result(0,0) = width/2;
  result(1,1) = height/2;
  result(0,3) = (width-1)/2;
  result(1,3) = (height-1)/2;
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

bool isInsideTriangle(Vector2i A, Vector2i B, Vector2i C, Vector2i P)
{
  Vector2i AB = B - A;
  Vector2i AC = C - A;
  Vector2i PA = A - P;

  Vector3f u = (Vector3f(AB(0), AC(0), PA(0))).cross(Vector3f(AB(1), AC(1), PA(1)));
  //u = Vector3f(u.x / u.z, u.y / u.z, u.z / u.z);

  return u(0) / u(2) >= 0. && u(1) / u(2) >= 0. && (u(0) / u(2) + u(1) / u(2)) <= 1.;
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

Vector3f barycentricCoords(Vector2i A, Vector2i B, Vector2i C, Vector2i P)
{
  Vector2i AB = B - A;
  Vector2i AC = C - A;
  Vector2i PA = A - P;

  Vector3f u = Vector3f(AB(0), AC(0), PA(0)).cross(Vector3f(AB(1), AC(1), PA(1)));
  u = Vector3f(u(0) / u(2), u(1) / u(2), u(2) / u(2));
  return Vector3f(1-u(0)-u(1), u(0), u(1));
}

// expects screen coordinates
// void triangle(std::array<Vector3f, 3> vertices, std::vector<std::vector<float>> &zbuffer, SDL_Renderer *renderer, TGAColor color, bool isWireframe)
// {
//   SDL_SetRenderDrawColor(renderer, color[2], color[1], color[0], color[3]);

//   if (isWireframe)
//   {
//     line(vertices[0].x, vertices[0].y,
//          vertices[1].x, vertices[1].y, renderer, color);

//     line(vertices[1].x, vertices[1].y,
//          vertices[2].x, vertices[2].y, renderer, color);

//     line(vertices[0].x, vertices[0].y,
//          vertices[2].x, vertices[2].y, renderer, color);
//   }
//   else
//   {
//     Vector2f bboxTopLeft(WINDOW_WIDTH - 1, WINDOW_HEIGHT - 1);
//     Vector2f bboxBottomRight(0, 0);

//     for (int i = 0; i < 3; i++)
//     {
//       bboxTopLeft.x = std::min(bboxTopLeft.x, vertices[i].x);
//       bboxTopLeft.y = std::min(bboxTopLeft.y, vertices[i].y);

//       bboxBottomRight.x = std::max(bboxBottomRight.x, vertices[i].x);
//       bboxBottomRight.y = std::max(bboxBottomRight.y, vertices[i].y);
//     }
//     Vector3f P;
//     Vector3f normal = (vertices[1] - vertices[0]) ^ (vertices[2] - vertices[1]);
//     normal.normalize();

//     for (P.x = bboxTopLeft.x; P.x <= bboxBottomRight.x; P.x++)
//     {
//       for (P.y = bboxTopLeft.y; P.y <= bboxBottomRight.y; P.y++)
//       {
//         if (isInsideTriangle(Vector2i(vertices[0].x,vertices[0].y),
//                              Vector2i(vertices[1].x,vertices[1].y),
//                              Vector2i(vertices[2].x,vertices[2].y),
//                              Vector2i(P.x, P.y)))
//         {
//           float A = normal.x * (vertices[0].x - P.x);
//           float B = normal.y * (vertices[0].y - P.y);
//           P.z = A + B / normal.z + vertices[0].z;
//           if (P.z > zbuffer[P.x][P.y])
//           {
//             SDL_RenderDrawPoint(renderer, P.x, WINDOW_HEIGHT-1-P.y);
//             zbuffer[P.x][P.y] = P.z;
//           }
//         }
//       }
//     }
//     // debug
//     //  line(bboxTopLeft.x,bboxTopLeft.y,
//     //      bboxBottomRight.x,bboxTopLeft.y, renderer, color);

//     // line(bboxTopLeft.x,bboxTopLeft.y,
//     //     bboxTopLeft.x,bboxBottomRight.y, renderer, color);

//     // line(bboxBottomRight.x,bboxTopLeft.y,
//     //     bboxBottomRight.x,bboxBottomRight.y, renderer, color);

//     // line(bboxTopLeft.x,bboxBottomRight.y,
//     //     bboxBottomRight.x,bboxBottomRight.y, renderer, color);
//   }
// }

void interpolatedTriangle(std::array<Vector3f, 3> vertices, SDL_Renderer *renderer,
                          std::array<Vector2f,3> uv, TGAImage &texture,
                          zbuffer &zBuffer, Matrix4f &projection,
                          Matrix4f &view)
{
  // Matrix4f projection = orthographic(-1,1,-1,1,1,-1);
  // Matrix4f view = lookAt(Vector3f(-1,1,-3),Vector3f(0,0,0),Vector3f(0,1,0));
  Matrix4f vp = viewport(WINDOW_HEIGHT, WINDOW_WIDTH);
  //Matrix4f model = translate(Matrix4f::identity(4),Vector3f(0,0,-3.0));
  Matrix4f M = projection*view;
  Matrix4f N = (M.inverse()).transpose();
  Vector3f normalVector = (vertices[2] - vertices[1]).cross((vertices[2] - vertices[0]));

  Vector4f temp;
  for (int i = 0; i < 3; i++)
  {
    temp = Vector4f(vertices[i](0), vertices[i](1), vertices[i](2), 1.0f);
    temp = vp*M*temp;
    vertices[i] = Vector3f(temp(0)/temp(3),temp(1)/temp(3),temp(2)/temp(3));
  }
  temp = N*Vector4f(normalVector(0),normalVector(1),normalVector(2),0.0);
  //normalVector = Vector3f(temp.x/temp.w,temp.y/temp.w,temp.z/temp.w);
  normalVector = Vector3f(temp(0),temp(1),temp(2));
  normalVector.normalize();
  float intensity = normalVector.dot(lightDirection);
  // if(intensity < 0)
  //   return;

  // vertices[0] = Vector3f(vp*Vec4f(vertices[0], 1.0f));
  // vertices[1] = Vector3f(vp*Vec4f(vertices[1], 1.0f));
  // vertices[2] = Vector3f(vp*Vec4f(vertices[2], 1.0f));

  // vertices[0] = worldToScreen(vertices[0]);
  // vertices[1] = worldToScreen(vertices[1]);
  // vertices[2] = worldToScreen(vertices[2]);
  boundingBox bbox(Vector2f(WINDOW_WIDTH - 1, WINDOW_HEIGHT - 1), Vector2f(0, 0));

  for (int i = 0; i < 3; i++)
  {
    bbox.TR()(0) = std::max(bbox.TR()(0), vertices[i](0));
    bbox.TR()(1) = std::max(bbox.TR()(1), vertices[i](1));

    bbox.BL()(0) = std::min(bbox.BL()(0), vertices[i](0));
    bbox.BL()(1) = std::min(bbox.BL()(1), vertices[i](1));
  }

  Vector3f P;
  Vector3f normal = (vertices[1] - vertices[0]).cross((vertices[2] - vertices[1]));
  normal.normalize();

  for (P(0) = bbox.BL()(0); P(0) <= bbox.TR()(0); P(0)++)
  {
    for (P(1) = bbox.BL()(1); P(1) <= bbox.TR()(1); P(1)++)
    {
      if(!(P(0) >=0 && P(0) <WINDOW_WIDTH && P(1) >= 0 && P(1) <WINDOW_HEIGHT))
        return;
      if (isInsideTriangle(Vector2i(vertices[0](0),vertices[0](1)),
                            Vector2i(vertices[1](0),vertices[1](1)),
                            Vector2i(vertices[2](0),vertices[2](1)),
                            Vector2i(P(0), P(1))))
      {
        float A = normal(0) * (vertices[0](0) - P(0));
        float B = normal(1) * (vertices[0](1) - P(1));
        P(2) = A + B / normal(2) + vertices[0](2);
        if (P(2) > zBuffer.get(P(0),P(1)))
        {
          //determine texture coordinates
          Vector3f barycentricCoords_ = barycentricCoords(Vector2i(vertices[0](0),vertices[0](1)), Vector2i(vertices[1](0),vertices[1](1)), Vector2i(vertices[2](0),vertices[2](1)), Vector2i(P(0),P(1)));
          Vector2f interpolatedUv;
          for (int i = 0; i < 3; i++)
          {
            interpolatedUv(0) = uv[0](0) * barycentricCoords_(0)
                             + uv[1](0) * barycentricCoords_(1)
                             + uv[2](0) * barycentricCoords_(2);

            interpolatedUv(1) = uv[0](1) * barycentricCoords_(0)
                             + uv[1](1) * barycentricCoords_(1)
                             + uv[2](1) * barycentricCoords_(2);
          }

          TGAColor color = texture.get(interpolatedUv(0) * texture.width(),
                                       interpolatedUv(1) * texture.height());
          SDL_SetRenderDrawColor(renderer, color[2]*1,
                                           color[1]*1,
                                           color[0]*1, 255);
          SDL_RenderDrawPoint(renderer, P(0), WINDOW_HEIGHT-P(1)-1);
          zBuffer.set(P(0),P(1),P(2));
        }
      }
    }
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

  zbuffer zBuffer(WINDOW_WIDTH,WINDOW_HEIGHT);

  TGAImage texture;
  texture.read_tga_file("obj/african_head_diffuse.tga");
  texture.flip_vertically();

  std::array<Vector2f,3> uv;
  float x,y;
  int angle;
  float r = 4.0;
  angle = 90;
  int delta = 2;
  while(1){
    x = cos(angle * M_PI / 180);
    y = sin(angle * M_PI / 180);
    angle+=delta%360;
    Matrix4f projection = perspective(-1,1,-1,1,-2,-3);
    Matrix4f view = lookAt(Vector3f(3*x,0,3*y),Vector3f(0,0,0),Vector3f(0,1,0));
    SDL_SetRenderDrawColor(renderer,0,0,0,255);
    SDL_RenderClear(renderer);
    zBuffer.resetBuffer();
    for (int i = 0; i < model->nfaces(); i++)
    {
      std::vector<int> face = model->face(i);
      std::vector<int> textureIdx = model->textureIdx(i);
      std::array<Vector2f,3> uv;
      for (int j = 0; j < 3; j++)
      {
        uv[j] = model->uv(textureIdx[j]);
      }

      interpolatedTriangle(std::array<Vector3f, 3>{model->vert(face[0]),
                                    model->vert(face[1]),
                                    model->vert(face[2])},
                                    renderer, uv, texture,
                                    zBuffer,projection,view);
    }
      SDL_RenderPresent(renderer);
      if (SDL_PollEvent(&event)){
        if(event.type == SDL_QUIT || (event.type==SDL_WINDOWEVENT && event.window.event==SDL_WINDOWEVENT_CLOSE))
          break;
      }
  }

  // for (int i = 0; i < model->nfaces(); i++)
  // {
  //   std::vector<int> face = model->face(i);
  //   std::vector<int> textureIdx = model->textureIdx(i);
  //   std::array<Vector2f,3> uv;
  //   for (int j = 0; j < 3; j++)
  //   {
  //     uv[j] = model->uv(textureIdx[j]);
  //   }

  //   interpolatedTriangle(std::array<Vector3f, 3>{model->vert(face[0]),
  //                                 model->vert(face[1]),
  //                                 model->vert(face[2])},
  //                                 renderer, uv, texture,
  //                                 zbuffer);
  // }

  //SDL_RenderPresent(renderer);
  std::cout << "Finished rendering\n";
  // while (1)
  // {
  //   if (SDL_PollEvent(&event) && event.type == SDL_QUIT)
  //     break;
  // }
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  delete model;
  return 0;
}
