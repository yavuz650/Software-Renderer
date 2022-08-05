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

bool isInsideTriangle(Vec2i A, Vec2i B, Vec2i C, Vec2i P)
{
  Vec2i AB = B - A;
  Vec2i AC = C - A;
  Vec2i PA = A - P;

  Vec3f u = Vec3f(AB.x, AC.x, PA.x) ^ Vec3f(AB.y, AC.y, PA.y);
  //u = Vec3f(u.x / u.z, u.y / u.z, u.z / u.z);

  return u.x / u.z >= 0. && u.y / u.z >= 0. && (u.x / u.z + u.y / u.z) <= 1.;
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

Vec3f barycentricCoords(Vec2i A, Vec2i B, Vec2i C, Vec2i P)
{
  Vec2i AB = B - A;
  Vec2i AC = C - A;
  Vec2i PA = A - P;

  Vec3f u = Vec3f(AB.x, AC.x, PA.x) ^ Vec3f(AB.y, AC.y, PA.y);
  u = Vec3f(u.x / u.z, u.y / u.z, u.z / u.z);
  return Vec3f(1-u.x-u.y, u.x, u.y);
}

// expects screen coordinates
void triangle(std::array<Vec3f, 3> vertices, std::vector<std::vector<float>> &zbuffer, SDL_Renderer *renderer, TGAColor color, bool isWireframe)
{
  SDL_SetRenderDrawColor(renderer, color[2], color[1], color[0], color[3]);

  if (isWireframe)
  {
    line(vertices[0].x, vertices[0].y,
         vertices[1].x, vertices[1].y, renderer, color);

    line(vertices[1].x, vertices[1].y,
         vertices[2].x, vertices[2].y, renderer, color);

    line(vertices[0].x, vertices[0].y,
         vertices[2].x, vertices[2].y, renderer, color);
  }
  else
  {
    Vec2f bboxTopLeft(WINDOW_WIDTH - 1, WINDOW_HEIGHT - 1);
    Vec2f bboxBottomRight(0, 0);

    for (int i = 0; i < 3; i++)
    {
      bboxTopLeft.x = std::min(bboxTopLeft.x, vertices[i].x);
      bboxTopLeft.y = std::min(bboxTopLeft.y, vertices[i].y);

      bboxBottomRight.x = std::max(bboxBottomRight.x, vertices[i].x);
      bboxBottomRight.y = std::max(bboxBottomRight.y, vertices[i].y);
    }
    Vec3f P;
    Vec3f normal = (vertices[0] - vertices[1]) ^ (vertices[2] - vertices[1]);
    normal.normalize();

    for (P.x = bboxTopLeft.x; P.x <= bboxBottomRight.x; P.x++)
    {
      for (P.y = bboxTopLeft.y; P.y <= bboxBottomRight.y; P.y++)
      {
        if (isInsideTriangle(Vec2i(vertices[0].x,vertices[0].y),
                             Vec2i(vertices[1].x,vertices[1].y),
                             Vec2i(vertices[2].x,vertices[2].y),
                             Vec2i(P.x, P.y)))
        {
          float A = normal.x * (vertices[0].x - P.x);
          float B = normal.y * (vertices[0].y - P.y);
          P.z = A + B / normal.z + vertices[0].z;
          if (P.z > zbuffer[P.x][P.y])
          {
            SDL_RenderDrawPoint(renderer, P.x, P.y);
            zbuffer[P.x][P.y] = P.z;
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
}

void interpolatedTriangle(std::array<Vec3f, 3> vertices, SDL_Renderer *renderer,
                          std::array<Vec2f,3> uv, TGAImage &texture, float intensity,
                          std::vector<std::vector<float>> &zbuffer)
{
  //SDL_SetRenderDrawColor(renderer, color[2], color[1], color[0], color[3]);

  Vec2f bboxTopLeft(WINDOW_WIDTH - 1, WINDOW_HEIGHT - 1);
  Vec2f bboxBottomRight(0, 0);

  for (int i = 0; i < 3; i++)
  {
    bboxTopLeft.x = std::min(bboxTopLeft.x, vertices[i].x);
    bboxTopLeft.y = std::min(bboxTopLeft.y, vertices[i].y);

    bboxBottomRight.x = std::max(bboxBottomRight.x, vertices[i].x);
    bboxBottomRight.y = std::max(bboxBottomRight.y, vertices[i].y);
  }
  Vec3f P;
  Vec3f normal = (vertices[0] - vertices[1]) ^ (vertices[2] - vertices[1]);
  normal.normalize();

  for (P.x = bboxTopLeft.x; P.x <= bboxBottomRight.x; P.x++)
  {
    for (P.y = bboxTopLeft.y; P.y <= bboxBottomRight.y; P.y++)
    {
      if (isInsideTriangle(Vec2i(vertices[0].x,vertices[0].y),
                            Vec2i(vertices[1].x,vertices[1].y),
                            Vec2i(vertices[2].x,vertices[2].y),
                            Vec2i(P.x, P.y)))
      {
        float A = normal.x * (vertices[0].x - P.x);
        float B = normal.y * (vertices[0].y - P.y);
        P.z = A + B / normal.z + vertices[0].z;
        if (P.z > zbuffer[P.x][P.y])
        {
          //determine texture coordinates
          Vec3f barycentricCoords_ = barycentricCoords(Vec2i(vertices[0].x,vertices[0].y), Vec2i(vertices[1].x,vertices[1].y), Vec2i(vertices[2].x,vertices[2].y), Vec2i(P.x,P.y));
          Vec2f interpolatedUv;
          for (int i = 0; i < 3; i++)
          {
            interpolatedUv.x = uv[0].x * barycentricCoords_.x
                             + uv[1].x * barycentricCoords_.y
                             + uv[2].x * barycentricCoords_.z;

            interpolatedUv.y = uv[0].y * barycentricCoords_.x
                             + uv[1].y * barycentricCoords_.y
                             + uv[2].y * barycentricCoords_.z;
          }

          TGAColor color = texture.get(interpolatedUv.x * texture.width(),
                                       interpolatedUv.y * texture.height());
          SDL_SetRenderDrawColor(renderer, intensity*color[2], 
                                           intensity*color[1], 
                                           intensity*color[0], 255);
          SDL_RenderDrawPoint(renderer, P.x, P.y);
          zbuffer[P.x][P.y] = P.z;
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

template <class T>
Vec2<T> worldToScreen(Vec2<T> v)
{
  return Vec2<T>((v.x + 1.) * WINDOW_WIDTH/2, (v.y + 1.) * WINDOW_HEIGHT/2);
}
template <class T>
Vec3<T> worldToScreen(Vec3<T> v)
{
  return Vec3<T>((v.x + 1.) * WINDOW_WIDTH/2, (v.y + 1.) * WINDOW_HEIGHT/2, v.z);
}

int main(int argc, char **argv)
{
  Model *model = new Model("obj/african_head.obj", 1);

  SDL_Event event;
  SDL_Renderer *renderer;
  SDL_Window *window;
  SDL_Init(SDL_INIT_VIDEO);
  SDL_CreateWindowAndRenderer(WINDOW_WIDTH, WINDOW_HEIGHT, 0, &window, &renderer);
  SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
  SDL_RenderClear(renderer);
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);

  Vec3f lightDirection(0, 0, -1);

  std::vector<std::vector<float>> zbuffer(WINDOW_WIDTH);
  for (int i = 0; i < WINDOW_WIDTH; i++)
  {
    zbuffer[i].resize(WINDOW_HEIGHT, -std::numeric_limits<float>::max());
  }

  TGAImage texture;
  texture.read_tga_file("obj/african_head_diffuse.tga");
  texture.flip_vertically();

  for (int i = 0; i < model->nfaces(); i++)
  {
    std::vector<int> face = model->face(i);
    std::vector<int> textureIdx = model->textureIdx(i);
    Vec3f normalVector = (model->vert(face[1]) - model->vert(face[0]))
                       ^ (model->vert(face[2]) - model->vert(face[0]));
    normalVector.normalize();
    float intensity = normalVector * lightDirection;
    if (intensity > 0)
    {
      std::array<Vec2f,3> uv;
      //std::array<TGAColor,3> colors;
      for (int j = 0; j < 3; j++)
      {
        uv[j] = model->uv(textureIdx[j]);
        // uv[j].x *= texture.width();
        // uv[j].y *= texture.height();
        // //printf("%f,%f\n", uv[j].x, uv[j].y);
        // colors[j] = texture.get(std::lround(uv[j].x), std::lround(uv[j].y));
        // colors[j] = colors[j]*intensity;
      }

      interpolatedTriangle(std::array<Vec3f, 3>{worldToScreen(model->vert(face[0])),
                                    worldToScreen(model->vert(face[1])),
                                    worldToScreen(model->vert(face[2]))},
                                    renderer, uv, texture, intensity,
                                    zbuffer);
    }
  }

  // interpolatedTriangle(std::array<Vec3f, 3>{worldToScreen(Vec3f(-0.9,0.9,5)),
  //                                           worldToScreen(Vec3f(0.4,0.2,5)),
  //                                           worldToScreen(Vec3f(0,-0.9,5))},
  //                                           renderer,
  //                      std::array<TGAColor, 3>{red,green,blue},zbuffer);

  // interpolatedTriangle(std::array<Vec3f, 3>{worldToScreen(Vec3f(0.9,0.9,0)),
  //                                           worldToScreen(Vec3f(0.1,0.2,0.1)),
  //                                           worldToScreen(Vec3f(0.4,-0.9,0.1))},
  //                                           renderer,
  //                      std::array<TGAColor, 3>{red,green,blue},zbuffer);

  //Vec3f dummy = barycentricCoords(Vec2i(1,2), Vec2i(3,6), Vec2i(6,2), Vec2i(2,3));
  //std::cout << dummy << std::endl;

  SDL_RenderPresent(renderer);
  while (1)
  {
    if (SDL_PollEvent(&event) && event.type == SDL_QUIT)
      break;
  }
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  delete model;
  return 0;
}
