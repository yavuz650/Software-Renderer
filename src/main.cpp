#include <iostream>
#include <algorithm>
#include <cmath>
#include <cassert>
#include "SDL2/SDL.h"

#include "tgaimage.hpp"
#include "model.hpp"
#include "rasterizer.hpp"
#include "shader.hpp"
//#include "utils.hpp"

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
