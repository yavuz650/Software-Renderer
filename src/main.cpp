#include <iostream>
#include <algorithm>
#include <cmath>
#include <cassert>
#include "SDL2/SDL.h"

#include "tgaimage.hpp"
#include "rasterizer.hpp"
#include "shader.hpp"
#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.hpp"
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
  std::string inputFile = "obj/african_head.obj";
  tinyobj::ObjReaderConfig reader_config;
  tinyobj::ObjReader reader;

  if (!reader.ParseFromFile(inputFile, reader_config)) {
    if (!reader.Error().empty()) {
      std::cerr << "TinyObjReader: " << reader.Error();
    }
    exit(1);
  }
  if (!reader.Warning().empty()) {
    std::cout << "TinyObjReader: " << reader.Warning();
  }

  auto& attrib = reader.GetAttrib();
  auto& shapes = reader.GetShapes();

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
  texture.read_tga_file("obj/african_head_diffuse.tga");
  texture.flip_vertically();
  std::vector<triangle> rawTriangles;
  // Loop over shapes
  for (size_t s = 0; s < shapes.size(); s++) {
    // Loop over faces(polygon)
    size_t index_offset = 0;
    for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
      size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
      std::array<Vertex,3> vertices;
      // Loop over vertices in the face.
      for (size_t v = 0; v < fv; v++) {
        // access to vertex
        float vx,vy,vz;
        float nx,ny,nz;
        float tx,ty;
        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
        vx = attrib.vertices[3*size_t(idx.vertex_index)+0];
        vy = attrib.vertices[3*size_t(idx.vertex_index)+1];
        vz = attrib.vertices[3*size_t(idx.vertex_index)+2];
        // Check if `normal_index` is zero or positive. negative = no normal data
        if (idx.normal_index >= 0) {
          nx = attrib.normals[3*size_t(idx.normal_index)+0];
          ny = attrib.normals[3*size_t(idx.normal_index)+1];
          nz = attrib.normals[3*size_t(idx.normal_index)+2];
        }
        // Check if `texcoord_index` is zero or positive. negative = no texcoord data
        if (idx.texcoord_index >= 0) {
          tx = attrib.texcoords[2*size_t(idx.texcoord_index)+0];
          ty = attrib.texcoords[2*size_t(idx.texcoord_index)+1];
        }
        vertices[v] = Vertex(Vector3f(vx,vy,vz),Vector2f(tx,ty),Vector3f(nx,ny,nz));
      }
      rawTriangles.emplace_back(vertices);
      index_offset += fv;
    }
  }

  float x,y;
  int angle;
  float r = 4.0;
  angle = 90;
  int delta = 2;
  Matrix4f modelMatrix = translate(Matrix4f::Identity(),Vector3f(0,0,-3.0));
  
  while(1){
    Vector3f lightDir = Vector3f(0,0,-1.f);
    x = cos(angle * M_PI / 180);
    y = sin(angle * M_PI / 180);
    angle+=delta%360;
    Matrix4f projection = perspective(-1,1,-1,1,-2,-4);
    Matrix4f view = lookAt(Vector3f(3*x,0,3*y-3),Vector3f(0,0,-3),Vector3f(0,1,0));
    Matrix4f vp = viewport(WINDOW_HEIGHT, WINDOW_WIDTH);
    Matrix4f M = projection*view*modelMatrix;
    std::vector<triangle> triangles(rawTriangles);
    SDL_SetRenderDrawColor(renderer,0,0,0,255);
    SDL_RenderClear(renderer);
    zBuffer.resetBuffer();
    vertShader.shade(triangles,modelMatrix,view,projection,vp);
    rasterizer.rasterize(triangles);
    fragShader.shade(triangles,zBuffer,texture,renderer,lightDir);
    //zBuffer.visualize(renderer,WINDOW_WIDTH,WINDOW_HEIGHT);
    // zBuffer.printBuffer();
    SDL_RenderPresent(renderer);
    //SDL_RenderClear(renderer);
    //zBuffer.visualize(renderer,WINDOW_WIDTH,WINDOW_HEIGHT);
    //SDL_RenderPresent(renderer);
    //while(1){
      if (SDL_PollEvent(&event)){
        if(event.type == SDL_QUIT || (event.type==SDL_WINDOWEVENT && event.window.
        event==SDL_WINDOWEVENT_CLOSE))
          break;
      }
    }
  //}

  std::cout << "Finished rendering\n";
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
