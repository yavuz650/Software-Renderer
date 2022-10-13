#ifndef GRAPHICSPIPELINE_HPP_
#define GRAPHICSPIPELINE_HPP_

#include <memory>

#include "rasterizer.hpp"
#include "shader.hpp"

class GraphicsPipeline {
 private:
  int screenWidth;
  int screenHeight;
  std::shared_ptr<Shader> shader;
  Rasterizer rasterizer;
  ZBuffer zbuffer;
  bool doBackfaceCulling;

 public:
  GraphicsPipeline(int screenWidth_, int screenHeight_,
                   std::shared_ptr<Shader> shader_);
  void draw(std::vector<triangle> &input, SDL_Renderer *renderer);
};

#endif