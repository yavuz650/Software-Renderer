#include "graphicsPipeline.hpp"

GraphicsPipeline::GraphicsPipeline(int screenWidth_, int screenHeight_,
                                   std::shared_ptr<Shader> shader_)
    : screenWidth(screenWidth_),
      screenHeight(screenHeight_),
      shader(shader_),
      zbuffer(screenWidth_, screenHeight_) {}

void GraphicsPipeline::draw(std::vector<triangle> &input, SDL_Renderer *renderer){
  zbuffer.resetBuffer();
  SDL_SetRenderDrawColor(renderer,0,0,0,255);
  SDL_RenderClear(renderer);
  shader->vertexShader(input);
  rasterizer.rasterize(input);
  shader->fragmentShader(input, zbuffer, renderer);
  SDL_RenderPresent(renderer);
}