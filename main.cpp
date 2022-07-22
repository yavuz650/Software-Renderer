#include "tgaimage.hpp"
#include "model.hpp"
#include <iostream>
#include <algorithm>
#include "SDL2/SDL.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);

#define WINDOW_HEIGHT 1000
#define WINDOW_WIDTH 1000

bool isInsideTriangle(Vec2i A, Vec2i B, Vec2i C, Vec2i P){
    Vec2i AB = B-A;
    Vec2i AC = C-A;
    Vec2i PA = A-P;

    Vec3f u = Vec3f(AB.x, AC.x, PA.x)^Vec3f(AB.y, AC.y, PA.y);
    u = Vec3f(u.x/u.z,u.y/u.z,u.z/u.z);

    return u.x/u.z > 0. && u.y/u.z > 0. && (u.x/u.z + u.y/u.z) < 1 ;
}

void line(int x0, int y0, int x1, int y1, SDL_Renderer *renderer, TGAColor color) {
    bool steep = false;
    if (std::abs(x0-x1)<std::abs(y0-y1)) {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0>x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    int dx = x1-x0;
    int dy = y1-y0;
    int derror2 = std::abs(dy)*2;
    int error2 = 0;
    int y = y0;
    for (int x=x0; x<=x1; x++) {
        if (steep) {
            SDL_RenderDrawPoint(renderer,y,x);
        } else {
            SDL_RenderDrawPoint(renderer,x,y);
        }
        error2 += derror2;
        if (error2 > dx) {
            y += (y1>y0?1:-1);
            error2 -= dx*2;
        }
    }
}

//expects normalized coordinates
void triangle(std::array<Vec2f,3> vertices, SDL_Renderer *renderer, TGAColor color, bool isWireframe){
    SDL_SetRenderDrawColor(renderer, color[2], color[1], color[0], color[3]);
    std::array<Vec2i,3> scaledVertices;
    for (int i = 0; i < 3; i++)
    {
        scaledVertices[i].x = (vertices[i].x+1.)*WINDOW_WIDTH/2;
        scaledVertices[i].y = (vertices[i].y+1.)*WINDOW_HEIGHT/2;
    }

    if(isWireframe)
    {
        line(scaledVertices[0].x,scaledVertices[0].y,
            scaledVertices[1].x,scaledVertices[1].y, renderer, color);

        line(scaledVertices[1].x,scaledVertices[1].y,
             scaledVertices[2].x,scaledVertices[2].y, renderer, color);

        line(scaledVertices[0].x,scaledVertices[0].y,
             scaledVertices[2].x,scaledVertices[2].y, renderer, color);
    }
    else
    {    
        Vec2i bboxTopLeft(WINDOW_WIDTH-1, WINDOW_HEIGHT-1);
        Vec2i bboxBottomRight(0, 0);

        for (int i=0; i<3; i++) {
            bboxTopLeft.x = std::min(bboxTopLeft.x, scaledVertices[i].x);
            bboxTopLeft.y = std::min(bboxTopLeft.y, scaledVertices[i].y);

            bboxBottomRight.x = std::max(bboxBottomRight.x, scaledVertices[i].x);
            bboxBottomRight.y = std::max(bboxBottomRight.y, scaledVertices[i].y);
        }
        Vec2i P; 
        for (P.x=bboxTopLeft.x; P.x<=bboxBottomRight.x; P.x++) { 
            for (P.y=bboxTopLeft.y; P.y<=bboxBottomRight.y; P.y++) { 
                if(isInsideTriangle(scaledVertices[0],scaledVertices[1],scaledVertices[2],P))
                    SDL_RenderDrawPoint(renderer,P.x,P.y);
            } 
        }         
        //debug
        // line(bboxTopLeft.x,bboxTopLeft.y,
        //     bboxBottomRight.x,bboxTopLeft.y, renderer, color);

        // line(bboxTopLeft.x,bboxTopLeft.y,
        //     bboxTopLeft.x,bboxBottomRight.y, renderer, color);

        // line(bboxBottomRight.x,bboxTopLeft.y,
        //     bboxBottomRight.x,bboxBottomRight.y, renderer, color);

        // line(bboxTopLeft.x,bboxBottomRight.y,
        //     bboxBottomRight.x,bboxBottomRight.y, renderer, color);              
    }

}

int main(int argc, char** argv) {
    Model *model = new Model("obj/african_head.obj",1);
    int height = 1000;
    int width = 1000;
    TGAImage image(width, height, TGAImage::RGB);

    SDL_Event event;
    SDL_Renderer *renderer;
    SDL_Window *window;
    SDL_Init(SDL_INIT_VIDEO);
    SDL_CreateWindowAndRenderer(width, height, 0, &window, &renderer);
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
    SDL_RenderClear(renderer);
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    Vec3f lightDirection(0,0,-1.);
    for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec3f normalVector = (model->vert(face[0]) - model->vert(face[1]))^(model->vert(face[0]) - model->vert(face[2]));
        normalVector.normalize();
        float intensity = lightDirection*normalVector;
        if(intensity > 0.){
            triangle(std::array<Vec2f,3>{Vec2f(model->vert(face[0]).x,model->vert(face[0]).y),
                                        Vec2f(model->vert(face[1]).x,model->vert(face[1]).y),
                                        Vec2f(model->vert(face[2]).x,model->vert(face[2]).y)}, 
                                        renderer, white.scale(intensity),0);
        }

    }
    //triangle(std::array<Vec2f,3>{Vec2f(0.3,-0.5),Vec2f(0.4,0.4), Vec2f(-0.1,0.2)}, renderer, white, 0);
    //printf("is inside %d\n", isInsideTriangle(Vec2i(1,2),Vec2i(3,6),Vec2i(6,2),Vec2i(2,3)));
    printf(isInsideTriangle(Vec2i(1,2),Vec2i(3,6),Vec2i(6,2),Vec2i(1,3)) ? "true\n" : "false\n");
    SDL_RenderPresent(renderer);
    while (1) {
        if (SDL_PollEvent(&event) && event.type == SDL_QUIT)
            break;
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    delete model;
    return 0;
}
