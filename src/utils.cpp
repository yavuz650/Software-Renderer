#include "utils.hpp"

Matrix4f viewport(int width, int height){
  assert(width!=0 && height!=0);
  Matrix4f result = Matrix4f::Identity();
  result(0,0) = width/2;
  result(1,1) = height/2;
  result(0,3) = (width-1)/2;
  result(1,3) = (height-1)/2;
  return result;
}

//Returns the view matrix
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
  Matrix4f Result(m);
  Result(0,3) = m(0,0) * v(0) + m(0,1) * v(1) + m(0,2) * v(2) + m(0,3);
  Result(1,3) = m(1,0) * v(0) + m(1,1) * v(1) + m(1,2) * v(2) + m(1,3);
  Result(2,3) = m(2,0) * v(0) + m(2,1) * v(1) + m(2,2) * v(2) + m(2,3);
  Result(3,3) = m(3,3);
  return Result;
}

boundingBox::boundingBox(Vector2f bottomLeft_, Vector2f topRight_)
    : bottomLeft(bottomLeft_), topRight(topRight_) {}
Vector2f& boundingBox::BL(){
  return bottomLeft;
}
Vector2f& boundingBox::TR(){
  return topRight;
}

ZBuffer::ZBuffer(int screenWidth_, int screenHeight_)
    : screenWidth(screenWidth_),
      screenHeight(screenHeight_),
      buffer(screenWidth_,
             std::vector<float>(screenHeight_,
                                -std::numeric_limits<float>::max())) {}

void ZBuffer::visualize(SDL_Renderer *renderer, int screenWidth, int screenHeight){
  for(int i=0; i<screenWidth; i++){
    for(int j=0; j<screenHeight; j++){
      SDL_SetRenderDrawColor(renderer, buffer[i][j]*10,
                                        buffer[i][j]*10,
                                        buffer[i][j]*10, 255);
      SDL_RenderDrawPoint(renderer, i, screenHeight-j);
    }
  }
}
float ZBuffer::get(int x, int y) const{
  return buffer[x][y];
}
void ZBuffer::set(int x, int y, float f){
  buffer[x][y] = f;
}
void ZBuffer::resetBuffer(){
  for (int i = 0; i < screenWidth; i++)
  {
    std::fill(buffer[i].begin(), buffer[i].end(), -std::numeric_limits<float>::max());
  }
}
void ZBuffer::printBuffer(){
  for(int i=0; i<screenWidth; i++){
    for(int j=0; j<screenHeight; j++){
      std::cout << buffer[i][j] << "\n";
    }
  }
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
