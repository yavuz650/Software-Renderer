#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include "model.hpp"

Model::Model(const char *filename, bool flip_vertical) : verts_(), faces_(), uv_(),
                                                         textureIdx_()
{
  std::ifstream in;
  in.open(filename, std::ifstream::in);
  if (in.fail())
    return;
  std::string line;
  if (flip_vertical)
  {
    while (!in.eof())
    {
      std::getline(in, line);
      std::istringstream iss(line.c_str());
      char trash;
      if (!line.compare(0, 2, "v "))
      {
        iss >> trash;
        Vec3f v;

        float temp;
        for (int i = 0; i < 3; i++)
        {
          iss >> temp;
          if (i == 1)
            temp = -temp;
          v.raw[i] = temp;
        }
        verts_.push_back(v);
      }
      else if (!line.compare(0, 2, "f "))
      {
        std::vector<int> f;
        std::vector<int> t;
        int itrash, idx, tidx;;
        iss >> trash;
        while (iss >> idx >> trash >> tidx >> trash >> itrash)
        {
          idx--; // in wavefront obj all indices start at 1, not zero
          tidx--;
          f.push_back(idx);
          t.push_back(tidx);
        }
        faces_.push_back(f);
        textureIdx_.push_back(t);
      }
      else if (!line.compare(0, 3, "vt "))
      {
        iss >> trash >> trash;

        float u,v;
        iss >> u >> v;

        uv_.push_back(Vec2f(u,v));
      }
    }
  }
  else
  {
    while (!in.eof())
    {
      std::getline(in, line);
      std::istringstream iss(line.c_str());
      char trash;
      if (!line.compare(0, 2, "v "))
      {
        iss >> trash;
        Vec3f v;
        float temp;
        for (int i = 0; i < 3; i++)
          iss >> v.raw[i];
        verts_.push_back(v);
      }
      else if (!line.compare(0, 2, "f "))
      {
        std::vector<int> f;
        int itrash, idx;
        iss >> trash;
        while (iss >> idx >> trash >> itrash >> trash >> itrash)
        {
          idx--; // in wavefront obj all indices start at 1, not zero
          f.push_back(idx);
        }
        faces_.push_back(f);
      }
    }
  }
  std::cerr << "# v# " << verts_.size() << " f# " << faces_.size() << std::endl;
}

Model::~Model()
{
}

int Model::nverts()
{
  return (int)verts_.size();
}

int Model::nfaces()
{
  return (int)faces_.size();
}

std::vector<int> Model::face(int idx)
{
  return faces_[idx];
}

Vec3f Model::vert(int i)
{
  return verts_[i];
}
