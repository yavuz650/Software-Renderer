#ifndef __MODEL_H__
#define __MODEL_H__

#include <vector>
#include "geometry.hpp"

class Model {
private:
	std::vector<Vec3f> verts_;
	std::vector<std::vector<int> > faces_;
	std::vector<Vec2f> uv_;
	std::vector<std::vector<int>> textureIdx_;
public:
	Model(const char *filename, bool flip_vertical);
	~Model();
	int nverts();
	int nfaces();
	Vec3f vert(int i);
	std::vector<int> face(int idx);
	Vec2f uv(int i);
	Vec3i textureIdx(int idx);
};

#endif //__MODEL_H__