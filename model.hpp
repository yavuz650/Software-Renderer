#ifndef __MODEL_H__
#define __MODEL_H__

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

class Model {
private:
	std::vector<Vector3f> verts_;
	std::vector<std::vector<int> > faces_;
	std::vector<Vector2f> uv_;
	std::vector<std::vector<int>> textureIdx_;
public:
	Model(const char *filename);
	~Model();
	int nverts();
	int nfaces();
	Vector3f vert(int i);
	std::vector<int> face(int idx);
	Vector2f uv(int i);
	std::vector<int> textureIdx(int idx);
};

#endif //__MODEL_H__