#ifndef MANIFOLD_MODEL_OBJ_H_
#define MANIFOLD_MODEL_OBJ_H_

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include <cmath>
#include <vector>
#include <map>
#include <set>
#include <queue>

#include "Octree.h"

class Model_OBJ
{
public:
    Model_OBJ();
    ~Model_OBJ();

    void Process_Manifold(int resolution);

    void Build_Tree(int resolution);
    void Construct_Manifold();
    void Project_Manifold();

protected:
    void Calc_Bounding_Box(glm::dvec3& min_corner, glm::dvec3& max_corner);
    glm::dvec3 Closest_Point( const glm::dvec3 triangle[3], const glm::dvec3& sourcePosition ) const;
    glm::dvec3 Find_Closest(int i) const;
    int is_manifold();
    bool Split_Grid(std::map<Grid_Index,int>& vcolor, std::vector<glm::dvec3>& nvertices, const std::vector<glm::ivec4>& nface_indices, std::vector<std::set<int> >& v_faces, std::vector<glm::ivec3>& triangles);

public:
    std::vector<std::set<int> > v_faces;
    std::vector<Grid_Index> v_info;

    std::vector<glm::dvec3> vertices, vertices_buf;
    std::vector<glm::dvec3> colors;
    std::vector<glm::ivec3> face_indices, face_indices_buf;

protected:
    Octree* tree;
};

#endif
