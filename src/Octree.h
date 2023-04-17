#ifndef MANIFOLD_OCTREE_H_
#define MANIFOLD_OCTREE_H_

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include <vector>
#include <list>
#include <set>
#include <map>

class Grid_Index
{
public:
    Grid_Index(){}
    Grid_Index(int x, int y, int z)
    : id(x,y,z)
    {}
    bool operator<(const Grid_Index& ind) const
    {
        int i = 0;
        while (i < 3 && id[i] == ind.id[i])
            i++;
        return (i < 3 && id[i] < ind.id[i]);
    }
    Grid_Index operator+(const Grid_Index& ind) const
    {
        Grid_Index grid(*this);
        grid.id += ind.id;
        return grid;
    }
    Grid_Index operator/(int x) const
    {
        return Grid_Index(id[0]/x,id[1]/x,id[2]/x);
    }
    glm::ivec3 id;
};

class Octree
{
public:
    Octree();

    Octree(const glm::dvec3& min_c, const glm::dvec3& max_c, const std::vector<glm::ivec3>& faces, float thickness);

    Octree(const glm::dvec3& min_c, const glm::dvec3& len);

    ~Octree();

    bool Is_Exterior(const glm::dvec3& p) const;

    bool Intersection(const glm::ivec3& sel_face_indices, const glm::dvec3& min_c, const glm::dvec3& len, const std::vector<glm::dvec3>& vertices) const;

    void Split(const std::vector<glm::dvec3>& vertices);

    void SwapFaces(const std::vector<glm::ivec3>& faces, const std::vector<glm::dvec3>& vertices);

    void SwapFacesRecursive(const std::vector<glm::dvec3>& vertices);

    void BuildConnection();

    void ConnectTree(Octree* l, Octree* r, int dim);

    void BuildEmptyConnection();

    void ConnectEmptyTree(Octree* l, Octree* r, int dim);

    void BuildEmptyList();

    void ExpandEmpty(std::list<Octree*>& empty_list, std::set<Octree*>& empty_set, int dim);

    void ConstructFace(std::map<Grid_Index,int>& vcolor,const glm::ivec3& start,std::vector<glm::dvec3>& vertices,std::vector<glm::ivec4>& faces, std::vector<std::set<int> >& v_faces) const;

    glm::dvec3 min_corner, length;
    int level;
    int number;
    int occupied;
    int exterior;
    std::list<Octree*> empty_neighbors;

protected:
    Octree* children[8];
    Octree* connection[6];
    Octree* empty_connection[6];

    std::vector<glm::ivec3> face_indices;
    std::vector<int> face_ind;
};

#endif
