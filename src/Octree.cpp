#include "Octree.h"

#include "Intersection.h"

#include <cstring>  // for memset
#include <iostream>

Octree::Octree()
{
    memset(children, 0, sizeof(Octree*)*8);
    memset(connection, 0, sizeof(Octree*)*6);
    memset(empty_connection, 0, sizeof(Octree*)*6);
    level = 0;
    number = 1;
    occupied = 1;
    exterior = 0;
}

Octree::Octree(const glm::dvec3& min_c, const glm::dvec3& max_c, const std::vector<glm::ivec3>& faces, float thickness)
{
    memset(children, 0, sizeof(Octree*)*8);
    memset(connection, 0, sizeof(Octree*)*6);
    memset(empty_connection, 0, sizeof(Octree*)*6);
    level = 0;
    number = 1;
    min_corner = min_c;
    length = max_c - min_c;
    int ind = 0;
    for (int i = 1; i < 3; ++i)
        if (length[i] > length[ind])
            ind = i;
    for (int i = 0; i < 3; ++i)
    {
        min_corner[i] -= (length[ind] - length[i]) * 0.5 + thickness * 0.5;
    }
    length = glm::dvec3(length[ind]+thickness, length[ind]+thickness, length[ind]+thickness);
    face_indices = faces;
    face_ind.resize(faces.size());
    for (int i = 0; i < (int)faces.size(); ++i)
        face_ind[i] = i;
    occupied = 1;
    exterior = 0;
}

Octree::Octree(const glm::dvec3& min_c, const glm::dvec3& len)
{
    memset(children, 0, sizeof(Octree*)*8);
    memset(connection, 0, sizeof(Octree*)*6);
    memset(empty_connection, 0, sizeof(Octree*)*6);
    level = 0;
    number = 1;
    min_corner = min_c;
    length = len;
    occupied = 1;
    exterior = 0;
}

Octree::~Octree()
{
    for (int i = 0; i < 8; ++i)
    {
        if (children[i])
            delete children[i];
        children[i] = 0;
    }
}

bool Octree::Is_Exterior(const glm::dvec3& p) const
{
    for (int i = 0; i < 3; ++i)
        if (p[i] < min_corner[i] || p[i] > min_corner[i] + length[i])
            return true;
    if (!occupied)
        return exterior;
    if (level == 0)
        return false;
    int index = 0;
    for (int i = 0; i < 3; ++i)
    {
        index *= 2;
        if (p[i] > min_corner[i] + length[i] / 2)
            index += 1;
    }
    return children[index]->Is_Exterior(p);
}

bool Octree::Intersection(const glm::ivec3& sel_face_indices, const glm::dvec3& min_c, const glm::dvec3& len, const std::vector<glm::dvec3>& vertices) const
{
    float boxcenter[3];
    float boxhalfsize[3];
    float triverts[3][3];
    for (int i = 0; i < 3; ++i)
    {
        boxhalfsize[i] = len[i] * 0.5;
        boxcenter[i] = min_c[i] + boxhalfsize[i];
    }
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            triverts[i][j] = vertices[sel_face_indices[i]][j];
        }
    }
    return triBoxOverlap(boxcenter, boxhalfsize, triverts);
}

void Octree::Split(const std::vector<glm::dvec3>& vertices)
{
    level += 1;
    number = 0;
    if (level > 1) {
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                for (int k = 0; k < 2; ++k) {
                    int ind = i * 4 + j * 2 + k;
                    if (children[ind] && children[ind]->occupied) {
                        children[ind]->Split(vertices);
                        number += children[ind]->number;
                    }
                }
            }
        }
        face_indices.clear();
        face_ind.clear();
        return;
    }
    glm::dvec3 halfsize = length * 0.5;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            for (int k = 0; k < 2; ++k) {
                int ind = i * 4 + j * 2 + k;

                glm::dvec3 startpoint = min_corner;
                startpoint[0] += i * halfsize[0];
                startpoint[1] += j * halfsize[1];
                startpoint[2] += k * halfsize[2];

                children[ind] = new Octree(startpoint, halfsize);
                children[ind]->occupied = 0;
                children[ind]->number = 0;

                for (int face = 0; face < (int)face_indices.size(); ++face) {
                    if (Intersection(face_indices[face], startpoint, halfsize, vertices)) {
                        children[ind]->face_indices.push_back(face_indices[face]);
                        children[ind]->face_ind.push_back(face_ind[face]);
                        if (children[ind]->occupied == 0) {
                            children[ind]->occupied = 1;
                            number += 1;
                            //children[ind]->number = 1;
                        }
                    }
                }
            }
        }
    }
    face_indices.clear();
    face_ind.clear();
}

void Octree::SwapFaces(const std::vector<glm::ivec3>& faces, const std::vector<glm::dvec3>& vertices)
{
    face_indices = faces;
    face_ind.resize(faces.size());
    for (int i = 0; i < (int)faces.size(); ++i)
        face_ind[i] = i;
    this->SwapFacesRecursive(vertices);
}

void Octree::SwapFacesRecursive(const std::vector<glm::dvec3>& vertices)
{
    if (level == 0)
        return;
    glm::dvec3 halfsize = length * 0.5;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            for (int k = 0; k < 2; ++k) {
                int ind = i * 4 + j * 2 + k;

                glm::dvec3 startpoint = min_corner;
                startpoint[0] += i * halfsize[0];
                startpoint[1] += j * halfsize[1];
                startpoint[2] += k * halfsize[2];

                children[ind]->face_indices.clear();
                children[ind]->face_ind.clear();

                for (int face = 0; face < (int)face_indices.size(); ++face) {
                    if (Intersection(face_indices[face], startpoint, halfsize, vertices)) {
                        children[ind]->face_indices.push_back(face_indices[face]);
                        children[ind]->face_ind.push_back(face_ind[face]);
                    }
                }
            }
        }
    }
    face_indices.clear();
    face_ind.clear();

    for (int i = 0; i < 8; ++i)
    {
        if (children[i])
        {
            children[i]->SwapFacesRecursive(vertices);
        }
    }
}

void Octree::BuildConnection()
{
    if (level == 0)
        return;
    for (int i = 0; i < 8; ++i)
    {
        if (children[i])
        {
            children[i]->BuildConnection();
        }
    }
    int y_index[] = {0, 1, 4, 5};
    for (int i = 0; i < 4; ++i)
    {
        if (children[i * 2] && children[i * 2 + 1])
            ConnectTree(children[i * 2], children[i * 2 + 1], 2);
        if (children[y_index[i]] && children[y_index[i] + 2])
            ConnectTree(children[y_index[i]], children[y_index[i] + 2], 1);
        if (children[i] && children[i + 4])
            ConnectTree(children[i], children[i + 4], 0);
    }
}

void Octree::ConnectTree(Octree* l, Octree* r, int dim)
{
    int y_index[] = {0, 1, 4, 5};
    if (dim == 2)
    {
        l->connection[2] = r;
        r->connection[5] = l;
        for (int i = 0; i < 4; ++i) {
            if (l->children[i * 2 + 1] && r->children[i * 2]) {
                ConnectTree(l->children[i * 2 + 1], r->children[i * 2], dim);
            }
        }
    } else
    if (dim == 1)
    {
        l->connection[1] = r;
        r->connection[4] = l;
        for (int i = 0; i < 4; ++i) {
            if (l->children[y_index[i] + 2] && r->children[y_index[i]]) {
                ConnectTree(l->children[y_index[i] + 2], r->children[y_index[i]], dim);
            }
        }
    } else
    if (dim == 0)
    {
        l->connection[0] = r;
        r->connection[3] = l;
        for (int i = 0; i < 4; ++i) {
            if (l->children[i + 4] && r->children[i]) {
                ConnectTree(l->children[i + 4], r->children[i], dim);
            }
        }
    }
}

void Octree::BuildEmptyConnection()
{
    if (level == 0)
        return;

    for (int i = 0; i < 8; ++i)
    {
        if (children[i]->occupied)
        {
            children[i]->BuildEmptyConnection();
        }
    }
    int pair_x[] = {0,2,4,6,0,1,4,5,0,1,2,3};
    int pair_y[] = {1,3,5,7,2,3,6,7,4,5,6,7};
    int dim[] = {2,2,2,2,1,1,1,1,0,0,0,0};
    for (int i = 0; i < 12; ++i)
    {
        ConnectEmptyTree(children[pair_x[i]], children[pair_y[i]], dim[i]);
    }
}

void Octree::ConnectEmptyTree(Octree* l, Octree* r, int dim)
{
    int y_index[] = {0, 1, 4, 5};
    if (l->occupied && r->occupied)
    {
        if (l->level == 0)
            return;
        if (dim == 2)
        {
            for (int i = 0; i < 4; ++i) {
                ConnectEmptyTree(l->children[i * 2 + 1], r->children[i * 2], dim);
            }
        } else
        if (dim == 1)
        {
            for (int i = 0; i < 4; ++i) {
                ConnectEmptyTree(l->children[y_index[i] + 2], r->children[y_index[i]], dim);
            }
        } else
        if (dim == 0)
        {
            for (int i = 0; i < 4; ++i) {
                ConnectEmptyTree(l->children[i + 4], r->children[i], dim);
            }
        }
        return;
    }
    if (!(l->occupied || r->occupied))
    {
        l->empty_neighbors.push_back(r);
        r->empty_neighbors.push_back(l);
        return;
    }
    if (!l->occupied)
    {
        if (dim == 2)
        {
            r->empty_connection[5] = l;
            if (r->level > 0)
            {
                for (int i = 0; i < 4; ++i)
                {
                    ConnectEmptyTree(l, r->children[i * 2], dim);
                }
            }
        } else
        if (dim == 1)
        {
            r->empty_connection[4] = l;
            if (r->level > 0)
            {
                for (int i = 0; i < 4; ++i)
                {
                    ConnectEmptyTree(l, r->children[y_index[i]], dim);
                }
            }
        } else
        if (dim == 0)
        {
            r->empty_connection[3] = l;
            if (r->level > 0)
            {
                for (int i = 0; i < 4; ++i)
                {
                    ConnectEmptyTree(l, r->children[i], dim);
                }
            }
        }
        return;
    }
    if (!r->occupied)
    {
        if (dim == 2)
        {
            l->empty_connection[2] = r;
            if (l->level > 0)
            {
                for (int i = 0; i < 4; ++i)
                {
                    ConnectEmptyTree(l->children[i * 2 + 1], r, dim);
                }
            }
        } else
        if (dim == 1)
        {
            l->empty_connection[1] = r;
            if (l->level > 0)
            {
                for (int i = 0; i < 4; ++i)
                {
                    ConnectEmptyTree(l->children[y_index[i] + 2], r, dim);
                }
            }
        } else
        if (dim == 0)
        {
            l->empty_connection[0] = r;
            if (l->level > 0)
            {
                for (int i = 0; i < 4; ++i)
                {
                    ConnectEmptyTree(l->children[i + 4], r, dim);
                }
            }
        }
    }
}

void Octree::BuildEmptyList()
{
    std::queue<Octree*> empty_list;
    std::set<Octree*> empty_set;
    for (int i = 0; i < 6; ++i)
    {
        ExpandEmpty(empty_list, empty_set, i);
    }

    while (!empty_list.empty())
    {
        Octree* empty = empty_list.front();
        empty_list.pop();

        empty->exterior = 1;
        for (std::vector<Octree*>::const_iterator it = empty->empty_neighbors.begin();
            it != empty->empty_neighbors.end(); ++it)
        {
            if (empty_set.find(*it) == empty_set.end())
            {
                empty_set.insert(*it);
                empty_list.push(*it);
            }
        }
    }
}

void Octree::ExpandEmpty(std::queue<Octree*>& empty_list, std::set<Octree*>& empty_set, int dim)
{
    if (!occupied)
    {
        if (empty_set.find(this) == empty_set.end())
        {
            empty_set.insert(this);
            empty_list.push(this);
        }
        return;
    }
    if (level == 0)
        return;
    int y_index[] = {0, 1, 4, 5};
    if (dim == 2 || dim == 5)
    {
        for (int i = 0; i < 4; ++i)
        {
            children[i * 2 + (dim == 5)]->ExpandEmpty(empty_list, empty_set, dim);
        }
        return;
    }
    if (dim == 1 || dim == 4)
    {
        for (int i = 0; i < 4; ++i)
        {
            children[y_index[i] + 2 * (dim == 4)]->ExpandEmpty(empty_list, empty_set, dim);
        }
        return;
    }
    if (dim == 0 || dim == 3)
    {
        for (int i = 0; i < 4; ++i)
        {
            children[i + 4 * (dim == 3)]->ExpandEmpty(empty_list, empty_set, dim);
        }
        return;
    }
}

void Octree::ConstructFace(std::map<Grid_Index,int>& vcolor,const glm::ivec3& start,std::vector<glm::dvec3>& vertices,std::vector<glm::ivec4>& faces, std::vector<std::set<int> >& v_faces) const
{
    if (level == 0)
    {
        if (!occupied)
            return;
        glm::ivec3 offset[6][4] = {{glm::ivec3(1,0,0),glm::ivec3(1,0,1),glm::ivec3(1,1,1),glm::ivec3(1,1,0)},
                                    {glm::ivec3(0,1,0),glm::ivec3(1,1,0),glm::ivec3(1,1,1),glm::ivec3(0,1,1)},
                                    {glm::ivec3(0,0,1),glm::ivec3(0,1,1),glm::ivec3(1,1,1),glm::ivec3(1,0,1)},
                                    {glm::ivec3(0,0,0),glm::ivec3(0,1,0),glm::ivec3(0,1,1),glm::ivec3(0,0,1)},
                                    {glm::ivec3(0,0,0),glm::ivec3(0,0,1),glm::ivec3(1,0,1),glm::ivec3(1,0,0)},
                                    {glm::ivec3(0,0,0),glm::ivec3(1,0,0),glm::ivec3(1,1,0),glm::ivec3(0,1,0)}};
        for (int i = 0; i < 6; ++i)
        {
            if (empty_connection[i] && empty_connection[i]->exterior)
            {
                if (connection[i] && connection[i]->occupied)
                {
                    std::cout << "Error!\n";
                    exit(0);
                }
                int id[4];
                for (int j = 0; j < 4; ++j)
                {
                    glm::ivec3 vind = start + offset[i][j];
                    Grid_Index v_id;
                    v_id.id = vind * 2;
                    std::map<Grid_Index,int>::const_iterator it = vcolor.find(v_id);
                    if (it == vcolor.end())
                    {
                        glm::dvec3 d = min_corner;
                        for (int k = 0; k < 3; ++k)
                            d[k] += offset[i][j][k] * length[k];
                        vcolor.insert(std::make_pair(v_id, vertices.size()));
                        id[j] = vertices.size();
                        vertices.push_back(d);
                        v_faces.emplace_back();
                        for (std::vector<int>::const_iterator it1 = face_ind.begin();
                            it1 != face_ind.end(); ++it1)
                            v_faces[id[j]].insert(*it1);
                    }
                    else {
                        id[j] = it->second;
                        for (std::vector<int>::const_iterator it1 = face_ind.begin();
                            it1 != face_ind.end(); ++it1)
                            v_faces[it->second].insert(*it1);
                    }
                }
                faces.push_back(glm::ivec4(id[0],id[1],id[2],id[3]));
            }
        }
    } else
    {
        for (int i = 0; i < 8; ++i)
        {
            if (children[i] && children[i]->occupied)
            {
                int x = i / 4;
                int y = (i - x * 4) / 2;
                int z = i - x * 4 - y * 2;
                glm::ivec3 nstart = start * 2 + glm::ivec3(x,y,z);
                children[i]->ConstructFace(vcolor, nstart, vertices, faces, v_faces);
            }
        }
    }
}
