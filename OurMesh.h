//
// Created by yarin on 25/11/2019.
//

#include <igl/circulation.h>
#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <iostream>
#include <set>
//#include "BetterViewer.h"
//#include <BetterViewer.h>

#ifndef ENGIENIGLNEW_MESH_H
#define ENGIENIGLNEW_MESH_H


class BetterViewer;

class OurMesh {
public:

    OurMesh();

    Eigen::MatrixXd V;
    Eigen::MatrixXd *facesToNormals;
    Eigen::MatrixXi F;
    // Prepare array-based edge data structures and priority queue
    Eigen::VectorXi EMAP;
    Eigen::MatrixXi E,EF,EI;
    typedef std::set<std::pair<double,int> > PriorityQueue;
    PriorityQueue *Q = new PriorityQueue;
    std::vector<PriorityQueue::iterator > *Qit = new std::vector<PriorityQueue::iterator >;

    // If an edge were collapsed, we'd collapse it to these points:
    Eigen::MatrixXd C;
    int numcollapsed;
    Eigen::Vector3d centerOfRotation;
    int parent,index;
    bool isYcylinder;
    int meshHeight;



    void init();
    void pAndC(
            const int e,
            const Eigen::MatrixXd & V,
            const Eigen::MatrixXi & F/*F*/,
            const Eigen::MatrixXi & E,
            const Eigen::VectorXi & EMAP/*EMAP*/,
            const Eigen::MatrixXi & EF/*EF*/,
            const Eigen::MatrixXi & EI/*EI*/,
            double & cost,
            Eigen::RowVectorXd & p);

private:

};


#endif //ENGIENIGLNEW_MESH_H