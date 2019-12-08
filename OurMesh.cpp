//
// Created by yarin on 25/11/2019.
//
#include <igl/read_triangle_mesh.h>
#include <GLFW/glfw3.h>
#include <igl/opengl/glfw/renderer.h>
#include <igl/vertex_triangle_adjacency.h>
#include "OurMesh.h"

OurMesh::OurMesh() {}

void OurMesh::init() {
    using namespace std;
    using namespace Eigen;
    using namespace igl;
    numcollapsed=0;
    edge_flaps(F,E,EMAP,EF,EI);

    Qit->resize(E.rows());
    C.resize(E.rows(),V.cols());
    Q->clear();

    for(int e = 0;e<E.rows();e++)
    {
        double cost = e;
        RowVectorXd p(1,3);
        pAndC(e,V,F,E,EMAP,EF,EI,cost,p);
        C.row(e) = p;
        (*Qit)[e] = Q->insert(std::pair<double,int>(cost,e)).first;
    }
};

void OurMesh::pAndC(
        const int e,
        const Eigen::MatrixXd & V,
        const Eigen::MatrixXi & F/*F*/,
        const Eigen::MatrixXi & E,
        const Eigen::VectorXi & EMAP/*EMAP*/,
        const Eigen::MatrixXi & EF/*EF*/,
        const Eigen::MatrixXi & EI/*EI*/,
        double & cost,
        Eigen::RowVectorXd & p)
{
    std::vector<int> p1  =
            igl::circulation(e,0 ,EMAP ,EF,EI);
    Eigen::Matrix4d newQ1 = Eigen::Matrix4d::Zero(),newQ2 = Eigen::Matrix4d::Zero(); ;
    for(int i=0; i<p1.size();i++){

        Eigen::Vector3d n=facesToNormals->row(p1[i]).normalized();
        Eigen::Vector3d B=V.row(E(e,0));
        double a=n(0);
        double b=n(1);
        double c=n(2);
        double d =(-1)*(B(0)*a +B(1)*b +B(2)*c);
        Eigen:: Vector4d p1=Eigen::Vector4d (a,b,c,d);
        Eigen:: Matrix4d p2=p1*p1.transpose();
        newQ1+=p2;

    }
    std::vector<int> p2  =
            igl::circulation(e,1 ,EMAP ,EF,EI);
    for(int i=0; i<p2.size();i++){
        Eigen::Vector3d n=facesToNormals->row(p2[i]).normalized();
        Eigen::Vector3d B=V.row(E(e,1));
        double a=n(0);
        double b=n(1);
        double c=n(2);
        double d =(-1)*(B(0)*a +B(1)*b +B(2)*c);

        Eigen:: Vector4d p1=Eigen::Vector4d (a,b,c,d);
        Eigen:: Matrix4d p2=p1*p1.transpose();
        newQ1+=p2;
    }

    Eigen::Vector4d toRet;
    Eigen::Matrix4d Qtag= Eigen::Matrix4d::Zero(),newQ3= Eigen::Matrix4d::Zero() ;

    Qtag << newQ1(0,0), newQ1(0,1), newQ1(0,2),newQ1(0,3),
            newQ1(0,1), newQ1(1,1), newQ1(1,2),newQ1(1,3),
            newQ1(0,2), newQ1(1,2), newQ1(2,2),newQ1(2,3),
            0,                0,               0,                     1;
    newQ3=Qtag.inverse();
    toRet = newQ3*Eigen::Vector4d(0 ,0 ,0, 1); // this is v tag.

    cost= toRet.transpose()*newQ1*toRet;
    p = Eigen::Vector3d(toRet[0], toRet[1], toRet[2]);

}




