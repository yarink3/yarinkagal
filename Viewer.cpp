// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"
#include <igl/collapse_edge.h>


#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <cassert>

#include <igl/project.h>
//#include <igl/get_seconds.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;


static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;


namespace igl {
    namespace opengl {
        namespace glfw {

            IGL_INLINE void Viewer::init() {
                string line;
                ifstream myfile("configuration.txt");
                if (myfile.is_open()) {
                    while (myfile.good()) {
                        getline(myfile, line);
                        if(line!="") {
                            line = line.substr(0, line.length() - 1);
                            int i = 1;
                            if (line != "" && line.substr(line.length() - 13, line.length() - 1) == "ycylinder.obj")
                                i = 4;
                            int j = 1;
                            for (int k = 0; k < i; k++, j++) {
                                load_mesh_from_file(line);
                                std::cout << "num of meshes loaded: " << j << std::endl;
                            }
                        }

                    }
                    myfile.close();
                }
            }

            //IGL_INLINE void Viewer::init_plugins()
            //{
            //  // Init all plugins
            //  for (unsigned int i = 0; i<plugins.size(); ++i)
            //  {
            //    plugins[i]->init(this);
            //  }
            //}

            //IGL_INLINE void Viewer::shutdown_plugins()
            //{
            //  for (unsigned int i = 0; i<plugins.size(); ++i)
            //  {
            //    plugins[i]->shutdown();
            //  }
            //}

            IGL_INLINE Viewer::Viewer() :
                    data_list(1),
                    selected_data_index(0),
                    next_data_id(1)
            {
                data_list.front().id = 0;



                // Temporary variables initialization
                // down = false;
                //  hack_never_moved = true;
                scroll_position = 0.0f;

                // Per face
                data().set_face_based(false);


#ifndef IGL_VIEWER_VIEWER_QUIET
                const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  [,]     Toggle between cameras
  1,2     Toggle between models
  ;       Toggle vertex labels
  :       Toggle face labels)"
                );
#endif
            }

            IGL_INLINE Viewer::~Viewer() {
            }

            IGL_INLINE bool Viewer::load_mesh_from_file(
                    const std::string &mesh_file_name_string) {

                // Create new data slot and set to selected
                if (!(data().F.rows() == 0 && data().V.rows() == 0)) {
                    append_mesh();
                }
                data().clear();

                size_t last_dot = mesh_file_name_string.rfind('.');
                if (last_dot == std::string::npos) {
                    std::cerr << "Error: No file extension found in " <<
                              mesh_file_name_string << std::endl;
                    return false;
                }

                std::string extension = mesh_file_name_string.substr(last_dot + 1);
                if (extension == "off" || extension == "OFF") {
                    OurMesh *mesh = new OurMesh();

                    if (!igl::readOFF(mesh_file_name_string, mesh->V, mesh->F))
                        return false;


                    data().set_mesh(mesh->V, mesh->F);
                    mesh->facesToNormals = &data().F_normals;
                    mesh->init();
                    meshes.push_back(*mesh);


                } else if (extension == "obj" || extension == "OBJ") {
                    OurMesh *mesh = new OurMesh();
                    Eigen::MatrixXd corner_normals;
                    Eigen::MatrixXi fNormIndices;

                    Eigen::MatrixXd UV_V;
                    Eigen::MatrixXi UV_F;


                    if (!(
                            igl::readOBJ(
                                    mesh_file_name_string,
                                    mesh->V, UV_V, corner_normals, mesh->F, UV_F, fNormIndices))) {
                        return false;
                    }



                    if(mesh_file_name_string.substr(mesh_file_name_string.length()-13,mesh_file_name_string.length()-1)=="ycylinder.obj")
                    {
                        mesh->index=numOfCylinder;

                        Eigen::Vector3d m1 = mesh->V.colwise().minCoeff();
                        Eigen::Vector3d M1 = mesh->V.colwise().maxCoeff();
                        double height=abs(M1(1)-m1(1));

                        data().Tout.translate(Eigen::Vector3d (0,1.6,0));

                        Eigen::Vector3d m = mesh->V.colwise().minCoeff();
                        Eigen::Vector3d M = mesh->V.colwise().maxCoeff();

                        if(numOfCylinder==0) {  // we are in the first link
                            mesh->parent = 0;
                            data().Tout.translate(Eigen::Vector3d (0,-1.6,0));


                        }



                        else {
                            mesh->parent = numOfCylinder - 1;
                            // drawAxises(); TODO- complete this method
                        }
                        mesh->centerOfRotation << Eigen::Vector3d((M(0)+m(0))/2,m(1),(M(2)+m(2))/2);
                        data().SetCenterOfRotation(Eigen::Vector3d((M(0)+m(0))/2,m(1),(M(2)+m(2))/2));

                        // draw thw axis
                        if(numOfCylinder!=3){

                            Eigen::MatrixXd V_box(6,3);
                            V_box <<
                                  (m(0)+M(0))/2, M(1),m(2)-1.6,// 1
                                    M(0)-1.6,M(1),(m(2)+M(2))/2, //2
                                    m(0)+1.6,M(1),(m(2)+M(2))/2, //3
                                    (m(0)+M(0))/2, M(1),M(2)+1.6,// 4
                                    (M(0)+m(0))/2,m(0),(M(2)+m(2))/2,
                                    0,2.5,0;


                            Eigen::MatrixXi E_box(3,2);
                            E_box <<
                                  0,3,
                                    1,2,
                                    4,5;

                                data().add_edges(V_box.row(E_box(0, 0)),V_box.row(E_box(0, 1)),Eigen::RowVector3d(1, 0, 0));
                                data().add_edges(V_box.row(E_box(1, 0)),V_box.row(E_box(1, 1)),Eigen::RowVector3d(0, 1, 0));
                                data().add_edges(V_box.row(E_box(2, 0)),V_box.row(E_box(2, 1)),Eigen::RowVector3d(0, 0, 1));

                        }
                        numOfCylinder++;

                        mesh->isYcylinder= true;
                    }
                    else {
                        mesh->isYcylinder = false;
                        data().MyTranslate(Eigen::Vector3f(5,0,0),true);
                    }



                    data().set_mesh(mesh->V, mesh->F);
                    data().set_uv(UV_V, UV_F);
                    data().show_overlay_depth=false;
                    data().line_width=3;
                    mesh->facesToNormals = &data().F_normals;
                    mesh->init();
                    meshes.push_back(*mesh);


                } else {
                    // unrecognized file type
                    printf("Error: %s is not a recognized file type.\n", extension.c_str());
                    return false;
                }

                data().compute_normals();
                data().uniform_colors(Eigen::Vector3d(51.0 / 255.0, 43.0 / 255.0, 33.3 / 255.0),
                                      Eigen::Vector3d(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0),
                                      Eigen::Vector3d(255.0 / 255.0, 235.0 / 255.0, 80.0 / 255.0));

                // Alec: why?
                if (data().V_uv.rows() == 0) {
                    data().grid_texture();
                }


                //for (unsigned int i = 0; i<plugins.size(); ++i)
                //  if (plugins[i]->post_load())
                //    return true;

                return true;
            }

            IGL_INLINE bool Viewer::save_mesh_to_file(
                    const std::string &mesh_file_name_string) {
                // first try to load it with a plugin
                //for (unsigned int i = 0; i<plugins.size(); ++i)
                //  if (plugins[i]->save(mesh_file_name_string))
                //    return true;

                size_t last_dot = mesh_file_name_string.rfind('.');
                if (last_dot == std::string::npos) {
                    // No file type determined
                    std::cerr << "Error: No file extension found in " <<
                              mesh_file_name_string << std::endl;
                    return false;
                }
                std::string extension = mesh_file_name_string.substr(last_dot + 1);
                if (extension == "off" || extension == "OFF") {
                    return igl::writeOFF(
                            mesh_file_name_string, data().V, data().F);
                } else if (extension == "obj" || extension == "OBJ") {
                    Eigen::MatrixXd corner_normals;
                    Eigen::MatrixXi fNormIndices;

                    Eigen::MatrixXd UV_V;
                    Eigen::MatrixXi UV_F;

                    return igl::writeOBJ(mesh_file_name_string,
                                         data().V,
                                         data().F,
                                         corner_normals, fNormIndices, UV_V, UV_F);
                } else {
                    // unrecognized file type
                    printf("Error: %s is not a recognized file type.\n", extension.c_str());
                    return false;
                }
                return true;
            }

            IGL_INLINE bool Viewer::load_scene() {
                std::string fname = igl::file_dialog_open();
                if (fname.length() == 0)
                    return false;
                return load_scene(fname);
            }

            IGL_INLINE bool Viewer::load_scene(std::string fname) {
                // igl::deserialize(core(),"Core",fname.c_str());
                igl::deserialize(data(), "Data", fname.c_str());
                return true;
            }

            IGL_INLINE bool Viewer::save_scene() {
                std::string fname = igl::file_dialog_save();
                if (fname.length() == 0)
                    return false;
                return save_scene(fname);
            }

            IGL_INLINE bool Viewer::save_scene(std::string fname) {
                //igl::serialize(core(),"Core",fname.c_str(),true);
                igl::serialize(data(), "Data", fname.c_str());

                return true;
            }

            IGL_INLINE void Viewer::open_dialog_load_mesh() {
                std::string fname = igl::file_dialog_open();

                if (fname.length() == 0)
                    return;

                this->load_mesh_from_file(fname.c_str());
            }

            IGL_INLINE void Viewer::open_dialog_save_mesh() {
                std::string fname = igl::file_dialog_save();

                if (fname.length() == 0)
                    return;

                this->save_mesh_to_file(fname.c_str());
            }

            IGL_INLINE ViewerData &Viewer::data(int mesh_id /*= -1*/) {
                assert(!data_list.empty() && "data_list should never be empty");
                int index;
                if (mesh_id == -1)
                    index = selected_data_index;
                else
                    index = mesh_index(mesh_id);

                assert((index >= 0 && index < data_list.size()) &&
                       "selected_data_index or mesh_id should be in bounds");
                return data_list[index];
            }

            IGL_INLINE const ViewerData &Viewer::data(int mesh_id /*= -1*/) const {
                assert(!data_list.empty() && "data_list should never be empty");
                int index;
                if (mesh_id == -1)
                    index = selected_data_index;
                else
                    index = mesh_index(mesh_id);

                assert((index >= 0 && index < data_list.size()) &&
                       "selected_data_index or mesh_id should be in bounds");
                return data_list[index];
            }

            IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/) {
                assert(data_list.size() >= 1);

                data_list.emplace_back();
                selected_data_index = data_list.size() - 1;
                data_list.back().id = next_data_id++;
                //if (visible)
                //    for (int i = 0; i < core_list.size(); i++)
                //        data_list.back().set_visible(true, core_list[i].id);
                //else
                //    data_list.back().is_visible = 0;
                return data_list.back().id;
            }

            IGL_INLINE bool Viewer::erase_mesh(const size_t index) {
                assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
                assert(data_list.size() >= 1);
                if (data_list.size() == 1) {
                    // Cannot remove last mesh
                    return false;
                }
                data_list[index].meshgl.free();
                data_list.erase(data_list.begin() + index);
                if (selected_data_index >= index && selected_data_index > 0) {
                    selected_data_index--;
                }

                return true;
            }

            IGL_INLINE size_t Viewer::mesh_index(const int id) const {
                for (size_t i = 0; i < data_list.size(); ++i) {
                    if (data_list[i].id == id)
                        return i;
                }
                return 0;
            }

            IGL_INLINE void Viewer::collapse() {
                auto PCOST = [=](
                        const int e,
                        const Eigen::MatrixXd &V,
                        const Eigen::MatrixXi &F/*F*/,
                        const Eigen::MatrixXi &E,
                        const Eigen::VectorXi &EMAP/*EMAP*/,
                        const Eigen::MatrixXi &EF/*EF*/,
                        const Eigen::MatrixXi &EI/*EI*/,
                        double &cost,
                        Eigen::RowVectorXd &p) {
                    std::vector<int> p1  =
                            igl::circulation(e,0 ,EMAP ,EF,EI);
                    Eigen::Matrix4d newQ1 = Eigen::Matrix4d::Zero() ;

                    for(int i=0; i<p1.size();i++){
                        Eigen::Vector3d n=data().F_normals.row(p1[i]).normalized();
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

                    for(int i=0; i<p2.size();i++) {
                        Eigen::Vector3d n = data().F_normals.row(p2[i]).normalized();
                        Eigen::Vector3d B = V.row(E(e, 1));
                        double a = n(0);
                        double b = n(1);
                        double c = n(2);
                        double d = (-1) * (B(0) * a + B(1) * b + B(2) * c);

                        Eigen::Vector4d p1 = Eigen::Vector4d(a, b, c, d);
                        Eigen::Matrix4d p2 = p1 * p1.transpose();
                        newQ1 += p2;
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
                    std::cout<<"edge < " << e <<" > , cost = <"<<cost << "> new v position (<" <<p[0]<< ">,<" <<p[1] <<">,<"<<p[2]<<">)"<<std::endl;

                };
                using namespace std;
                using namespace Eigen;
                using namespace igl;
                // If animating then collapse 10% of edges

                int i = selected_data_index;
                if (!meshes[i].Q->empty()) {
                    bool something_collapsed = false;
                    // collapse edge
                    const int max_iter = std::ceil(0.05 * meshes[i].Q->size());
                    for (int j = 0; j < max_iter; j++) {
                        if (!collapse_edge(
                                PCOST, meshes[i].V,
                                meshes[i].F,
                                meshes[i].E,
                                meshes[i].EMAP,
                                meshes[i].EF,
                                meshes[i].EI,
                                *meshes[i].Q,
                                *meshes[i].Qit,
                                meshes[i].C))
                        {
                            break;
                        }

                        something_collapsed = true;
                        meshes[i].numcollapsed++;

                    }

                    if (something_collapsed) {

                        std::cout << "collapsed:    " << meshes[i].numcollapsed << std::endl;
                        data().clear();
                        data().set_mesh(meshes[i].V, meshes[i].F);
                        data().set_face_based(true);
                    }
                }
            }

            Eigen:: Vector3d Viewer:: getNewRotationVector(int indexOfLink){
                Eigen::Vector3d vecY=Eigen::Vector3d (0,1.0,0);
                Eigen:: Matrix3d mat=Eigen::Matrix3d::Identity();
                for(int i=0; i<indexOfLink+1;i++){
                    mat=mat*data(i).Tout.rotation().matrix();

                }
                return mat*vecY;

            }

            Eigen:: Matrix4d Viewer:: getParentsTrans(int indexOfLink){
                Eigen:: Matrix4d mat=Eigen:: Matrix4d::Identity();
                if(indexOfLink==4)
                    return mat;
                for(int i=0; i<indexOfLink;i++){
                    mat=mat*data(i).MakeTransd();
                }
                return mat;

            }

            Eigen:: Matrix3d Viewer:: getParentsRotationMatrixes(int indexOfLink){
                if(indexOfLink==0)
                    return Eigen:: Matrix3d::Identity();
                else {
                    Eigen::Matrix3d mat = data(1).Tout.rotation().matrix();
                    for (int i = 2; i < indexOfLink; i++) {
                        mat = mat * data(i).Tout.rotation().matrix();
                    }
                    return mat;
                }

            }

            void Viewer::toFirstTip() {

            }


            IGL_INLINE Eigen::Vector3d Viewer::getTip(int index) {
                Eigen::Vector3d ret = Eigen::Vector3d::Zero();
                for (int j = 0; j <index+1 ; j++) {
                    ret = ret+ getNewRotationVector(j);
                }
                return ret*1.6;
            }






        } // end namespace
    } // end namespace
}

