#pragma once
// #include <Eigen/core>
#include <Eigen/Geometry>


class Movable
{
public:
    Movable();
    Movable(const Movable &mov);
    Eigen::Matrix4f MakeTrans();
    Eigen::Matrix4d MakeTransd();
    Eigen::Matrix4f MakeTransScale();
    Eigen::Matrix4d MakeTransScaled();
    void MyTranslate(Eigen::Vector3f amt,bool preRotation);
    void MyRotate(Eigen::Vector3d rotAxis,double angle);
    void MyScale(Eigen::Vector3d amt);
    void TranslateInSystem(Eigen::Matrix4f mat, Eigen::Vector3f amt, bool preRotation);


    Eigen::Transform<float,3,Eigen::Affine> T;
    Eigen::Affine3d Tin;
    Eigen::Affine3d Tout;

    void SetCenterOfRotation(Eigen::Vector3d amt);

    Eigen::Vector3d GetCenterOfRotation();


    void RotateInSystem(Eigen::Matrix4d mat,  Eigen::Vector3d rotAxis, double angle, bool preRotation);
};