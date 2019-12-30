#include "Movable.h"

Movable::Movable()
{
    T = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
    Tin = Eigen::Affine3d::Identity();
    Tout = Eigen::Affine3d::Identity();
}

Movable::Movable(const Movable& mov) {
    Tout=mov.Tout;
    Tin=mov.Tin;
}



Eigen::Matrix4d Movable::MakeTransd()
{
    Eigen::Matrix4d mat=Eigen::Matrix4d::Identity();
    mat.col(3) << Tin.translation(),1;
    return (Tout.matrix()*mat);
}
Eigen::Matrix4f Movable::MakeTrans()
{
    return MakeTransd().cast<float>();
}

Eigen::Matrix4f Movable::MakeTransScale() {

    return (Tout.matrix()*Tin.matrix()).cast<float>();
}

Eigen::Matrix4d Movable::MakeTransScaled() {

    return (Tout.matrix()*Tin.matrix());
}



//void Movable::MyTranslate(Eigen::Vector3f amt)
//{
//	T.pretranslate(amt);
//}

//angle in radians
void Movable::MyRotate(Eigen::Vector3d rotAxis, double angle)
{
    Tout.rotate(Eigen::AngleAxisd(angle, rotAxis.normalized()));
}

void Movable::MyScale(Eigen::Vector3d amt)
{
    Tin.scale(amt);
}

void Movable::SetCenterOfRotation(Eigen::Vector3d amt)
{
    Tin.translate(-amt);
    Tout.pretranslate(amt);
}

Eigen::Vector3d Movable::GetCenterOfRotation()
{
    return -Tin.translation();
}

void Movable::MyTranslate(Eigen::Vector3f amt, bool preRotation)
{
    if (preRotation)
        Tout.pretranslate(amt.cast<double>());
    else
        Tout.translate(amt.cast<double>());
}

void Movable::TranslateInSystem(Eigen::Matrix4f mat, Eigen::Vector3f amt, bool preRotation)
{
    MyTranslate((mat.block<3, 3>(0, 0).transpose() * amt), preRotation);
}

void Movable::RotateInSystem(Eigen::Matrix4d mat,  Eigen::Vector3d rotAxis, double angle, bool preRotation){
    MyRotate((mat.block<3, 3>(0, 0).transpose()*rotAxis) , angle);
}