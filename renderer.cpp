#include "igl/opengl/glfw/renderer.h"

#include <GLFW/glfw3.h>
#include <igl/unproject_onto_mesh.h>
#include "igl/look_at.h"
#include <Eigen/Dense>
Renderer::Renderer() : selected_core_index(0),
                       next_core_id(2)
{
    core_list.emplace_back(igl::opengl::ViewerCore());
    core_list.front().id = 1;
    // C-style callbacks
    callback_init = nullptr;
    callback_pre_draw = nullptr;
    callback_post_draw = nullptr;
    callback_mouse_down = nullptr;
    callback_mouse_up = nullptr;
    callback_mouse_move = nullptr;
    callback_mouse_scroll = nullptr;
    callback_key_down = nullptr;
    callback_key_up = nullptr;

    callback_init_data = nullptr;
    callback_pre_draw_data = nullptr;
    callback_post_draw_data = nullptr;
    callback_mouse_down_data = nullptr;
    callback_mouse_up_data = nullptr;
    callback_mouse_move_data = nullptr;
    callback_mouse_scroll_data = nullptr;
    callback_key_down_data = nullptr;
    callback_key_up_data = nullptr;
    highdpi = 1;

    xold = 0;
    yold = 0;

}

IGL_INLINE void Renderer::draw( GLFWwindow* window)
{
    using namespace std;
    using namespace Eigen;

    int width, height;
    glfwGetFramebufferSize(window, &width, &height);

    int width_window, height_window;
    glfwGetWindowSize(window, &width_window, &height_window);

    auto highdpi_tmp = (width_window == 0 || width == 0) ? highdpi : (width / width_window);

    if (fabs(highdpi_tmp - highdpi) > 1e-8)
    {
        post_resize(window,width, height);
        highdpi = highdpi_tmp;
    }

    for (auto& core : core_list)
    {
        core.clear_framebuffers();
    }

    for (auto& core : core_list)
    {
        int index=0;
        for (auto& mesh : scn->data_list)
        {
            if (mesh.is_visible & core.id)
            {
                core.draw((scn->MakeTransd()*scn->getParentsTrans(index)).cast<float>(),mesh,true);
//                core.draw(scn->MakeTrans(),mesh,true);
            }
            index++;
        }
    }

}

void Renderer::SetScene(igl::opengl::glfw::Viewer* viewer)
{
    scn = viewer;
}

IGL_INLINE void Renderer::init(igl::opengl::glfw::Viewer* viewer)
{
    scn = viewer;
    core().init();

    core().align_camera_center(scn->data().V, scn->data().F);
}

void Renderer::UpdatePosition(double xpos, double ypos)
{
    xrel = xold - xpos;
    yrel = yold - ypos;
    xold = xpos;
    yold = ypos;
}

void Renderer::MouseProcessing(int button)
{
    int index=scn->selected_data_index;

    if (button == 1) {
        if(!scn->meshPicked){
            scn->MyTranslate(Eigen::Vector3f(-xrel / 2000.0f, 0, 0), true);
            scn->MyTranslate(Eigen::Vector3f(0, yrel / 2000.0f, 0), true);
        }
        else if (scn->meshes[index].isYcylinder) {
            scn->data(0).MyTranslate(Eigen::Vector3f(-xrel / 2000.0f, 0, 0), true);
            scn->data(0).MyTranslate(Eigen::Vector3f(0, yrel / 2000.0f, 0), true);
        }
        else{
            scn->data().MyTranslate(Eigen::Vector3f(-xrel / 2000.0f, 0, 0), true);
            scn->data().MyTranslate(Eigen::Vector3f(0, yrel / 2000.0f, 0), true);
        }


    }
    else
    {
            scn->data(index).MyRotate(Eigen::Vector3d(0, 0, 1),yrel/180.0f);
            scn->data(index).MyRotate(Eigen::Vector3d(1, 0, 0),xrel/180.0f);

    }

}

Renderer::~Renderer()
{
    //if (scn)
    //	delete scn;
}

bool Renderer::Picking(double newx, double newy, float *pInt)
{
    int fid;
    //Eigen::MatrixXd C = Eigen::MatrixXd::Constant(scn->data().F.rows(), 3, 1);
    Eigen::Vector3f bc;
    double x = newx;
    double y = core().viewport(3) - newy;
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    igl::look_at(core().camera_eye, core().camera_center, core().camera_up, view);
    view = view * (core().trackball_angle * Eigen::Scaling(core().camera_zoom * core().camera_base_zoom)
                   * Eigen::Translation3f(core().camera_translation + core().camera_base_translation)).matrix() * scn->MakeTrans() *scn->getParentsTrans(scn->selected_data_index).cast<float>()* scn->data().MakeTrans();
    if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), view,
                                 core().proj, core().viewport, scn->data().V, scn->data().F, fid, bc))
    {
        Eigen::MatrixXi F = scn->data().F;
        Eigen::MatrixXd V = scn->data().V;

        //Vector4f v5 = Vector4f (1.0 f , 2.0 f , 3.0 f , 4.0 f ) ;

        Eigen::Vector3f v0,v1,v2,p;
        v0=V.row(F.row(fid)(0)).cast<float>();
        v1=V.row(F.row(fid)(1)).cast<float>();
        v2=V.row(F.row(fid)(2)).cast<float>();

        Eigen::Vector4f u0,u1,u2;
        u0=Eigen::Vector4f(v0[0] , v0[1], v0[2],1.0f);
        u1=Eigen::Vector4f(v1[0] , v1[1], v1[2],1.0f);
        u2=Eigen::Vector4f(v2[0] , v2[1], v2[2],1.0f);

        u0=view*u0;
        u1=view*u1;
        u2=view*u2;

        v0=Eigen::Vector3f(u0[0] , u0[1], u0[2]);
        v1=Eigen::Vector3f(u1[0] , u1[1], u1[2]);
        v2=Eigen::Vector3f(u2[0] , u2[1], u2[2]);

        p=v0*bc[0]+v1*bc[1]+v2*bc[2];
        *pInt=p[2];

        return true;
    }

    *pInt=-INFINITY;
    return false;

}

IGL_INLINE void Renderer::resize(GLFWwindow* window,int w, int h)
{
    if (window) {
        glfwSetWindowSize(window, w / highdpi, h / highdpi);
    }
    post_resize(window,w, h);
}

IGL_INLINE void Renderer::post_resize(GLFWwindow* window, int w, int h)
{
    if (core_list.size() == 1)
    {
        core().viewport = Eigen::Vector4f(0, 0, w, h);
    }
    else
    {
        // It is up to the user to define the behavior of the post_resize() function
        // when there are multiple viewports (through the `callback_post_resize` callback)
    }
    //for (unsigned int i = 0; i < plugins.size(); ++i)
    //{
    //	plugins[i]->post_resize(w, h);
    //}
    if (callback_post_resize)
    {
        callback_post_resize(window, w, h);
    }
}

IGL_INLINE igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/)
{
    assert(!core_list.empty() && "core_list should never be empty");
    int core_index;
    if (core_id == 0)
        core_index = selected_core_index;
    else
        core_index = this->core_index(core_id);
    assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
    return core_list[core_index];
}

IGL_INLINE const igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/) const
{
    assert(!core_list.empty() && "core_list should never be empty");
    int core_index;
    if (core_id == 0)
        core_index = selected_core_index;
    else
        core_index = this->core_index(core_id);
    assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
    return core_list[core_index];
}

IGL_INLINE bool Renderer::erase_core(const size_t index)
{
    assert((index >= 0 && index < core_list.size()) && "index should be in bounds");
    //assert(data_list.size() >= 1);
    if (core_list.size() == 1)
    {
        // Cannot remove last viewport
        return false;
    }
    core_list[index].shut(); // does nothing
    core_list.erase(core_list.begin() + index);
    if (selected_core_index >= index && selected_core_index > 0)
    {
        selected_core_index--;
    }
    return true;
}

IGL_INLINE size_t Renderer::core_index(const int id) const {
    for (size_t i = 0; i < core_list.size(); ++i)
    {
        if (core_list[i].id == id)
            return i;
    }
    return 0;
}

IGL_INLINE int Renderer::append_core(Eigen::Vector4f viewport, bool append_empty /*= false*/)
{
    core_list.push_back(core()); // copies the previous active core and only changes the viewport
    core_list.back().viewport = viewport;
    core_list.back().id = next_core_id;
    next_core_id <<= 1;
    if (!append_empty)
    {
        for (auto& data : scn->data_list)
        {
            data.set_visible(true, core_list.back().id);
            //data.copy_options(core(), core_list.back());
        }
    }
    selected_core_index = core_list.size() - 1;
    return core_list.back().id;
}

IGL_INLINE void Renderer::toFirstTip() {
    using namespace Eigen;
    Vector3d c0 = scn->getNewRotationVector(1);
    Vector3d c1 = scn->data(1).GetCenterOfRotation();
    Matrix3d m1 = scn->data(1).Tout.rotation().matrix();
    Matrix3d m2 = scn->getParentsRotationMatrixes(1);
    std::cout <<"m1  is\n " <<m1 <<std::endl;


//    scn->data(1).Tout = scn->data(0).Tout;
//    scn->data(1).Tin = scn->data(0).Tin;
    scn->data(1).Tout.prerotate(m2);
//    scn->data(1).Tout.rotate(scn->data(1).Tout.rotation().transpose());


//    scn->data(1).MyTranslate(scn->getNewRotationVector(0)*1.6,true);

//    scn->data(1).Tout.rotate(m1 );


    std::cout <<"m111  is\n " <<scn->data(1+1).Tout.rotation().matrix() <<std::endl;

    //    scn->data(1).Tout.rotate(scn->data(0).Tout.rotation().matrix()) ;


}
//
//void Renderer::RotateForward(int i) {
//
//    using namespace std;
//
//    using namespace Eigen;
//    Vector3d ci = scn->getNewRotationVector(i);
////    Vector3d c1 = scn->data(1).GetCenterOfRotation();
//    Matrix3d m1 = scn->data(i).Tout.rotation().matrix();
//    Matrix3d m2 = scn->getParentsRotationMatrixes(i);
//    Vector3d ci1= scn->getNewRotationVector(i-1);
//
//
//
//    scn->data(i).Tout = scn->data(i-1).Tout;
//    scn->data(i).Tin = scn->data(i-1).Tin;
//
//
////    scn->data(i).Tout.rotate(scn->data(1).Tout.rotation().inverse());
////    scn->data(1).Tout.rotate(scn->data(1).Tout.rotation().transpose());
//
//
//    scn->data(i).MyTranslate(scn->getNewRotationVector(i-1)*1.6*i,true);
//    scn->data(i).Tout.prerotate(m2);
//    //    scn->data(i).MyRotate(Eigen::Vector3d(0, 0, 1),angle1);
////    scn->data(i).MyRotate(Eigen::Vector3d(1, 0, 0),angle2);
////
////    scn->data(i).MyRotate(Eigen::Vector3d(0, 0, 1),angle3);
//
//
//
//    //    ci.normalize(); ci1.normalized();
////    double alphaY = acos(ci.dot(ci1));
//    //    scn->data(i).MyRotate(Eigen::Vector3d(1,0,0), acos(ci.dot(ci1)));
////    scn->data(1).Tout.rotate(m1 );
//
//
//
//
//
//
//
////    using namespace Eigen;
////    Vector3d c0 = scn->data(i).GetCenterOfRotation() +
////                  scn->data(i).Tout.rotation().matrix()*Eigen::Vector3d (0,1.6,0);
////    Vector3d c1 = scn->data(i+1).GetCenterOfRotation();
////    Matrix3d m1 = scn->data(i+1).Tout.rotation().matrix();
////
////    Affine3d to = scn->data(i+1).Tout;
////    Affine3d ti = scn->data(i+1).Tin;
////
////    scn->data(i+1).Tout = scn->data(i).Tout;
////    scn->data(i+1).Tin = scn->data(i).Tin;
////
//////    scn->data(i+1).SetCen
////    scn->data(i+1).MyTranslate( (scn->data(0).MakeTransd()*Eigen::Vector4d (0,0,0,1)).head(3)+scn->getNewRotationVector(i)*1.6,true);
////    scn->data(i+1).Tout.rotate(scn->data(i).Tout.rotation().matrix());
////    scn->data(i+1).Tout.rotate(scn->data(i+1).Tout.rotation().inverse());
////    scn->data(i+1).Tout.rotate(m1);
//
//
//
//
//
//
//}

IGL_INLINE void Renderer::PrintSphereLocation() {
    using namespace Eigen;
    using namespace std;
    cout<< " size is :\n " <<scn->data_list.size() <<endl;

    Vector3d mt = getSphere();
    cout<< " the target is at is:\n " <<mt.transpose() <<endl;

}

Eigen::Vector3d Renderer::getSphere() {

   return scn->data(scn->data_list.size()-1).Tout.translation().matrix();

}

Eigen::Vector3d Renderer::getButtom() {

    return scn->data(0).Tout.translation().matrix();
}






IGL_INLINE void Renderer::PrintTip() {
    using namespace std;
    Eigen::Vector3d ret;
    ret = printTip(scn->numOfCylinder-1);

    cout<< "the tip is:\n " <<ret.transpose() <<endl;

}


IGL_INLINE Eigen::Vector3d Renderer::printTip(size_t index) {
    Eigen::Vector3d ret = Eigen::Vector3d::Zero();
    for (int j = 0; j <index+1 ; j++) {
        ret = ret+ scn->getNewRotationVector(j);
    }
    return ret*1.6;
}
IGL_INLINE void Renderer::RotateY(bool right) {
    using namespace Eigen;;
    using namespace std;
    if(!scn->meshPicked){
        //scn rotation
        if(right){  scn->MyRotate(Eigen::Vector3d(0, 1,0 ), 10 / 180.0f);}
        else{   scn->MyRotate(Eigen::Vector3d(0, 1,0 ), -10 / 180.0f);}
    }
    else {
        Matrix3d m = scn->data().Tout.rotation().matrix();

        if (right) {
            scn->data().Tout.rotate(m.inverse());
            scn->data().MyRotate(Eigen::Vector3d(0,1,0), 10 / 180.0f);
            scn->data().Tout.rotate(m);



        } else {
            scn->data().Tout.rotate(m.inverse());
            scn->data().MyRotate(Eigen::Vector3d(0,1,0), -10 / 180.0f);
            scn->data().Tout.rotate(m);

        }

    }

}


IGL_INLINE void Renderer::RotateX(bool up) {
    using namespace Eigen;
    using namespace std;
    if(!scn->meshPicked){
        //scn rotation
        if(up){  scn->MyRotate(Eigen::Vector3d(0,0,1), 10 / 180.0f);}
        else{   scn->MyRotate(Eigen::Vector3d(0,0,1), -10 / 180.0f);}
    }
    else {
        Matrix3d m = scn->data().Tout.rotation().matrix();
        if (up) {
            scn->data().MyRotate(Eigen::Vector3d(0,0,1), 10 / 180.0f);
        } else {
            scn->data().MyRotate(Eigen::Vector3d(0,0,1), -10 / 180.0f);
        }
    }

}

IGL_INLINE void Renderer::toggleIK() {
    if((getSphere()-getButtom()).norm()<=6.4) {
        shouldIK = !shouldIK;

    }

    else
        std::cout<<"can not reach!" << std:: endl;


}

IGL_INLINE void Renderer::animateIK() {
    using namespace Eigen;
    Vector3d sphereCenter=getSphere();
    Vector3d e = getSphere() -( scn->getTip(scn->numOfCylinder-1)+getButtom());// get the tip of the last cylinder
    double d = e.norm();
    std::cout<< "norm is: "<<d <<std::endl;

//    while (d >0.1 && shouldIK){ // TODO change 0.1 to scn->tol
    for(int i = scn->numOfCylinder-1 ; i>-1 ; i--)
    {
        Vector3d er;
        Vector3d d;
        Vector3d rd;
        Vector3d r;
        if(i==0) {
            r = getButtom();
            std::cout<<"buttom is:  " <<r.transpose()<<std::endl;
        } else{
            r=scn->getTip(i-1)+getButtom();
        }

        er=(r-e).normalized();
        rd=(sphereCenter-r).normalized();

//            er.normalize();
//            rd.normalize();
        double distance = (getSphere() - (scn->getTip(scn->numOfCylinder-1)+getButtom())).norm();// get the tip of the last cylinder
        if(distance<0.1){
            shouldIK= false;
            return;
        }
        double alpha=acos(er.dot(rd));
        if(alpha>1)
            alpha=1;
        if(alpha<-1)
            alpha=-1;

        std::cout<<"alfa is: " <<alpha<<std::endl;
        scn->data(i).MyRotate(er.cross(rd).normalized() ,alpha/10);










//            Vector3d v1 =  scn->getNewRotationVector(i);// the Vi vector from the class
//            Vector3d v2 = sphereCenter-dis;
//            v1=v1.normalized() ;
//            v2=v2.normalized();
//            double alpha = acos(v1.dot(v2));
//
//            std::cout<<"alfa is: " <<alpha<<std::endl;
//            scn->data(i).MyRotate(v1.cross(v2).normalized() ,alpha/10);
////            scn->data(i).MyRotate(Eigen::Vector3d(0,1,0),alpha);


        e = getSphere() - (scn->getTip(scn->numOfCylinder-1)+getButtom());// get the tip of the last cylinder

//        }
//        Vector3d dis = getSphere() - scn->getTip(scn->numOfCylinder-1);// get the tip of the last cylinder
//        d = dis.norm();

//        break;
    }


}

void Renderer::printAngle() {
    using namespace std;
    if(!scn->meshPicked){
        //print the scene p t
        cout<< "the pie is "<< asin(scn->MakeTransd()(1,1)) <<endl;
        cout<< "the theta is "<< acos(scn->MakeTransd() (0,0))<<endl;
    }

    else{
        cout<< "the pie of link num "<< scn->selected_data_index<<" is "<< asin(scn->data().MakeTransd()(1,1)) <<endl;
        cout<< "the theta of link num "<< scn->selected_data_index<<" is "<< acos(scn->data().MakeTransd()(0,0)) <<endl;
    }


}



//IGL_INLINE void Viewer::select_hovered_core()
//{
//	int width_window, height_window = 800;
//   glfwGetFramebufferSize(window, &width_window, &height_window);
//	for (int i = 0; i < core_list.size(); i++)
//	{
//		Eigen::Vector4f viewport = core_list[i].viewport;

//		if ((current_mouse_x > viewport[0]) &&
//			(current_mouse_x < viewport[0] + viewport[2]) &&
//			((height_window - current_mouse_y) > viewport[1]) &&
//			((height_window - current_mouse_y) < viewport[1] + viewport[3]))
//		{
//			selected_core_index = i;
//			break;
//		}
//	}
//}

//void Renderer::fabrik() {
//    using namespace std;
//    double target  = scn->data(scn->data_list.size()-1).getLocation; //get the location of the ball
//    vector<double> pos, gap;   //vector of positions
//
//    for (ViewerData  cyl: scn->data_list )
//    {
//        pos.push_back( cyl.location) ; //push the location of the cylyndar
//    }
//    int n = pos.size()-1;  //-1 ??
//    gap.resize(n-1);
//    for(int i =0 ;i<n-1 ; i++){
//        gap[i] = abs(pos[i+1] -pos[i] );
//    }
//
//    double dist = abs(pos[0] - target);
//    double sum = 0;
//    for( double x : pos){
//        sum =+ x;
//    }
//    if (dist >sum ){
//        //target is unreachable
//        vector<double> fromRoot , lamd;
//        lamd.resize(n-1);
//        fromRoot.resize(n-1);
//        for(int i =0 ;i<n-1 ; i++){
//            fromRoot[i] = (abs(target - pos[i]));
//            lamd[i] = (gap[i] /fromRoot[i] );
//            pos[i+1] = (1-lamd[i])*pos[i] +lamd[i]*target;
//        }
//
//
//    }
//
//    else{
//        //target is reachable
//        vector<double> fromRoot , lamd;
//        lamd.resize(n-1);
//        fromRoot.resize(n-1);
//        double b = pos[0];
//        double difA = abs(pos[n-1] - target);
//        while(difA>scn->tol) {
//            pos[n - 1] = target;
//            for (int i = n - 2; i >= 0; i--) {
//                fromRoot[i] = abs(pos[i + 1] - pos[i]);
//                lamd[i] = (gap[i] / fromRoot[i]);
//                pos[i + 1] = (1 - lamd[i]) * pos[i] + lamd[i] * target;
//
//            }
//
//            pos[0] = b;
//            for (int i = 0; i > n - 1; i++) {
//                fromRoot[i] = abs(pos[i + 1] - pos[i]);
//                lamd[i] = (gap[i] / fromRoot[i]);
//                pos[i + 1] = (1 - lamd[i]) * pos[i] + lamd[i] * target;
//
//            }
//            difA = abs(pos[pos.size() - 1] - target);
//            updatePosition(pos);
//
//        }
//    }
//}


//IGL_INLINE void Viewer::select_hovered_core()
//{
//	int width_window, height_window = 800;
//   glfwGetFramebufferSize(window, &width_window, &height_window);
//	for (int i = 0; i < core_list.size(); i++)
//	{
//		Eigen::Vector4f viewport = core_list[i].viewport;

//		if ((current_mouse_x > viewport[0]) &&
//			(current_mouse_x < viewport[0] + viewport[2]) &&
//			((height_window - current_mouse_y) > viewport[1]) &&
//			((height_window - current_mouse_y) < viewport[1] + viewport[3]))
//		{
//			selected_core_index = i;
//			break;
//		}
//	}
//}