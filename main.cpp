
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "OurMesh.h"

#include <stdio.h>  /* defines FILENAME_MAX */
// #define WINDOWS  /* uncomment this line to use it for windows.*/
#ifdef WINDOWS
#include <direct.h>
#define GetCurrentDir _getcwd
#else
#include <unistd.h>
#define GetCurrentDir getcwd
#endif
#include<iostream>
#include <igl/opengl/glfw/BetterViewer.h>


std::string GetCurrentWorkingDir( void ) {
    char buff[FILENAME_MAX];
    GetCurrentDir( buff, FILENAME_MAX );
    std::string current_working_dir(buff);
    return current_working_dir;
}
int main(int argc, char *argv[])
{
    Display *disp = new Display(1000, 800, "Wellcome");
    Renderer renderer;
    igl::opengl::glfw::Viewer viewer;
    viewer.init();
    std::string  path = GetCurrentWorkingDir();
    path = path.replace(path.find("cmake-build-debug/tutorial"), strlen("cmake-build-debug/tutorial") , "tutorial/data/");
    int i=0;


    Init(*disp);
    renderer.init(&viewer);
    disp->SetRenderer(&renderer);
    disp->launch_rendering(true);




    delete disp;
}




