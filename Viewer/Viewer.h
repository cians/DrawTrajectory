
#pragma once
#include <stdio.h>
#include "Eigen/Eigen"
#include <pangolin/pangolin.h>
#include <boost/graph/graph_concepts.hpp>
#include <unistd.h>
using namespace std;
using namespace pangolin;

class Viewer
{
public:
    //pangolin relate
    OpenGlRenderState* s_cam;
    Handler3D *handler3d;
    View d_cam;
    //data
   vector< vector<Eigen::Vector3d> > trajsss;
    
    //odometry paras
     int TimeCounter;
    // int shutdownCounter;
    // void color2rgb(string color, Eigen::Matrix<GLubyte,3,1> &GLColor);
    void drawCameraTrajectory();
    void onDraw();
   // void Viewer::drawLine(Eigen::Vector3f M0, Eigen::Vector3f M1);
    void Run();
    Viewer( vector< vector<Eigen::Vector3d> > &trajs);
};

// void Viewer::color2rgb(string color,Eigen::Matrix<GLubyte,3,1> &RGB)
// {
//     if (color == "red")
//     {
//       RGB(0) = 255;
//       RGB(1) = 0;
//       RGB(2) = 0;
//     }
//     if (color == "green")
//     {
//       RGB(0) = 0;
//       RGB(1) = 255;
//       RGB(2) = 0;
//     }
//     if (color == "yellow")
//     {
//       RGB(0) = 255;
//       RGB(1) = 255;
//       RGB(2) = 0;
//     }
//     if (color == "other")
//     {
//       RGB(0) = 0;
//       RGB(1) = 0;
//       RGB(2) = 0;
//     }
// }
 
// void Viewer::drawLine(Eigen::Vector3f M0, Eigen::Vector3f M1)
//     {
//       glLineWidth(2.0);
//       glColor4ub(123, 104, 238, 255);
//       glBegin(GL_LINES);
//       glVertex3f(M0(0), M0(1), M0(2));
//       glVertex3f(M1(0), M1(1), M1(2));
//       glEnd();
//     }
    //draw camera trajectory and plane
// void Viewer::drawCameraModel(int TimeCounter)
//     {
//       Eigen::Vector3f Cam_center = maps.mv_cameras[TimeCounter].m_pose.center;
//       Eigen::Matrix3f Cam_M = maps.mv_cameras[TimeCounter].m_pose.m_rotation;
//       //rotation的逆才是 transform matrix!
//       Eigen::Vector3f Cam_R = Cam_M.row(0) / Cam_M.row(0).norm();
//       Eigen::Vector3f Cam_UP = -Cam_M.row(1) / Cam_M.row(1).norm();
//       Eigen::Vector3f Cam_BK = -Cam_M.row(2) / Cam_M.row(2).norm();
//       Eigen::Vector3f Model_center;
//       Model_center = Cam_center - Model_Len *Cam_BK / Cam_BK.norm();

//       Eigen::Vector3f M00, M01, M11, M10;
//       M00 = Model_center - Model_Xoffset*Cam_R - Model_Yoffset*Cam_UP;
//       M01 = Model_center - Model_Xoffset*Cam_R + Model_Yoffset*Cam_UP;
//       M11 = Model_center + Model_Xoffset*Cam_R + Model_Yoffset*Cam_UP;
//       M10 = Model_center + Model_Xoffset*Cam_R - Model_Yoffset*Cam_UP;

//       drawLine(M00, M01);
//       drawLine(M01, M11);
//       drawLine(M11, M10);
//       drawLine(M10, M00);
//       drawLine(M10, M00);
//       drawLine(M10, Cam_center);
//       drawLine(M11, Cam_center);
//       drawLine(M00, Cam_center);
//       drawLine(M01, Cam_center);
//     }
    
// maps is input,other are output
  
void Viewer::drawCameraTrajectory()
    {
      glPointSize(2.0);
      glColor4ub(0,0,255,255);
      for(int j =0; j < trajsss.size(); j++)
      {
	auto trajectory = trajsss[j];
	if(j%2==0)
		glColor4ub(255,0,0,255);
	else
		glColor4ub(0,0,255,255);
        if(TimeCounter != trajectory.size())
          TimeCounter = trajectory.size();
        for(int i = 0; i < TimeCounter;i++)
            {
              auto v = trajectory[i];       
              glBegin(GL_POINTS);
              glVertex3dv(v.data());
              glEnd();
            }

      }
      
    }
    
void Viewer::onDraw()
    {
      sleep(0.2);
      TimeCounter++;
      drawCameraTrajectory();
     // cout<<s_cam->GetModelViewMatrix()<<'\n';
    }
void Viewer::Run()
  {
    while(!ShouldQuit())
    {
       
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(*s_cam);
        onDraw();
      	FinishFrame();
    }
  }

Viewer::Viewer( vector< vector<Eigen::Vector3d> > &trajs)
  {
    trajsss = trajs;
    int win_width = 1200;
    int win_height = 800;
    TimeCounter =0;
    //for viewer
    CreateWindowAndBind("Viewer",win_width,win_height);
    glEnable(GL_DEPTH_TEST);
    //Generate glulookat style model view matrix, looking at (lx,ly,lz) Assumes forward is -z and up is +y
    s_cam = new OpenGlRenderState(
      ProjectionMatrix(win_width,win_height,-2000,2000,-2000,2000,-20000,6000),
      ModelViewLookAt(0, 0, -2, 0, 0, 0, AxisY));
   // s_cam->SetModelViewMatrix(ModelViewMat);
    handler3d = new Handler3D( *s_cam);
    d_cam = CreateDisplay().SetBounds(0.0,1.0,0.0,1.0,(-1.0f*win_width)/win_height).SetHandler(handler3d);
  }

