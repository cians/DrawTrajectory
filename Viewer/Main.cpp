#include "Viewer.h"
#include <fstream>
int ReadConfig(int argc, char** argv,
               std::vector<std::string>& file_names)
{
    //Load input arguments
    std::string file_name;
    if (argc > 1)
    {
        for (int i = 1; i < argc; ++i)
        {
            file_name = argv[i];
            file_names.push_back(file_name);
        }
    }
    else
    {
        //default act for test
#ifdef _WIN32
        file_name = "./result.act";
#else
        file_name = "./images/test.act";
#endif
        file_names.push_back(file_name);
    }
    return 0;
}

void ReadFiles(std::vector<std::string> &files, vector< vector<Eigen::Vector3d> > &trajs)
{
    
    for(auto filePath : files)
      {
        ifstream f(filePath);
        if(!f)
            cerr<<"can not open the file\n";
        vector<Eigen::Vector3d> trajectory;
        double data[12];
        Eigen::Matrix<double,3,4> Pose;
        Eigen::Vector3d center;
        int tm;
	    bool printed = false;
        while(!f.eof())
        {
            string line;
            getline(f,line);
          if(sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &data[0],&data[1],&data[2],&data[3],&data[4],&data[5],
                                                               &data[6],&data[7],&data[8],&data[9],&data[10],&data[11]) == 12)
          {
	          if(!printed)
                {
                    printf("found 12 paras, take it as transform matrix\n");
                    printed = true;
                }
             Pose<<data[0],data[1],data[2],data[3],
                  data[4],data[5],data[6],data[7],
                data[8],data[9],data[10],data[11];
             center = -Pose.block<3,3>(0,0).transpose()*Pose.block<3,1>(0,3);
             center = Eigen::Vector3d(data[3],data[7],data[11]);
             trajectory.push_back(center);
             cout<<center.transpose()<<endl;
          }
          else  if(sscanf(line.c_str(), "%d %lf %lf %lf %lf %lf %lf %lf", &tm, &data[0],&data[1],&data[2],
                                    &data[3],&data[4],&data[5],&data[6]) == 8)
                                    //q0,     q1,      q2 ,     q3
          {
	         if(!printed)
                 {
                printf("found 7 paras, take it as T + quaternion\n");
                printed = true;
                }
             //quaternion ->R
             printf("%d    ",tm);
             Pose(0,0) = 1 - 2*data[5]*data[5] - 2*data[6]*data[6];
             Pose(0,1) = 2*data[4]*data[5] + 2*data[3]*data[6];
             Pose(0,2) = 2*data[4]*data[6] - 2*data[3]*data[5];
             Pose(1,0) = 2*data[4]*data[5] - 2*data[3]*data[6];
             Pose(1,1) = 1 - 2*data[4]*data[4] - 2*data[6]*data[6];
             Pose(1,2) = 2*data[5]*data[6] + 2*data[3]*data[4];
             Pose(2,0) = 2*data[4]*data[6] + 2*data[3]*data[5];
             Pose(2,1) = 2*data[5]*data[6] - 2*data[3]*data[4];
             Pose(2,2) = 1 - 2*data[4]*data[4] - 2*data[5]*data[5];
             //T
             Pose(0,3) = data[0];
             Pose(1,3) = data[1];
             Pose(2,3) = data[2];
             center = -Pose.block<3,3>(0,0).transpose()*Pose.block<3,1>(0,3);
             center = Eigen::Vector3d(data[0],data[1],data[2]);
             trajectory.push_back(center);
             cout<<center.transpose()<<endl;
          }
          else
             {
                 cerr<<"not defined file format!\n";
                 break;
             }
        }
       
 	    printf("drawed %d points\n",trajectory.size());
          trajs.push_back(trajectory);
      }

}


int main(int argc, char** argv)
{
    //Load input arguments
    vector<string> file_names;
    ReadConfig(argc, argv, file_names);
    printf("found %d files\n",file_names.size());

    vector< vector<Eigen::Vector3d> > trajs;
    ReadFiles(file_names,trajs);


    //opengl window to show our map or trajectory
    Viewer my_viewer(trajs);//check to remove const int parameters
    my_viewer.Run();
    return 0;
}
