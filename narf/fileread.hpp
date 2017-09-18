  #include<pcl/common/file_io.h>
  #include<pcl/point_cloud.h>
  #include<vector>
  #include<ctime>
  #include<fstream>
  #include<iostream>
 #include<cassert>
  
#include<Eigen/Dense>
#include<Eigen/Geometry>
#include <pcl/common/common_headers.h>
#include "types.hpp"

using namespace std;

 istream & operator >> (istream &in, Eigen::Quaternion<float> &s)
 {
   float x,y,z,w;
   in>>x>>y>>z>>w;
   Eigen::Quaternion<float> q(w,x,y,z);
   s=q;
   
   /*
   if(!in)
   {
     s=Eigen::Quaternion<float>::Quaternion();
  }
  */
   return in;   
}
istream& operator>>(istream& in, Eigen::Vector3f &s)
{
  in>>s(0)>>s(1)>>s(2);
  /*
  if(!in)
  {
    s=Eigen::Vector3f::vector();
  }
  */
  return in;
}

  namespace map3d
  {
      /**
       * \brief This function deals with files named like scan001.pose and scan001.3d according to related style. Any other style file will be illegal. 
       * \param[in] dir is the diretory which store the files,
       *\param[in] nfile is the number of current reading file. 
       * \param[out] cloudPtr is current read point_cloud. 
       * \param[out]  R_odometry  is a initial pose for registering current point cloud into global point cloud. 
       * \param[out]  t_odometry is a initial pose for registering current point cloud into global point cloud.
      */
      void readfile(const char * dir, const int nfile,pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr, Eigen::Matrix3f &R_odometry, Eigen::Vector3f &t_odometry)
      {
	char poseFileName[50],pointFileName[50];
	string mode="3d";
	//sprintf是把后面格式的文字输出到缓存，这里poseFileName是缓存的地址,.3d表示了
	sprintf(poseFileName,"%sscan%.3d.pose",dir,nfile);
	sprintf(pointFileName,"%sscan%.3d.3d",dir,nfile);
	cout<<"reading pose from "<<poseFileName<<endl;
	ifstream is(poseFileName,ios_base::in);
	double rpos[3],rposTheta[3];//用来存放pose
	if(is.good())
	{
		if(mode=="3d")
		{
		for(unsigned int i=0;i<3;is>>rpos[i++]);
		for(unsigned int i=0;i<3;is>>rposTheta[i++]);
		}
		else
		{
			//***********************************
			//由于是在平面上，且在左手坐标系下，只有pith角度保留，且，y方向上平移设置为0
			//************************************
			for(unsigned int i=0;i<3;i++)
			{
			  if(i==1)
			  {
			    is>>rpos[i];
			    rpos[i]=0;
			  }
			  else{
			    is>>rpos[i];
			  }
			}
			
			for(unsigned int i=0;i<3;i++)
			{
			if(i!=1)
			{
			  is>>rposTheta[i]; 
			}
			else
			{
			  is>>rposTheta[i];
			  rposTheta[i]=0;
			}
			}	
		      }
	       }
	else{cout<<"ERROR: Cannot open file 'scan000.pose'."<<endl;}
	is.close();
	//显示读取的结果
	for(unsigned int i=0;i<3;cout<<rpos[i++]<<" ");cout<<endl;
	for(unsigned int i=0;i<3;cout<<rposTheta[i++]<<" ");cout<<endl;

	//*************************************************
	//欧拉角转换为旋转矩阵,注意Eigen使用的角度均为弧度
	//*************************************************
	t_odometry<<rpos[0],rpos[1],rpos[2];
	R_odometry=Eigen::AngleAxisf(pcl::deg2rad(rposTheta[2]), Eigen::Vector3f::UnitZ())
			    * Eigen::AngleAxisf(pcl::deg2rad(rposTheta[1]), Eigen::Vector3f::UnitY())
			    * Eigen::AngleAxisf(pcl::deg2rad(rposTheta[0]), Eigen::Vector3f::UnitX());
	cout<<"current Rotation Matix is equal to:"<<endl
	      <<R_odometry<<endl
	      <<"the tranlate data from odometry"<<endl
	      <<t_odometry<<endl;

	  //打开点云文件读取点云数据
	  ifstream file(pointFileName,ios_base::in);
	  if(file.good())
	  {
	  //******************
	  //读取文件的初始信息
	  //******************
	  int n;
	  double size1;file>>size1;cout<<size1;
	  char xxx;
	  file>>xxx;
	  cout<<xxx;double size2;
	  file>>size2;cout<<size2<<endl;
	  n=size1*size2;
	  //**********************************
	  //读取点云
	  //**********************************
	  float r_filter=1000;
	  while(1)
	  {
	      if(file.good())
	      {
		  pcl::PointXYZ point;
		  float temp;
		  file>>point.x;file>>point.y;file>>point.z;file>>temp;
		  //cout<<cloud.x<<" "<<cloud.y<<" "<<cloud.z<<" "<<i<<endl;
		  float distance=sqrt(pow(point.x,2)+pow(point.y,2)+pow(point.z,2));
		  if(distance<=r_filter)
		  {
		    cloudPtr->points.push_back(point);
		  }
	      }
	      else
	      {
		cout<<"file were read completely"<<endl;break;
	      }
	    }
	  }
      }
      
      /**
       * \brief this function is used to read optimized poses. This class file are usually named as poses_optimized.txt. Ohter types file will be illegal.
       * \param[in] dir is the diretory that poses_optimized locate
       * \param[out] R_optimized is the Rotation Matix transformed from quaternion
       * \param[out] t_optimized is the translation
       */
      void readg2ofile(const char * dir, vector<map3d::pose> &pose_optimizied)
      {
	char poseFileName[255];
	sprintf(poseFileName,"%sposes_optimized.txt",dir);
	ifstream file;
	file.open(poseFileName,ios_base::in);

	if(file.good())
	{
	 while(file.good())
	 {
	  map3d::pose tempose;
	  int id;
	  file>>id;
	  file>>tempose.t;
	  file>>tempose.q;
	  pose_optimizied.push_back(tempose);
	 }
	}
	else{
	  cout<<"can not open file name poses_optimized.txt in "<<dir<<endl; 
	  assert(file.good());
      }
      }
      
      /**
       * \brief read poin cloud from the file named as scanNum.3d
       * \param[in] dir
       * \param[out] cloudPtr 
       */
      
      void readPointCloud(const char* dir, int nfile,pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr)
      {
	char pointFileName[255];
	sprintf(pointFileName,"%sscan%.3d.3d",dir,nfile);
	ifstream file;
	file.open(pointFileName,ios_base::in);
	if(file.good())
	  {
	  //******************
	  //读取文件的初始信息
	  //******************
	  int n;
	  double size1;file>>size1;cout<<size1;
	  char xxx;
	  file>>xxx;
	  cout<<xxx;double size2;
	  file>>size2;cout<<size2<<endl;
	  n=size1*size2;
	  //**********************************
	  //读取点云
	  //**********************************
	  float r_filter=1000;
	  while(1)
	  {
	      if(file.good())
	      {
		  pcl::PointXYZ point;
		  float temp;
		  file>>point.x;file>>point.y;file>>point.z;file>>temp;
		  //cout<<cloud.x<<" "<<cloud.y<<" "<<cloud.z<<" "<<i<<endl;
		  float distance=sqrt(pow(point.x,2)+pow(point.y,2)+pow(point.z,2));
		  if(distance<=r_filter)
		  {
		    cloudPtr->points.push_back(point);
		  }
	      }
	      else
	      {
		cout<<"file were read completely"<<endl;break;
	      }
	    }
	  }
      }
}