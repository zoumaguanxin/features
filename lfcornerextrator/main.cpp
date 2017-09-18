#include <iostream>
#include<fstream>
#include<cstring>
#include<vector>
#include <boost/concept_check.hpp>
#include <boost/graph/graph_concepts.hpp>
#include<cmath>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include<opencv2/imgproc.hpp>
#include<opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/imgcodecs.hpp"

using namespace cv;
using namespace std;


struct LaserScan{
    string frame_id;
    double stamp;
    double angle_min;
    double angle_max;
    double angle_increment;
    vector<float> range;
    int rows, cols;
    
    void track(const pcl::PointXY& beginPt,const pcl::PointXY& endPt,  vector<int>& griddescription) 
	{
	    //判断所有使用点画线算法经过的点被占有的情况，free累积放在的第一个位置，未知放在第二个位置
	   for(vector<int>::iterator it=griddescription.begin();it!=griddescription.end();it++)
	   {
	     *it=0;
	   }
	   
	   /*********************************bresenham algorithm***********************/
	    int deltax=endPt.x-beginPt.x;
	    int deltay=endPt.y-beginPt.y;
	    int x=beginPt.x;
	    int y=beginPt.y;
	    bool flag=0;//用来判断是否改变了主轴，当它等于1时表示主轴方向变为y轴
	    if(abs(deltax)<abs(deltay))
	    {
	      flag=1;
	      int temp;
	      temp=deltax;      deltax=deltay;      deltay=temp;
	      x=beginPt.y;      y=beginPt.x;
	     //cout<<"成功交换主轴"<<endl;
	    }
	    float absslope=abs((float)deltay/(float)deltax);//之前忘记转换为浮点型，结果为0，便划线出现错误
	   // cout<<absslope;
	    float deltaD=absslope-0.5;
	   // cout<<deltaD<<endl;
	    for(int i=0;i<abs(deltax);i++)
	    {
	      if(deltax>0)
	      {
		x+=1;
	      }
	      else
	      {
		x-=1;
	      } 
	      if(deltaD>0)
	      {
		if(deltay>0)
		{
		  y+=1;
		}
		else
		{
		y-=1;
		}
		deltaD-=1;
	      }
	    if(i>=1)  deltaD+=absslope;
	    //cout<<deltaD<<endl;
	    /***************************************************************************/
	    
	    
	   /*************check the usage of  grid: free, unexplored, occupancy**********************/
		   int intensity;
		     float laser_x,laser_y;
		   if(flag==1) 
		    {
		     //reproject the point from frame of image  to polar frame		   
		     laser_x=(y-cols/2)*0.05;
		     laser_y=-(rows/2-x)*0.05;//due to the inversed installation of laser   
		    }
		   else 
		   {
		     laser_x=(x-cols/2)*0.05;
		     laser_y=-(rows/2-y)*0.05;//due to the inversed installation of laser   
		     }
		     float p=sqrt(pow<float>(laser_x,2)+pow<float>(laser_y,2));		   
		     float theta=atan(laser_y/laser_x);
		     if((laser_x<0&&laser_y>0)||(laser_x<0&&laser_y<0))
		     {
			 theta=3.14962+theta;
		      }
		      int index=ceil((theta-angle_min)/angle_increment);
		      /******************************************************************/
		      
		      if(theta<angle_min||theta>angle_max)
		      {
			intensity=205;
		      }
		      else if(p>range[index]+0.05)
		      {
			intensity=205;
		      }
		      else if(p<range[index]+0.1)
		      {
			intensity=254;
		       }
		       else{
			 intensity=0;
		      }
		  switch (intensity)
		    {
		    case 254:
		    {
		      griddescription[0]+=1;
		      break;
		    }
		    case 205:
		    {
		      griddescription[1]+=1;
		      break;
		    }
		    case 0:
		    {
		      griddescription[2]+=1;
		      break;
		    }
		    default:
		      //int  xxx=1;
		    cout<<"the value is unexpected!"<<endl; 
		    }
	/***********************************************************************************/   
	      
	}
}
    
    void ShowPolartoCartian(const vector<int> indexKeypoints, vector<Point>& laserKeypoints)
      {
	    Mat laser=255*Mat::ones(rows,cols,CV_8UC1);//创建全白的一个图像矩阵
	    float angle=angle_min;
	    for(vector<float>::iterator it=range.begin();it!=range.end();it++)
	      {
		    angle+=angle_increment;
		    float x=(*it)*cos(angle);
		    float y=-(*it)*sin(angle); //因为激光被沿着x方向旋转了180度，所以y要取负
		    //地图的坐标系和左手坐标系（x,y）一样,而图像的坐标系原点在左上角，横着的方向为u,竖着的方向为v。
		    int index_x=cols/2+ceil(x/0.05);
		    int index_y=rows/2-ceil(y/0.05);     
		    laser.at<uchar>(index_y,index_x)=0;//因为读取时是按照矩阵坐标系,先行后列。图像坐标下说(u,v)处有个黑点，在矩阵表示就是在u列v行出有个黑点，所以at<uchar>(v,u)。
	      }
	    for( vector<int>::const_iterator t=indexKeypoints.begin();t!=indexKeypoints.end();t++)
	    {
		  float angleKey=angle_min+(*t)*angle_increment;    
		  float x=range[(*t)]*cos(angleKey);
		  float y=-range[*t]*sin(angleKey); //因为激光被沿着x方向旋转了180度，所以y要取负
		  //地图的坐标系和左手坐标系（x,y）一样,而图像的坐标系原点在左上角，横着的方向为u,竖着的方向为v。
		  int index_x=cols/2+ceil(x/0.05);
		  int index_y=rows/2-ceil(y/0.05);
		  laserKeypoints.push_back(Point(index_x,index_y));
		  circle(laser,Point(index_x,index_y),3,Scalar(0));//Point的坐标系是在图像坐标系下面表示的。
	    }
	    Mat img_part=laser(Range(800,1500),Range(800,1500));
	    Mat scaled_img(img_part.rows*2,img_part.cols*2,CV_8UC1);
	    resize(img_part,scaled_img,scaled_img.size(),0,0,CV_INTER_CUBIC);
	    namedWindow("laser",CV_WINDOW_AUTOSIZE);
	    imshow("laser",scaled_img);
      }
    
    void detectCorner(vector<int>& indexKeyPoint)
    {
	size_t i=0;
	/*************************************************************8
	vector<int> continuous_flag;
	for(vector<float>::iterator it=range.begin();it!=range.end()-3;it++)
	{
	    if(i>=1)
	  {
	    if(abs(range[i]-range[i-1])>0.1&&abs(range[i+1]-range[i])>0.1)
	    {
	      continuous_flag.push_back(0);//不连续
	    }
	    else if(abs(range[i]-range[i-1])>0.1&&abs(range[i+1]-range[i])<0.1)
	    {
	      continuous_flag.push_back(1);//右连续
	    }
	    else if(abs(range[i]-range[i-1])<0.1&&abs(range[i+1]-range[i])>0.1)
	    {
	      continuous_flag.push_back(-1);//左连续
	    }
	    else{
	       continuous_flag.push_back(2);//连续
	    }
	  }
	    i++;
	}
	*********************************************************/
	i=0;
	for(vector<float>::iterator it=range.begin()-3;it!=range.end();it++)
	{
	  //cout<<*it<<endl;
	   float d;
	   float curvdiff_last,curvdiff_current;
	  
	  //膨胀一下试一下效果
	  /***********************************************************************************
	  if(i>=2)
	  {
	      curvdiff_last=1/range[i-1]-1/range[i-2];
	      curvdiff_current=1/range[i]-1/range[i-1];
	      //激光源到直线的距离
	      float deltax=range[i-1]*cos(angle_min+(i-1)*angle_increment)-range[i-2]*cos(angle_min+(i-2)*angle_increment);
	      float deltay=range[i-1]*sin(angle_min+(i-1)*angle_increment)-range[i-2]*sin(angle_min+(i-2)*angle_increment);
	     //d=range[i-2]*range[i-1]*sin(2*angle_min+angle_increment*(2*i-3))-pow(range[i-1],2)*sin(2*(angle_min+(i-1)*angle_increment));	
	      d=range[i-2]*range[i-1]*sin(angle_increment)/sqrt(pow(deltax,2)+pow(deltay,2));
	      cout<<d;
	      //threshold必须要随着距离变化而变化,近距离的时候大一些，远距离的时候小一些
	      float threshold=pow(angle_increment,2)/abs(d)+0.05/abs(d);
	      if(abs(curvdiff_current-curvdiff_last)>threshold)
	      {
		  // cout<<abs(curvdiff_current-curvdiff_last)*20;
		    indexKeyPoint.push_back(i-1);
	      }
	   }
	   ********************************************************************************/
	     if(i>=3)
	   {
	      // if(continuous_flag[i]!=0&&continuous_flag[i-2]!=1&&continuous_flag[i-1]!=0&&continuous_flag[i+1]!=-1&&continuous_flag[i+1]!=0)
	       //{
		    curvdiff_last=range[i]-range[i-3];
		    curvdiff_current=range[i+3]-range[i];
		  if(curvdiff_last>0.01*range[i]&&curvdiff_current<-0.01*range[i])
		  {
		    if(indexKeyPoint.size()>1)
		    {
		      vector<int>::iterator it=indexKeyPoint.end();
		      if(*it!=(i-1)&&*it!=(i-2)&&*it!=(i-3))
		      {
			 indexKeyPoint.push_back(i);
		      }
		    }
		   else{
		      indexKeyPoint.push_back(i);
		  }
		    
		  }
		  else if(curvdiff_current>0.01*range[i]&&curvdiff_last<-0.01*range[i])
		  {
		      if(indexKeyPoint.size()>1)
		    {
		      vector<int>::iterator it=indexKeyPoint.end();
		      if(*it!=(i-1)&&*it!=(i-2)&&*it!=(i-3))
		      {
			 indexKeyPoint.push_back(i);
		      }
		    }
		   else{
		      indexKeyPoint.push_back(i);
		  }
		  }
		    /*
		    //激光源到直线的距离
		    float deltax=range[i]*cos(angle_min+i*angle_increment)-range[i-1]*cos(angle_min+(i-1)*angle_increment);
		    float deltay=range[i]*sin(angle_min+i*angle_increment)-range[i-1]*sin(angle_min+(i-1)*angle_increment);
		    //d=range[i-2]*range[i-1]*sin(2*angle_min+angle_increment*(2*i-3))-pow(range[i-1],2)*sin(2*(angle_min+(i-1)*angle_increment));	
		    d=range[i]*range[i-1]*sin(angle_increment)/sqrt(pow(deltax,2)+pow(deltay,2));
		    cout<<"d等于"<<d<<endl;
		    //threshold必须要随着距离变化而变化,近距离的时候大一些，远距离的时候小一些
		    float threshold=pow(angle_increment,2)/abs(d)+1/abs(d);
		    if(abs(curvdiff_current-curvdiff_last)>threshold)
		    {
			// cout<<abs(curvdiff_current-curvdiff_last)*20;
			indexKeyPoint.push_back(i-1);
		    }
		    */
	      // }
	   }

	   i++;
	} 
    }
    void ExGeoDescriptor(vector<Point> laserKeypoints,Eigen::MatrixXf& descriptor)
    {
       const int K=3;//找出三个最近邻居，因为自己必然是第一个，所以它的最近邻居是第二个和第三个
	      vector<int> index(K);
	      vector<float> distance(K);
	      pcl::PointCloud<pcl::PointXY>::Ptr keypoint2d(new pcl::PointCloud<pcl::PointXY>);//注意一定是要申请一个动态指针，不然编译可以通过，但是运行时会报错。
	      
	      for(size_t i=0;i<laserKeypoints.size();i++)
	      {
		  pcl::PointXY temp;
		  temp.x=laserKeypoints[i].x;
		  temp.y=laserKeypoints[i].y;
		  keypoint2d->points.push_back(temp);
	      }
	      pcl::KdTreeFLANN<pcl::PointXY> kdtree;
	      kdtree.setInputCloud(keypoint2d);
	      for(size_t j=0;j<keypoint2d->points.size();j++)
	      {
		  kdtree.nearestKSearch(keypoint2d->points[j],K,index,distance);
		 double d3;
		 vector<int> griddescription(3);
		  d3=pow(abs(keypoint2d->points[index[1]].x-keypoint2d->points[index[2]].x),2)+pow(abs(keypoint2d->points[index[1]].y-keypoint2d->points[index[2]].y),2);
		  track(keypoint2d->points[index[1]],keypoint2d->points[index[2]], griddescription);
		  descriptor.col(j)<<distance[1],distance[2],d3,griddescription[0],griddescription[1],griddescription[2];
	      }      
    }  
 };


int main(int argc, char **argv)
{
     /*********************************read data from file****************************/
      string dir="scan001";
      ifstream file;
      const char *t=dir.c_str();
      file.open(t,ios_base::in);
      LaserScan scan;      
      if(file.good())
      {
	  file>>scan.frame_id>>scan.stamp>>scan.angle_min>>scan.angle_max>>scan.angle_increment;
	  while(file.good())
	  {
	    float temp;       file>>temp;
	    scan.range.push_back(temp);       
	  }
      }
      else
      {
	cout<<"can't open file in"<<dir<<endl;
      }
      /*********************************************************************************/
    scan.cols=2000;
    scan.rows=2000;
     vector<int> indexKeyPoint;
     vector<Point> laserKeypoints;    
     scan.detectCorner(indexKeyPoint);
     scan.ShowPolartoCartian(indexKeyPoint,laserKeypoints);     
     Eigen::MatrixXf laserKeypointsDes(6,indexKeyPoint.size());
     scan.ExGeoDescriptor(laserKeypoints,laserKeypointsDes);
     cout<<laserKeypointsDes;
     waitKey();    
    return 0;
}
