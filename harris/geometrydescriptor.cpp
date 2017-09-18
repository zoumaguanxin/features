#include <opencv2/core/core.hpp> 
  #include <opencv2/highgui/highgui.hpp> 
  #include <opencv2/imgproc/imgproc.hpp> 
  #include <opencv2/features2d/features2d.hpp>
  #include "opencv2/imgcodecs.hpp"
  #include<iostream>
  #include<fstream>
  
#include <boost/graph/graph_concepts.hpp>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

  //#include <boost/thread/thread.hpp>


  using namespace cv; 
  using namespace std;
  
  namespace track{
	void track(pcl::PointXY initalposization,pcl::PointXY endposiztion, vector<int> griddescription[3]) 
	{
	    //判断所有使用点画线算法经过的点被占有的情况，free累积放在的第一个位置，未知放在第二个位置
	  
	}
      
  }
  
  class geometryDescriptorExtrator{
  public:
	Mat image;
	vector<KeyPoint> keypoints;
	/*
	void geometryDescriptorExtrator(Mat src_img,  vector<KeyPoint> src_keypoints){
	  image=src_img.clone();
	  keypoints(src_keypoints);
	}*/
	void compute(Mat src_img,  vector<KeyPoint> src_keypoints, Mat descriptor)
	{
	      const int K=2;
	      vector<int> index(K);
	      vector<float> distance(K);
	      pcl::PointCloud<pcl::PointXY>::Ptr keypoint2d;	
	      for(size_t i=0;i<src_keypoints.size();i++)
	      {
		  pcl::PointXY temp;
		  temp.x=src_keypoints[i].pt.x;
		  temp.y=src_keypoints[i].pt.y;
		  keypoint2d->points.push_back(temp);
	      }
	      pcl::KdTreeFLANN<pcl::PointXY> kdtree;
	      kdtree.setInputCloud(keypoint2d);
	      for(size_t j=0;j<keypoint2d->points.size();j++)
	      {
		  kdtree.nearestKSearch(keypoint2d->points[j],K,index,distance);
		 double d3;
		  d3=pow(abs(keypoint2d->points[index[0]].x-keypoint2d->points[index[1]].x),2)+pow(abs(keypoint2d->points[index[0]].y-keypoint2d->points[index[1]].y),2);
		  //descriptor.col(j)<<distance[0],distance[1];
	      }
	     
	}
	
  };