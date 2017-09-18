  #include <opencv2/core/core.hpp> 
  #include <opencv2/highgui/highgui.hpp> 
  #include <opencv2/imgproc/imgproc.hpp> 
  #include <opencv2/features2d/features2d.hpp>
  #include "opencv2/imgcodecs.hpp"
  #include<iostream>
  #include<fstream>
  #include<ctime>
  
#include <boost/graph/graph_concepts.hpp>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>

  //#include <boost/thread/thread.hpp>


  using namespace cv; 
  using namespace std;
  
  namespace track{
	void track(const pcl::PointXY& beginPt,const pcl::PointXY& endPt,  const Mat& src_img, Mat& label_img, vector<int>& griddescription) 
	//void track(const pcl::PointXY& beginPt,const pcl::PointXY& endPt,  const Mat& src_img,vector<int>& griddescription) 
	{
	    //判断所有使用点画线算法经过的点被占有的情况，free累积放在的第一个位置，未知放在第二个位置
	  //Mat label_img=src_img.clone();
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
	      cout<<"成功交换主轴"<<endl;
	    }
	    float absslope=abs((float)deltay/(float)deltax);//之前忘记转换为浮点型，结果为0，便划线出现错误
	    cout<<absslope;
	    float deltaD=absslope-0.5;
	    cout<<deltaD<<endl;
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
	    cout<<deltaD<<endl;
	    /***************************************************************************/
	    
	    
	   /*************check the usage of  grid: free, unexplored, occupancy**********************/
		   int intensity;
		   if(flag==1)  {intensity=src_img.at<uchar>(x,y);  cout<<"ready to depict point in ("<<y<<", "<<x<<")"<<endl;circle(label_img, Point(y,x),5,Scalar(0));}//之前uchar为int就不行
		   else {intensity=src_img.at<uchar>(y,x);    cout<<"ready to depict point in ("<<x<<", "<<y<<")"<<endl;circle(label_img, Point(x,y),5,Scalar(0));}
		   cout<<intensity<<endl;
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
	
	
	/************************** show trace**********************************
	  namedWindow( "tracker", WINDOW_AUTOSIZE );
          imshow( "tracker", label_img );
        /*********************************************************************/
  }
  }
  
  class geometryDescriptorExtrator{
  public:
	Mat image;
	vector<KeyPoint> keypoints;
	//注意构造函数前面不要加void
	geometryDescriptorExtrator(const Mat& src_img, const vector<KeyPoint>& src_keypoints){
	  image=src_img.clone();
	  keypoints=src_keypoints;
	}
	void compute(Eigen::MatrixXf& descriptor)
	{   //compute discriptors. save keypoints as point cloud style and search of neighborhood of each point using k-d tree 
	      const int K=3;//找出三个最近邻居，因为自己必然是第一个，所以它的最近邻居是第二个和第三个
	      vector<int> index(K);
	      vector<float> distance(K);
	      Mat label_img=image.clone();
	      pcl::PointCloud<pcl::PointXY>::Ptr keypoint2d(new pcl::PointCloud<pcl::PointXY>);//注意一定是要申请一个动态指针，不然编译可以通过，但是运行时会报错。
	      for(size_t i=0;i<keypoints.size();i++)
	      {
		  pcl::PointXY temp;
		  temp.x=keypoints[i].pt.x;
		  temp.y=keypoints[i].pt.y;
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
		 track::track(keypoint2d->points[index[1]],keypoint2d->points[index[2]], image,label_img,griddescription);
		   //track::track(keypoint2d->points[index[0]],keypoint2d->points[index[1]], image,griddescription);
		  descriptor.col(j)<<distance[1],distance[2],d3,griddescription[0],griddescription[1],griddescription[2];
	      }
	  namedWindow( "tracker", WINDOW_AUTOSIZE );
          imshow( "tracker", label_img);
	}
  };

  int main(int argc, char** argv) 
  { 
      Mat img = imread("map.pgm",CV_8UC1); 
      Mat img1=img.clone();
      // cvtColor( img, img1, COLOR_BGR2GRAY );//非常重要的一句话，因为imread默认的读入方式是clolor,so it is essiential to invert it into gray.
     // cout<<img<<endl;
      vector<KeyPoint> keypoints; 
      //-- Draw keypoints 
      Mat img_keypoints;    
     // Mat img_part(img1, Rect(1500,1500,1000,1000));
      Mat img_part=img1(Range(1500,2500),Range(1500,2500));
      //imshow("initial picture", img);
     // cout<<img_part.at<uchar>(2000,2000)<<endl;
      time_t t_begin,t_end;
      t_begin=clock();//注意clock得到的是时钟的周期数，是一个整数，1ms为一个周期，CLOCKS_PER_SEC为一秒钟的周期数，值为1000；
      FAST(img_part,keypoints,100);//问题，KeyPoints的坐标系指图像坐标下，还是图像的矩阵坐标系？这里应该指的是图像坐标系下的点。因为这样刚好画出的是对的。
      //cout<<keypoints[1].pt.x;
      drawKeypoints(img_part,keypoints,img_keypoints,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
      imshow("feature in RIO", img_keypoints);
      
     /************************check the bresenham algorithm*****************
     pcl::PointCloud<pcl::PointXY>::Ptr line(new pcl::PointCloud<pcl::PointXY>);      
     pcl::PointXY linebegin, linend;
     linebegin.x=800;linebegin.y=5;
     linend.x=500;linend.y=800;
     line->points.push_back(linebegin);
     line->points.push_back(linend);
     track::track(line->points[0],line->points[1],img_part,griddescription);
     /********************************************************************/
      geometryDescriptorExtrator GDE(img_part,keypoints);
      Eigen::MatrixXf discriptors(6,keypoints.size());
      GDE.compute(discriptors); 
      t_end=clock();
      cout<<double(t_end-t_begin)/CLOCKS_PER_SEC<<endl;
      cout<<discriptors<<endl;
 
     
     /********************************************************** */
      waitKey(0); 
      return 0; 
  }