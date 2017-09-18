    #include "opencv2/imgcodecs.hpp"
    #include "opencv2/highgui.hpp"
    #include "opencv2/imgproc.hpp"
    #include <iostream>
    using namespace cv;
    using namespace std;
    Mat src, src_gray;
    int thresh = 200;
    int max_thresh = 255;
    const char* source_window = "Source image";
    const char* corners_window = "Corners detected";
    void cornerHarris_demo( int, void* );
    int main( int, char** argv )
    {
      Mat img = imread("/home/shaoan/projects/feature2d/build/map.pgm"); 
      Mat img_part(img, Rect(1500,1500,1000,1000));
      src=img_part.clone();
       cvtColor(src, src_gray, COLOR_BGR2GRAY );//非常重要的一句话，因为imread默认的读入方式是clolor,so it is essiential to invert it into gray.
	  //cout<<img<<endl;
	  //-- Draw keypoints 
       cout<<src_gray;
      namedWindow( source_window, WINDOW_AUTOSIZE );//创建一个窗口，并命名为source_window, 尺寸的大小由图片大小确定
      createTrackbar( "Threshold: ", source_window, &thresh, max_thresh, cornerHarris_demo );//create a trackbar in source_window, the bar will change the value of thresh and result in the change of function of cornerHarris_demo;
      imshow( source_window, src );//在source_window窗口上现实src图像
      cornerHarris_demo( 0, 0 );
      waitKey(0);
      return(0);
    }
    void cornerHarris_demo( int, void* )
    {
      Mat dst, dst_norm, dst_norm_scaled;
      dst = Mat::zeros( src.size(), CV_32FC1 );
      int blockSize = 2;//邻居的范围
      int apertureSize = 3;//扩展的微分算子(Sobel算子)的模板大小为3*3
      double k = 0.04;//parameter of harris, usually it varies from 0.04 to 0.05
      cornerHarris( src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT );//角点检测，和keypoint不同，dst保存的是原图像是每一个点的检测是否为角点的值，输出到dst中
      normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
      convertScaleAbs( dst_norm, dst_norm_scaled );
      for( int j = 0; j < dst_norm.rows ; j++ )
	{
	  for( int i = 0; i < dst_norm.cols; i++ )
	      {
		if( (int) dst_norm.at<float>(j,i) > thresh )
		  {
		  circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
		  }
	      }
	}
	//cout<<dst.at<int>(1,2);
      namedWindow( corners_window, WINDOW_AUTOSIZE );
      imshow( corners_window, dst_norm_scaled );
    }