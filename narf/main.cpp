#include <iostream>
#include<pcl/range_image/range_image.h>
#include "fileread.hpp"
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/visualization/range_image_visualizer.h>
#include <boost/graph/graph_concepts.hpp>

bool live_update = false;

void 
setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}

int main(int argc, char **argv) {
   char dir[100]="/home/shaoan/projects/SLAM6D/dat_et4/";

   pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
   
   map3d::readPointCloud(dir,0,target_cloud);
   map3d::readPointCloud(dir,1,source_cloud);
   pcl::PointCloud<pcl::PointXYZ>  scP=*source_cloud;
    //scP.width = (uint32_t) scP.points.size();   
    //scP.height = 1;
   Eigen::Vector4f temp;
   temp<<0,0,0,1;
   Eigen::Quaternionf q(1,0,0,0);
   scP.sensor_origin_=temp;
   scP.sensor_orientation_=q;
    
   float angularResolution=pcl::deg2rad(1.0f);
   //max_angle_width为模拟的深度传感器的水平最大采样角度，
  float maxAngleWidth=pcl::deg2rad (360.f);  // 360.0 degree in radians
  //max_angle_height为模拟传感器的垂直方向最大采样角度  都转为弧度
  float maxAngleHeight=pcl::deg2rad(180.f);  // 180.0 degree in radians
   //传感器的采集位置
  
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
   //深度图像遵循坐标系统
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  float noiseLevel=0.00;    //noise_level获取深度图像深度时，近邻点对查询点距离值的影响水平
  float minRange = 0.0f;     //min_range设置最小的获取距离，小于最小获取距离的位置为传感器的盲区
  int  borderSize = 1;        //border_size获得深度图像的边缘的宽度
  Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
   pcl::RangeImage range_image;
   cout<<" label"<<endl;
   range_image.createFromPointCloud(scP, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  cout<<" label2"<<endl;
  
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor (1, 1, 1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image.makeShared(), 255, 255, 255);  
  //addPointCloud被重载
  viewer.addPointCloud (range_image.makeShared(), range_image_color_handler, "range image");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
  //viewer.addCoordinateSystem (1.0f, "global");
  //PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
  //viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
  viewer.initCameraParameters ();
  //range_image.getTransformationToWorldSystem ()的作用是获取从深度图像坐标系统（应该就是传感器的坐标）转换为世界坐标系统的转换矩阵
  setViewerPose(viewer, range_image.getTransformationToWorldSystem ());  //设置视点的位置
  
  //可视化深度图
  pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
  range_image_widget.showRangeImage (range_image);
  
   while (!viewer.wasStopped ())
  {
    range_image_widget.spinOnce ();
    viewer.spinOnce ();
    pcl_sleep (0.01);
    
    if (live_update)
    {
      //如果选择的是——l的参数说明就是要根据自己选择的视点来创建深度图。
     // live update - update the range image according to the selected view in the 3D viewer.
      scene_sensor_pose = viewer.getViewerPose();
        range_image.createFromPointCloud(scP, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
      range_image_widget.showRangeImage (range_image);
    }
  }


  /*
   pcl::PointCloud<pcl::PointXYZ> pointCloud;   //定义点云的对象
  
  // 循环产生点云的数据
  for (float y=-0.5f; y<=0.5f; y+=0.01f) {
    for (float z=-0.5f; z<=0.5f; z+=0.01f) {
      pcl::PointXYZ point;
      point.x = 2.0f - y;
      point.y = y;
      point.z = z;
      pointCloud.points.push_back(point); //循环添加点数据到点云对象
    }
  }
  pointCloud.width = (uint32_t) pointCloud.points.size();
  pointCloud.height = 1;                                        //设置点云对象的头信息
    //实现一个呈矩形形状的点云
  // We now want to create a range image from the above point cloud, with a 1deg angular resolution
   //angular_resolution为模拟的深度传感器的角度分辨率，即深度图像中一个像素对应的角度大小
  float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
   //max_angle_width为模拟的深度传感器的水平最大采样角度，
  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
  //max_angle_height为模拟传感器的垂直方向最大采样角度  都转为弧度
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
   //传感器的采集位置
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
   //深度图像遵循坐标系统
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  float noiseLevel=0.00;    //noise_level获取深度图像深度时，近邻点对查询点距离值的影响水平
  float minRange = 0.0f;     //min_range设置最小的获取距离，小于最小获取距离的位置为传感器的盲区
  int borderSize = 1;        //border_size获得深度图像的边缘的宽度
  
  pcl::RangeImage rangeImage;
  rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
   // rangeImage.createFromPointCloud(pointCloud);
  
  std::cout << rangeImage << "\n";
   
 pcl::visualization::RangeImageVisualizer  viewer1("3d viewer");
   viewer1.showRangeImage(rangeImage);
   viewer1.setSize(10,10);
  
while(!viewer1.wasStopped())
{
  viewer1.spinOnce();
  pcl_sleep(0.01);
}*/
    return 0;
}
