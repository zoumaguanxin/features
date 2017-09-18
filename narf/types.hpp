#include<Eigen/Dense>
#include<Eigen/Geometry>
#include<cstring>
#include <boost/iterator/iterator_concepts.hpp>
using namespace std;
namespace map3d{
  
template <typename PoseT>
struct edge{

  string Header;
  int a_ID;
  int b_ID;
  PoseT pose_ab;
  Eigen::Matrix<float,6,6> informationMatrix_ab;  
};

struct pose
{
    Eigen::Vector3f t;
    Eigen::Quaternion<float> q;   
};
template <typename PoseT>
struct vertix{
  string Header;
  int pose_ID;
  PoseT initialPose;
};
}
