#include "nano_test.h"

ros::Subscriber tag_sub;
ros::Subscriber link_sub;
ros::Subscriber fin_sub;

bool init = true;
bool finished = false;
int first= 0;
int quat_count = 20;
int count = 80;

//parcour 4
// const int NUMBER_OF_TAGS = 4;
// // 
//   int OFFSET = 2;
//   int CAMERA_OFFSET = 10;

// parcour 3
const int NUMBER_OF_TAGS = 35;

int OFFSET = 3;
int CAMERA_OFFSET = 43;


//parcour 2
// const int NUMBER_OF_TAGS = 24;

//   int OFFSET = 3;
//   int CAMERA_OFFSET = 31;

//parcour 1
// const int NUMBER_OF_TAGS = 16;

//   int OFFSET = 2;
//   int CAMERA_OFFSET = 22;


//updated camera 
Eigen::Vector3f camera_vec;
Eigen::Quaternionf camera_quat;

//collect tag in world properties
Eigen::Vector3f tags[NUMBER_OF_TAGS];
Eigen::Quaternionf quats[NUMBER_OF_TAGS];

//collect over time
std::list<Eigen::Vector3f> found_vec[NUMBER_OF_TAGS][2];
std::list<float> abs_dist[NUMBER_OF_TAGS];
std::list<float> difference_m[NUMBER_OF_TAGS];

std::list<Eigen::Quaternionf> found_quat[NUMBER_OF_TAGS][2];
std::list<float> difference_deg[NUMBER_OF_TAGS];



//manage camerafound tags
void tagCb(const apriltags_ros::AprilTagDetectionArray::ConstPtr &found_tags)
{
  if(!finished){

for (int i = 0; i < found_tags->detections.size(); i++)
{
  //collect vector & distance
  Eigen::Vector3f tag_world_vec = tags[found_tags->detections[i].id] - camera_vec;
  
  //store actual world pos
  found_vec[found_tags->detections[i].id][0].push_back(tag_world_vec);

  //x,y,z from tag
  Eigen::Vector3f tag_vec(found_tags->detections[i].pose.pose.position.x, found_tags->detections[i].pose.pose.position.y, found_tags->detections[i].pose.pose.position.z);
  
  //store detected tag
  found_vec[found_tags->detections[i].id][1].push_back(tag_vec);



  // calculate distance of tag, real and measured
  float tag = sqrt(pow(tag_world_vec[0], 2) + pow(tag_world_vec[1], 2) + pow(tag_world_vec[2], 2));
  float dist = sqrt(pow(tag_vec[0], 2) + pow(tag_vec[1], 2) + pow(tag_vec[2], 2));

//store difference
  difference_m[found_tags->detections[i].id].push_back(tag - dist);
  abs_dist[found_tags->detections[i].id].push_back(dist);
  if(first)
  {
std::cout << "dist tag " << found_tags->detections[i].id << " = "  << tag - dist << "\n";
first--;
  }
  

//---------------------------------------------------------------------------------------------------------------
//compare quaternions

//get quat from camera
Eigen::Quaternionf tag_quat(found_tags->detections[i].pose.pose.orientation.w, found_tags->detections[i].pose.pose.orientation.x, found_tags->detections[i].pose.pose.orientation.y, found_tags->detections[i].pose.pose.orientation.z);

// create translationMatrix for camera
  Eigen::Matrix4f world_to_camera_trans = Eigen::Matrix4f::Identity();
  Eigen::Matrix3f cq = camera_quat.toRotationMatrix();
  world_to_camera_trans(0,0) = cq(0,0);
  world_to_camera_trans(0,1) = cq(0,1);
  world_to_camera_trans(0,2) = cq(0,2);
  world_to_camera_trans(1,0) = cq(1,0);
  world_to_camera_trans(1,1) = cq(1,1);
  world_to_camera_trans(1,2) = cq(1,2);
  world_to_camera_trans(2,0) = cq(2,0);
  world_to_camera_trans(2,1) = cq(2,1);
  world_to_camera_trans(2,2) = cq(2,2);
  world_to_camera_trans(0,3) = camera_vec[1];
  world_to_camera_trans(1,3) = camera_vec[0];
  world_to_camera_trans(2,3) = camera_vec[2];

Eigen::Matrix4f world_to_camera_rot = Eigen::Matrix4f::Identity();
  Eigen::Matrix3f cqr = camera_quat.toRotationMatrix();
  world_to_camera_rot(0,0) = cqr(0,0);
  world_to_camera_rot(0,1) = cqr(0,1);
  world_to_camera_rot(0,2) = cqr(0,2);
  world_to_camera_rot(1,0) = cqr(1,0);
  world_to_camera_rot(1,1) = cqr(1,1);
  world_to_camera_rot(1,2) = cqr(1,2);
  world_to_camera_rot(2,0) = cqr(2,0);
  world_to_camera_rot(2,1) = cqr(2,1);
  world_to_camera_rot(2,2) = cqr(2,2);

Eigen::Matrix4f world_to_camera_move = Eigen::Matrix4f::Identity();
  world_to_camera_move(0,3) = camera_vec[1];
  world_to_camera_move(1,3) = camera_vec[0];
  world_to_camera_move(2,3) = camera_vec[2];

//correct for camera coordinates
// Eigen::Matrix2f correct;
// correct <<  1, 0, 0,
//             0, 1, 0,
//             0, 0, 1;

// translate tag position into camera
// Eigen::Vector4f tag_pos_in_world(tags[found_tags->detections[i].id](0), tags[found_tags->detections[i].id](1), tags[found_tags->detections[i].id](2), 1);
// Eigen::Vector4f tag_vec4(tag_vec[0], tag_vec[1], tag_vec[2], 1);
// Eigen::Vector4f tag_pos_in_cam_calc = world_to_camera_trans * tag_vec4;
// Eigen::Vector4f reverse = (world_to_camera_move * world_to_camera_rot).inverse() * tag_pos_in_world;

  // std::cout << "Here is tag cam" << "\n" <<  tag_pos_in_cam_calc << "\n";
  // std::cout << "Here is reverse" << "\n" <<  reverse   << "\n";
//create translationmatrix for tag in worldcoordinates
  Eigen::Matrix4f world_to_tag_trans = Eigen::Matrix4f::Identity();
  Eigen::Matrix3f qr = quats[found_tags->detections[i].id].toRotationMatrix();
  world_to_tag_trans(0,0) = qr(0,0);
  world_to_tag_trans(0,1) = qr(0,1);
  world_to_tag_trans(0,2) = qr(0,2);
  world_to_tag_trans(1,0) = qr(1,0);
  world_to_tag_trans(1,1) = qr(1,1);
  world_to_tag_trans(1,2) = qr(1,2);
  world_to_tag_trans(2,0) = qr(2,0);
  world_to_tag_trans(2,1) = qr(2,1);
  world_to_tag_trans(2,2) = qr(2,2);
  world_to_tag_trans(0,3) = tags[found_tags->detections[i].id](0);
  world_to_tag_trans(1,3) = tags[found_tags->detections[i].id](1);
  world_to_tag_trans(2,3) = tags[found_tags->detections[i].id](2);

//create trnaslationmatrix for visual tag
Eigen::Matrix4f camera_to_tag_trans = Eigen::Matrix4f::Identity();
Eigen::Matrix3f qcr = tag_quat.toRotationMatrix();
camera_to_tag_trans(0,0) = qcr(0,0);
camera_to_tag_trans(0,1) = qcr(0,1);
camera_to_tag_trans(0,2) = qcr(0,2);
camera_to_tag_trans(1,0) = qcr(1,0);
camera_to_tag_trans(1,1) = qcr(1,1);
camera_to_tag_trans(1,2) = qcr(1,2);
camera_to_tag_trans(2,0) = qcr(2,0);
camera_to_tag_trans(2,1) = qcr(2,1);
camera_to_tag_trans(2,2) = qcr(2,2);
camera_to_tag_trans(0,3) = tags[found_tags->detections[i].id](0);
camera_to_tag_trans(1,3) = tags[found_tags->detections[i].id](1);
camera_to_tag_trans(2,3) = tags[found_tags->detections[i].id](2);


Eigen::Quaternionf correction(1,0,0,0);  
Eigen::Quaternionf test = quats[found_tags->detections[i].id].inverse() * camera_quat * correction;

  if(quat_count-- > 0)
  {
std::cout << "quat_dist/dist " << found_tags->detections[i].id << " : " << test.inverse().angularDistance(tag_quat) << " : " << dist << "\n";// << "," << test.x() << "," <<  test.y()  << "," << test.z()  << "\n";
  }


}
}
}

//update camera position
void posCb(const gazebo_msgs::LinkStates::ConstPtr &msg)
{
  if(true)
  {
  //for(int i = OFFSET; i <= NUMBER_OF_TAGS + OFFSET ; i++)
  for(int i = OFFSET; i < NUMBER_OF_TAGS + OFFSET ; i++)
  {
 


    tags[i - OFFSET][0] = msg->pose[i].position.x;
    tags[i - OFFSET][1] = msg->pose[i].position.y;
    tags[i - OFFSET][2] = msg->pose[i].position.z;

    quats[i - OFFSET].w() = msg->pose[i].orientation.w;
    quats[i - OFFSET].x() = msg->pose[i].orientation.x;
    quats[i - OFFSET].y() = msg->pose[i].orientation.y;
    quats[i - OFFSET].z() = msg->pose[i].orientation.z;
    //display items and their index
    // std::cout << "item " << i << " = " << msg->name[i] << " - in  " << i-OFFSET << "\n" << tags[i - OFFSET] << "\n";
  }
  init = false;
  }
    camera_vec = Eigen::Vector3f(msg->pose[CAMERA_OFFSET].position.x, msg->pose[CAMERA_OFFSET].position.y, msg->pose[CAMERA_OFFSET].position.z);
    camera_quat = Eigen::Quaternionf(msg->pose[CAMERA_OFFSET].orientation.w, msg->pose[CAMERA_OFFSET].orientation.x, msg->pose[CAMERA_OFFSET].orientation.y, msg->pose[CAMERA_OFFSET].orientation.z);
    
    
}

void finCb(const std_msgs::Bool &msg)
{
  finished = true;
  for (int i = 0; i < NUMBER_OF_TAGS; i++)
  {
    std::cout << i << "\n";
    for(int j = 0; j < difference_m[i].size(); j++){
    std::cout << abs_dist[i].front() << " , " << difference_m[i].front() << "\n";
    abs_dist[i].pop_front();
    difference_m[i].pop_front();
    }
  }
}





int main(int argc, char **argv)
{

  Eigen::Matrix3f test1;
test1 << 1,2,3,4,5,6,7,8,9;
std::cout << "test 1" << "\n" << test1 << "\n";

  Eigen::Matrix4f test11 = Eigen::Matrix4f::Identity();
test11(0,0) = test1(0,0);
test11(0,1) = test1(0,1);
test11(0,2) = test1(0,2);

std::cout << "test 11" << "\n" << test11 << "\n";



  // initializeTags();
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  tag_sub = n.subscribe("tag_detections", 1, tagCb);
  link_sub = n.subscribe("/gazebo/link_states", 1, posCb);
  fin_sub = n.subscribe("/finished", 1, finCb);
  
  ros::Rate r(2);

  ROS_INFO("launched, waiting for Tags");
  ros::spin();
}

