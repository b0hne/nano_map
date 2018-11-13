#include "nano_map.h"

// // //parcour 1
// const int NUMBER_OF_TAGS = 16;

// //parcour 2
// const int NUMBER_OF_TAGS = 24;


// parcour 3
const int NUMBER_OF_TAGS = 35;


//max number of tag arrays till remaping
int map_counter = 30;

  //collect three tag touple
struct connection {
  Eigen::Vector3f vec_0;
  Eigen::Vector3f vec_1;
 };


ros::Publisher marker_pub;
ros::Subscriber tag_sub;

bool finished = false;
bool touched[NUMBER_OF_TAGS];
bool seen[NUMBER_OF_TAGS];

std::list<connection> connections[NUMBER_OF_TAGS];
connection averaged_connection[NUMBER_OF_TAGS];
Eigen::Quaternionf rot_sum = Eigen::Quaternionf::Identity();

//store gazebo given coordinates and directions
Eigen::Vector3f positions[NUMBER_OF_TAGS];
Eigen::Vector3f orientations[NUMBER_OF_TAGS];

//collection of calculated data
geometry_msgs::PoseArray connected_dots;

int found_tags = 0;

Eigen::Matrix3f rot_glob;


//get average of vector
Eigen::Vector3f average_vector(std::list<Eigen::Vector3f> list)
{
  Eigen::Vector3f answer(0,0,0);
  for(std::list<Eigen::Vector3f>::iterator it=list.begin(); it != list.end(); ++it)
    answer += *it;
  answer /= list.size();
  return answer;
}

//compare position.x of apriltags
bool sortByX(const apriltags_ros::AprilTagDetection &tag0, const apriltags_ros::AprilTagDetection &tag1)
{
  return tag0.pose.pose.position.x > tag1.pose.pose.position.x;
}


void average_out()
{
  for(int i = 0; i< NUMBER_OF_TAGS; i++)
  {
    if(touched[i])
    {
      // prepare Eigen collections
      connection new_average;
      std::list<Eigen::Vector3f> vec_0;
      std::list<Eigen::Vector3f> vec_1;
 
      for(std::list<connection>::iterator it=connections[i].begin(); it != connections[i].end(); ++it)
      {
       vec_0.push_back(it->vec_0); 
       vec_1.push_back(it->vec_1); 
      }
      
      //calculate tag position
      new_average.vec_0 = average_vector(vec_0);
      new_average.vec_1 = average_vector(vec_1);
      
      averaged_connection[i] = new_average;
    }
  }
}


void recalculate_map(){
  average_out();

  // collect accumilating rotation
  rot_sum = Eigen::Quaternionf::Identity();
  // rot_sum = quats[0];
  connected_dots.poses.clear();

  for(int i = 0; i < NUMBER_OF_TAGS; i++){
    
    // abort calculation if base tag is not found yet
    if(connections[i].size() == 0 || connections[0].size() == 0)
      {
        break;
      }
     
    Eigen::Quaternionf rotate = Eigen::Quaternionf().FromTwoVectors(Eigen::Vector3f(1,1,1),averaged_connection[0].vec_0);
      
        //prepare first tag
    if(i==0){
      geometry_msgs::Pose pose0;
      pose0.position.x = 0;
      pose0.position.y = 0;
      pose0.position.z = 0;
      connected_dots.poses.push_back(pose0);

      geometry_msgs::Pose pose1;
      Eigen::Vector3f new_pos = rot_sum * averaged_connection[0].vec_1;
      
      // Eigen::Vector3f new_pos = averaged_connection[0].vec_1;
      pose1.position.x = new_pos.x();
      pose1.position.y = new_pos.y();
      pose1.position.z = new_pos.z();
      

      connected_dots.poses.push_back(pose1);

    }
    else{
      geometry_msgs::Pose poseN;
      Eigen::Quaternionf correction(1,0,0,0);
      Eigen::Quaternionf rotate = Eigen::Quaternionf().FromTwoVectors(averaged_connection[i-1].vec_1,averaged_connection[i].vec_0);
      
      rot_sum = rot_sum * rotate.inverse();
      
      Eigen::Vector3f new_pos = rot_sum * averaged_connection[i].vec_1;
      poseN.position.x = connected_dots.poses[i].position.x + new_pos.x();
      poseN.position.y = connected_dots.poses[i].position.y + new_pos.y();
      poseN.position.z = connected_dots.poses[i].position.z + new_pos.z();
      
      connected_dots.poses.push_back(poseN);
        }
  }
  marker_pub.publish(connected_dots);
  if (connected_dots.poses.size() >= NUMBER_OF_TAGS)
    {
      std:: cout << "spacial dist1 : "<< "\n" << (rot_sum * averaged_connection[NUMBER_OF_TAGS-1].vec_1 - averaged_connection[0].vec_0).norm() << "\n";
      
    // finished = true;
    }
  std::cout << "publishing " << connected_dots.poses.size() << "\n";
} 

  
  
  
 

void tagCb(const apriltags_ros::AprilTagDetectionArray &tags)
{
  int found_tags_size = tags.detections.size();
  if(!finished){

    // sort tags by z value
    apriltags_ros::AprilTagDetection found_tags_sorted[found_tags_size];
    for (int i = 0; i < found_tags_size; i++)
      found_tags_sorted[i] = tags.detections[i];
    std::sort(found_tags_sorted, found_tags_sorted + found_tags_size, sortByX);

    //start collecting
    for(int i = 0; i < found_tags_size-2; i++)
      {
        //if tags are adjacent
      if(((found_tags_sorted[i].id + 2)%NUMBER_OF_TAGS) == found_tags_sorted[i+2].id)
        { 
          seen[found_tags_sorted[i].id] = true;


          //prepare rotation matrices
          Eigen::Vector3f first_pos(found_tags_sorted[i].pose.pose.position.x, found_tags_sorted[i].pose.pose.position.y, found_tags_sorted[i].pose.pose.position.z);
          Eigen::Vector3f second_pos(found_tags_sorted[i+1].pose.pose.position.x, found_tags_sorted[i+1].pose.pose.position.y, found_tags_sorted[i+1].pose.pose.position.z);
          Eigen::Vector3f third_pos(found_tags_sorted[i+2].pose.pose.position.x, found_tags_sorted[i+2].pose.pose.position.y, found_tags_sorted[i+2].pose.pose.position.z);

          connection new_connection;
          new_connection.vec_0 = second_pos- first_pos;
          new_connection.vec_1 = third_pos- second_pos;


          // push to collection
          connections[found_tags_sorted[i].id].push_back(new_connection);
          touched[found_tags_sorted[i].id] = true;

          
        }
      }
    map_counter--;
    int count = 0;
    for (int i= 0; i < NUMBER_OF_TAGS; i++){
      if(seen[i])
        count++;}
    if(found_tags < count || map_counter < 0)
          {
          found_tags = count;
          map_counter = 100;
            recalculate_map();
          }
    }
}


int main(int argc, char **argv)
{
  for (int i = 0; i < NUMBER_OF_TAGS; i++)
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  connected_dots.header.frame_id = "/world";
  marker_pub = n.advertise<geometry_msgs::PoseArray>("visualization_marker", 1);
  tag_sub = n.subscribe("tag_detections", 1, tagCb);
  ros::Rate r(2);

  ROS_INFO("launched, waiting for Tags");
  ros::spin();
}
