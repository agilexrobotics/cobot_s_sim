#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc,argv,"record");
  ros::NodeHandle nh("~");
  std::string file_name;
  std::string robot_frame, global_frame;
  nh.param<std::string>("file_name", file_name, "");
  nh.param<std::string>("robot_frame", robot_frame, "base_link");
  nh.param<std::string>("global_frame", global_frame, "map");

  FILE *f =fopen(file_name.c_str(), "w");

  uint n = 0;
  tf::TransformListener listener;
  tf::StampedTransform transform;

  ROS_INFO_STREAM("PRESS ENTER TO RECORD A POSE, PRESS Q TO QUIT!");
  while(ros::ok()) {
    char key = getchar();
    if(key=='q' || key=='Q')break;
    try {
      listener.waitForTransform(robot_frame, global_frame, ros::Time(0), ros::Duration(10.0));
      listener.lookupTransform(global_frame, robot_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    char str[100]={0};
    sprintf(str, "{x: %f, y: %f, z: %f, qx: %f, qy: %f, qz: %f, qw: %f}",
          transform.getOrigin().getX(),transform.getOrigin().getY(),transform.getOrigin().getZ(),
          transform.getRotation().x(),transform.getRotation().y(),transform.getRotation().z(),transform.getRotation().w());
    fprintf(f, "nav_goal_%d: %s\r\n", ++n, str);
    ROS_INFO("Record pose %d in map frame: %s", n, str);
  }
  fprintf(f, "nav_goal_num: %d\r\n", n);
  fclose(f);
}
