#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
​
double my_abs(double a)
{
    return a>0? a:-a;
}
​
int main(int argc, char** argv){
  ros::init(argc, argv, "box_frame_correction");
​
  ros::NodeHandle node;
​
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
​
  static tf2_ros::TransformBroadcaster br;
​
  ros::Rate rate(5.0);
  while (node.ok()){
    geometry_msgs::TransformStamped box_msg;
    try{
      box_msg = tfBuffer.lookupTransform("ar_master_22", "ar_master_0",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(0.2).sleep();
      continue;
    }
​
​
    tf::StampedTransform box_tf;
    tf::transformStampedMsgToTF (box_msg, box_tf);
​
    double x,y,z;
    box_tf.getBasis().getEulerYPR(z,y,x);
    // std::cout<<"z:"<<z<<" y:"<<y<<" x:"<<x<<std::endl;
​
    double pi = 3.14159265;
​
    tf::Transform offset_tf;
    offset_tf.setIdentity();
    bool error = true;
​
    tf::Vector3 base_x(1,0,0);
    tf::Vector3 base_y(0,1,0);
    tf::Vector3 base_z(0,0,1);
    tf::Vector3 current_base_x;
    tf::Vector3 current_base_y;
    tf::Vector3 current_base_z;
    
    current_base_x = box_tf * base_x;
    current_base_y = box_tf * base_y;
    current_base_z = box_tf * base_z;
​
    if (my_abs(1.0 - base_z.dot(current_base_z)) <1e-1 ) {
        offset_tf.setIdentity();
        error = false;
    }
    if (my_abs(-1.0 - base_z.dot(current_base_z)) <1e-1 ) {
        offset_tf.setRotation(tf::createQuaternionFromRPY(pi, 0, 0));
        error = false;
    }
    if (my_abs(1.0 - base_z.dot(current_base_x)) <1e-1 ) {
        offset_tf.setRotation(tf::createQuaternionFromRPY(0, pi/2, 0));
        error = false;
    }
    if (my_abs(-1.0 - base_z.dot(current_base_x)) <1e-1 ) {
        offset_tf.setRotation(tf::createQuaternionFromRPY(0, -pi/2, 0));
        error = false;
    }
     if (my_abs(1.0 - base_z.dot(current_base_y)) <1e-1 ) {
        offset_tf.setRotation(tf::createQuaternionFromRPY(-pi/2, 0, 0));
        error = false;
    }
    if (my_abs(-1.0 - base_z.dot(current_base_y)) <1e-1 ) {
        offset_tf.setRotation(tf::createQuaternionFromRPY(pi/2, 0, 0));
        error = false;
    }
​
    if (error) {
        ROS_WARN("Invalid Rotation Angle: R: %.4f  P: %.4f  Y: %.4f", x, y, z);
    }
​
    geometry_msgs::TransformStamped corrected_box_msg;
    tf::StampedTransform correst_box_tf;
    correst_box_tf.setData(box_tf * offset_tf);
    correst_box_tf.stamp_ = ros::Time::now();
    tf::transformStampedTFToMsg (correst_box_tf, corrected_box_msg);
    corrected_box_msg.header.frame_id = "ar_master_22";
    corrected_box_msg.child_frame_id = "corrected_box_frame";
    br.sendTransform(corrected_box_msg);
​
    rate.sleep();
  }
  return 0;
};