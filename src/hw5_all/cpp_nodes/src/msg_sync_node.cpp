// Include core ROS 2 client library
#include "rclcpp/rclcpp.hpp"

// Include message types
#include "sensor_msgs/msg/imu.hpp"          
#include "nav_msgs/msg/odometry.hpp"

// Include message_filters for synchronization
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

// For binding placeholders used in callback
using std::placeholders::_1;
using std::placeholders::_2;

// Define a class that inherits from rclcpp::Node
class SyncNode : public rclcpp::Node{
public:
  // Constructor for the node
  SyncNode()
  : Node("sync_node")  // Initialize node with the name "sync_node"
  {
    // ------------ TBD --------------
    // Create a message_filters::Subscriber for IMU topic
    // Use rmw_qos_profile_sensor_data for low-latency, best-effort delivery
    imu_sub_ = ...; // subscription to /imu using rmw_qos_profile_sensor_data QoS

    // Create a message_filters::Subscriber for Odometry topic
    odom_sub_ = ...; // subscription to /odom using rmw_qos_profile_sensor_data

    // ------------- TBD END -----------------

    // Define the approximate time policy with a queue size of 10
    // This allows the synchronizer to match messages with slightly different timestamps
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *imu_sub_, *odom_sub_);

    // Register the callback function to be called when synchronized messages are received
    sync_->registerCallback(std::bind(&SyncNode::callback, this, _1, _2));
  }

private:
  // Callback function triggered when synchronized IMU and Odometry messages are received
  void callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
                const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
  {
	// Explicit types for IMU data fields
	const geometry_msgs::msg::Quaternion & ori = imu_msg->orientation;
	const geometry_msgs::msg::Vector3 & ang_vel = imu_msg->angular_velocity;
	const geometry_msgs::msg::Vector3 & lin_acc = imu_msg->linear_acceleration;

	RCLCPP_INFO(this->get_logger(),"  IMU orientation: [x=%.3f, y=%.3f, z=%.3f, w=%.3f]", ori.x, ori.y, ori.z, ori.w);
	RCLCPP_INFO(this->get_logger(),"  IMU angular velocity: [x=%.3f, y=%.3f, z=%.3f]", ang_vel.x, ang_vel.y, ang_vel.z);
	RCLCPP_INFO(this->get_logger(),"  IMU linear acceleration: [x=%.3f, y=%.3f, z=%.3f]", lin_acc.x, lin_acc.y, lin_acc.z);

	// Explicit types for Odometry data fields
	const geometry_msgs::msg::Point & pos = odom_msg->pose.pose.position;
	const geometry_msgs::msg::Quaternion & ori_odom = odom_msg->pose.pose.orientation;

	RCLCPP_INFO(this->get_logger(),	"  Odometry position: [x=%.3f, y=%.3f, z=%.3f]", pos.x, pos.y, pos.z);
	RCLCPP_INFO(this->get_logger(),	"  Odometry orientation: [x=%.3f, y=%.3f, z=%.3f, w=%.3f]", ori_odom.x, ori_odom.y, ori_odom.z, ori_odom.w);

    //---------------------- TBD Comp Filter Stuff -------------------
	  // Create a publisher for the fused data here and convert your HW4 code as well
    
    //----------------------------------------------------------------
  }

  // Define the sync policy type: ApproximateTime syncs messages with similar timestamps
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Imu,
    nav_msgs::msg::Odometry>;

  // Subscribers for the two message types
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> imu_sub_;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> odom_sub_;

  // Synchronizer instance using the defined policy
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

// Main function: initialize, spin the node, and shut down
int main(int argc, char **argv){
  rclcpp::init(argc, argv);                       // Initialize ROS 2
  rclcpp::spin(std::make_shared<SyncNode>());     // Create and run the SyncNode
  rclcpp::shutdown();                             // Clean shutdown
  return 0;
}

