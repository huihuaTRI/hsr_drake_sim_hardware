#include "ros/package.h"
#include "ros/ros.h"

#include <XmlRpcValue.h>

#include "controller_manager/controller_manager.h"
#include "hardware_interface/hardware_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"

#include "lcm_to_ros/lcmt_hsr_sim_command.h"
#include "lcm_to_ros/lcmt_hsr_sim_status.h"

#define USE_PID_CONTROLLER 0

// Main class to run simulation and provide ROS control API.
// It's mandatory to inherit hardware_interface::RobotHW.
// Beacuse controller_manager::ControllerManager uses information registered in
// RobotHW.
class HsrDrakeSimHardware : public hardware_interface::RobotHW {
public:
  HsrDrakeSimHardware();

  // Initialize hardware interfaces and Drake simlulation
  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;

  // Read robots' state from the simulation
  void HsrSimStatusCallBack(const lcm_to_ros::lcmt_hsr_sim_status &msg);

  // Write commands to the simluation
  void HsrSimCommandPub();

private:
  int num_joints_;
  std::vector<std::string> joint_names_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> efforts_;
  std::vector<double> commands_;

  // A hardware interface that reads states of joints.
  hardware_interface::JointStateInterface state_interface_;
  // A hardware interface that sends commands to joints.
  hardware_interface::EffortJointInterface command_interface_;

  std::unique_ptr<ros::Publisher> sim_command_pub_ptr_;
  std::unique_ptr<ros::Subscriber> sim_status_sub_ptr_;
};

HsrDrakeSimHardware::HsrDrakeSimHardware() {}

bool HsrDrakeSimHardware::init(ros::NodeHandle &root_nh,
                               ros::NodeHandle &robot_hw_nh) {
  sim_command_pub_ptr_ = std::make_unique<ros::Publisher>(
      root_nh.advertise<lcm_to_ros::lcmt_hsr_sim_command>(
          "hsr_sim_command_channel", 1000));
  sim_status_sub_ptr_ = std::make_unique<ros::Subscriber>(
      root_nh.subscribe("hsr_sim_status_channel", 1000,
                        &HsrDrakeSimHardware::HsrSimStatusCallBack, this));

  // Get ros parameter to initialize internal states.
  root_nh.getParam("joint_trajectory_controller/num_joints", num_joints_);
  joint_names_.resize(num_joints_);
  positions_.resize(num_joints_);
  velocities_.resize(num_joints_);
  efforts_.resize(num_joints_);
  commands_.resize(num_joints_);

  XmlRpc::XmlRpcValue joint_params;
  root_nh.getParam("joint_trajectory_controller", joint_params);
  ROS_ASSERT(joint_params.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it =
           joint_params.begin();
       it != joint_params.end(); ++it) {
    if ((std::string)(it->first) == "joints") {
      for (int i = 0; i < num_joints_; ++i) {
        const std::string joint_name =
            static_cast<std::string>(joint_params[it->first][i]);
        ROS_INFO_STREAM("Joint name:" << joint_name);
        joint_names_[i] = joint_name;
      }
      break;
    }
  }

  // Register joint interfaces for ros-control
  for (int i = 0; i < num_joints_; ++i) {
    ROS_ASSERT(joint_names_[i] != "");
    // Create a JointHandle for every joint
    hardware_interface::JointStateHandle state_handle(
        joint_names_[i], &positions_[i], &velocities_[i], &efforts_[i]);
    state_interface_.registerHandle(state_handle);
    hardware_interface::JointHandle joint_handle(state_handle, &commands_[i]);
    command_interface_.registerHandle(joint_handle);
  }
  registerInterface(&state_interface_);
  registerInterface(&command_interface_);

  return 0;
}

// Read robot states from the simulation
void HsrDrakeSimHardware::HsrSimStatusCallBack(
    const lcm_to_ros::lcmt_hsr_sim_status &msg) {

  std::cout << "Message received!" << std::endl;
  for (int i = 0; i < msg.num_joints; ++i) {
    positions_[i] = msg.joint_position[i];
    velocities_[i] = msg.joint_velocity[i];
  }
}

// Publish the commands from a ros-controller to the simulation
void HsrDrakeSimHardware::HsrSimCommandPub() {
  lcm_to_ros::lcmt_hsr_sim_command msg;
  msg.num_joints = num_joints_;
  for (int i = 0; i < num_joints_; ++i) {
    msg.joint_position.push_back(0.0);
    msg.joint_velocity.push_back(0.0);
  }
  sim_command_pub_ptr_->publish(msg);
}

// The main loop
int main(int argc, char **argv) {
  ros::init(argc, argv, "hsr_hardware");

  // Create a node handle
  ros::NodeHandle nh("hsr");

  // Initialize a hardware object
  HsrDrakeSimHardware hw;
  hw.init(nh, nh);
  // ControllerManager handle loading/running/etc of controller-plugins.
  controller_manager::ControllerManager cm(&hw, nh);

  // Launch ROS event loop in a dedicated thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time start = ros::Time::now();
  ros::Time end = start;
  double control_rate;
  nh.param("hw_config/control_rate", control_rate, 100.0);
  ros::Rate rate(control_rate);
  double cycle_time = 1.0 / control_rate;
  ros::Duration period(cycle_time);

  // Simulation loop
  while (ros::ok()) {
    start = ros::Time::now();

    hw.HsrSimCommandPub();

    // Simple cycle over detection
    ros::Duration elapsed = end - start;
    if (elapsed.toSec() > cycle_time) {
      ROS_WARN_STREAM("elapsed = " << elapsed.toSec()
                                   << " / cycle = " << cycle_time);
    } else {
      ROS_INFO_STREAM_THROTTLE(1, "elapsed = " << elapsed.toSec()
                                               << " / cycle = " << cycle_time);
    }

    rate.sleep();
  }
  spinner.stop();
  ros::shutdown();

  return 0;
}
