
//
// Created by qiayuan on 1/24/22.
//

#include "legged_unitree_hw/UnitreeHW.h"

#ifdef UNITREE_SDK_3_3_1
#include "unitree_legged_sdk_3_3_1/unitree_joystick.h"
#elif UNITREE_SDK_3_8_0
#include "unitree_legged_sdk_3_8_0/joystick.h"
#endif

#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace legged {
bool UnitreeHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!LeggedHW::init(root_nh, robot_hw_nh)) {
    return false;
  }

  robot_hw_nh.getParam("power_limit", powerLimit_);

  setupJoints();
  setupImu();
  setupContactSensor(robot_hw_nh);

  // Subscribe to IMU data
  imu_sub_ = root_nh.subscribe<sensor_msgs::Imu>("/imu/data_raw", 10, &UnitreeHW::imuCallback, this);
  joint_state_sub_ = root_nh.subscribe<sensor_msgs::JointState>("/motor_states", 10, &UnitreeHW::jointStateCallback, this);
  command_pub_ = root_nh.advertise<trajectory_msgs::JointTrajectory>("/joint_controller/command", 10);

  // joyPublisher_ = root_nh.advertise<sensor_msgs::Joy>("/joy", 10);
  contactPublisher_ = root_nh.advertise<std_msgs::Int16MultiArray>(std::string("/contact"), 10);
  return true;
}

void UnitreeHW::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  // Orientation (quaternion)
  imuData_.ori_[0] = msg->orientation.x;
  imuData_.ori_[1] = msg->orientation.y;
  imuData_.ori_[2] = msg->orientation.z;
  imuData_.ori_[3] = msg->orientation.w;

  // Angular velocity
  imuData_.angularVel_[0] = msg->angular_velocity.x;
  imuData_.angularVel_[1] = msg->angular_velocity.y;
  imuData_.angularVel_[2] = msg->angular_velocity.z;

  // Linear acceleration
  imuData_.linearAcc_[0] = msg->linear_acceleration.x;
  imuData_.linearAcc_[1] = msg->linear_acceleration.y;
  imuData_.linearAcc_[2] = msg->linear_acceleration.z;
}

void UnitreeHW::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  for (size_t i = 0; i < msg->name.size(); ++i) {
    const std::string& name = msg->name[i];

    // Loop through your registered joints and find matching index
    for (int j = 0; j < 12; ++j) {
      if (jointNames_[j] == name) {
        jointData_[j].pos_ = msg->position[i];
        jointData_[j].vel_ = msg->velocity[i];
        jointData_[j].tau_ = msg->effort[i];
      }
    }
  }
}

void UnitreeHW::read(const ros::Time& time, const ros::Duration& /*period*/) {

  ROS_INFO_STREAM_THROTTLE(0.3, 
    "IMU Data:\n"
    << "Orientation: [" << imuData_.ori_[0] << ", " << imuData_.ori_[1] << ", " 
    << imuData_.ori_[2] << ", " << imuData_.ori_[3] << "]\n"
    << "Angular Velocity: [" << imuData_.angularVel_[0] << ", " << imuData_.angularVel_[1] << ", " 
    << imuData_.angularVel_[2] << "]\n"
    << "Linear Acceleration: [" << imuData_.linearAcc_[0] << ", " << imuData_.linearAcc_[1] << ", " 
    << imuData_.linearAcc_[2] << "]"
);




  // for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
  //   contactState_[i] = lowState_.footForce[i] > contactThreshold_;
  // }
  int j=0;
  for (size_t i = 2; i < 12; i=i+3) {
    // contactState_[j] = jointData_[i].tau_ > contactThreshold_;
    contactState_[j] = 1;
    j=j+1;
  }

  std::ostringstream contact_oss;
  contact_oss << "Contact States:\n";
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contact_oss << CONTACT_SENSOR_NAMES[i] << ": " << (contactState_[i]) << "\n";
  }
  ROS_INFO_STREAM_THROTTLE(0.5, contact_oss.str());

  // Joint states printout
  std::ostringstream oss;
  oss << "Received Joint States (from /joint_states):\n";
  for (int i = 0; i < 12; ++i) {
    oss << "Joint " << i << " (" << jointNames_[i] << "): "
        << "pos = " << jointData_[i].pos_ << ", "
        << "vel = " << jointData_[i].vel_ << ", "
        << "tau = " << jointData_[i].tau_ << "\n";
  }
  ROS_INFO_STREAM_THROTTLE(0.5, oss.str());

  // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
  std::vector<std::string> names = hybridJointInterface_.getNames();
  for (const auto& name : names) {
    HybridJointHandle handle = hybridJointInterface_.getHandle(name);
    handle.setFeedforward(0.);
    handle.setVelocityDesired(0.);
    handle.setKd(3.);
  }

  // updateJoystick(time);
  updateContact(time);
}

void UnitreeHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {

  trajectory_msgs::JointTrajectory traj_msg;
  traj_msg.header.stamp = ros::Time::now();

  // One trajectory point containing all desired values
  trajectory_msgs::JointTrajectoryPoint point;

  traj_msg.joint_names = jointNames_;

  // Positions (dummy i here, replace with your posDes if available)
  for (int i = 0; i < 12; ++i)
    point.positions.push_back(jointData_[i].posDes_);

  // Velocities
  for (int i = 0; i < 12; ++i)
    point.velocities.push_back(jointData_[i].velDes_);

  // Efforts (using ff_ for feedforward torques/forces)
  for (int i = 0; i < 12; ++i)
    point.effort.push_back(jointData_[i].ff_);

  // Set time_from_start (required field)
  point.time_from_start = ros::Duration(0.01); // e.g., 10ms

  // Add the point to trajectory
  traj_msg.points.push_back(point);

  // Publish
  command_pub_.publish(traj_msg);

  // std_msgs::Float64MultiArray cmd_msg;
  // cmd_msg.data.reserve(12 * 5);

  // // Append posDes
  // for (int i = 0; i < 12; ++i) cmd_msg.data.push_back(i);
  // // Append velDes
  // for (int i = 0; i < 12; ++i) cmd_msg.data.push_back(jointData_[i].velDes_);
  // // Append kp
  // for (int i = 0; i < 12; ++i) cmd_msg.data.push_back(jointData_[i].kp_); 
  // // Append kd
  // for (int i = 0; i < 12; ++i) cmd_msg.data.push_back(jointData_[i].kd_);
  // // Append ff
  // for (int i = 0; i < 12; ++i) cmd_msg.data.push_back(jointData_[i].ff_);

  // command_pub_.publish(cmd_msg);

  ROS_INFO_STREAM_THROTTLE(0.5,
    "Publishing Joint Commands (posDes, velDes, kp, kd, ff):\n" <<
    [&]() {
    std::ostringstream oss;
    for (int i = 0; i < 12; ++i) {
      oss << "Joint " << i << ": ["
          << jointData_[i].posDes_ << ", "
          << jointData_[i].velDes_ << ", "
          << jointData_[i].kp_ << ", "
          << jointData_[i].kd_ << ", "
          << jointData_[i].ff_ << "]\n";
    }
    return oss.str();
    }()
    );
}

bool UnitreeHW::setupJoints() {
  jointNames_.resize(12);
  for (const auto& joint : urdfModel_->joints_) {
    int leg_index = 0;
    int joint_index = 0;
    if (joint.first.find("RF") != std::string::npos) {
      leg_index = UNITREE_LEGGED_SDK::FR_;
    } else if (joint.first.find("LF") != std::string::npos) {
      leg_index = UNITREE_LEGGED_SDK::FL_;
    } else if (joint.first.find("RH") != std::string::npos) {
      leg_index = UNITREE_LEGGED_SDK::RR_;
    } else if (joint.first.find("LH") != std::string::npos) {
      leg_index = UNITREE_LEGGED_SDK::RL_;
    } else {
      continue;
    }

    if (joint.first.find("HAA") != std::string::npos) {
      joint_index = 0;
    } else if (joint.first.find("HFE") != std::string::npos) {
      joint_index = 1;
    } else if (joint.first.find("KFE") != std::string::npos) {
      joint_index = 2;
    } else {
      continue;
    }

    int index = leg_index * 3 + joint_index;
    jointNames_[index] = joint.first;
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].posDes_, &jointData_[index].velDes_,
                                                           &jointData_[index].kp_, &jointData_[index].kd_, &jointData_[index].ff_));
  }
  return true;
}

bool UnitreeHW::setupImu() {
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("base_imu", "base_imu", imuData_.ori_, imuData_.oriCov_,
                                                                         imuData_.angularVel_, imuData_.angularVelCov_, imuData_.linearAcc_,
                                                                         imuData_.linearAccCov_));
  // imuData_.oriCov_[0] = 0.0012;
  // imuData_.oriCov_[4] = 0.0012;
  // imuData_.oriCov_[8] = 0.0012;
  imuData_.oriCov_[0] = 1.218e-05;
  imuData_.oriCov_[4] = 1.218e-05;
  imuData_.oriCov_[8] = 1.218e-05;

  // imuData_.angularVelCov_[0] = 0.0004;
  // imuData_.angularVelCov_[4] = 0.0004;
  // imuData_.angularVelCov_[8] = 0.0004;
  imuData_.angularVelCov_[0] = 7.31e-07;
  imuData_.angularVelCov_[4] = 7.31e-07;
  imuData_.angularVelCov_[8] = 7.31e-07;

  imuData_.linearAccCov_[0] = 7.38e-05;
  imuData_.linearAccCov_[4] = 7.38e-05;
  imuData_.linearAccCov_[8] = 7.38e-05;


  return true;
}

bool UnitreeHW::setupContactSensor(ros::NodeHandle& nh) {
  nh.getParam("contact_threshold", contactThreshold_);
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactSensorInterface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
  }
  return true;
}

void UnitreeHW::updateJoystick(const ros::Time& time) {
  if ((time - lastJoyPub_).toSec() < 1 / 50.) {
    return;
  }
  lastJoyPub_ = time;
  xRockerBtnDataStruct keyData;
  memcpy(&keyData, &lowState_.wirelessRemote[0], 40);
  sensor_msgs::Joy joyMsg;  // Pack as same as Logitech F710
  joyMsg.axes.push_back(-keyData.lx);
  joyMsg.axes.push_back(keyData.ly);
  joyMsg.axes.push_back(-keyData.rx);
  joyMsg.axes.push_back(keyData.ry);
  joyMsg.buttons.push_back(keyData.btn.components.X);
  joyMsg.buttons.push_back(keyData.btn.components.A);
  joyMsg.buttons.push_back(keyData.btn.components.B);
  joyMsg.buttons.push_back(keyData.btn.components.Y);
  joyMsg.buttons.push_back(keyData.btn.components.L1);
  joyMsg.buttons.push_back(keyData.btn.components.R1);
  joyMsg.buttons.push_back(keyData.btn.components.L2);
  joyMsg.buttons.push_back(keyData.btn.components.R2);
  joyMsg.buttons.push_back(keyData.btn.components.select);
  joyMsg.buttons.push_back(keyData.btn.components.start);
  joyPublisher_.publish(joyMsg);
}

void UnitreeHW::updateContact(const ros::Time& time) {
  if ((time - lastContactPub_).toSec() < 1 / 50.) {
    return;
  }
  lastContactPub_ = time;

  std_msgs::Int16MultiArray contactMsg;
  // for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
  //   contactMsg.data.push_back(lowState_.footForce[i]);
  // }

  for (size_t i = 2; i < 12; i=i+3) {
    // contactMsg.data.push_back(jointData_[i].tau_);
    contactMsg.data.push_back(1);
  }

  contactPublisher_.publish(contactMsg);
}

}  // namespace legged
