/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015-2018 PX4 Pro Development Team
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <vector>
#include <regex>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <cassert>
#include <stdexcept>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_array.hpp>
#include <boost/system/system_error.hpp>

#include <iostream>
#include <random>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>

#include <Eigen/Eigen>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <ignition/math.hh>
#include <sdf/sdf.hh>
#include <common.h>
#include <CommandMotorSpeed.pb.h>
#include <MotorSpeed.pb.h>
#include <Imu.pb.h>
#include <OpticalFlow.pb.h>
#include <Range.pb.h>
#include <SITLGps.pb.h>
#include <IRLock.pb.h>
#include <Groundtruth.pb.h>
#include <Odometry.pb.h>
#include <MagneticField.pb.h>
#include <Pressure.pb.h>

#include <mavlink/v2.0/common/mavlink.h>
#include "msgbuffer.h"

static const uint32_t kDefaultMavlinkUdpPort = 14560;
static const uint32_t kDefaultMavlinkTcpPort = 4560;
static const uint32_t kDefaultQGCUdpPort = 14550;
static const uint32_t kDefaultSDKUdpPort = 14540;

using lock_guard = std::lock_guard<std::recursive_mutex>;
static constexpr auto kDefaultDevice = "/dev/ttyACM0";
static constexpr auto kDefaultBaudRate = 921600;

//! Maximum buffer size with padding for CRC bytes (280 + padding)
static constexpr ssize_t MAX_SIZE = MAVLINK_MAX_PACKET_LEN + 16;
static constexpr size_t MAX_TXQ_SIZE = 1000;

//! Default distance sensor model joint naming
static const std::regex kDefaultLidarModelLinkNaming("(lidar|sf10a)(.*::link)");
static const std::regex kDefaultSonarModelLinkNaming("(sonar|mb1240-xl-ez4)(.*::link)");

namespace gazebo {

typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;
typedef const boost::shared_ptr<const nav_msgs::msgs::Odometry> OdomPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Groundtruth> GtPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Imu> ImuPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::IRLock> IRLockPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::OpticalFlow> OpticalFlowPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Range> SonarPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Range> LidarPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::SITLGps> GpsPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::MagneticField> MagnetometerPtr;
typedef const boost::shared_ptr<const sensor_msgs::msgs::Pressure> BarometerPtr;

typedef std::pair<const int, const ignition::math::Quaterniond> SensorIdRot_P;
typedef std::map<transport::SubscriberPtr, SensorIdRot_P > Sensor_M;

// Default values
static const std::string kDefaultNamespace = "";

// This just proxies the motor commands from command/motor_speed to the single motors via internal
// ConsPtr passing, such that the original commands don't have to go n_motors-times over the wire.
static const std::string kDefaultMotorVelocityReferencePubTopic = "/gazebo/command/motor_speed";

static const std::string kDefaultImuTopic = "/imu";
static const std::string kDefaultOpticalFlowTopic = "/px4flow/link/opticalFlow";
static const std::string kDefaultIRLockTopic = "/camera/link/irlock";
static const std::string kDefaultGPSTopic = "/gps";
static const std::string kDefaultVisionTopic = "/vision_odom";
static const std::string kDefaultMagTopic = "/mag";
static const std::string kDefaultBarometerTopic = "/baro";

//! Rx packer framing status. (same as @p mavlink::mavlink_framing_t)
enum class Framing : uint8_t {
	incomplete = MAVLINK_FRAMING_INCOMPLETE,
	ok = MAVLINK_FRAMING_OK,
	bad_crc = MAVLINK_FRAMING_BAD_CRC,
	bad_signature = MAVLINK_FRAMING_BAD_SIGNATURE,
};


//! Enumeration to use on the bitmask in HIL_SENSOR
enum class SensorSource {
  ACCEL		= 0b111,
  GYRO		= 0b111000,
  MAG		= 0b111000000,
  BARO		= 0b1101000000000,
  DIFF_PRESS	= 0b10000000000,
};

//! OR operation for the enumeration and unsigned types that returns the bitmask
template<typename A, typename B>
static inline uint32_t operator |(A lhs, B rhs) {
  // make it type safe
  static_assert((std::is_same<A, uint32_t>::value || std::is_same<A, SensorSource>::value),
		"first argument is not uint32_t or SensorSource enum type");
  static_assert((std::is_same<B, uint32_t>::value || std::is_same<B, SensorSource>::value),
		"second argument is not uint32_t or SensorSource enum type");

  return static_cast<uint32_t> (
    static_cast<std::underlying_type<SensorSource>::type>(lhs) |
    static_cast<std::underlying_type<SensorSource>::type>(rhs)
  );
}

class GazeboMavlinkInterface : public ModelPlugin {
public:
  GazeboMavlinkInterface() : ModelPlugin(),
    received_first_actuator_(false),
    namespace_(kDefaultNamespace),
    protocol_version_(2.0),
    motor_velocity_reference_pub_topic_(kDefaultMotorVelocityReferencePubTopic),
    use_propeller_pid_(false),
    use_elevator_pid_(false),
    use_left_elevon_pid_(false),
    use_right_elevon_pid_(false),
    vehicle_is_tailsitter_(false),
    send_vision_estimation_(false),
    send_odometry_(false),
    imu_sub_topic_(kDefaultImuTopic),
    opticalFlow_sub_topic_(kDefaultOpticalFlowTopic),
    irlock_sub_topic_(kDefaultIRLockTopic),
    gps_sub_topic_(kDefaultGPSTopic),
    vision_sub_topic_(kDefaultVisionTopic),
    mag_sub_topic_(kDefaultMagTopic),
    baro_sub_topic_(kDefaultBarometerTopic),
    sensor_map_ {},
    model_ {},
    world_(nullptr),
    left_elevon_joint_(nullptr),
    right_elevon_joint_(nullptr),
    elevator_joint_(nullptr),
    propeller_joint_(nullptr),
    gimbal_yaw_joint_(nullptr),
    gimbal_pitch_joint_(nullptr),
    gimbal_roll_joint_(nullptr),
    input_offset_ {},
    input_scaling_ {},
    zero_position_disarmed_ {},
    zero_position_armed_ {},
    input_index_ {},
    mag_updated_(false),
    baro_updated_(false),
    diff_press_updated_(false),
    groundtruth_lat_rad(0.0),
    groundtruth_lon_rad(0.0),
    groundtruth_altitude(0.0),
    mavlink_udp_port_(kDefaultMavlinkUdpPort),
    mavlink_tcp_port_(kDefaultMavlinkTcpPort),
    simulator_socket_fd_(0),
    simulator_tcp_client_fd_(0),
    use_tcp_(false),
    qgc_udp_port_(kDefaultQGCUdpPort),
    sdk_udp_port_(kDefaultSDKUdpPort),
    remote_qgc_addr_ {},
    local_qgc_addr_ {},
    remote_sdk_addr_ {},
    local_sdk_addr_ {},
    qgc_socket_fd_(0),
    sdk_socket_fd_(0),
    serial_enabled_(false),
    tx_q {},
    rx_buf {},
    m_status {},
    m_buffer {},
    io_service(),
    serial_dev(io_service),
    device_(kDefaultDevice),
    baudrate_(kDefaultBaudRate),
    hil_mode_(false),
    hil_state_level_(false)
    {}

  ~GazeboMavlinkInterface();

  void Publish();

protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo&  /*_info*/);

private:
  bool received_first_actuator_;
  Eigen::VectorXd input_reference_;

  float protocol_version_;

  std::string namespace_;
  std::string motor_velocity_reference_pub_topic_;
  std::string mavlink_control_sub_topic_;
  std::string link_name_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr motor_velocity_reference_pub_;
  transport::SubscriberPtr mav_control_sub_;

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::JointPtr left_elevon_joint_;
  physics::JointPtr right_elevon_joint_;
  physics::JointPtr elevator_joint_;
  physics::JointPtr propeller_joint_;
  physics::JointPtr gimbal_yaw_joint_;
  physics::JointPtr gimbal_pitch_joint_;
  physics::JointPtr gimbal_roll_joint_;
  common::PID propeller_pid_;
  common::PID elevator_pid_;
  common::PID left_elevon_pid_;
  common::PID right_elevon_pid_;
  bool use_propeller_pid_;
  bool use_elevator_pid_;
  bool use_left_elevon_pid_;
  bool use_right_elevon_pid_;

  bool vehicle_is_tailsitter_;

  bool send_vision_estimation_;
  bool send_odometry_;

  std::vector<physics::JointPtr> joints_;
  std::vector<common::PID> pids_;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;
  event::ConnectionPtr sigIntConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  void ImuCallback(ImuPtr& imu_msg);
  void GpsCallback(GpsPtr& gps_msg);
  void GroundtruthCallback(GtPtr& groundtruth_msg);
  void LidarCallback(LidarPtr& lidar_msg, const int& id);
  void SonarCallback(SonarPtr& sonar_msg, const int& id);
  void OpticalFlowCallback(OpticalFlowPtr& opticalFlow_msg);
  void IRLockCallback(IRLockPtr& irlock_msg);
  void VisionCallback(OdomPtr& odom_msg);
  void MagnetometerCallback(MagnetometerPtr& mag_msg);
  void BarometerCallback(BarometerPtr& baro_msg);
  void send_mavlink_message(const mavlink_message_t *message);
  void forward_mavlink_message(const mavlink_message_t *message);
  void handle_message(mavlink_message_t *msg, bool &received_actuator);
  void acceptConnections();
  void pollForMAVLinkMessages();
  void pollFromQgcAndSdk();
  void SendSensorMessages();
  void SendGroundTruth();
  void handle_control(double _dt);
  bool IsRunning();
  void onSigInt();

  /**
   * @brief Set the MAV_SENSOR_ORIENTATION enum value based on the sensor orientation
   *
   * @param[in] rootModel		The root model where the sensor is attached
   * @param[in] u_Xs				Unit vector of X-axis sensor in `base_link` frame
   * @param[in] sensor_msg	The Mavlink DISTANCE_SENSOR message struct
   */
  template <class T>
  void setMavlinkSensorOrientation(const ignition::math::Vector3d& u_Xs, T& sensor_msg);

  /**
   * @brief A helper class that allows the creation of multiple subscriptions to sensors.
   *	    It gets the sensor link/joint and creates the subscriptions based on those.
   *	    It also allows to set the initial rotation of the sensor, to allow computing
   *	    the sensor orientation quaternion.
   * @details GazeboMsgT  The type of the message that will be subscribed to the Gazebo framework.
   */
  template <typename GazeboMsgT>
  void CreateSensorSubscription(
      void (GazeboMavlinkInterface::*fp)(const boost::shared_ptr<GazeboMsgT const>&, const int&),
      GazeboMavlinkInterface* ptr, const physics::Link_V& links);

  // Serial interface
  void open();
  void close();
  void do_read();
  void parse_buffer(const boost::system::error_code& err, std::size_t bytes_t);
  void do_write(bool check_tx_state);
  inline bool is_open(){
    return serial_dev.is_open();
  }

  static const unsigned n_out_max = 16;

  double input_offset_[n_out_max];
  double input_scaling_[n_out_max];
  std::string joint_control_type_[n_out_max];
  std::string gztopic_[n_out_max];
  double zero_position_disarmed_[n_out_max];
  double zero_position_armed_[n_out_max];
  int input_index_[n_out_max];
  transport::PublisherPtr joint_control_pub_[n_out_max];

  transport::SubscriberPtr imu_sub_;
  transport::SubscriberPtr opticalFlow_sub_;
  transport::SubscriberPtr irlock_sub_;
  transport::SubscriberPtr gps_sub_;
  transport::SubscriberPtr groundtruth_sub_;
  transport::SubscriberPtr vision_sub_;
  transport::SubscriberPtr mag_sub_;
  transport::SubscriberPtr baro_sub_;

  Sensor_M sensor_map_; // Map of sensor SubscriberPtr, IDs and orientations

  std::string imu_sub_topic_;
  std::string opticalFlow_sub_topic_;
  std::string irlock_sub_topic_;
  std::string gps_sub_topic_;
  std::string groundtruth_sub_topic_;
  std::string vision_sub_topic_;
  std::string mag_sub_topic_;
  std::string baro_sub_topic_;

  std::mutex last_imu_message_mutex_ {};
  std::condition_variable last_imu_message_cond_ {};
  sensor_msgs::msgs::Imu last_imu_message_;
  common::Time last_time_;
  common::Time last_imu_time_;
  common::Time last_actuator_time_;

  bool mag_updated_;
  bool baro_updated_;
  bool diff_press_updated_;

  double groundtruth_lat_rad;
  double groundtruth_lon_rad;
  double groundtruth_altitude;

  double imu_update_interval_ = 0.004; ///< Used for non-lockstep

  ignition::math::Vector3d gravity_W_;
  ignition::math::Vector3d velocity_prev_W_;
  ignition::math::Vector3d mag_n_;

  double temperature_;
  double pressure_alt_;
  double abs_pressure_;

  std::default_random_engine random_generator_;
  std::normal_distribution<float> standard_normal_distribution_;

  struct sockaddr_in local_simulator_addr_;
  socklen_t local_simulator_addr_len_;
  struct sockaddr_in remote_simulator_addr_;
  socklen_t remote_simulator_addr_len_;

  int qgc_udp_port_;
  struct sockaddr_in remote_qgc_addr_;
  socklen_t remote_qgc_addr_len_;
  struct sockaddr_in local_qgc_addr_;
  socklen_t local_qgc_addr_len_;

  int sdk_udp_port_;
  struct sockaddr_in remote_sdk_addr_;
  socklen_t remote_sdk_addr_len_;
  struct sockaddr_in local_sdk_addr_;
  socklen_t local_sdk_addr_len_;

  unsigned char _buf[65535];
  enum FD_TYPES {
    LISTEN_FD,
    CONNECTION_FD,
    N_FDS
  };
  struct pollfd fds_[N_FDS];
  bool use_tcp_ = false;
  bool close_conn_ = false;

  double optflow_distance;
  double sonar_distance;

  in_addr_t mavlink_addr_;
  int mavlink_udp_port_; // MAVLink refers to the PX4 simulator interface here
  int mavlink_tcp_port_; // MAVLink refers to the PX4 simulator interface here

  int simulator_socket_fd_;
  int simulator_tcp_client_fd_;

  int qgc_socket_fd_ {-1};
  int sdk_socket_fd_ {-1};

  bool enable_lockstep_ = false;
  double speed_factor_ = 1.0;
  int64_t previous_imu_seq_ = 0;
  unsigned update_skip_factor_ = 1;

  // Serial interface
  mavlink_status_t m_status;
  mavlink_message_t m_buffer;
  bool serial_enabled_;
  std::thread io_thread;
  std::string device_;
  std::array<uint8_t, MAX_SIZE> rx_buf;
  std::recursive_mutex mutex;
  unsigned int baudrate_;
  std::atomic<bool> tx_in_progress;
  std::deque<MsgBuffer> tx_q;
  boost::asio::io_service io_service;
  boost::asio::serial_port serial_dev;

  bool hil_mode_;
  bool hil_state_level_;

  std::atomic<bool> gotSigInt_ {false};
};
}
