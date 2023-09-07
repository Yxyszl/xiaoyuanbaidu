#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Geometry> 
#include <chrono>
#include <locale>
#include <tuple>
#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <boost/assert.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

extern "C" {
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <time.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <unistd.h> //  close
#include <string.h> //  strerror
}

using namespace std;

#define IMU_HEADER_1 0xA5
#define IMU_HEADER_2 0x5A
#define IMU_TAILER   0xAA

enum {
  CMD_START = 0x01,
  CMD_STOP  = 0x02,
  CMD_MAG_CALIBRATE = 0xE3,
  CMD_SAVE_MAG_CALI = 0xE1,
  CMD_BEGIN_MAG_AZIMUTH_FIX = 0xE2,
  CMD_END_MAG_AZIMUTH_FIX = 0xE4,
};

typedef struct __attribute__ ((packed)) _tagImuCommand {
  uint8_t   ucHeader[2];
  uint8_t   ucDataLen;
  uint8_t   ucCommand;
  uint8_t   ucChecksum;
  uint8_t   ucTail;
} IMU_CMD;
static_assert(6 == sizeof(IMU_CMD), "structure size no align to one byte");

typedef struct __attribute__ ((packed)) _tagImuData {
  uint8_t   ucHeader[2];
  uint8_t   ucDataLen;

  uint8_t   ucAzimuth[2];
  uint8_t   ucPitch[2];
  uint8_t   ucRollAngle[2];

  uint8_t   ucAccelX[2];
  uint8_t   ucAccelY[2];
  uint8_t   ucAccelZ[2];
  
  uint8_t   ucAngularVelX[2];
  uint8_t   ucAngularVelY[2];
  uint8_t   ucAngularVelZ[2];
  
  uint8_t   ucMagneticX[2];
  uint8_t   ucMagneticY[2];
  uint8_t   ucMagneticZ[2];
  
  uint8_t   ucReserved[4];
  
  uint8_t   ucTemperature[2];
  uint8_t   ucTimeStamp[4];
  uint8_t   ucMagFlag;
  uint8_t   ucChecksum;
  uint8_t   ucTail;
} IMU_DATA;
static_assert(40 == sizeof(IMU_DATA), "structure size no align to one byte");

static inline float d2f_acc(uint8_t a[2]) {
    int16_t acc = a[0];
    acc = (acc << 8) | a[1];
    return ((float) acc) / 16384.0f;
}

static inline float d2f_gyro(uint8_t a[2]) {
    int16_t acc = a[0];
    acc = (acc << 8) | a[1];
    return ((float) acc) / 32.8f;
}

static inline float d2f_mag(uint8_t a[2]) {
    int16_t acc = a[0];
    acc = (acc << 8) | a[1];
    return ((float) acc) / 1.0f;
}

static inline float d2f_euler(uint8_t a[2]) {
    int16_t acc = a[0];
    acc = (acc << 8) | a[1];
    return ((float) acc) / 10.0f;
}

int main(int argc, char** argv) {
  int fd_ = -1;
  boost::asio::serial_port* serial_port = nullptr;
  const uint8_t stop[6] = {0xA5, 0x5A, 0x04, 0x02, 0x06, 0xAA};
  const uint8_t mode[6] = {0xA5, 0x5A, 0x04, 0x01, 0x05, 0xAA};
  uint8_t data_raw[4096], tmp[2048];

  std::vector<uint8_t> buffer_;
  std::deque<uint8_t> queue_;
  std::string name, frame_id;

  double linear_acceleration_stddev;
  double angular_velocity_stddev;
  double orientation_stddev;

  sensor_msgs::Imu msg;
  sensor_msgs::MagneticField msg_mag;
  sensor_msgs::NavSatFix msg_gps;

  ros::Publisher pub, pub_mag;

  ros::init(argc, argv, "imu");
  ros::NodeHandle n("~");

  name = ros::this_node::getName();

  std::string port;
  if (n.hasParam("port")) {
    n.getParam("port", port);
  } else {
    ROS_ERROR("%s: must provide a port", name.c_str());
    return -1;
  }

  std::string model;
  if (n.hasParam("model")) {
    n.getParam("model", model);
  } else {
    ROS_ERROR("%s: must provide a model name", name.c_str());
    return -1;
  }
  ROS_WARN("Model set to %s", model.c_str());

  int baud;
  if (n.hasParam("baud")) {
    n.getParam("baud", baud);
  } else {
    ROS_ERROR("%s: must provide a baudrate", name.c_str());
    return -1;
  }
  ROS_WARN("Baudrate set to %d", baud);

  n.param("frame_id", frame_id, string("IMU_link"));
  double delay;
  n.param("delay", delay, 0.0);
  // Standard Deviation,StdDev,标准偏差指标,目的是用来衡量**的波动性
  n.param<double>("linear_acceleration_stddev", linear_acceleration_stddev, 0.0);
  n.param<double>("angular_velocity_stddev", angular_velocity_stddev, 0.0);
  n.param<double>("orientation_stddev", orientation_stddev, 0.0);
  
  boost::asio::io_service io_service;
  serial_port = new boost::asio::serial_port(io_service);
  try {
    serial_port->open(port);
  } catch (boost::system::system_error &error) {
    ROS_ERROR("%s: Failed to open port %s with error %s",
              name.c_str(), port.c_str(), error.what());
    return -1;
  }

  if (!serial_port->is_open()) {
    ROS_ERROR("%s: failed to open serial port %s",
              name.c_str(), port.c_str());
    return -1;
  }

  typedef boost::asio::serial_port_base sb;

  sb::baud_rate baud_option(baud);
  sb::flow_control flow_control(sb::flow_control::none);
  sb::parity parity(sb::parity::none);
  sb::stop_bits stop_bits(sb::stop_bits::one);

  serial_port->set_option(baud_option);
  serial_port->set_option(flow_control);
  serial_port->set_option(parity);
  serial_port->set_option(stop_bits);

  const char *path = port.c_str();
  fd_ = open(path, O_RDWR);
  if(fd_ < 0) {
    ROS_ERROR("Port Error!: %s", path);
    return -1;
  }

  int kk = 0;
  double vyaw_sum = 0;
  double vyaw_bias = 0;
  pub = n.advertise<sensor_msgs::Imu>("/imu_data", 1);
  pub_mag = n.advertise<sensor_msgs::MagneticField>("mag", 1);

  if(model == "imu_100d2") {
    write(fd_, stop, 6);
    usleep(1000 * 1000);
    write(fd_, mode, 6);
    usleep(1000 * 1000);
  }

  ROS_WARN("Streaming Data...");
  int nbuf_length = 0;
  uint8_t *pbuf = &data_raw[0];

  while (n.ok()) {
    int nbytes = read(fd_, tmp, sizeof(tmp));
	double timestamp = ros::Time::now().toSec();
	if (nbytes < 1) continue;
	if (nbuf_length + nbytes > sizeof(data_raw)) {
      //TODO:
      nbuf_length = 0;
    }
    memcpy(&data_raw[nbuf_length], tmp, nbytes);
    nbuf_length += nbytes;

    bool found = false;
    pbuf = &data_raw[0];
    while (nbuf_length >= sizeof(IMU_DATA)) {
      IMU_DATA *pImu = reinterpret_cast<IMU_DATA *>(pbuf);
      if(model == "imu_100d2" && pImu->ucHeader[0] == IMU_HEADER_1 \
         && pImu->ucHeader[1] == IMU_HEADER_2) {
        if (IMU_TAILER == pImu->ucTail && pImu->ucDataLen == sizeof(IMU_DATA) - 3) {
          uint8_t ucChecksum = 0;
          for(int i = 2; i < sizeof(IMU_DATA) - 2; ++i)
            ucChecksum += pbuf[i];

          if (pImu->ucChecksum != ucChecksum) {
            --nbuf_length; ++pbuf;
            ROS_ERROR("Checksum Error [%x, %x].\n", pImu->ucChecksum, ucChecksum);
            continue;
          }

          Eigen::Vector3d ea0(-d2f_euler(pImu->ucAzimuth) * M_PI / 180.0, 
		                       d2f_euler(pImu->ucRollAngle) * M_PI / 180.0, 
							   d2f_euler(pImu->ucPitch) * M_PI / 180.0);
          Eigen::Matrix3d R;
          R = Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
              * Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
              * Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());
          Eigen::Quaterniond q;
          q = R;
          msg.orientation.w = (double)q.w();
          msg.orientation.x = (double)q.x();
          msg.orientation.y = (double)q.y();
          msg.orientation.z = (double)q.z();
          msg.orientation_covariance[0] = orientation_stddev;
          msg.orientation_covariance[4] = orientation_stddev;
          msg.orientation_covariance[8] = orientation_stddev;

          msg.header.stamp = ros::Time::now();
          msg.header.frame_id = frame_id;
		  // deg/s or rad/s
          // 角速度
          msg.angular_velocity.x = d2f_gyro(pImu->ucAngularVelX) * M_PI / 180.0;
          msg.angular_velocity.y = d2f_gyro(pImu->ucAngularVelY) * M_PI / 180.0;
          msg.angular_velocity.z = d2f_gyro(pImu->ucAngularVelZ) * M_PI / 180.0;
          msg.angular_velocity_covariance[0] = angular_velocity_stddev;
          msg.angular_velocity_covariance[4] = angular_velocity_stddev;
          msg.angular_velocity_covariance[8] = angular_velocity_stddev;
          // 线加速度
          msg.linear_acceleration.x = d2f_acc(pImu->ucAccelX) * 9.81;
          msg.linear_acceleration.y = d2f_acc(pImu->ucAccelY) * 9.81;
          msg.linear_acceleration.z = d2f_acc(pImu->ucAccelZ) * 9.81;
          msg.linear_acceleration_covariance[0] = linear_acceleration_stddev;
          msg.linear_acceleration_covariance[4] = linear_acceleration_stddev;
          msg.linear_acceleration_covariance[8] = linear_acceleration_stddev;

          //tf::quaternionTFToMsg(differential_rotation, msg.orientation); // 把tf四元数转化为geomsg四元数
          pub.publish(msg);

          msg_mag.magnetic_field.x = d2f_mag(pImu->ucMagneticX);
          msg_mag.magnetic_field.y = d2f_mag(pImu->ucMagneticY);
          msg_mag.magnetic_field.z = d2f_mag(pImu->ucMagneticZ);
          msg_mag.header.stamp = msg.header.stamp;
          msg_mag.header.frame_id = msg.header.frame_id;
          pub_mag.publish(msg_mag);

          double pitch = 180 * atan2(msg.linear_acceleration.x,
                         sqrt(msg.linear_acceleration.y * msg.linear_acceleration.y
                         + msg.linear_acceleration.z * msg.linear_acceleration.z)) / M_PI;
          double roll = 180 * atan2(msg.linear_acceleration.y,
                        sqrt(msg.linear_acceleration.x * msg.linear_acceleration.x 
                        + msg.linear_acceleration.z * msg.linear_acceleration.z)) / M_PI;
          double magx = msg_mag.magnetic_field.x * cos(pitch) +
                        msg_mag.magnetic_field.y * sin(roll) * sin(pitch) + 
                        msg_mag.magnetic_field.z * cos(roll) * sin(pitch);
          double magy = msg_mag.magnetic_field.y * cos(roll) -
                        msg_mag.magnetic_field.z * sin(roll);
          double yaw = 180 * atan2(-magy, magx) / M_PI;
#if 0
          tf::Matrix3x3 obs_mat;
          obs_mat.setEulerYPR(yaw, pitch, roll);
          tf::Quaternion orientation;
          obs_mat.getRotation(orientation);
          tf::Quaternion orientation2(msg.orientation.x, msg.orientation.y, 
                                      msg.orientation.z, msg.orientation.w);
ROS_INFO("\n=== Got Quaternion ===\n"
             " Quaternion\n"
             " x : %f\n y : %f\n z : %f\n w : %f\n"
             " Quaternion2\n"
             " x : %f\n y : %f\n z : %f\n w : %f\n"
             " RPY\n"
             " R : %f\n P : %f\n Y : %f",
             orientation.x(), orientation.y(), orientation.z(), orientation.w(),
             orientation2.x(), orientation2.y(), orientation2.z(), orientation2.w(),
             roll, pitch, yaw);
          static tf::Quaternion zero_orientation;
          static bool zero_orientation_set = false;
          if (!zero_orientation_set) {
            zero_orientation = orientation;
            zero_orientation_set = true;
            ROS_INFO_STREAM("zero_orientation = " << zero_orientation);
          }
          tf::Quaternion differential_rotation;
          differential_rotation = zero_orientation.inverse() * orientation;
ROS_INFO("\n=== Got Differential Rotation ===\n"
             " Quaternion\n"
             " x : %f\n y : %f\n z : %f\n w : %f\n",
             differential_rotation.x(), differential_rotation.y(), 
			 differential_rotation.z(), differential_rotation.w());
#endif
		  //          ROS_INFO("pitch = %lf, roll = %lf, yaw = %lf", pitch, roll, yaw);
          //printf("pitch = %lf, roll = %lf, yaw = %lf\n", pitch / 180 * M_PI, roll / 180 * M_PI, yaw / 180 * M_PI);

          found = true;
		  nbuf_length -= sizeof(IMU_DATA);
		  pbuf += sizeof(IMU_DATA);
        } else {
          --nbuf_length; ++pbuf;
        }
      } else {
        --nbuf_length; ++pbuf;
      }
    }
	
    assert(nbuf_length < sizeof(IMU_DATA));
    if (nbuf_length > 0 && pbuf != data_raw)
      memmove(&data_raw[0], pbuf, nbuf_length);
  
//    ROS_INFO("Time Elapse : %.6lf ms", (ros::Time::now().toSec() - timestamp) * 1000.);
  }

  // Stop continous and close device
  ROS_WARN("Wait 0.1s");
  ros::Duration(0.1).sleep();
  ::close(fd_);

  return 0;
}
