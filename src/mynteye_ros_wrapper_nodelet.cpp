#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>

#include "calibration_parameters.h"
#include "camera.h"
#include "init_parameters.h"

namespace mynt_wrapper
{
#ifdef USE_OPENCV2
void CompatDistCoeffs(cv::Mat &distCoeffs)
{
  int w = distCoeffs.cols;
  if(w >= 8)
    w = 8;
  else if(w >= 5)
    w = 5;
  else if(w >= 4)
    w = 4;
  else
    CV_Assert(false);
  distCoeffs = distCoeffs.row(0).colRange(0, w);
}
#endif

class MYNTWrapperNodelet : public nodelet::Nodelet
{
  ros::NodeHandle nh_ns;
  boost::thread device_poll_thread;

  mynteye::Camera cam;

  mynteye::Resolution resolution = cam.GetResolution();

  image_transport::Publisher pub_raw_right;
  image_transport::Publisher pub_raw_left;

  ros::Publisher pub_left_cam_info;
  ros::Publisher pub_right_cam_info;
  ros::Publisher pub_imu;

  std::string right_frame_id;
  std::string left_frame_id;
  std::string depth_frame_id;
  std::string imu_frame_id;

  int device_name;
  mynteye::Rate camera_rate;

  std::uint32_t imu_time_beg = -1;
  double imu_ros_time_beg;

  std::uint32_t img_time_beg = -1;
  double img_ros_time_beg;

  sensor_msgs::ImagePtr imageToROSmsg(const cv::Mat &img,
                                      const std::string &encodingType,
                                      const std::string &frameId,
                                      const ros::Time &stamp)
  {
    auto imgMessage = boost::make_shared<sensor_msgs::Image>();
    imgMessage->header.stamp = stamp;
    imgMessage->header.frame_id = frameId;
    imgMessage->height = img.rows;
    imgMessage->width = img.cols;
    imgMessage->encoding = encodingType;
    int num = 1;
    imgMessage->is_bigendian = !(*(char *)&num == 1);
    imgMessage->step = img.cols * img.elemSize();
    size_t size = imgMessage->step * img.rows;
    imgMessage->data.resize(size);

    if(img.isContinuous())
      memcpy(&imgMessage->data[0], img.data, size);
    else
    {
      uchar *opencvData = img.data;
      uchar *rosData = &imgMessage->data[0];
      for(unsigned int i = 0; i < img.rows; i++)
      {
        memcpy(rosData, opencvData, imgMessage->step);
        rosData += imgMessage->step;
        opencvData += img.step;
      }
    }

    return imgMessage;
  }

  void publishImage(const cv::Mat &img,
                    const image_transport::Publisher &pub_img,
                    const std::string &img_frame_id, const ros::Time &stamp)
  {
    pub_img.publish(imageToROSmsg(img, sensor_msgs::image_encodings::MONO8,
                                  img_frame_id, stamp));
  }

  void publishIMU(const mynteye::IMUData &imudata, const ros::Time &stamp)
  {
    auto msg = boost::make_shared<sensor_msgs::Imu>();

    msg->header.stamp = stamp;
    msg->header.frame_id = imu_frame_id;

    msg->linear_acceleration.x = imudata.accel_x * 9.81;
    msg->linear_acceleration.y = imudata.accel_y * 9.81;
    msg->linear_acceleration.z = imudata.accel_z * 9.81;

    msg->linear_acceleration_covariance[0] = 0.04;
    msg->linear_acceleration_covariance[1] = 0;
    msg->linear_acceleration_covariance[2] = 0;

    msg->linear_acceleration_covariance[3] = 0;
    msg->linear_acceleration_covariance[4] = 0.04;
    msg->linear_acceleration_covariance[5] = 0;

    msg->linear_acceleration_covariance[6] = 0;
    msg->linear_acceleration_covariance[7] = 0;
    msg->linear_acceleration_covariance[8] = 0.04;

    msg->angular_velocity.x = imudata.gyro_x * M_PI / 180;
    msg->angular_velocity.y = imudata.gyro_y * M_PI / 180;
    msg->angular_velocity.z = imudata.gyro_z * M_PI / 180;

    msg->angular_velocity_covariance[0] = 0.02;
    msg->angular_velocity_covariance[1] = 0;
    msg->angular_velocity_covariance[2] = 0;

    msg->angular_velocity_covariance[3] = 0;
    msg->angular_velocity_covariance[4] = 0.02;
    msg->angular_velocity_covariance[5] = 0;

    msg->angular_velocity_covariance[6] = 0;
    msg->angular_velocity_covariance[7] = 0;
    msg->angular_velocity_covariance[8] = 0.02;

    pub_imu.publish(msg);
  }

  void publishCamInfo(const sensor_msgs::CameraInfo &cam_info,
                      const ros::Publisher &pub_cam_info,
                      const ros::Time &stamp)
  {
    // Need to allocate a new message for every publish in a nodelet
    const auto cam_info_msg =
        boost::make_shared<sensor_msgs::CameraInfo>(cam_info);
    // static int seq = 0;
    cam_info_msg->header.stamp = stamp;
    // cam_info_msg->header.seq = seq;
    pub_cam_info.publish(cam_info_msg);
    //++seq;
  }

  void fillCamInfo(const mynteye::Resolution &resolution,
                   const mynteye::CalibrationParameters &calibration_parameters,
                   sensor_msgs::CameraInfo &left_cam_info_msg,
                   sensor_msgs::CameraInfo &right_cam_info_msg,
                   const std::string &left_frame_id,
                   const std::string &right_frame_id)
  {
    int width = resolution.width;
    int height = resolution.height;

    left_cam_info_msg.distortion_model =
        sensor_msgs::distortion_models::PLUMB_BOB;
    right_cam_info_msg.distortion_model =
        sensor_msgs::distortion_models::PLUMB_BOB;

    left_cam_info_msg.D.resize(5);
    right_cam_info_msg.D.resize(5);

    left_cam_info_msg.D[0] = calibration_parameters.D1.at<double>(0, 0);
    right_cam_info_msg.D[0] = calibration_parameters.D2.at<double>(0, 0);

    left_cam_info_msg.D[1] = calibration_parameters.D1.at<double>(0, 1);
    right_cam_info_msg.D[1] = calibration_parameters.D2.at<double>(0, 1);

    left_cam_info_msg.D[2] = calibration_parameters.D1.at<double>(0, 7);
    right_cam_info_msg.D[2] = calibration_parameters.D2.at<double>(0, 7);

    left_cam_info_msg.D[3] = 0.0;
    right_cam_info_msg.D[3] = 0.0;

    left_cam_info_msg.D[4] = 0.0;
    right_cam_info_msg.D[4] = 0.0;

    for(int i = 0; i < 3; i++)
    {
      for(int j = 0; j < 3; j++)
      {
        left_cam_info_msg.K[i * 3 + j] =
            calibration_parameters.M1.at<double>(i, j);
        right_cam_info_msg.K[i * 3 + j] =
            calibration_parameters.M2.at<double>(i, j);

        left_cam_info_msg.R[i * 3 + j] =
            calibration_parameters.R.at<double>(i, j);
        right_cam_info_msg.R[i * 3 + j] =
            calibration_parameters.R.at<double>(i, j);
      }
    }

    cv::Size img_size(width, height);
    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect leftROI, rightROI;
#ifdef USE_OPENCV2
    CompatDistCoeffs(calibration_parameters.D1);
    CompatDistCoeffs(calibration_parameters.D2);
#endif
    cv::stereoRectify(
        calibration_parameters.M1, calibration_parameters.D1,
        calibration_parameters.M2, calibration_parameters.D2, img_size,
        calibration_parameters.R, calibration_parameters.T, R1, R2, P1, P2, Q,
        cv::CALIB_ZERO_DISPARITY, 0, img_size, &leftROI, &rightROI);

    for(int i = 0; i < R1.rows; i++)
    {
      for(int j = 0; j < R1.cols; j++)
      {
        left_cam_info_msg.R[i * R1.cols + j] = R1.at<double>(i, j);
        right_cam_info_msg.R[i * R1.cols + j] = R2.at<double>(i, j);
      }
    }
    for(int i = 0; i < P1.rows; i++)
    {
      for(int j = 0; j < P1.cols; j++)
      {
        left_cam_info_msg.P[i * P1.cols + j] = P1.at<double>(i, j);
        right_cam_info_msg.P[i * P1.cols + j] = P2.at<double>(i, j);
      }
    }
    left_cam_info_msg.width = right_cam_info_msg.width = width;
    left_cam_info_msg.height = right_cam_info_msg.height = height;
    left_cam_info_msg.header.frame_id = left_frame_id;
    right_cam_info_msg.header.frame_id = right_frame_id;
  }

  ros::Time getImgStamp(bool reset = false)
  {
    if(reset)
    {
      img_time_beg = -1;
      return ros::Time::now();
    }
    if(img_time_beg == -1)
    {
      img_time_beg = cam.GetTimestamp();
      img_ros_time_beg = ros::Time::now().toSec();
    }
    return ros::Time(img_ros_time_beg +
                     (cam.GetTimestamp() - img_time_beg) * 0.0001f);
  }

  ros::Time getIMUStamp(const mynteye::IMUData *imudata, bool reset = false)
  {
    if(reset)
    {
      imu_time_beg = -1;
      return ros::Time::now();
    }
    if(imu_time_beg == -1)
    {
      imu_time_beg = imudata->time;
      imu_ros_time_beg = ros::Time::now().toSec();
    }
    return ros::Time(imu_ros_time_beg +
                     (imudata->time - imu_time_beg) * 0.0001f);
  }

  void device_poll()
  {
    using namespace mynteye;

    InitParameters params(std::to_string(device_name));
    cam.Open(params);

    if(!cam.IsOpened())
    {
      NODELET_ERROR("Error: Open camera failed");
      return;
    }

    cam.SetRate(camera_rate);

    const auto calib_params = cam.GetCalibrationParameters();

    sensor_msgs::CameraInfo left_cam_info, right_cam_info;
    fillCamInfo(resolution, calib_params, left_cam_info, right_cam_info,
                left_frame_id, right_frame_id);

    cv::Mat img_left, img_right;
    std::vector<IMUData> imudatas;

    while(nh_ns.ok())
    {
      if(cam.Grab() != ErrorCode::SUCCESS)
        continue;

      if(cam.RetrieveImage(img_left, View::VIEW_LEFT_UNRECTIFIED) ==
             ErrorCode::SUCCESS &&
         cam.RetrieveImage(img_right, View::VIEW_RIGHT_UNRECTIFIED) ==
             ErrorCode::SUCCESS)
      {
        const auto timestamp = getImgStamp();
        publishImage(img_left, pub_raw_left, left_frame_id, timestamp);
        publishImage(img_right, pub_raw_right, right_frame_id, timestamp);
        publishCamInfo(left_cam_info, pub_left_cam_info, timestamp);
        publishCamInfo(right_cam_info, pub_right_cam_info, timestamp);
      }
      else
      {
        NODELET_WARN("Reset Image begin time");
        getImgStamp(true); // reset
      }

      if(cam.RetrieveIMUData(imudatas) == ErrorCode::SUCCESS &&
         !imudatas.empty())
      {
        for(const auto &imudata : imudatas)
        {
          ros::Time imu_ros_time = getIMUStamp(&imudata);

          publishIMU(imudata, imu_ros_time);
          // Sleep 1ms, otherwise publish may drop some IMUs.
          ros::Duration(0.001).sleep();
        }
      }
      else
      {
        NODELET_WARN("Reset IMU begin time");
        getIMUStamp(nullptr, true); // reset
      }
    }
  }

  void onInit()
  {
    nh_ns = getPrivateNodeHandle();

    nh_ns.param("device_name", device_name, 1);

    std::string left_raw_topic, left_cam_info_topic, right_raw_topic,
        right_cam_info_topic, imu_topic;
    nh_ns.param("left_raw_topic", left_raw_topic,
                std::string("left/image_raw"));
    nh_ns.param("left_cam_info_topic", left_cam_info_topic,
                std::string("left/camera_info"));
    nh_ns.param("right_raw_topic", right_raw_topic,
                std::string("right/image_raw"));
    nh_ns.param("right_cam_info_topic", right_cam_info_topic,
                std::string("right/camera_info"));
    nh_ns.param("imu_topic", imu_topic, std::string("imu"));

    nh_ns.param("left_frame_id", left_frame_id, std::string("mynt_left_frame"));
    nh_ns.param("right_frame_id", right_frame_id,
                std::string("mynt_right_frame"));
    nh_ns.param("imu_frame_id", imu_frame_id, std::string("mynt_imu_frame"));

    int camera_rate_int;
    nh_ns.param("camera_rate", camera_rate_int, 0);
    if(camera_rate_int == 0)
      camera_rate = mynteye::RATE_50_FPS_500_HZ;
    else if(camera_rate_int == 1)
      camera_rate = mynteye::RATE_25_FPS_500_HZ;
    else if(camera_rate_int == 2)
      camera_rate = mynteye::RATE_10_FPS_200_HZ;
    else
    {
      NODELET_WARN("Invalid camera_rate value, setting to RATE_50_FPS_500_HZ");
      camera_rate = mynteye::RATE_50_FPS_500_HZ;
    }

    image_transport::ImageTransport it_mynteye(nh_ns);

    pub_raw_left = it_mynteye.advertise(left_raw_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << left_raw_topic);
    pub_left_cam_info =
        nh_ns.advertise<sensor_msgs::CameraInfo>(left_cam_info_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << left_cam_info_topic);

    pub_raw_right = it_mynteye.advertise(right_raw_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << right_raw_topic);
    pub_right_cam_info =
        nh_ns.advertise<sensor_msgs::CameraInfo>(right_cam_info_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << right_cam_info_topic);

    pub_imu = nh_ns.advertise<sensor_msgs::Imu>(imu_topic, 1);
    NODELET_INFO_STREAM("Advertized on topic " << imu_topic);

    device_poll_thread =
        boost::thread(boost::bind(&MYNTWrapperNodelet::device_poll, this));
  }

  ~MYNTWrapperNodelet() { cam.Close(); }
};

} // namespace mynt_wrapper

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mynt_wrapper::MYNTWrapperNodelet, nodelet::Nodelet);

