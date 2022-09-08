#include "common.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <signal.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <chrono>
#include <Eigen/Core>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <cstring>

#include </usr/local/include/librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_sensor.hpp>
#include <librealsense2/hpp/rs_device.hpp>





class RealSense
{
    public:
        rs2::pipeline m_pipeline;
        rs2::frameset m_data;
        bool mirror_; //jing xiang
        rs2::config m_cfg;
        rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
        float qnan_;
        Eigen::Matrix<float, IMG_WIDTH, 1> colmap;
        Eigen::Matrix<float, IMG_HEIGHT, 1> rowmap;       
        std::string m_serial;
        int m_map[IMG_WIDTH * IMG_HEIGHT];

    public:
        RealSense(bool mirror_);
        bool start();
        bool stop();
        bool shutdown();
       
        void getDepth(cv::Mat depth_mat);
        //get images that color image aligned to depth image
        rs2::depth_frame getAligned();
        void getImuData(float &pitch, float &roll, float &yaw);


        // All frame and cloud are aligned. There is a small overhead in the double call to registration->apply which has to be removed
        void get(cv::Mat &color_mat, cv::Mat &depth_mat, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
        
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloud();
        
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getCloud(const rs2::video_frame &rgb, const rs2::depth_frame &depth, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr updateCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr updateCloud(const rs2::video_frame &rgb, const rs2::depth_frame &depth, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

        //RS2 TO PCL
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr points_to_pcl(const rs2::points& points);
       
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PCL_Conversion(const rs2::points& points, const rs2::video_frame& color);
        std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);
        void computeRotationMatrix(const float W, const float X, const float Y, const float Z, Eigen::Matrix3f &rotation_matrix);
        
        void computeRotationMatrix(const float pitch, const float roll, const float yaw, Eigen::Matrix3f &rotation_matrix);
        void rotationCloudPoints(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in, const Eigen::Matrix3f rotation_matrix, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotation);
        
};
