
#include <common.h>
#include <librealsense2/rs.hpp>
#include "/home/zk02/Downloads/stairDetection/stairperception/StairPerception.hpp"
#include "/home/zk02/Downloads/stairDetection/stairperception/Stair.hpp"

// include OpenCV header file
#include <opencv2/opencv.hpp>
#include </home/zk02/Downloads/stairDetection/rs2/realSense.h>

using namespace std;
using namespace cv;

#define width 640 
#define height 480 
#define fps 30
void update_planes(const std::vector<Plane> *vector_plane_sorted, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, int v2);

int main(int argc, char** argv) try
{
    RealSense m_rs2(true);

    ParameterDef parameters;
    
    //init parameters
    parameters.x_min_ = -5.;
    parameters.x_max_= 5.;
    parameters.y_min_= -5.;
    parameters.y_max_= 5.;
    parameters.z_min_= -5.;
    parameters.z_max_= 5.;
    parameters.normal_compute_points_=69;
    parameters.voxel_x_=1.0000000000000000e-02;
    parameters.voxel_y_= 1.0000000000000000e-02;
    parameters.voxel_z_=1.0000000000000000e-02;
    parameters.parallel_angle_diff_=1.9577777777777779e+01;
    parameters.perpendicular_angle_diff_=2.4466666666666665e+01;
    parameters.cluster_tolerance_=2.9600000000000001e-02;
    parameters.min_cluster_size_= 9;
    parameters.max_cluster_size_= 100000;
    parameters.seg_threshold_ =1.0000000000000000e-02;
    parameters.seg_max_iters_= 100;
    parameters.seg_rest_point_=236;
    parameters.seg_plane_angle_diff_= 7.;
    parameters.merge_threshold_= 2.0000000000000000e-02;
    parameters.merge_angle_diff_= 7.;
    parameters.min_num_points_= 150;
    parameters.min_length_=1.4999999999999999e-01;
    parameters.max_length_=1.;
    parameters.min_width_=5.0000000000000003e-02;
    parameters.max_width_=3.8000000000000000e-01;
    parameters.max_g_height_= 5.0000000000000012e-02;
    parameters.counter_max_distance_=2.5000000000000000e-01;
    parameters.min_height_=5.0000000000000003e-02;
    parameters.max_height_= 2.0000000000000001e-01;
    parameters.vertical_angle_diff_= 20.;
    parameters.mcv_angle_diff_= 80.;
    parameters.center_vector_angle_diff_= 70.;
    parameters.vertical_plane_angle_diff_=20.;
    parameters.noleg_distance_= 8.1100000000000005e-01;
    
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in;
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in;
    //pcl::visualization::CloudViewer viewer("Cloud Viewer");//创建对象
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud = CloudType::Ptr(new CloudType(width, height));
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in = CloudType::Ptr(new CloudType(width, height));
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_pre = CloudType::Ptr(new CloudType(width, height));
    //pointcloud
    rs2::pointcloud pc;
    rs2::points points;
    //rotation matrix
    Eigen::Matrix3f rotation_matrix;
    //用于显示输出
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("cloud_window"));
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_out(new pcl::visualization::PCLVisualizer("cloud_out"));
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_plane(new pcl::visualization::PCLVisualizer("plane_out"));


    ShowModeDef show_mode = seg_vertical_plane;
    //CloudType::Ptr cloud_in;  //
    CloudType cloud_in_copy;    
    CloudType cloud_out;
    pcl::PointCloud<pcl::Normal> normal_out;
    //记录已经分割出的平面
    std::vector<Plane> vector_plane_sorted;   
    bool has_stair;      
    //for show the line
    CloudType addon_cloud;

    //load parameters
    StairDetection stair_detection(parameters);
    Stair stair;
        
    // judge whether devices is exist or not 
	rs2::context ctx;
	auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
	if (list.size() == 0) 
		throw std::runtime_error("No device detected. Is it plugged in?");
	rs2::device dev = list.front();
    
    //
    rs2::frameset frames;
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;//创建一个通信管道//https://baike.so.com/doc/1559953-1649001.html pipeline的解释
    //Create a configuration for configuring the pipeline with a non default profile
	rs2::config cfg;//创建一个以非默认配置的配置用来配置管道
    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);//向配置添加所需的流
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16,fps);
    //cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
	//cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
   
    // get depth scale 
    // float depth_scale = get_depth_scale(profile.get_device());

    // start stream 
    pipe.start(cfg);//指示管道使用所请求的配置启动流
    int v1(0);
    int v2(0);
    
    while(1)
    {
        frames = pipe.wait_for_frames();//等待所有配置的流生成框架
        
        //清理所有的视窗
        viewer->removeAllPointClouds();
	    viewer->removeAllShapes();
        viewer_out->removeAllPointClouds();
	    viewer_out->removeAllShapes();
        viewer_plane->removeAllPointClouds();
        viewer_plane->removeAllShapes();
        vector_plane_sorted.clear();



        // Align to depth 
        rs2::align align_to_depth(RS2_STREAM_DEPTH);
        frames = align_to_depth.process(frames);
    

        // Get imu data---Gyro
        if (rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO))
        {
            rs2_vector gyro_sample = gyro_frame.get_motion_data();
            std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
            //计算旋转矩阵 realsense----->x:pitch y:yaw z:roll
            m_rs2.computeRotationMatrix(gyro_sample.x, gyro_sample.z, gyro_sample.y, rotation_matrix);
        }
        else
        {
            std::cout<<"Catch IMU Data Failed!"<<std::endl;
            continue;
        }
        


        //Get each frame
        rs2::frame color_frame = frames.get_color_frame();
        rs2::frame depth_frame = frames.get_depth_frame();
        //rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
        //rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);
        Mat color(Size(width, height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        Mat pic_depth(Size(width,height), CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
        
        /*
        namedWindow("Display Image", WINDOW_AUTOSIZE );
        imshow("Display Image", color);
        waitKey(1);
        namedWindow("Display Depth", WINDOW_AUTOSIZE );
        imshow("Display depth", pic_depth*15);
        waitKey(1);
        */

        //获得带RGB的三维点云图
        pc.map_to(color_frame);
        points = pc.calculate(depth_frame);

        //rs2转PCL
        cloud_pre = m_rs2.PCL_Conversion(points, color_frame);
        //cloud_in = m_rs2.PCL_Conversion(points, color_frame);
        //is rotation the right gyro?
        //旋转点云，旋转目的是使点云x轴与重力方向对齐，y轴与前视方向对齐，z轴对前视方向右侧
        m_rs2.rotationCloudPoints(cloud_pre, rotation_matrix, cloud_in);
        //cloud_in = m_rs2.points_to_pcl(points);
        //cloud_in = m_rs2.updateCloud(color_frame, depth_frame, cloud_in);
        //viewer.showCloud(cloud_in);
        cloud_in_copy = *cloud_in;
        //process，运行楼梯检测算法
        has_stair = stair_detection.process(cloud_in_copy, show_mode, stair, vector_plane_sorted, cloud_out, normal_out, addon_cloud);

        //show
        viewer->addPointCloud<pcl::PointXYZRGBA> (cloud_in, "sample cloud", v1);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	    //viewer->addCoordinateSystem(1.0);//显示坐标轴，
	    viewer->initCameraParameters(); 
	    viewer->setBackgroundColor(0, 0, 0);
	    viewer->setCameraPosition(1, 0, -2, 0, -1, 1, 0);

        //如果找到楼梯就输出“findstair！”
        if(has_stair == true){
            std::cout<<"find stair!"<<std::endl;
        }



        viewer_out->addPointCloud<pcl::PointXYZRGBA> (cloud_out.makeShared(), "output cloud");
        viewer_out->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud");
        //viewer->addCoordinateSystem(1.0);//显示坐标轴，
        viewer_out->initCameraParameters();
        viewer_out->setBackgroundColor(0, 0, 0);
        viewer_out->setCameraPosition(1, 0, -2, 0, -1, 1, 0);


        viewer_plane->initCameraParameters();
        viewer_plane->setBackgroundColor(0, 0, 0);
        viewer_plane->setCameraPosition(1, 0, -2, 0, -1, 1, 0);
        update_planes(&vector_plane_sorted, viewer, v2);
        viewer->spin();

        //show cloud
        // Creating OpenCV Matrix from a color image
        
        //Mat pic_right(Size(width,height), CV_8UC1, (void*)ir_frame_right.get_data());
        //Mat pic_left(Size(width,height), CV_8UC1, (void*)ir_frame_left.get_data());
        
        
        //cloud_in = m_rs2.getCloud(color, pic_depth, cloud_in);
        // Display in a GUI
 
        //imshow("Display pic_left", pic_left);
        //waitKey(1);
        //imshow("Display pic_right",pic_right);
        //waitKey(1);
        //viewer.showCloud(cloud_in);//显示

        //get cloud

    }
    return 0;
}

// error
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}



void update_planes(const std::vector<Plane> *vector_plane_sorted, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_plane, int v2)
{

    std::string viewr_cloud_name = "rcloud_";
    if (vector_plane_sorted->size() > 0)
    {
        srand(0);
        for (unsigned int i = 0; i < vector_plane_sorted->size(); i++)
        {
            if ((*vector_plane_sorted)[i].ptype == Plane::Ptype::stair_component)
            {
                double r, g, b;
                r = int(255.0 * rand() / (RAND_MAX + 1.0));
                g = int(255.0 * rand() / (RAND_MAX + 1.0));
                b = int(255.0 * rand() / (RAND_MAX + 1.0));
                pcl::visualization::PointCloudColorHandlerCustom<PointType>
                        single_color((*vector_plane_sorted)[i].cloud.makeShared(), r, g, b);

                std::stringstream ss;

                // add cloud
                ss << viewr_cloud_name << i << "c";
                viewer_plane->addPointCloud<PointType>((*vector_plane_sorted)[i].cloud.makeShared(), single_color, ss.str(), v2);
                viewer_plane->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());

                // add center
                ss << "c";
                // viewer->addSphere(vector_plane_sorted[i].center, 0.02, r / 255.0, g / 255.0, b / 255.0, ss.str());

                // add counter
                ss << "c";
                viewer_plane->addPolygon<pcl::PointXYZ>((*vector_plane_sorted)[i].counter.makeShared(), r / 255.0, g / 255.0,
                                                  b / 255.0, ss.str());

            } else if ((*vector_plane_sorted)[i].ptype == Plane::Ptype::others)
            {
                pcl::visualization::PointCloudColorHandlerCustom<PointType>
                        single_color((*vector_plane_sorted)[i].cloud.makeShared(), 255, 0, 0);

                std::stringstream ss;

                // add cloud
                ss << viewr_cloud_name << i << "c";
                viewer_plane->addPointCloud<PointType>((*vector_plane_sorted)[i].cloud.makeShared(), single_color, ss.str());
                viewer_plane->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());
            } else if ((*vector_plane_sorted)[i].ptype == Plane::Ptype::ground)
            {
                pcl::visualization::PointCloudColorHandlerCustom<PointType>
                        single_color((*vector_plane_sorted)[i].cloud.makeShared(), 0, 255, 0);

                std::stringstream ss;

                // add cloud
                ss << viewr_cloud_name << i << "c";
                viewer_plane->addPointCloud<PointType>((*vector_plane_sorted)[i].cloud.makeShared(), single_color, ss.str());
                viewer_plane->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, ss.str());
            }
        }
    }

    //ui->qvtkwidget->update();
}