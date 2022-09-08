#include "realSense.h"


bool stop = false;

void sigint_handler(int s)
{
    stop = true;
}


RealSense::RealSense(bool mirror_)
{
    //m_undistorted(512, 424, 4);
    //m_registerted(512, 424, 4);
    mirror_ = true;
    //std::cout<<"RealSense::RealSense(bool mirror)"<<std::endl;
    qnan_ = std::numeric_limits<float>::quiet_NaN();

}




bool RealSense::start()
{
    std::cout<<"START ENGINE!"<<std::endl;
    signal(SIGINT, sigint_handler);
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    m_cfg.enable_stream(RS2_STREAM_COLOR, IMG_WIDTH, IMG_HEIGHT, RS2_FORMAT_BGR8, 30);
    m_cfg.enable_stream(RS2_STREAM_DEPTH, IMG_WIDTH, IMG_HEIGHT, RS2_FORMAT_Z16, 30);
    m_cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    
    //m_pipeline.start(m_cfg);
    m_pipeline.start(m_cfg);
    //std::cout<<" m_cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F)"<<std::endl;

}



bool RealSense::stop()
{

}


bool RealSense::shutdown()
{

}


// All frame and cloud are aligned. There is a small overhead in the double call to registration->apply which has to be removed
void RealSense::get(cv::Mat &color_mat, cv::Mat &depth_mat, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    //std::cout<<"void RealSense::get(cv::Mat &color_mat, cv::Mat &depth_mat, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)"<<std::endl;
    m_data = m_pipeline.wait_for_frames(15000U);
    //std::cout<<"rs2::frameset data_ = m_pipeline.wait_for_frames();"<<std::endl;
    rs2::video_frame rgb = m_data.get_color_frame();
    rs2::depth_frame depth = m_data.get_depth_frame();


    std::cout<<"frame_height:"<<rgb.get_height()<<"\n"<<"frame_width:"<<rgb.get_width()<<std::endl;
    cv::Mat tmp_depth(depth.get_height(), depth.get_width(), CV_32FC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat tmp_color;
    
    tmp_color = cv::Mat(rgb.get_height(), rgb.get_width(), CV_8UC4, (void*)rgb.get_data());

    cv::flip(tmp_depth, depth_mat, 1);
    cv::flip(tmp_color, color_mat, 1);
    
    cloud = getCloud(rgb, depth, cloud);    
}



pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RealSense::getCloud(const rs2::video_frame &rgb, const rs2::depth_frame &depth, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
    //std::cout<<"pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RealSense::getCloud(const rs2::video_frame &rgb, const rs2::depth_frame &depth, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)"<<std::endl;
    return updateCloud(rgb, depth, cloud);
}



pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RealSense::updateCloud(const rs2::video_frame &rgb, const rs2::depth_frame &depth, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{

    //registration_->apply(rgb, depth, &undistorted_, &registered_, true, &big_mat_, map_);
    //std::cout<<"pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RealSense::updateCloud(const rs2::video_frame &rgb, const rs2::depth_frame &depth, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)"<<std::endl;
    const std::size_t w = IMG_WIDTH;
    const std::size_t h = IMG_HEIGHT;

    if (cloud->size() != w * h)
        cloud->resize(w * h);

    cv::Mat tmp_itD0(depth.get_height(), depth.get_width(), CV_8UC4, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat tmp_itRGB0(rgb.get_height(), rgb.get_width(), CV_8UC4, (void*)rgb.get_data(), cv::Mat::AUTO_STEP);
    //std::cout<<" cv::Mat tmp_itRGB0(rgb.get_height(), rgb.get_width(), CV_8UC4, const_cast<void*> (rgb.get_data()));"<<std::endl;
           
        cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
        cv::imshow("Display Image", tmp_itRGB0);
        cv::waitKey(1);
        cv::namedWindow("Display depth", cv::WINDOW_AUTOSIZE );
        cv::imshow("Display depth", tmp_itD0*15);
        cv::waitKey(1);
        

    if (mirror_ == true)
    {

        cv::flip(tmp_itD0, tmp_itD0, 1);
        cv::flip(tmp_itRGB0, tmp_itRGB0, 1);

    }

    const float *itD0 = (float *) tmp_itD0.ptr();
    const char *itRGB0 = (char *) tmp_itRGB0.ptr();

    pcl::PointXYZRGBA *itP = &cloud->points[0];
    bool is_dense = true;
    //std::cout<<"bool is_dense = true;"<<std::endl;
//#pragma omp parallel for

    for (std::size_t y = 0; y < h; ++y)
    {

        const unsigned int offset = y * w;
        const float *itD = itD0 + offset;  //locate the position
        //const char *itRGB = itRGB0 + offset * 4;
        //const float dy = rowmap(y);
        const float dy = y;
        //bool is_dense = true;
        pcl::PointXYZRGBA *itPc = itP + offset; //locate the position
        
        /*
        for (std::size_t x = 0; x < w; ++x, ++itPc, ++itD, itRGB += 4)
        {
            const float depth_value = *itD / 1000.0f;

            if (!std::isnan(depth_value) && !(std::abs(depth_value) < 0.0001))
            {

                const float rx = colmap(x) * depth_value;
                const float ry = dy * depth_value;
                itPc->z = depth_value;
                itPc->x = rx;
                itPc->y = ry;

                itPc->b = itRGB[0];
                itPc->g = itRGB[1];
                itPc->r = itRGB[2];
                itPc->a = 255;
            } 
            else
            {
                itPc->z = qnan_;
                itPc->x = qnan_;
                itPc->y = qnan_;

                itPc->b = qnan_;
                itPc->g = qnan_;
                itPc->r = qnan_;
                is_dense = false;
            }
        }
        */
        for (std::size_t x = 0; x < w; ++x, ++itPc, ++itD)
        {
            //const float depth_value = *itD / 1000.0f;
            float depth_value = *itD / 1000.0f;

            if (!std::isnan(depth_value) && !(std::abs(depth_value) < 0.0001))
            {


                //const float rx = colmap(x) * depth_value;
                //const float ry = dy * depth_value;
                float rx = x * depth_value;                
                float ry = dy * depth_value;
                
                std::cout<<"rx: "<< rx <<" \n"<<"ry: "<<ry<<"\n"<<"dy: "<<dy<<std::endl;
                std::cout<<"colmap(x): "<< colmap(x) <<" \n"<<"depth_value: "<<depth_value<<std::endl;
                //itPc->z = depth_value;
                itPc->z = depth_value;
                itPc->x = rx;
                itPc->y = ry;
                

                itPc->b = 0;
                itPc->g = 255;
                itPc->r = 0;
                itPc->a = 255;
            } 
            else
            {
                itPc->z = qnan_;
                itPc->x = qnan_;
                itPc->y = qnan_;

                itPc->b = qnan_;
                itPc->g = qnan_;
                itPc->r = qnan_;
                is_dense = false;
            }
        }
    }
    cloud->is_dense = is_dense;
    /*
#ifdef WITH_SERIALIZATION
    if (serialize_)
        serializeCloud(cloud);
#endif
*/
    std::cout<<" pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RealSense::updateCloud finished!"<<std::endl;
    std::cout<<cloud->points.size()<<std::endl;
    return cloud;
}



void RealSense::getDepth(cv::Mat depth_mat)
{   
    std::cout<<"void RealSense::getDepth(cv::Mat depth_mat)"<<std::endl;
    m_data = m_pipeline.wait_for_frames();
    auto depth = m_data.get_depth_frame();

    cv::Mat depth_tmp(depth.get_height(), depth.get_width(), CV_32FC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

    if (mirror_ == true)
    {
        cv::flip(depth_tmp, depth_mat, 1);
    } else
    {
        depth_mat = depth_tmp.clone();
    }

}


/*

 //   rs2_vector      translation;          /**< X, Y, Z values of translation, in meters (relative to initial position)                                    */
 //   rs2_vector      velocity;             /**< X, Y, Z values of velocity, in meters/sec                                                                  */
  //  rs2_vector      acceleration;         /**< X, Y, Z values of acceleration, in meters/sec^2                                                            */
 //   rs2_quaternion  rotation;             /**< Qi, Qj, Qk, Qr components of rotation as represented in quaternion rotation (relative to initial position) */
 //   rs2_vector      angular_velocity;     /**< X, Y, Z values of angular velocity, in radians/sec                                                         */
 //   rs2_vector      angular_acceleration; /**< X, Y, Z values of angular acceleration, in radians/sec^2                                                   */
 //   unsigned int    tracker_confidence;   /**< Pose confidence 0x0 - Failed, 0x1 - Low, 0x2 - Medium, 0x3 - High                                          */
  //  unsigned int    mapper_confidence;    /**< Pose map confidence 0x0 - Failed, 0x1 - Low, 0x2 - Medium, 0x3 - High                                      */

void RealSense::getImuData(float &pitch, float &roll, float &yaw)
{
    std::cout<<"void RealSense::getImuData(float Quat[4])"<<std::endl;
    //get imu
    //rs2::config cfg;
    //cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    //m_pipeline.start(cfg);  

    rs2::frameset frames = m_pipeline.wait_for_frames();
    std::cout<<"rs2::frameset frames = m_pipeline.wait_for_frames();"<<std::endl;
    auto f = frames.first_or_default(RS2_STREAM_GYRO);
    rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);
    rs2_vector gyro_sample = gyro_frame.get_motion_data();
    pitch = gyro_sample.x;
    roll = gyro_sample.y;
    yaw = gyro_sample.z;
    
    // Cast the frame to pose_frame and get its data
    //auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

    std::cout<<"void RealSense::getImuData finished!"<<std::endl;
}


/*
rs2::depth_frame RealSense::getAligned()
{
    rs2::colorizer c;                     // Helper to colorize depth images
       // Create a pipeline to easily configure and start the camera
    rs2::pipeline pipe;
    //Calling pipeline's start() without any additional parameters will start the first device
    // with its default streams.
    //The start function returns the pipeline profile which the pipeline used to start the device
    rs2::pipeline_profile profile = pipe.start();

    // Each depth camera might have different units for depth pixels, so we get it here
    // Using the pipeline's profile, we can retrieve the device that the pipeline uses
    float depth_scale = profile.get_device().get_depth_scale();

    //Pipeline could choose a device that does not have a color stream
    //If there is no color stream, choose to align depth to another stream
    rs2_stream align_to = find_stream_to_align(profile.get_streams());

    //The "align_to" is the stream type to which we plan to align depth frames.
    rs2::align align(align_to);

    // Define a variable for controlling the distance to clip
    float depth_clipping_distance = 1.f;

    // Using the align object, we block the application until a frameset is available
    rs2::frameset frameset = pipe.wait_for_frames();

    // rs2::pipeline::wait_for_frames() can replace the device it uses in case of device error or disconnection.
    // Since rs2::align is aligning depth to some other stream, we need to make sure that the stream was not changed
    //  after the call to wait_for_frames();
    if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
    {
        //If the profile was changed, update the align object, and also get the new device's depth scale
        profile = pipe.get_active_profile();
        align_to = find_stream_to_align(profile.get_streams());
        align = rs2::align(align_to);
        depth_scale = get_depth_scale(profile.get_device());
    }

    //Get processed aligned frame
    auto processed = align.process(frameset);

    // Trying to get both other and aligned depth frames
    rs2::video_frame other_frame = processed.first(align_to);
    rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

    //If one of them is unavailable, continue iteration
    return aligned_depth_frame;    
}


*/


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RealSense::points_to_pcl(const rs2::points& points)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
 
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        p.r = 0;
        p.g = 255;
        p.b = 0;
        p.a = 0;
        ptr++;
    }
 
    return cloud;
}


pcl::PointCloud<pcl::PointXYZRGBA>::Ptr RealSense::PCL_Conversion(const rs2::points& points, const rs2::video_frame& color){
 
    // Object Declaration (Point Cloud)
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
 
    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;
 
    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    
    cloud->width  = static_cast<uint32_t>( sp.width()  );   
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize( points.size() );
 
    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();
 
    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {   
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;
 
        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(color, Texture_Coord[i]);
 
        // Mapping Color (BGR due to Camera Model)
        cloud->points[i].r = std::get<2>(RGB_Color); // Reference tuple<2>
        cloud->points[i].g = std::get<1>(RGB_Color); // Reference tuple<1>
        cloud->points[i].b = std::get<0>(RGB_Color); // Reference tuple<0>
 
    }
    
   return cloud; // PCL RGB Point Cloud generated
}


std::tuple<int, int, int> RealSense::RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels
    
    // Normals to Texture Coordinates conversion
    int x_value = std::min(std::max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = std::min(std::max(int(Texture_XY.v * height + .5f), 0), height - 1);
 
    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);
 
    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());
    
    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];
 
    return std::tuple<int, int, int>(NT1, NT2, NT3);
}




void RealSense::computeRotationMatrix(const float W, const float X, const float Y, const float Z, Eigen::Matrix3f &rotation_matrix)
{
    Eigen::Quaternionf quaternionf(W, X, Y, Z);
    Eigen::Matrix3f m1,m2,m3;

    m1 <<   0, 0, 1,
            -1, 0, 0,
            0, -1, 0;

    m2 = quaternionf.normalized().toRotationMatrix();

    m3 <<   0, 0, -1,
            1, 0, 0,
            0,-1, 0;

    rotation_matrix = m3 * m2 * m1;
}

void RealSense::rotationCloudPoints(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in, const Eigen::Matrix3f rotation_matrix, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rotation)
{
    std::cout<<"void rotationCloudPoints start"<<std::endl;
    cloud_rotation->is_dense = false;
    cloud_rotation->resize(IMG_WIDTH * IMG_HEIGHT);
    float r00, r01, r02, r10, r11, r12, r20, r21, r22;

    r00 = rotation_matrix(0, 0);
    r01 = rotation_matrix(0, 1);
    r02 = rotation_matrix(0, 2);
    r10 = rotation_matrix(1, 0);
    r11 = rotation_matrix(1, 1);
    r12 = rotation_matrix(1, 2);
    r20 = rotation_matrix(2, 0);
    r21 = rotation_matrix(2, 1);
    r22 = rotation_matrix(2, 2);

#ifndef _OPENMP
    std::cerr<< "OpenMP not supported";
#endif

    size_t height = IMG_HEIGHT, width = IMG_WIDTH;
    std::cout<<"size_t height = 480, width = 640;"<<std::endl;
//#pragma omp parallel for

    for (size_t i = 0; i < height; i = i + 1)
    {   
        std::cout<<" i=:" <<i <<std::endl;
        for (size_t j = 0; j < width; j = j + 1)
        {
            
            cloud_rotation->points[i * width + j].r = cloud_in->points[i * width + j].r;
            //std::cout<<" cloud_in.points--r:" <<cloud_in->points[i * width + j].r <<std::endl;
            cloud_rotation->points[i * width + j].g = cloud_in->points[i * width + j].g;
            cloud_rotation->points[i * width + j].b = cloud_in->points[i * width + j].b;
            cloud_rotation->points[i * width + j].a = cloud_in->points[i * width + j].a;
            //std::cout<<" cloud_in.points--a:" <<cloud_in->points[i * width + j].a <<std::endl;
            if (!std::isnan(cloud_in->points[i * width + j].x))
            {
                cloud_rotation->points[i * width + j].x =
                        r00 * cloud_in->points[i * width + j].x +
                        r01 * cloud_in->points[i * width + j].y +
                        r02 * cloud_in->points[i * width + j].z;
                cloud_rotation->points[i * width + j].y =
                        r10 * cloud_in->points[i * width + j].x +
                        r11 * cloud_in->points[i * width + j].y +
                        r12 * cloud_in->points[i * width + j].z;
                cloud_rotation->points[i * width + j].z =
                        r20 * cloud_in->points[i * width + j].x +
                        r21 * cloud_in->points[i * width + j].y +
                        r22 * cloud_in->points[i * width + j].z;
            }
        }
    }

    //std::cout<<"cloud_rotation.width = static_cast<uint32_t>(width);"<<std::endl;
    cloud_rotation->width = static_cast<uint32_t>(width);
    cloud_rotation->height = static_cast<uint32_t>(height);
}


void RealSense::computeRotationMatrix(const float pitch, const float roll, const float yaw, Eigen::Matrix3f &rotation_matrix)
{
    Eigen::Matrix3f m1,m2,m3;
    m1 <<   0, 0, 1,
            -1, 0, 0,
            0, -1, 0;
//    m1 = Eigen::AngleAxisf(yaw / 180.0 * M_PI, Eigen::Vector3f::UnitY())
//         * Eigen::AngleAxisf(-pitch / 180.0 * M_PI, Eigen::Vector3f::UnitX())
//         * Eigen::AngleAxisf(roll / 180.0 * M_PI, Eigen::Vector3f::UnitZ());
    m2 = Eigen::AngleAxisf(roll / 180.0 * M_PI, Eigen::Vector3f::UnitX())
         * Eigen::AngleAxisf(pitch / 180.0 * M_PI, Eigen::Vector3f::UnitY())
         * Eigen::AngleAxisf(yaw / 180.0 * M_PI, Eigen::Vector3f::UnitZ());

    m3 <<   0, 0, -1,
            1, 0, 0,
            0,-1, 0;

    rotation_matrix = m3 * m2 * m1;
}
