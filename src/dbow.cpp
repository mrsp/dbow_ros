#include <dbow_ros/dbow.h>

dbow::dbow(ros::NodeHandle nh_) : it(nh_)
{

    nh = nh_;
    img_inc = false;
    frame = 0;
    cam_intrinsics = cv::Mat::zeros(3, 3, CV_64F);
    ros::NodeHandle n_p("~");
    curr_pose = Eigen::Affine3d::Identity();
    n_p.param<std::string>("image_topic", image_topic, "camera/rgb/image_rect_color");
    n_p.param<std::string>("cam_info_topic", cam_info_topic, "camera/rgb/camera_info");
    n_p.param<std::string>("vocabulary_path", vocabulary_path, "config/orbVoc.voc");

    n_p.param<double>("publish_rate", publish_rate, 50);
    image_sub = it_.subscribe(image_topic, 1, &dbow::imageCb, this);
    ROS_INFO("Waiting camera info");
    while (ros::ok())
    {
        sensor_msgs::CameraInfoConstPtr cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam_info_topic);
        if (cam_info)
        {
            cameraInfoCb(cam_info);
            break;
        }
    }
    ROS_INFO("Initializing DBoW");

    voc.readFromFile(vocabulary_path);
    desc_name=voc.getDescName();
    //select detector
    if (desc_name == "orb")        
        fdetector = cv::ORB::create(2000);
    else if (desc_name == "brisk") 
        fdetector = cv::BRISK::create();
    #ifdef OPENCV_VERSION_3
    else if (desc_name == "akaze") fdetector = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 1e-4);
    #endif
    #ifdef USE_CONTRIB
    else if (desc_name == "surf")  fdetector = cv::xfeatures2d::SURF::create(15, 4, 2);
    #endif
}

void dbow::imageCb(const sensor_msgs::ImageConstPtr &img_msg)
{

    ROS_INFO("Image Cb");
    cv_bridge::CvImagePtr cv_ptr;
    img_inc = true;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge RGB exception: %s", e.what());
        return;
    }

    if (cv_ptr->image.channels() == 3)
    {
        cvtColor(cv_ptr->image, currImage, cv::COLOR_BGR2GRAY);
    }
    else
    {
        currImage = cv_ptr->image;
    }
    

    features.push_back(computeFeatures(currImage));
    fbow::fBow vv, vv2;
    vv = voc.transform(features.front());
    map<double, int> score;
    for (size_t j = 1; j < features.size(); ++j)
    {
        vv2 = voc.transform(features[j]);
        double score1 = vv.score(vv, vv2);
        counter++;
        score.insert(pair<double, int>(score1, j));
        printf("%f, ", score1);
    }

}

cv::Mat dbow::computeFeatures(cv::Mat gray_img)
{
        vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors; 
        std::cout << "extracting features" <<std::endl;
        fdetector->detectAndCompute(gray_img, cv::Mat(), keypoints, descriptors);
        return descriptors;
}


void dbow::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg)
{

    height = msg->height;
    width = msg->width;

    k1 = msg->D[0];
    k2 = msg->D[1];
    t1 = msg->D[2];
    t2 = msg->D[3];
    k3 = msg->D[4];

    fx = msg->K[0];
    cx = msg->K[2];
    fy = msg->K[4];
    cy = msg->K[5];

    cam_intrinsics.at<double>(0, 0) = fx;
    cam_intrinsics.at<double>(0, 2) = cx;
    cam_intrinsics.at<double>(1, 1) = fx;
    cam_intrinsics.at<double>(1, 2) = cy;
    cam_intrinsics.at<double>(2, 2) = 1;
    cout << "Cam Int" << cam_intrinsics << std::endl;
}
