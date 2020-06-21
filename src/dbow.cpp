#include <dbow_ros/dbow.h>

dbow::dbow(ros::NodeHandle nh_) : it(nh_)
{

    nh = nh_;
    img_inc = false;
    frame = 0;
    cam_intrinsics = cv::Mat::zeros(3, 3, CV_64F);
    ros::NodeHandle n_p("~");
    n_p.param<std::string>("image_topic", image_topic, "camera/rgb/image_rect_color");
    n_p.param<std::string>("cam_info_topic", cam_info_topic, "camera/rgb/camera_info");
    n_p.param<std::string>("vocabulary_path", vocabulary_path, "config/orbVoc.voc");
    n_p.param<int>("max_kf_rate",max_kf_rate,30);
    n_p.param<int>("max_kfs",max_kfs,100);

    image_sub = it.subscribe(image_topic, 1, &dbow::imageCb, this);
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
        fdetector = cv::ORB::create(5000);
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
    frame++;
    if(frame%max_kf_rate!=0)
        return;
    
    frame=0;
    features.push_back(computeFeatures(currImage));

    if(features.size()>max_kfs)
        features.pop_front();

    fbow::fBow vv, vv2;
    vv = voc.transform(features.back());
    std::cout << "new KF "<<features.size()<<std::endl;
    map<double, int> score;
    std::cout << "scores: " <<std::endl;

    for (size_t j = 0; j < features.size()-1; ++j)
    {
        vv2 = voc.transform(features[j]);
        double score1 = vv.score(vv, vv2);
        score.insert(pair<double, int>(score1, j));
        std::cout << score1 <<" ";
    }
    std::cout<<std::endl<< "--------" <<std::endl;

}

cv::Mat dbow::computeFeatures(cv::Mat gray_img)
{
        vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors; 
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
