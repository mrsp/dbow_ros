/* 
 * Copyright 2020 Stylianos Piperakis, Foundation for Research and Technology Hellas (FORTH)
 * License: BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Foundation for Research and Technology Hellas (FORTH) 
 *		 nor the names of its contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @brief Bag Of Words with openCV and fbow
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#ifdef USE_CONTRIB
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>
#endif
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <fbow/fbow.h>


using namespace std;


class dbow
{
    /// current image frame
    int frame;
    ///image dimensions
    int height, width;
    /// camera distorsion
    double k1, k2, k3, t1, t2;
    /// camera calibration
    double cx, cy, fx, fy;
    /// ROS nodehanlder
    ros::NodeHandle nh;
    /// ROS image transport for image callback
    image_transport::ImageTransport it;
    /// ROS image subscriber
    image_transport::Subscriber image_sub;
    ///placeholders for  current Grayscale Image
    cv::Mat currImage;
    /// camera matrix
    cv::Mat cam_intrinsics; 
    /// ROS image, camera info topics and vocabulary path
    std::string image_topic, cam_info_topic, vocabulary_path;
    fbow::Vocabulary voc;
    string desc_name;
    cv::Ptr<cv::Feature2D> fdetector;
    std::vector<cv::Mat> features;
    bool img_inc;
public:
    /** @fn  dbow_ros(ros::NodeHandle nh_);
	 *  @brief Initializes the dbow
     *  @param nh_ ros nodehandler 
	 */
    dbow(ros::NodeHandle nh_);
    /** @fn void imageCb(const sensor_msgs::ImageConstPtr &img_msg);
     *  @brief callback for RGB 
     */
    void imageCb(const sensor_msgs::ImageConstPtr &img_msg);
    /** @fn void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg);
     * @brief Camera Info Callback
     */
    void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr &msg);
    cv::Mat computeFeatures(cv::Mat gray_img);
};
