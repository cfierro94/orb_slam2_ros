/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raul MurArtal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud.h>
#include "pcl_ros/point_cloud.h"

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include"../../../../include/System.h"


using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}
    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    ORB_SLAM2::System* mpSLAM;
};

ORB_SLAM2::System* my_SLAM;
void chatterCalback(const std_msgs::Int8 msg);
ros::Publisher publisher;
ros::Publisher pub_pos_robot;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    my_SLAM = &SLAM;

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    ros::NodeHandle nh;
    pub_pos_robot = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/robot_pos", 1);

    ros::NodeHandle node_chatter;
    ros::Subscriber sub2 = node_chatter.subscribe("/request_points", 100, chatterCalback);

    ros::NodeHandle node_pub;
    publisher = node_pub.advertise<pcl::PointCloud<pcl::PointXYZ>>("/map_points", 1);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void chatterCalback(const std_msgs::Int8 msg){
    ROS_INFO("requested? %d, track_state: %d", msg.data, my_SLAM->GetTrackingState());
    if(msg.data == 1 && my_SLAM->GetTrackingState()){
        // get points
        std::vector<ORB_SLAM2::MapPoint*> mapPoints = my_SLAM->GetTrackedMapPoints();
        ROS_INFO("got points");
        cv::Mat position;
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.header.frame_id = "global_frame";
        for(unsigned int i = 0; i < mapPoints.size(); i++){
            if(mapPoints[i] != NULL) {
                position = mapPoints[i]->GetWorldPos();
                pcl::PointXYZRGB point;
                point.x = position.at<float>(0);
		point.y = position.at<float>(1);
		point.z = position.at<float>(2);
	        cloud.push_back(point);
            }
        }
        ROS_INFO("got positions");
        // make message
        publisher.publish(cloud);
        ROS_INFO("points sent");
    }
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat tracked_pos = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    if(tracked_pos.rows > 0){
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.header.frame_id = "global_frame";
        for(int i = 0; i < 4; i++){
	    pcl::PointXYZRGB point;
            point.x = tracked_pos.at<float>(i,0);
            point.y = tracked_pos.at<float>(i,1);
            point.z = tracked_pos.at<float>(i,2);
            point.rgb = tracked_pos.at<float>(i,3);
            cloud.push_back(point);
        }
        pub_pos_robot.publish(cloud);
        cout << "position sent" << endl;
    }
}
