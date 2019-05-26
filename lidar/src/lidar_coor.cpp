#include <ros/ros.h>
#include "string.h"
#include <cmath>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <algorithm>

#include <fstream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/extract_indices.h>

#include "lidar/coor.h"



#define RADtoDEG(x) ((x)*180./M_PI)

using namespace std;


// Global variable
pcl::PointCloud<pcl::PointXYZ>::Ptr prev_cloud (new::pcl::PointCloud<pcl::PointXYZ>);
Eigen::Matrix4f transMtx_now;
Eigen::Matrix4f transMtx_prev;
Eigen::Matrix4f transMtx_delta;

double pos[3];

ros::Publisher pub_lidar;

void lidar_cb(sensor_msgs::LaserScan msg){

    // angle in radian
    float angle_min = msg.angle_min;
    float angle_max = msg.angle_max;
    float angle_increment = msg.angle_increment;
    std::vector<float> range = msg.ranges;

    // size of range vector
    int len = range.size();
    float angle_temp;

    /// 1. LaserScan msg to PCL::PointXYZ

    // initializae pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud (new::pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr inf_points(new pcl::PointIndices());

    new_cloud->is_dense = false;
    new_cloud->width = len;
    new_cloud->height = 1;
    new_cloud->points.resize(len);

    // fill the pointcloud
    for(int i = 0; i < len; i++){

        // TO DO START

        angle_temp = angle_min + i*angle_increment;

        // TO DO END

        if (std::isinf(range[i])==false){

            // TO DO START

            new_cloud->points[i].x = range[i]*cos(angle_temp);
            new_cloud->points[i].y = range[i]*sin(angle_temp);
            new_cloud->points[i].z = 0;

            // TO DO END

        }
        else{
            // indices of infinite distance points
            inf_points->indices.push_back(i);

        }
    }

    // Remove infinite distance points from new_cloud
    extract.setInputCloud(new_cloud);
    extract.setIndices(inf_points);
    extract.setNegative(true);
    extract.filter(*new_cloud);

    // 2. Get transformation between previous pointcloud and current pointcloud

    // transMtx_prev : transformation matrix at time (t-1)
    // transMtx_now : transformation matrix at time (t)
    // 4X4 transformation matrix (3X3: rotation matrix, 3X1: translation vector)

    if(prev_cloud->width == 0){

        // initialize transformation matrix. initial posiiton: x = 0, y = 0, theta = 0;
        transMtx_prev << cos(0), -sin(0), 0, 0,
                        sin(0), cos(0), 0, 0,
                        0, 0, 1, 0,
                        0, 0, 0, 1;
    }

    else{

        // ICP algorithm
        // http://pointclouds.org/documentation/tutorials/iterative_closest_point.php
        // http://pointclouds.org/documentation/tutorials/interactive_icp.php#interactive-icp
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(prev_cloud);
        icp.setInputTarget(new_cloud);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);
        //std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        //icp.getFitnessScore() << std::endl;
        //std::cout << icp.getFinalTransformation() << std::endl;

        transMtx_delta = icp.getFinalTransformation();

    // 3. Get current transformation matrix using previous transformation and ICP result
    //  (Odometry calculation)

        // TO DO START

        transMtx_now = transMtx_prev*transMtx_delta;

        // TO DO END

    // 4. Get current position from transformation matrix

        // TO DO START

        pos[0] = transMtx_now(0,3);
        pos[1] = transMtx_now(1,3);
        pos[2] = RADtoDEG(acos(transMtx_now(0,0)));

        // TO DO END
        pos[0] = round(pos[0]*100);
        pos[1] = round(pos[1]*100);
        pos[2] = round(pos[2]);

        transMtx_prev = transMtx_now; // Save current transformation matrix in transMtx_prev
        lidar::coor coor_data;
        coor_data.coor_x = pos[0];
        coor_data.coor_y = pos[1];
        coor_data.coor_theta = pos[2];

        pub_lidar.publish(coor_data);
    }


    // 5. Save new_cloud in prev_cloud

    prev_cloud = new_cloud;

}

int main(int argc, char **argv){

    ros::init(argc, argv, "lidar_coor_node");
    ros::NodeHandle nh;/* message */

    ros::Subscriber sub_lidar = nh.subscribe("/scan", 1, lidar_cb);
    pub_lidar = nh.advertise<lidar::coor>("/lidar_coor", 100);
    ros::Rate loop_rate(5);

    // initialize prev_cloud
    prev_cloud->width = 0;

    // File write

    // TO DO START
    // string filePath = "/home/thanhndv212/capstone_1_ROS/coordinate.txt"; //<- Change to your own directory
    // TO DO END

    // ofstream txtFile(filePath);

    while(ros::ok()){
        ros::spinOnce();

        ROS_INFO("pos : x = %f | y = %f | theta = %f", pos[0], pos[1], pos[2]);
        // txtFile << "position: " << pos[0] << "\t" << pos[1] << "\t" << pos[2] << endl;

        loop_rate.sleep();
    }

    // txtFile.close();
    return 0;
}
