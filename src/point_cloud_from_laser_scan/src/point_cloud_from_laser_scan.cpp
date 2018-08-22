//
// Created by dtrimoul on 8/16/18.
//
#include <ros/ros.h>
#include <laser_assembler/AssembleScans.h>
#include <sensor_msgs/PointCloud.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

using namespace laser_assembler;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "assembled_point_cloud");
    ros::NodeHandle n;
    ros::Rate freq(1);
    ros::Time last_call = ros::Time(0,0);

    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("assembled_point_cloud", 1000);

    ros::service::waitForService("assemble_scans");
    ros::ServiceClient client = n.serviceClient<AssembleScans>("assemble_scans");
    AssembleScans srv;

    while(ros::ok()){
        srv.request.begin = last_call;
        srv.request.end   = ros::Time::now();
        if(client.call(srv)){
            sensor_msgs::PointCloud2 cloud;
            if(sensor_msgs::convertPointCloudToPointCloud2(srv.response.cloud, cloud))
                pub.publish(cloud);
        }else{
            ROS_INFO("No server message received");
        }
        last_call = srv.request.end;

        ros::spinOnce();
        freq.sleep();
    }


}