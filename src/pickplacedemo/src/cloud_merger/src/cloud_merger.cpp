//
// Created by dtrimoul on 8/17/18.
//



//Take two cloud as input and send one merged as output

#include "sensor_msgs/PointCloud2.h"
#include "ros/ros.h"
#include "pcl_ros/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tf/transform_listener.h"


std::vector<sensor_msgs::PointCloud2> cloud_A_data;
int cloud_A_data_counter=0;

std::vector<sensor_msgs::PointCloud2> cloud_B_data;
int cloud_B_data_counter=0;

bool publish = false;

ros::Publisher outputCloud;

void storeCloudA(const sensor_msgs::PointCloud2::ConstPtr& msg);
void storeCloudB(const sensor_msgs::PointCloud2::ConstPtr& msg);

bool
transformPointCloud (const std::string &target_frame, const sensor_msgs::PointCloud2 &in,
                     sensor_msgs::PointCloud2 &out, const tf::TransformListener &tf_listener);

template <typename PointIn1T, typename PointIn2T, typename PointOutT>
void concatenateFieldsCustom (pcl::PointCloud<PointIn1T> &cloud1_in,
                   pcl::PointCloud<PointIn2T> &cloud2_in,
                   pcl::PointCloud<PointOutT> &cloud_out);

bool fastestA_B();

int main(int argc, char **argv ){

    ros::init( argc, argv, "cloud_merger");
    ros::NodeHandle n;

    ros::Subscriber cloudA = n.subscribe("input_cloud_A", 1000, storeCloudA);
    ros::Subscriber cloudB = n.subscribe("input_cloud_B", 1000, storeCloudB);

    outputCloud = n.advertise<sensor_msgs::PointCloud2>("merged_cloud", 1000);

    while(ros::ok()){

        if(publish){
            sensor_msgs::PointCloud2 buffer_A;
            sensor_msgs::PointCloud2 buffer_B;
            tf::TransformListener listener;
            ROS_INFO("frame of A= %s", cloud_A_data.front().header.frame_id.c_str());
            ROS_INFO("frame of B= %s", cloud_B_data.front().header.frame_id.c_str());
            transformPointCloud("odom_combined", cloud_A_data.front(), buffer_A, listener);
            transformPointCloud("odom_combined", cloud_B_data.front(), buffer_B, listener);
            ROS_INFO("frame of A= %s", buffer_A.header.frame_id.c_str());
            ROS_INFO("frame of B= %s", buffer_B.header.frame_id.c_str());
            ROS_INFO("SIZE of A= %i", buffer_A.data.size());
            ROS_INFO("SIZE of B= %i", buffer_B.data.size());

            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::PointCloud<pcl::PointXYZ> A ;
            pcl::PointCloud<pcl::PointXYZ> B ;
            pcl::fromROSMsg(buffer_A, A);
            pcl::fromROSMsg(buffer_B, B);

            concatenateFieldsCustom(A, B, cloud);

            sensor_msgs::PointCloud2 final_cloud;

            pcl::toROSMsg(cloud, final_cloud);
            ROS_INFO("size of concatenate cloud : %i", final_cloud.data.size());
            //ROS_INFO("publishing cloud");
            //ROS_INFO("%s", final_cloud.header.frame_id.c_str());
            final_cloud.header.frame_id = "odom_combined";
            outputCloud.publish(final_cloud);
            publish = false;
        }

        ros::spinOnce();
    }

    return 0;
}


void storeCloudA(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    cloud_A_data.insert(cloud_A_data.begin(), *msg);
    cloud_A_data_counter++;
    if(!fastestA_B())
        publish = true;
}

void storeCloudB(const sensor_msgs::PointCloud2::ConstPtr& msg) {

    cloud_B_data.insert(cloud_B_data.begin(), *msg);
    cloud_B_data_counter++;
    if(fastestA_B())
        publish = true;
}



bool fastestA_B() {
    return (cloud_A_data_counter - cloud_B_data_counter) > 0 ;
}

bool transformPointCloud (const std::string &target_frame, const sensor_msgs::PointCloud2 &in, sensor_msgs::PointCloud2 &out, const tf::TransformListener &tf_listener)
{
    if (in.header.frame_id == target_frame)
    {
        out = in;
        return (true);
    }

    // Get the TF transform
    tf::StampedTransform transform;
    try
    {
        const std::string out_frame( in.header.frame_id);
        ros::Time time_trans(in.header.stamp);
        tf_listener.waitForTransform (target_frame, out_frame, time_trans , ros::Duration(0.5) );
    }
    catch (tf::LookupException &e)
    {
        ROS_ERROR ("%s", e.what ());
        return (false);
    }
    catch (tf::ExtrapolationException &e)
    {
        ROS_ERROR ("%s", e.what ());
        return (false);
    }

    // Convert the TF transform to Eigen format
    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix (transform, eigen_transform);

    pcl_ros::transformPointCloud (eigen_transform, in, out);

    out.header.frame_id = target_frame;
    return (true);
}

template <typename PointIn1T, typename PointIn2T, typename PointOutT>
void concatenateFieldsCustom (pcl::PointCloud<PointIn1T> &cloud1_in,
                        pcl::PointCloud<PointIn2T> &cloud2_in,
                        pcl::PointCloud<PointOutT> &cloud_out)
{
    typedef typename pcl::traits::fieldList<PointIn1T>::type FieldList1;
    typedef typename pcl::traits::fieldList<PointIn2T>::type FieldList2;

    ROS_INFO("cloud1 size = %i, cloud2 size = %i", cloud1_in.points.size() , cloud2_in.points.size() );
    if(cloud1_in.points.size() < cloud2_in.points.size()){
        pcl::PointCloud<PointIn1T> cloud;
        cloud = cloud1_in;
        cloud1_in = cloud2_in;
        cloud2_in = cloud;
    }

    // Resize the output dataset
    cloud_out.points.resize (cloud1_in.points.size ());
    cloud_out.header   = cloud1_in.header;
    cloud_out.width    = cloud1_in.width;
    cloud_out.height   = cloud1_in.height;
    if (!cloud1_in.is_dense || !cloud2_in.is_dense)
        cloud_out.is_dense = false;
    else
        cloud_out.is_dense = true;
    ROS_INFO("cloud out size = %i", cloud_out.points.size() );

    // Iterate over each point
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
        // Iterate over each dimension
        pcl::for_each_type <FieldList1> (pcl::NdConcatenateFunctor <PointIn1T, PointOutT> (cloud1_in.points[i], cloud_out.points[i]));
        pcl::for_each_type <FieldList2> (pcl::NdConcatenateFunctor <PointIn2T, PointOutT> (cloud2_in.points[i], cloud_out.points[i]));
    }
}