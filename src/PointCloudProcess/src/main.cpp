#include <iostream>
#include <thread>
#include <fstream>
#include <string>
#include <mutex>
#include <memory>
#include <condition_variable>
#include <fstream>
#include <queue>
#include <map>
#include <set>
#include <functional>
#include <memory>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#include <unordered_set>
#include <arpa/inet.h>
#include <sys/uio.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <cstring>
#include <algorithm>
#include <array>
#include <ros/ros.h>
#include <ros/assert.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

#include "filter/filter.h"
#include "websocket/websocket.h"

using namespace std;
ros::Publisher pointcloud_pub;
ros::Publisher pointcloud_remove_pub;


#define PADDDOWNX 0

#define RANSAC 1
// 轴长
#define XPIXEL (40.)
#define YPIXEL (40.)
#define ZPIXEL (40.)
#define INTERVAL (0.1)


glm::vec3 CENTER(XPIXEL / 2., YPIXEL / 2., ZPIXEL / 2.);
enum COLOR
{
    BLUE = 0,
    GREEN,
    YELLOW,
    RED
};
std::shared_ptr<WEB> web = nullptr;
std::shared_ptr<Filter> filter = nullptr;



typedef pcl::PointXYZ PointType;
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> BarrierCloudVector;
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> GroupCloudVector;
std::unordered_set<int> endIndexs;


void pubMsg(typename pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &link, ros::Publisher &pub)
{
    sensor_msgs::PointCloud2 outMsg;
    pcl::toROSMsg(*cloud, outMsg);
    outMsg.header.frame_id = link;
    outMsg.header.stamp = ros::Time::now();
    pub.publish(outMsg);
}
int GroundExtraction(typename pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    if (cloud->points.size() <= 0)
    {
        std::cout << " point cloud raw data's size is null. Extract group failed! " << std::endl;
        return -1;
    }
#if RANSAC
    struct timespec start,end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    pcl::PointCloud<pcl::PointXYZ>::Ptr groupCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr barrierCloud(new pcl::PointCloud<pcl::PointXYZ>);
    filter->PassThrough<pcl::PointXYZ>(cloud, groupCloud, "z", -2.0f, 0.0f);
    filter->PassThrough<pcl::PointXYZ>(cloud, barrierCloud, "z", 0.0f, 2.0f);

    BarrierCloudVector.emplace_back(barrierCloud);

    if (groupCloud->points.size() > 0)
    {
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setInputCloud(groupCloud);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setDistanceThreshold(0.05);
        seg.segment(*inliers, *coefficients);

        std::cout << "x : " << coefficients->values[0] << "y : " << coefficients->values[1] << "z : " << coefficients->values[2] << std::endl;

        pcl::PointIndices::Ptr outliers(new pcl::PointIndices);
        for (size_t i = 0; i < groupCloud->points.size(); ++i)
        {
            if (std::find(inliers->indices.begin(), inliers->indices.end(), i) == inliers->indices.end())
            {
                outliers->indices.push_back(i);
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(groupCloud);
        extract.setIndices(inliers);
        extract.filter(*cloud2);
        GroupCloudVector.emplace_back(cloud2);

        // 提取外点
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> out_extract;
        out_extract.setInputCloud(groupCloud);
        out_extract.setIndices(outliers);
        out_extract.filter(*cloud_outliers);
        BarrierCloudVector.emplace_back(cloud_outliers);
        pubMsg(cloud2, "rgb_camera_link", pointcloud_remove_pub);
    }
    clock_gettime(CLOCK_MONOTONIC, &end);
    double diff_time = 1e9 * (end.tv_sec - start.tv_sec) + end.tv_nsec - start.tv_nsec;
    printf("The time required for the RANSAC algorithm is : %2.5f(us) \n", diff_time/1000.);
#else

    // 创建法向量估计对象
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // 创建 KD 树对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    // 设置法向量估计的半径
    ne.setRadiusSearch(0.03); // 设置搜索半径为0.03米

    // 计算法向量
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    // 打印法向量
    for (size_t i = 0; i < normals->points.size(); ++i)
    {
        std::cout << "点 " << i << " 的法向量： "
                  << normals->points[i].normal_x << " "
                  << normals->points[i].normal_y << " "
                  << normals->points[i].normal_z << std::endl;
    }

#endif

    return 0;
}
void InterPolation()
{
    std::vector<int> interpolation_value;
    int min_num = *std::min_element(endIndexs.begin(), endIndexs.end());
    int max_num = *std::max_element(endIndexs.begin(), endIndexs.end());
    for (int i = min_num; i < max_num; ++i) {
        if (endIndexs.find(i) == endIndexs.end()) {
            interpolation_value.emplace_back(i);
        }
    }
    for (auto it = std::begin(interpolation_value); it != std::end(interpolation_value); ++it)
    {
        endIndexs.insert(*it);
    }
}
void Index_Conversions(pcl::PointXYZ &point, int color)
{
    int index, color_index;
    /// 相对于CENTER的格子位置
    int box_x = static_cast<int>(point.x * 10) + CENTER.x;
    int box_y = static_cast<int>(point.y * 10) + CENTER.y;
    int box_z = static_cast<int>(point.z * 10) + CENTER.z;

    int abs_x = std::abs(box_x), abs_z = std::abs(box_z) - 1, abs_y = std::abs(box_y);
    if (abs_x <= 0)
    {
        index = abs_z * XPIXEL * YPIXEL + abs_y;
    }
    else
    {
        int plane_offset = (abs_x - 1) * YPIXEL;
        index = abs_z * XPIXEL * YPIXEL + plane_offset + abs_y;
    }
    if ((index >= 0) && index < (XPIXEL * YPIXEL * ZPIXEL))
    {
        color_index = color << 16 | index;
        endIndexs.insert(color_index);
    }
}
void Point2IndexConver()
{
    for (auto it = BarrierCloudVector.begin(); it != BarrierCloudVector.end(); ++it)
    {
        for (pcl::PointXYZ& point : (*it)->points)
        {
            Index_Conversions(point, RED);
        }    
    }
    for (auto it = GroupCloudVector.begin(); it != GroupCloudVector.end(); ++it)
    {
        for (pcl::PointXYZ& point : (*it)->points)
        {
            Index_Conversions(point, GREEN);
        }    
    } 
    // InterPolation();



}
void WebSend()
{
    std::cout << "发送的个数为: " << endIndexs.size() << std::endl;
    if (endIndexs.size() > 0)
    {
        std::vector<int> indexs(endIndexs.begin(), endIndexs.end());
        web->Push(indexs);
    }
    endIndexs.clear();
    GroupCloudVector.clear();
    BarrierCloudVector.clear();
}
void send_point(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    // 读取RGB-D点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    std::cout << "输入的点云数量: " << cloud->points.size() << std::endl;
    filter->PassThrough<pcl::PointXYZ>(cloud, "x", -2.0, 2.0);
    filter->PassThrough<pcl::PointXYZ>(cloud, "y", -2.0, 2.0);
    filter->PassThrough<pcl::PointXYZ>(cloud, "z", -1.0, 1.0);
    filter->RadiusOutlierRemoval<pcl::PointXYZ>(cloud, INTERVAL, 10);
    filter->VoxelGrid<pcl::PointXYZ>(cloud, INTERVAL);


    GroundExtraction(cloud);
    Point2IndexConver();
    WebSend();
}
int main(int argc, char **argv)
{
    web = std::make_shared<WEB>();
    filter = std::make_shared<Filter>();

    ros::init(argc, argv, "avm");
    ros::NodeHandle nh, nh_private("~");

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/output", 10, send_point);
    // 定义发布点云的话题
    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cj", 10);
    pointcloud_remove_pub = nh.advertise<sensor_msgs::PointCloud2>("cj_remove_point", 10);

    ros::Rate rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "bye main" << std::endl;

    return 0;
}
