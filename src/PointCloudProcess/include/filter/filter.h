
#pragma once 


#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>


class Filter
{
    public:
    Filter() = default;
    virtual ~Filter() = default;
    template<typename PointTYPE>
    void PassThrough(typename pcl::PointCloud<PointTYPE>::Ptr& cloud,  const std::string& axes,
                   float min_limit, float max_limit)
    {
        pcl::PassThrough<PointTYPE> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName(axes);
        pass.setFilterLimits(min_limit, max_limit);
        pass.filter(*cloud);
    }
    template<typename PointTYPE>
    void PassThrough(typename pcl::PointCloud<PointTYPE>::Ptr& cloudIn, typename pcl::PointCloud<PointTYPE>::Ptr& cloudOut,
                const std::string& axes, float min_limit, float max_limit)
    {
        pcl::PassThrough<PointTYPE> pass;
        pass.setInputCloud(cloudIn);
        pass.setFilterFieldName(axes);
        pass.setFilterLimits(min_limit, max_limit);
        pass.filter(*cloudOut);
    }
    template<typename PointTYPE>
    void RadiusOutlierRemoval(typename pcl::PointCloud<PointTYPE>::Ptr& cloud, float radius, int nums)
    {
        pcl::RadiusOutlierRemoval<PointTYPE> outrem;
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(radius);
        outrem.setMinNeighborsInRadius(nums);
        outrem.filter(*cloud);
    }
    template<typename PointTYPE>
    void VoxelGrid(typename pcl::PointCloud<PointTYPE>::Ptr& cloud,float leafsize)
    {
        pcl::VoxelGrid<PointTYPE> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(leafsize, leafsize , leafsize);
        sor.filter(*cloud);
    }

    
};