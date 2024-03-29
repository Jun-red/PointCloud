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

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"


#include "websocket/websocket.h"

using namespace std;
ros::Publisher pointcloud_pub;
ros::Publisher pointcloud_remove_pub;

#define TCPSOCKET 0
#define OFFLINEDATA 0
#define TOPICPUB 0
#define REALPOINT 1
#define WARNREGION 0
#define PADDUPZ  0
#define DEBUGMSG 0
#define PADDDOWNX 0
// 轴长
#define XPIXEL (40.)
#define YPIXEL (40.)
#define ZPIXEL (40.)
#define INTERVAL (0.1) 

#define ROBOTHIGHT (0.5)
#define ROBOTLEN   (0.8 + 0.1)
#define ROBOTWIDTH (0.8)

#define RUP    (0)
#define RDOWN  (1)
#define RLEFT  (2)
#define RRIGHT (3)
#define ROFFSET (4)

glm::vec3 CENTER(XPIXEL/2., YPIXEL/2., ZPIXEL/2.);
enum COLOR
{
    BLUE = 0,
    GREEN,
    YELLOW,
    RED
};
std::shared_ptr<WEB> web = nullptr;
std::vector<int> box_index;
std::vector<glm::vec3> box_data;

struct PlaneInfo
{
    int index = 0;
    int num = 0;
    float altitude = 0.0;
};
struct BaseInfo      
{
    int color ;
    std::vector<PlaneInfo> plane {};
    std::map<int , pcl::PointXYZ> data{};   
};
BaseInfo edge{YELLOW};
BaseInfo pit{BLUE};
BaseInfo barrier{RED};  
BaseInfo group{GREEN};
struct Fragment
{
    std::array<int, 8> max_region;  // group: up down left right; barrier : ....
    std::map<int , pcl::PointXYZ> group_fragment;
    std::vector<std::map<int , pcl::PointXYZ>> barrier_fragment;
    std::map<int , pcl::PointXYZ> pit_fragment;
    std::vector<std::map<int , pcl::PointXYZ>> edge_fragment;
};
Fragment fragment;

typedef pcl::PointXYZ PointType;

#if TCPSOCKET
std::vector<int> net_connect;
std::vector<int> net_disconnect;
std::unique_ptr<std::thread> net_thread;
std::map<int, std::function<void(int)>> net_events;
fd_set read_set;
fd_set write_set;
fd_set error_set;

int host_socket;

std::mutex net_mutex;

void remove_client(int client_fd)
{
    auto it = std::find(net_connect.begin(), net_connect.end(), client_fd);
    if (it != net_connect.end())
        net_connect.erase(it);

    auto iter = net_events.find(client_fd);
    if (iter != net_events.end()) 
        net_events.erase(iter);
    
    FD_CLR(client_fd, &read_set);
    close(client_fd);
}
void readIO()
{
    fd_set read_bak = read_set;
    fd_set write_bak = write_set;
    fd_set error_bak = error_set;
    struct timeval scheduler_time;
    scheduler_time.tv_sec = 0;
    scheduler_time.tv_usec = 0;
    if (net_events.size() <= 0) return;
    int max_socket = net_events.rbegin()->first + 1;
    
    int ret = select(max_socket, &read_bak, &write_bak, &error_bak, &scheduler_time);

    if (ret <= 0)
    {
        return;
    }
    for (auto it = net_events.begin(); it != net_events.end(); ++it)
    {
        if (FD_ISSET(it->first, &read_bak))
        {
            it->second(it->first);
        }
    }
    for (auto fd : net_disconnect)
        remove_client(fd);
    net_disconnect.clear();
}

void client_event(int client_fd)
{
    char extrabuf[1024];
    memset(extrabuf, 0 ,sizeof(extrabuf));
    const int n = ::recv(client_fd, extrabuf, sizeof(extrabuf), 0);
    if (n <= 0)
    {
        net_disconnect.emplace_back(client_fd);
    } else {
        std::cout << extrabuf << std::endl;
    }
}
void updateIO(int& t_fd)
{
    FD_CLR(t_fd, &read_set);
    FD_SET(t_fd, &read_set); 
}
void net_request(int host_socket)
{
    struct sockaddr_in addr = { 0 };
    socklen_t addrlen = sizeof(struct sockaddr_in);
    int client_fd = ::accept(host_socket, (struct sockaddr*)&addr, &addrlen);

    int flags = ::fcntl(client_fd, F_GETFL, 0);
    flags |= O_NONBLOCK;
    int ret = ::fcntl(client_fd, F_SETFL, flags);
    flags = ::fcntl(client_fd, F_GETFD, 0);
    flags |= FD_CLOEXEC;
    ret = ::fcntl(client_fd, F_SETFD, flags);

    updateIO(client_fd);
    net_connect.emplace_back(client_fd);
    net_events.insert(std::make_pair(client_fd, std::bind(&client_event,std::placeholders::_1)));
    std::cout << "New Connect. the socket is " << client_fd << " , the IP is " << inet_ntoa(addr.sin_addr) << " , the Port is " << addr.sin_port << std::endl;
}


void net_register()
{
    struct sockaddr_in host_addr;

    host_addr.sin_family = AF_INET;	
    host_addr.sin_addr.s_addr = INADDR_ANY ; //inet_addr(sdt::string(HOSTIP).c_str()); 
    host_addr.sin_port = htons(HOSTPORT);

    host_socket = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, IPPROTO_TCP);
    int optval = 1 ;
    setsockopt(host_socket, SOL_SOCKET, SO_REUSEADDR, (const char*)&optval, sizeof(optval));

    if (::bind(host_socket, (struct sockaddr*)&host_addr, sizeof(host_addr)) < 0) 
    {
        std::cerr << " bind host ip failed! "  << std::endl;
    }
    if (::listen(host_socket, 10) < 0) 
    {
        std::cerr << " host listen failed! "  << std::endl;
    }   
    updateIO(host_socket);
    net_events.insert(std::make_pair(host_socket, std::bind(&net_request, std::placeholders::_1)));
}
#endif
float computeDistance(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) 
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2)) ;
}

std::vector<std::map<int ,pcl::PointXYZ>> PointClustering(BaseInfo& info, float threshold)
{
    const std::map<int , pcl::PointXYZ>& map_data = info.data;

    int i,j;
    std::vector<bool> processed(map_data.size(), false);
    std::vector<std::map<int , pcl::PointXYZ>> regions;

    for (auto it = map_data.begin(); it != map_data.end(); ++it) 
    {
        i = std::distance(map_data.begin(), it);
        if (processed[i])
            continue;

        std::map<int , pcl::PointXYZ> region;
        region.insert(std::make_pair(it->first, it->second));
        processed[i] = true;

        for (auto iter = std::next(it); iter != map_data.end(); ++iter) 
        {
            j = std::distance(map_data.begin(), iter);
            float distance = computeDistance(it->second, iter->second);
            if (distance <= threshold) 
            {
                region.insert(std::make_pair(iter->first, iter->second));
                processed[j] = true;
            }
        }

        regions.push_back(region);
    }

    return regions;
}
std::vector<std::map<int ,pcl::PointXYZ>> EuclideanCluster(BaseInfo& info, float threshold)
{
    const std::map<int , pcl::PointXYZ>& map_data = info.data;

    int i,j;
    std::vector<bool> processed(map_data.size(), false);
    std::vector<std::map<int , pcl::PointXYZ>> regions;
    
    for (auto it = map_data.begin(); it != map_data.end(); ++it) 
    {
        i = std::distance(map_data.begin(), it);
        if (processed[i])
            continue;
        PlaneInfo plane;
        std::map<int , pcl::PointXYZ> region;
        region.insert(std::make_pair(it->first, it->second));
        plane.num++;
        plane.altitude += it->second.z;
        processed[i] = true;

        for (auto iter = std::next(it); iter != map_data.end(); ++iter) 
        {
            j = std::distance(map_data.begin(), iter);
            float distance = std::abs(it->first - iter->first);
            if (distance < threshold && not processed[j]) 
            {
                plane.num++;
                plane.altitude += it->second.z;
                region.insert(std::make_pair(iter->first, iter->second));
                processed[j] = true;   
            }
        }
        regions.push_back(region);
        plane.index = info.plane.size();
        plane.altitude /= plane.num;
        info.plane.emplace_back(plane);
    }
    return regions;
}
int Compare(int d1, int d2)
{
    return d1 > d2 ? d1 : d2;
}

int GetCol(int base)
{
    return base % (int)YPIXEL;
}
int GetRow(int base, int init)
{
    return (int)(base - init) / (int)YPIXEL;
}
std::vector<int> GetVertexPos(int bottom_left , int min_col, int max_col, int min_row, int max_row)
{
    if (min_col > (int)XPIXEL) min_col = (int)XPIXEL - 1;
    if (max_col > (int)XPIXEL) max_col = (int)XPIXEL - 1;
    if (min_row > (int)XPIXEL) min_row = (int)XPIXEL - 1;
    if (max_row > (int)XPIXEL) max_row = (int)XPIXEL - 1;

    std::vector<int> pos;
    std::set<int> vertex;
    int min_col_base = bottom_left + min_col;
    int max_col_base = bottom_left + max_col;
    for (int i = 0; i < (int)XPIXEL; ++i)
    {
        vertex.insert(min_col_base + int(i*XPIXEL));
        vertex.insert(max_col_base + int(i*XPIXEL));
    }
    int min_row_base = bottom_left + min_row * XPIXEL;
    int max_row_base = bottom_left + max_row * XPIXEL;
    for (int i = 0; i < (int)YPIXEL; ++i)
    {
        int td1 =  min_row_base + i;
        int td2 =  max_row_base + i;
        auto it1 = vertex.find(td1);
        if (it1 != vertex.end())
        {
            pos.emplace_back(td1);
        }
        auto it2 = vertex.find(td2);
        if (it2 != vertex.end())
        {
            pos.emplace_back(td2);
        } 
    }  
    return  pos;
}
void XYHorizontalPlane(int group_plane)
{
#if PADDDOWNX
    std::vector<int> down_indexs;
    std::vector<int> left_indexs;
    std::vector<int> up_indexs;
    std::vector<int> right_indexs;

    int bottom_left = group_plane * static_cast<int>(XPIXEL * YPIXEL);
    int top_left = bottom_left + ((int)XPIXEL - 1) * (int)YPIXEL;
    int bottom_right = bottom_left +( (int)YPIXEL - 1);
    int top_right = top_left +( (int)YPIXEL - 1);

    int max_plane = bottom_left * static_cast<int>(XPIXEL * YPIXEL);
    if (fragment.max_region[RDOWN] >= max_plane)
    {
        fragment.max_region[RDOWN] = max_plane - 1;
    }
    if(fragment.max_region[RUP] > max_plane)
    {
        fragment.max_region[RUP] = max_plane - 1;
    }  
    if(fragment.max_region[RLEFT]> max_plane)
    {
        fragment.max_region[RLEFT] = max_plane - 1;
    }
    if(fragment.max_region[RRIGHT]> max_plane)
    {
        fragment.max_region[RRIGHT] = max_plane - 1;
    }

    std::vector<int> col;
    std::vector<int> row;
    // 列
    col.emplace_back(GetCol(fragment.max_region[RDOWN]) );
    col.emplace_back(GetCol(fragment.max_region[RUP])   );
    col.emplace_back(GetCol(fragment.max_region[RLEFT]) );
    col.emplace_back(GetCol(fragment.max_region[RRIGHT]));
    // 行
    row.emplace_back(GetRow(fragment.max_region[RDOWN],bottom_left) );
    row.emplace_back(GetRow(fragment.max_region[RUP],bottom_left)   );
    row.emplace_back(GetRow(fragment.max_region[RLEFT],bottom_left) );
    row.emplace_back(GetRow(fragment.max_region[RRIGHT],bottom_left));

    int min_col = *(std::min_element(col.begin(), col.end()));
    int max_col = *(std::max_element(col.begin(), col.end()));
    int min_row = *(std::min_element(row.begin(), row.end()));
    int max_row = *(std::max_element(row.begin(), row.end())); 

    std::vector<int> vertex = GetVertexPos(bottom_left, min_col, max_col, min_row, max_row);

    if (vertex.size() == 4 && ((vertex[3] - vertex[1]) == (vertex[2] - vertex[0])))
    {
        std::sort(vertex.begin(), vertex.end()); 
        std::vector<int> interpolation;
        int temp_data = vertex[0];
        for (int y = 0; temp_data < vertex[2]; y *= (int)(XPIXEL))
        {
            temp_data += y;
            for (int x = 0; x < (vertex[1] - vertex[0]); ++x)
            {
                int offset_y = temp_data + x;
                offset_y = GREEN << 16 | offset_y;
                auto iter = fragment.group_fragment.find(offset_y) ;
                if (iter == fragment.group_fragment.end())
                {
                    int color = GREEN << 16 | offset_y;
                    interpolation.emplace_back(color);
                }
            }             
        }
        if (interpolation.size() > 0)
        {
            for (auto i : interpolation)
            {
                fragment.group_fragment.insert(std::make_pair(i, pcl::PointXYZ()));
            }
            std::cout << "interpolation.size() = " << interpolation.size() << std::endl;
        }

#if DEBUGMSG
        std::cout   << " vertex[0] = "  << vertex[0] \
                    << " vertex[1] =  " << vertex[1] \
                    << " vertex[2] = "  << vertex[2] \
                    << " vertex[3] = "  << vertex[3]  << std::endl;
#endif
    }
#if DEBUGMSG
    std::cout << "fragment.max_region[RDOWN] = "  << fragment.max_region[RDOWN] \
              << " fragment.max_region[RUP]  =  " << fragment.max_region[RUP] \
              << " fragment.max_region[RLEFT]= "  << fragment.max_region[RLEFT] \
              << " fragment.max_region[RRIGHT]= "  << fragment.max_region[RRIGHT]  << std::endl;
    std::cout << "bottom_left = "<< bottom_left << " min_col = " << min_col << " max_col = " << max_col \
              << " min_row = " << min_row << " max_row = " << max_row \
              << " vertex.size() = " << vertex.size() << std::endl;
#endif

#else ///PADDDOWNX


#endif ///PADDDOWNX


}
int ParseCurPlane(const std::map<int , pcl::PointXYZ>& data)
{
    int index = data.begin()->first & 0xffff;
    return index / (int)(XPIXEL * YPIXEL) ;
}
int ParseCurPlane(const int& data)
{
    int index = data & 0xffff;
    return index / (int)(XPIXEL * YPIXEL)  ;
}
int GetUpRegion(const std::map<int , pcl::PointXYZ>& data)
{
    return data.rbegin()->first & 0xffff;
}
int GetDownRegion(const std::map<int , pcl::PointXYZ>& data)
{
    return data.begin()->first & 0xffff;
}
int GetLeftRegion(const std::map<int , pcl::PointXYZ>& data)
{
    float max_z = 0;
    int max_index = 0;
    for (auto iter = data.cbegin(); iter !=  data.cend(); ++iter)
    {
        if (iter->second.y > max_z)
        {
            max_z = iter->second.y;
            max_index = iter->first;
        }
    }
    return max_index & 0xffff;
}
int GetRightRegion(const std::map<int , pcl::PointXYZ>& data)
{
    float max_z = 0;
    int max_index = 0;
    for (auto iter = data.cbegin(); iter !=  data.cend(); ++iter)
    {
        if (iter->second.y < max_z)
        {
            max_z = iter->second.y;
            max_index = iter->first;
        }
    }
    return max_index & 0xffff;
}
void PaddVerticalDirt(int group_plane)
{
    std::vector<int> indexs;
    for (int i = 0; i < fragment.barrier_fragment.size(); ++i)
    {
        // using map_data = fragment.barrier_fragment[i];
        for (auto iter = fragment.barrier_fragment[i].begin(); iter != fragment.barrier_fragment[i].end(); ++iter)
        {
            int cur_plane = ParseCurPlane(iter->first);
            int plane_dist = cur_plane - group_plane -1 ;
            if ( plane_dist > 0)
            {
                for (int z = 0; z < plane_dist; ++z)
                {
                    int index = iter->first & 0xffff - (z+1) * (int)(XPIXEL * YPIXEL);
                    index = RED << 16 | index;
                    indexs.emplace_back(index);
                }
            }
        }
    }
    std::map<int , pcl::PointXYZ> pad_data;
    for (int i = 0; i < indexs.size(); ++i)
    {
        pad_data.insert(std::make_pair(indexs[i],pcl::PointXYZ()));
    }
    fragment.barrier_fragment.emplace_back(pad_data);
}
void ZUPVerticalPlane(int group_plane)
{
    if (barrier.data.size() <= 0) return; 

    int barrier_thres = 15;

    std::vector<std::map<int , pcl::PointXYZ>> regions = PointClustering(barrier,  0.5);
    for (auto iter = std::begin(regions); iter != std::end(regions) ; )
    {
        if (iter->size() < barrier_thres)
        {
            iter = regions.erase(iter);
        } else {
            fragment.barrier_fragment.emplace_back(*iter);
            ++iter;
        }
    }
#if PADDUPZ 
    float  max_left = 0.0, max_right = 0.0;
    int max_left_index, max_right_index, max_down, max_up ;
    bool first = true;
    for (int i = 0; i < fragment.barrier_fragment.size(); ++i)
    { 
        // using map_data = fragment.barrier_fragment[i];

        int t_down = fragment.barrier_fragment[i].begin()->first & 0xffff;
        int t_up   = fragment.barrier_fragment[i].rbegin()->first & 0xffff;

        if (first)
        {
            first = not first;
            max_down =  t_down;
            max_up = t_up;
        }

        if (t_down < max_down)
        {
            max_down = t_down;
        }
        if (t_up > max_up)
        {
            max_up = t_up;
        }       

        for (auto iter = fragment.barrier_fragment[i].begin(); iter != fragment.barrier_fragment[i].end() ; ++iter)
        {
            if (iter->second.y > max_left)
            {
                max_left = iter->second.y;
                max_left_index = iter->first & 0xffff;
            }
            if (iter->second.y < max_right)
            {
                max_right = iter->second.y;
                max_right_index = iter->first & 0xffff;
            }
        }

    }

    fragment.max_region[ROFFSET+RDOWN]  =  max_down - (int)(XPIXEL*YPIXEL) * ((max_down / (int)(XPIXEL*YPIXEL)) - group_plane) ;
    fragment.max_region[ROFFSET+RUP]    = max_up - (int)(XPIXEL*YPIXEL) * ((max_up / (int)(XPIXEL*YPIXEL)) - group_plane) ;
    fragment.max_region[ROFFSET+RLEFT]  = max_left_index - (int)(XPIXEL*YPIXEL) * ((max_left_index / (int)(XPIXEL*YPIXEL)) - group_plane) ;
    fragment.max_region[ROFFSET+RRIGHT] = max_right_index - (int)(XPIXEL*YPIXEL) * ((max_right_index / (int)(XPIXEL*YPIXEL)) - group_plane) ;
#endif
    // PaddVerticalDirt(group_plane);
}

int ZDOWNVerticalPlane()
{
    if (group.data.size() <= 0) return -1; 

    std::vector<std::map<int , pcl::PointXYZ>> regions = EuclideanCluster(group, XPIXEL * YPIXEL);
    std::sort(group.plane.begin(), group.plane.end(),
              [&](const PlaneInfo& a, const PlaneInfo& b) {
                  return a.num > b.num;
              });
#if WARNREGION
    if (group.plane[0].altitude > -1 * ROBOTHIGHT)
    {
        fragment.group_fragment = regions[group.plane[0].index];
        if (group.plane[0].altitude > group.plane[group.plane.size() - 1 ].altitude)
        {
            fragment.pit_fragment = regions[group.plane[group.plane.size() - 1].index];
        } 
    } else {
        std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;  // 执行了这里???
        fragment.pit_fragment = regions[group.plane[0].index];
    }
#else
    fragment.group_fragment = regions[group.plane[0].index];
#endif
    fragment.max_region[RDOWN]  = GetDownRegion(regions[group.plane[0].index]);
    fragment.max_region[RUP]    = GetUpRegion(regions[group.plane[0].index]);
    fragment.max_region[RLEFT]  = GetLeftRegion(regions[group.plane[0].index]);
    fragment.max_region[RRIGHT] = GetRightRegion(regions[group.plane[0].index]);

    group.plane.clear();

    if (fragment.group_fragment.size() <= 0)
        return -1;
    return ParseCurPlane(fragment.group_fragment);
}

void Index_Conversions(pcl::PointXYZ& point)
{
#if REALPOINT
    int index, color_index ;
    /// 相对于CENTER的格子位置
    int box_x = static_cast<int>(point.x * 10) + CENTER.x;
    int box_y = static_cast<int>(point.y * 10) + CENTER.y; 
    int box_z = static_cast<int>(point.z * 10) + CENTER.z; 

    int abs_x = std::abs(box_x), abs_z = std::abs(box_z) - 1 , abs_y = std::abs(box_y);
    if ( abs_x <= 0) {
        index = abs_z * XPIXEL * YPIXEL + abs_y;
    } else {
        int plane_offset = ( abs_x - 1 ) * YPIXEL;
        index = abs_z * XPIXEL * YPIXEL + plane_offset + abs_y;
    }
    if ((index >= 0) && index < (XPIXEL * YPIXEL * ZPIXEL))
    {
        if (point.z < (-1. * ROBOTHIGHT / 2.0) )
        {
            color_index = group.color << 16 | index;
            group.data[color_index] = point;
        } else {
            color_index = barrier.color << 16 | index;
            barrier.data[color_index] = point;
        }
    } 
#else

#endif
}


void WebSend()
{
    std::vector<int> indexs;
    for (auto iter = fragment.group_fragment.begin(); iter != fragment.group_fragment.end(); ++iter)
        indexs.emplace_back(iter->first);
    for (int i = 0; i < fragment.barrier_fragment.size(); ++i)
        for (auto j = fragment.barrier_fragment[i].begin(); j != fragment.barrier_fragment[i].end(); ++j)
            indexs.emplace_back(j->first);
    for (int i = 0; i < fragment.edge_fragment.size(); ++i)
        for (auto j = fragment.edge_fragment[i].begin(); j != fragment.edge_fragment[i].end(); ++j)
            indexs.emplace_back(j->first);
    // for (auto iter = fragment.pit_fragment.begin(); iter != fragment.pit_fragment.end(); ++iter)
    //     indexs.emplace_back(iter->first);

    fragment.group_fragment.clear();
    fragment.barrier_fragment.clear();
    fragment.edge_fragment.clear();
    fragment.pit_fragment.clear();
    std::cout << "发送的个数为: " << indexs.size() << std::endl;
    if (indexs.size() > 0)
    {
        
        web->Push(indexs);
    }
#if OFFLINEDATA   
    static int file_index = 0;
    if (file_index++ % 5 == 0)
    {
    	std::string file_name = std::to_string(file_index) + ".txt";
    	std::ofstream ofs(file_name.c_str(), std::ios::out);
    	for (int i  = 0 ; i < indexs.size(); ++i)
    	{
    		ofs <<  indexs[i] << endl;
    	}
    	ofs.close();    
    	std::cout << "file_index = " << file_index << "  box_index.size() = " << box_index.size() << std::endl;
    }
#endif  
}
void send_point(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    // 读取RGB-D点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);


    // std::cout << "输入的点云数量: " << cloud->points.size() << std::endl;

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-2.0, 2.0);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2.0, 2.0);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-2.0, 2.0);
    pass.filter(*cloud);
    //std::cout << "直通滤波后的点云数量: " << cloud->points.size() << std::endl;

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(INTERVAL);
    outrem.setMinNeighborsInRadius(10);
    outrem.filter(*cloud);
    //std::cout << "半径滤波点云数量: " << cloud->points.size() << std::endl;

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(INTERVAL, INTERVAL , INTERVAL);
    sor.filter(*cloud);
    std::cout << "体素滤波后的点云数量: " << cloud->points.size() << std::endl;


    group.data.clear();
    barrier.data.clear();
    edge.data.clear();
    pit.data.clear();

    for (pcl::PointXYZ& point : cloud->points)
    {
        if ((point.x >= -2.0 && point.x <= 2.0 )
            && (point.y >= -2.0 && point.y <= 2.0 )
            && (point.z >= -2.0 && point.z <= 2.0 ))
        {
            Index_Conversions(point);
        }
    }
    
    int group_plane = ZDOWNVerticalPlane();
    ZUPVerticalPlane(group_plane);

    XYHorizontalPlane(group_plane);
    WebSend();




    ///////////////////////////////////////////////////////////////
#if TOPICPUB
    float dist_thres = 2 * INTERVAL ;
    std::vector<std::vector<PointType>> regions = clustering(cloud, dist_thres);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::size_t i = 0; i < regions.size(); ++i)
    {
        if (regions[i].size() < 5) 
        {
            for (int j = 0; j < regions[i].size(); j++)
                cloud2->points.emplace_back(regions[i][j]);
        }
        else
            for (int j = 0; j < regions[i].size(); j++)
                cloud1->points.emplace_back(regions[i][j]);
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud1, output);
    output.header.frame_id = "rgb_camera_link";  // 指定坐标系
    output.header.stamp = ros::Time::now();
    pointcloud_pub.publish(output);

    sensor_msgs::PointCloud2 output_remove;
    pcl::toROSMsg(*cloud2, output_remove);
    output_remove.header.frame_id = "rgb_camera_link";  // 指定坐标系
    output_remove.header.stamp = ros::Time::now();
    pointcloud_remove_pub.publish(output_remove);
#else
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "rgb_camera_link";  // 指定坐标系
    output.header.stamp = ros::Time::now();
    pointcloud_pub.publish(output);
#endif 
    //////////////////////////////////////////////////////////////
   

    
}
int main(int argc, char **argv)
{
	web = std::make_shared<WEB>();
    ros::init(argc,argv,"avm");
    ros::NodeHandle nh, nh_private("~");
#if TCPSOCKET
    net_register();
#endif

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/output",10, send_point);
    // 定义发布点云的话题
    pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cj", 10);
    pointcloud_remove_pub = nh.advertise<sensor_msgs::PointCloud2>("cj_remove_point", 1);



    ros::Rate rate(50); 
    while(ros::ok() )  
    {
#if TCPSOCKET
        readIO();
#endif

        ros::spinOnce();    
        rate.sleep();
    }
    std::cout << "bye main" << std::endl;
#if TCPSOCKET
    for (auto fd : net_connect)
        close(fd);
#endif
    return 0;
}
