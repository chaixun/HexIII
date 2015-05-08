#include "kinect_test.h"

void Kinect::start()
{
    interface->start();
}

void Kinect::capture()
{
    frames_num++;
    save_one = true;
}

void Kinect::pointcloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    //Remove NAN
    pcl::PointCloud<pcl::PointXYZ> cloud1;
    cloud1 = *cloud;
    cloud1.width = 640*480;
    cloud1.height = 1;
    std::vector<int> mapping;
    pcl::PointCloud<pcl::PointXYZ>::Ptr newcloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::removeNaNFromPointCloud(*cloud,cloud1,mapping);
    //delete newcloud;

    //Filtering VoxelGrid
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtercloudgrid(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::VoxelGrid<pcl::PointXYZ> filtergrid;
    //filtergrid.setInputCloud(newcloud);
    //filtergrid.setLeafSize(0.005f,0.005f,0.005f);
    //filtergrid.filter(*filtercloudgrid);

    //if(save_one)
    {
       // save_one = false;
      // std::stringstream out;
        //out<<frames_num;
       // std::string outnanname = "outnancloud" + out.str() + ".pcd";
        //std::string voxelname = "voxelcloud" + out.str() + ".pcd";

        //pcl::io::savePCDFileASCII(outnanname,*newcloud);
        //pcl::io::savePCDFileASCII(voxelname,*filtercloudgrid);

        cout<<"Capture "<<frames_num-1<<"end"<<endl;
    }
}
