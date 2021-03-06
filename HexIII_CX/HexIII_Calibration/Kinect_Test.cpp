#include "Kinect_Test.h"

bool Kinect::IsStepCaptureEnd;

Kinect::Kinect():interface(new pcl::OpenNIGrabber())
{
    frames_num = 0;
    save_one = false;
    Kinect::IsStepCaptureEnd = false;
}
Kinect::~Kinect()
{
    interface->stop();
    cout<<"Depth Close"<<endl;
}

void Kinect::start()
{
    interface->start();
    cout<<"Depth Open"<<endl;
}

void Kinect::capture(int *num)
{
    (*num)++;
    frames_num++;
    save_one = true;
    cout<<"NUM "<<*num<<endl;
}

void Kinect::pointcloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    //Remove NAN
    pcl::PointCloud<pcl::PointXYZ> cloud1;
    cloud1 = *cloud;
    cloud1.width = 640*480;
    cloud1.height = 1;

    //std::vector<int> mapping;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr newcloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::removeNaNFromPointCloud(*cloud,*newcloud,mapping);

    //    //Filtering VoxelGrid
    //    pcl::PointCloud<pcl::PointXYZ>::Ptr filtercloudgrid(new pcl::PointCloud<pcl::PointXYZ>);
    //    pcl::VoxelGrid<pcl::PointXYZ> filtergrid;
    //    filtergrid.setInputCloud(newcloud);
    //    filtergrid.setLeafSize(0.005f,0.005f,0.005f);
    //    filtergrid.filter(*filtercloudgrid);

    if(save_one)
    {
        save_one = false;
        std::stringstream out;
        out<<frames_num-1;
        std::string dataname = "cloud" + out.str() + ".pcd";
        //        std::string outnanname = "outnancloud" + out.str() + ".pcd";
        //        std::string voxelname = "voxelcloud" + out.str() + ".pcd";
        pcl::io::savePCDFileASCII(dataname,cloud1);
        //        pcl::io::savePCDFileASCII(outnanname,*newcloud);
        //        pcl::io::savePCDFileASCII(voxelname,*filtercloudgrid);

        cout<<"Capture "<<frames_num-1<<" end"<<endl;
        Kinect::IsStepCaptureEnd = true;
    }
}

void Kinect::viewcloud()
{
    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
            boost::bind (&Kinect::pointcloud, this, _1);

    interface->registerCallback (f);
}

/*
void Kinect::map(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    //Remove NAN
    std::vector<int> mapping;
    pcl::PointCloud<pcl::PointXYZ>::Ptr newcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::removeNaNFromPointCloud(*cloud,*newcloud,mapping);

    //Filtering VoxelGrid
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtercloudgrid(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> filtergrid;
    filtergrid.setInputCloud(newcloud);
    filtergrid.setLeafSize(0.005f,0.005f,0.005f);
    filtergrid.filter(*filtercloudgrid);

    //Transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    pcl::transformPointCloud(*filtercloudgrid,*transformed,transformation);

    //Filtering Range
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtertransformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition(new pcl::ConditionAnd<pcl::PointXYZ>);
    condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr
                             (new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, 0.0)));
    condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr
                             (new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 3.0)));
    condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr
                             (new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, -1.5)));
    condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr
                             (new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 1.5)));

    pcl::ConditionalRemoval<pcl::PointXYZ> filterrange;
    filterrange.setCondition(condition);
    filterrange.setInputCloud(transformed);
    filterrange.setKeepOrganized(false);
    filterrange.filter(*filtertransformed);

    //Mapping GridMap
    Eigen::MatrixXf GridMap = Eigen::MatrixXf::Zero(120,120);
    Eigen::MatrixXf GridNum = Eigen::MatrixXf::Zero(120,120);
    for (size_t i = 0;i < filtertransformed->points.size(); ++i)
    {
        int m, n;
        n = floor(filtertransformed->points[i].x/0.025) + 60;
        m = floor(filtertransformed->points[i].z/0.025);

        //Mean
        GridMap(m,n) = (GridMap(m,n)*GridNum(m,n) + filtertransformed->points[i].y)/(GridNum(m,n) + 1);
        GridNum(m,n) = GridNum(m,n) + 1;

        //Max

     //if (GridMap(m,n) < filtertransformed->points[i].y)
     //{
      //  GridMap(m,n) = filtertransformed->points[i].y;
    // }

    }


    std::ofstream Gridmapfile("GridMap.txt");
    if (Gridmapfile.is_open())
    {
        Gridmapfile<<GridMap<<endl;
    }
}
    */

/*
void Kinect::getmap()
{
    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
            boost::bind (&Kinect::map, this, _1);
    boost::signals2::connection c = interface->registerCallback (f);
}
*/
void Kinect::stop()
{
    interface->stop();
}
