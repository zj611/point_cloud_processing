#include <iostream>
#include <pcl/io/pcd_io.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>


pcl::PointCloud<pcl::PointXYZ>::Ptr read_cloud_point(std::string const &file_path)
{
    // Loading first scan.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_path, *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read the pcd file\n");
        return nullptr;
    }
    return cloud;
}

//输入两个点云，使其可视化
void visualizer(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud)
{
    // Initializing point cloud visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer_final->setBackgroundColor (0, 0, 0);//设置背景为全黑色

    // Coloring and visualizing target cloud (red).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color (target_cloud, 255, 200, 0);
    viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "target cloud");

    // Coloring and visualizing transformed input cloud (green).(0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color (output_cloud, 0, 255, 100);
    viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1, "output cloud");

    // Starting visualizer
    viewer_final->addCoordinateSystem (1.0, "global");
    viewer_final->initCameraParameters ();

    // Wait until visualizer window is closed.
    while (!viewer_final->wasStopped ())
    {
        viewer_final->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

int main(int argc, char **argv)
{
    //auto target_cloud = read_cloud_point(argv[1]);
    auto target_cloud = read_cloud_point("../cloud1.pcd");
    std::cout << "Loaded " << target_cloud->size () << " data points from cloud1.pcd" << std::endl;

    //auto input_cloud = read_cloud_point(argv[2]);
    auto input_cloud = read_cloud_point("../cloud2.pcd");
    std::cout << "Loaded " << input_cloud->size () << " data points from cloud2.pcd" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //对输入点云进行滤波
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);

    approximate_voxel_filter.setInputCloud(input_cloud);
    approximate_voxel_filter.filter(*filtered_cloud);

    std::cout<<"Filtered cloud contains "<< filtered_cloud->size() << "data points from cloud2.pcd" << std::endl;

    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;//正态分布变换NDT算法的配准
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);
    ndt.setResolution(1.0);//设置分辨率

    ndt.setMaximumIterations(20);//设置最大迭代次数
    ndt.setInputSource(filtered_cloud);
    ndt.setInputTarget(target_cloud);
    //设置初始值
    Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    ndt.align(*output_cloud, init_guess);
    std::cout << "Normal Distribution Transform has converged:" << ndt.hasConverged()
              << "  score: " << ndt.getFitnessScore() << std::endl;

    pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());
    pcl::io::savePCDFileASCII("../cloud3.pcd", *output_cloud);

    visualizer(target_cloud, output_cloud);

    return 0;
}