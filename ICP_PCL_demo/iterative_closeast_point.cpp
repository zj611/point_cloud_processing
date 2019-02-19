#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr read_cloud_point(std::string const &file_path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(file_path,*cloud)==-1)
    {
        PCL_ERROR("Could not read the pcd file\b");
        return nullptr;
    }
    return cloud;
}

bool next_interation = false;
void keyboardEvent(const pcl::visualization::KeyboardEvent &event,void *nothing)
{
    if(event.getKeySym()=="space"&&event.keyDown())
        next_interation=true;
}



int main()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("icp --view1 red:input green:target --view2 green:target white:aligend"));
    int v1,v2;//定义两个窗口，v1用来显示初始值，v2用来显示配准过程
    view->createViewPort(0.0,0.0,0.5,1.0,v1);//(xmin,ymin,xmax,ymax,int &viewport)
    view->createViewPort(0.5,0.0,1.0,1.0,v2);

    //输入点云
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source = read_cloud_point("cloud1.pcd");
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target = read_cloud_point("cloud2.pcd");
    auto cloud_input = read_cloud_point("../cloud1.pcd");
    auto cloud_target = read_cloud_point("../cloud2.pcd");

    //设置点云颜色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_input_color(cloud_input,250,0,0);//输入点云red
    view->addPointCloud(cloud_input,cloud_input_color,"cloud_input_v1",v1);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_target_color(cloud_target,0,255,0);//目标点云green
    view->addPointCloud(cloud_target,cloud_target_color,"cloud_target_v1",v1);

    view->setBackgroundColor(0.0,0.05,0.05,v1);//设置窗口背景颜色
    view->setBackgroundColor(0.05,0.05,0.05,v2);
    //设置显示点的大小
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"cloud_input_v1");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"cloud_target_v1");

    //设置配准结果为白色
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligend(new pcl::PointCloud<pcl::PointXYZ>);//已配准点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_aligend_color(cloud_aligend,255,255,255);
    view->addPointCloud(cloud_aligend,cloud_aligend_color,"cloud_aligend_v2",v2);
    view->addPointCloud(cloud_target,cloud_target_color,"cloud_target_v2",v2);
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"cloud_aligend_v2");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,"cloud_target_v2");

    //创建icp
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
    //点云的输入
    icp.setInputSource(cloud_input);
    icp.setInputTarget(cloud_target);
    icp.setMaxCorrespondenceDistance(0.05);//设置最大对应点距离，大于该值会被忽略
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(1);
    icp.setMaximumIterations(100);
    //执行对齐操作
    //icp.align(cloud_final);


    //设置键盘回调函数
    view->registerKeyboardCallback(&keyboardEvent,(void*)NULL);
    int iterations = 0;//迭代次数

    while(!view->wasStopped())
    {
        view->spinOnce();//运行视图
        if(next_interation)
        {
            icp.align(*cloud_aligend);//ICP计算，将输入点云和目标点云进行对齐配准操作
            cout<<"has converged: "<<icp.hasConverged()<<" score:"<<icp.getFitnessScore()<<endl;
            cout<<"matrix:\n"<<icp.getFinalTransformation()<<endl;
            cout<<"iterations = "<<++iterations<<endl;
            cout<<"number of cloud_aligend = "<<cloud_aligend->size()<<endl;
            if(iterations == 1000)
                return 0;
            view->updatePointCloud(cloud_aligend,cloud_aligend_color,"cloud_aligend_v2");
        }
        next_interation = false;//本次迭代结束，等待触发
    }
}

