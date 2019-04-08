#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

/**********************************************************************************
  函数是作为回调函数，在主函数中只注册一次 ，函数实现对可视化对象背景颜色的设置，添加一个圆球几何体
*********************************************************************************/
int user_data;

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 0.5, 1.0);       //设置背景颜色
	pcl::PointXYZ o;                                //存储球的圆心位置
	o.x = 1.0;
	o.y = 0;
	o.z = 0;
	viewer.addSphere(o, 0.25, "sphere", 0);         //添加圆球几何对象
	std::cout << "i only run once" << std::endl;

}
/***********************************************************************************
作为回调函数，在主函数中注册后每帧显示都执行一次，函数具体实现在可视化对象中添加一个刷新显示字符串
*************************************************************************************/
void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	static unsigned count = 0;
	std::stringstream ss;
	ss << "Once per viewer loop: " << count++;
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);

	//FIXME: possible race condition here:
	user_data++;
}
/**************************************************************
首先加载点云文件到点云对象，并初始化可视化对象viewer，注册上面的回
 调函数，执行循环直到收到关闭viewer的消息退出程序
 *************************************************************/
int main()
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);    //声明cloud 
	pcl::io::loadPCDFile("txt2pcd_bunny1.pcd", *cloud);         //加载点云文件

	pcl::visualization::CloudViewer viewer("Cloud Viewer");      //创建viewer对象

	//showCloud函数是同步的，在此处等待直到渲染显示为止
	viewer.showCloud(cloud);

	//该注册函数在可视化的时候只执行一次
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	//该注册函数在渲染输出时每次都调用
	viewer.runOnVisualizationThread(viewerPsycho);
	while (!viewer.wasStopped())
	{
		//此处可以添加其他处理
		//FIXME: Note that this is running in a separate thread from viewerPsycho
		//and you should guard against race conditions yourself...
		user_data++;
	}
	return 0;
}
