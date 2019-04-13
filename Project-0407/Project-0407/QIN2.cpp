#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <kinect.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace cv;
IKinectSensor* pSensor = nullptr;
ICoordinateMapper* pMapper = nullptr;
const int iWidth = 512, iHeight = 424;
CameraSpacePoint depth2xyz[iWidth*iHeight];
ColorSpacePoint depth2rgb[iWidth*iHeight];


void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1, 1, 1);//设置背景颜色 
}

bool initKinect()
{
	if (FAILED(GetDefaultKinectSensor(&pSensor))) return false;
	if (pSensor)
	{
		pSensor->get_CoordinateMapper(&pMapper);
		pSensor->Open();
		return true;
	}
	else return false;
}

void getPointCloudFromImage(Mat depthImage, Mat rgbImage, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out)
{

	pMapper->MapDepthFrameToCameraSpace(iWidth*iHeight, reinterpret_cast<UINT16*>(depthImage.data), iWidth*iHeight, depth2xyz);
	pMapper->MapDepthFrameToColorSpace(512 * 424, reinterpret_cast<UINT16*>(depthImage.data), 512 * 424, depth2rgb);

	cloud_out.height = 1;
	cloud_out.is_dense = 1;

	for (size_t i = 0; i < iWidth; i++)
	{
		for (size_t j = 0; j < iHeight; j++)
		{
			pcl::PointXYZRGB pointTemp;
			if (depth2xyz[i + j * iWidth].Z > 0.5)
			{

				pointTemp.x = depth2xyz[i + j * iWidth].X;
				pointTemp.y = depth2xyz[i + j * iWidth].Y;
				pointTemp.z = depth2xyz[i + j * iWidth].Z;
				int X = static_cast<int>(depth2rgb[j * 512 + i].X);
				int Y = static_cast<int>(depth2rgb[j * 512 + i].Y);
				if (X > 0 && Y > 0 && X < 1920 && Y < 1080)
				{
					Vec3b* pixelsRGBImage = rgbImage.ptr<Vec3b>(Y);
					pointTemp.g = pixelsRGBImage[X][0];
					pointTemp.b = pixelsRGBImage[X][1];
					pointTemp.r = pixelsRGBImage[X][2];
					cloud_out.push_back(pointTemp);
				}
				else continue;

			}
		}
	}
}

int main()
{
	initKinect();
	Mat rgbImage = imread("GDB.jpg");
	Mat depthImage = imread("Depth.jpg", -1);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
	getPointCloudFromImage(depthImage, rgbImage, *cloud_out);
	pcl::visualization::CloudViewer viewerG("Cloud Viewe");
	pcl::io::savePCDFileASCII("输出点云.pcd", *cloud_out);
	//pcl::PLYWriter writer;
	//writer.write("输出点云.ply", *cloud_out);
	viewerG.runOnVisualizationThreadOnce(viewerOneOff);
	viewerG.showCloud(cloud_out);
	while (true) if (cv::waitKey(30) == VK_ESCAPE) break;
	return 0;
}
