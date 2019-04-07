//Kinect 相机库函数
#include "kinect.h"
//C++ 标准IO库
#include <iostream>
//Opencv coro库 highgui库
#include <opencv2/core/core.hpp>  
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp> 
//pcl io库
#include <pcl/io/io.h>
//pcl io库 读写.pcd 文件
#include <pcl/io/pcd_io.h>
//pcl 点类型
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace cv;
using namespace std;
using namespace pcl;

// 相机内参(还需要进行大量修改！！！！)
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

////自定义定义点类型结构
//struct MyPointType {
//
//	PCL_ADD_POINT4D;		//该点类型有四个元素
//	float test;
//	/*EIGEN_MAKE_ALIGEND_OPERATOR_NEW;*/
//	EIGEN_ALIGN16;
//};
//
////注册点类型宏
//POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointType,
//	(float, x, x) (float, y, y) (float, z, z) (float, test, test))
//
//


// 安全释放指针
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

/*typedef pcl:PointCloud colud*/;

void pcl_init() {
	cv::Mat color = cv::imread("GDB.jpg");
	cv::Mat depth = cv::imread("Depth.jpg");

	int rowNumber = color.rows;
	int colNumber = color.cols;

	pcl::PointCloud<pcl::PointXYZRGB> cloud;

	cloud.height = rowNumber;
	cloud.width = colNumber;
	cloud.points.resize(cloud.width * cloud.height);

	for (unsigned int u = 0; u < rowNumber; ++u)
	{
		for (unsigned int v = 0; v < colNumber; ++v)
		{
			unsigned int num = u * colNumber + v;
			double Xw = 0, Yw = 0, Zw = 0;

			Zw = ((double)depth.at<uchar>(u, v)) / 255.0 * 10001.0;
			Xw = (u - camera_cx) * Zw / camera_fx;
			Yw = (v - camera_cy) * Zw / camera_fy;

			cloud.points[num].b = color.at<cv::Vec3b>(u, v)[0];
			cloud.points[num].g = color.at<cv::Vec3b>(u, v)[1];
			cloud.points[num].r = color.at<cv::Vec3b>(u, v)[2];

			cloud.points[num].x = Xw;
			cloud.points[num].y = Yw;
			cloud.points[num].z = Zw;
		}
	}

	/**cloud = cloud;*/
	pcl::io::savePCDFile("colorImage.pcd", cloud);
	/*pcl::visualization::CloudViewer viewer("Cloud Viewer");

	viewer.showCloud(cloud);

	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	while (!viewer.wasStopped())
	{
		user_data = 9;
	}*/
	printf("点云生成完毕\n");
}

int main()
{
	// 获取Kinect设备
	IKinectSensor* m_pKinectSensor;
	HRESULT hr;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	IMultiSourceFrameReader* m_pMultiFrameReader = nullptr;
	if (m_pKinectSensor)
	{
		hr = m_pKinectSensor->Open();
		if (SUCCEEDED(hr))
		{
			// 获取多数据源到读取器  
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Color |
				FrameSourceTypes::FrameSourceTypes_Depth,
				&m_pMultiFrameReader);
		}
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		return E_FAIL;
	}
	// 三个数据帧及引用
	IDepthFrameReference* m_pDepthFrameReference = nullptr;
	IColorFrameReference* m_pColorFrameReference = nullptr;
	IDepthFrame* m_pDepthFrame = nullptr;
	IColorFrame* m_pColorFrame = nullptr;

	// 三个图片格式
	Mat i_rgb(1080, 1920, CV_8UC4);      //注意：这里必须为4通道的图，Kinect的数据只能以Bgra格式传出
	Mat i_depth(424, 512, CV_8UC1);
	Mat i_src_depth(424, 512, CV_16UC1);

	UINT16 *depthData = new UINT16[424 * 512];
	IMultiSourceFrame* m_pMultiFrame = nullptr;
	while (true)
	{
		// 获取新的一个多源数据帧
		hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);
		if (FAILED(hr) || !m_pMultiFrame)
		{
			//cout << "!!!" << endl;
			continue;
		}

		// 从多源数据帧中分离出彩色数据，深度数据和红外数据
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_ColorFrameReference(&m_pColorFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pColorFrameReference->AcquireFrame(&m_pColorFrame);
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_DepthFrameReference(&m_pDepthFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pDepthFrameReference->AcquireFrame(&m_pDepthFrame);
		

		// color拷贝到图片中
		UINT nColorBufferSize = 1920 * 1080 * 4;
		if (SUCCEEDED(hr))
			hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);

		// depth拷贝到图片中
		if (SUCCEEDED(hr))
		{
			hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, depthData);
			//for (int i = 0; i < 512 * 424; i++)
			//{
			//  // 0-255深度图，为了显示明显，只取深度数据的低8位
			//  BYTE intensity = static_cast<BYTE>(depthData[i] % 256);
			//  reinterpret_cast<BYTE*>(i_depth.data)[i] = intensity;
			//}

			// 实际是16位unsigned int数据
			hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_src_depth.data));
		}


		//Mat i_rgb_resize = i_rgb.clone();       // 缩小方便看
		//cv::resize(i_rgb_resize, i_rgb_resize, Size(512, 424));
		////// 显示
		//cv::imshow("rgb", i_rgb_resize);
		//if (waitKey(1) == VK_ESCAPE)
		//	break;
		//cv::imshow("i_src_depth", i_src_depth);
		//if (waitKey(1) == VK_ESCAPE)
		//	break;
		
		if (waitKey(100))
		{
			Mat i_rgb_resize = i_rgb.clone();       // 缩小方便看
			cv::resize(i_rgb_resize, i_rgb_resize, Size(512, 424));
			//// 显示
			cv::imshow("rgb", i_rgb_resize);
			if (waitKey(1) == VK_ESCAPE)
				break;
			cv::imshow("i_src_depth", i_src_depth);
			if (waitKey(1) == VK_ESCAPE)
				break;
			imwrite("Depth.jpg", i_src_depth);
			imwrite("GDB.jpg", i_rgb_resize);
			pcl_init();

			//Sleep(10000);
			//break;
		}
	}
	// 释放资源
	SafeRelease(m_pColorFrame);
	SafeRelease(m_pDepthFrame);
	SafeRelease(m_pColorFrameReference);
	SafeRelease(m_pDepthFrameReference);
	SafeRelease(m_pMultiFrame);
	// 关闭窗口，设备
	destroyAllWindows();
	m_pKinectSensor->Close();
	system("pause");
	return 0;
}




