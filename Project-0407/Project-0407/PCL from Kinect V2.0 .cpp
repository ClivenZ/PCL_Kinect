//Kinect ����⺯��
#include "kinect.h"
//C++ ��׼IO��
#include <iostream>
//Opencv coro�� highgui��
#include <opencv2/core/core.hpp>  
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp> 
//pcl io��
#include <pcl/io/io.h>
//pcl io�� ��д.pcd �ļ�
#include <pcl/io/pcd_io.h>
//pcl ������
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace cv;
using namespace std;
using namespace pcl;

// ����ڲ�(����Ҫ���д����޸ģ�������)
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

////�Զ��嶨������ͽṹ
//struct MyPointType {
//
//	PCL_ADD_POINT4D;		//�õ��������ĸ�Ԫ��
//	float test;
//	/*EIGEN_MAKE_ALIGEND_OPERATOR_NEW;*/
//	EIGEN_ALIGN16;
//};
//
////ע������ͺ�
//POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointType,
//	(float, x, x) (float, y, y) (float, z, z) (float, test, test))
//
//


// ��ȫ�ͷ�ָ��
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
	printf("�����������\n");
}

int main()
{
	// ��ȡKinect�豸
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
			// ��ȡ������Դ����ȡ��  
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
	// ��������֡������
	IDepthFrameReference* m_pDepthFrameReference = nullptr;
	IColorFrameReference* m_pColorFrameReference = nullptr;
	IDepthFrame* m_pDepthFrame = nullptr;
	IColorFrame* m_pColorFrame = nullptr;

	// ����ͼƬ��ʽ
	Mat i_rgb(1080, 1920, CV_8UC4);      //ע�⣺�������Ϊ4ͨ����ͼ��Kinect������ֻ����Bgra��ʽ����
	Mat i_depth(424, 512, CV_8UC1);
	Mat i_src_depth(424, 512, CV_16UC1);

	UINT16 *depthData = new UINT16[424 * 512];
	IMultiSourceFrame* m_pMultiFrame = nullptr;
	while (true)
	{
		// ��ȡ�µ�һ����Դ����֡
		hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);
		if (FAILED(hr) || !m_pMultiFrame)
		{
			//cout << "!!!" << endl;
			continue;
		}

		// �Ӷ�Դ����֡�з������ɫ���ݣ�������ݺͺ�������
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_ColorFrameReference(&m_pColorFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pColorFrameReference->AcquireFrame(&m_pColorFrame);
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_DepthFrameReference(&m_pDepthFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pDepthFrameReference->AcquireFrame(&m_pDepthFrame);
		

		// color������ͼƬ��
		UINT nColorBufferSize = 1920 * 1080 * 4;
		if (SUCCEEDED(hr))
			hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);

		// depth������ͼƬ��
		if (SUCCEEDED(hr))
		{
			hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, depthData);
			//for (int i = 0; i < 512 * 424; i++)
			//{
			//  // 0-255���ͼ��Ϊ����ʾ���ԣ�ֻȡ������ݵĵ�8λ
			//  BYTE intensity = static_cast<BYTE>(depthData[i] % 256);
			//  reinterpret_cast<BYTE*>(i_depth.data)[i] = intensity;
			//}

			// ʵ����16λunsigned int����
			hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_src_depth.data));
		}


		//Mat i_rgb_resize = i_rgb.clone();       // ��С���㿴
		//cv::resize(i_rgb_resize, i_rgb_resize, Size(512, 424));
		////// ��ʾ
		//cv::imshow("rgb", i_rgb_resize);
		//if (waitKey(1) == VK_ESCAPE)
		//	break;
		//cv::imshow("i_src_depth", i_src_depth);
		//if (waitKey(1) == VK_ESCAPE)
		//	break;
		
		if (waitKey(100))
		{
			Mat i_rgb_resize = i_rgb.clone();       // ��С���㿴
			cv::resize(i_rgb_resize, i_rgb_resize, Size(512, 424));
			//// ��ʾ
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
	// �ͷ���Դ
	SafeRelease(m_pColorFrame);
	SafeRelease(m_pDepthFrame);
	SafeRelease(m_pColorFrameReference);
	SafeRelease(m_pDepthFrameReference);
	SafeRelease(m_pMultiFrame);
	// �رմ��ڣ��豸
	destroyAllWindows();
	m_pKinectSensor->Close();
	system("pause");
	return 0;
}




