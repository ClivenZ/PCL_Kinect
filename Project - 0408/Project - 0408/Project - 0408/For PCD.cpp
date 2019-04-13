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
////pcl ���ӻ�����
//#include <pcl/visualization/cloud_viewer.h>

#include <GLFW/glfw3.h>

using namespace cv;
using namespace std;
using namespace pcl;


// ����ڲ�(����Ҫ���д����޸ģ�������)
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;


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



void pcl_init() {
	cv::Mat color = cv::imread("GDB.jpg");
	cv::Mat depth = cv::imread("Depth.jpg");

	int rowNumber = color.rows;
	int colNumber = color.cols;

	pcl::PointCloud<pcl::PointXYZRGBA> cloud;

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
	pcl::io::savePCDFile("MyPCL.pcd", cloud);
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
	//����ڲ�
	Eigen::Matrix3d intricRGB;
	intricRGB << 1094.75283, 0, 942.00992,
		0, 1087.37528, 530.35240,
		0, 0, 1;
	Eigen::Matrix3d intricDepth;
	intricDepth << camera_fx, 0, camera_cx,
		0, camera_fy, camera_cy,
		0, 0, 1;
	Eigen::Matrix3d intricdepth2RGB;
	intricdepth2RGB = intricRGB * intricDepth.inverse();
	
	// ��ȡKinect�豸
	IKinectSensor* m_pKinectSensor;
	HRESULT hr;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	IMultiSourceFrameReader* m_pMultiFrameReader;
	IMultiSourceFrameReader* m_pDepthFrameReader;
	IMultiSourceFrameReader* m_pColorFrameReader;

	if (m_pKinectSensor)
	{
		hr = m_pKinectSensor->Open();
		if (SUCCEEDED(hr))
		{
			// ��ȡ������Դ����ȡ��  
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Color |
				FrameSourceTypes::FrameSourceTypes_Infrared |
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


	DepthSpacePoint* m_pDepthCoordinates = NULL;
	ColorSpacePoint* m_pColorCoordinates = NULL;
	CameraSpacePoint* m_pCameraCoordinates = NULL;


	// ����ͼƬ��ʽ
	Mat i_rgb(1080, 1920, CV_8UC4);      //ע�⣺�������Ϊ4ͨ����ͼ��Kinect������ֻ����Bgra��ʽ����
	Mat i_depth(424, 512, CV_8UC1);
	Mat i_src_depth(424, 512, CV_16UC1);
	Mat result(424, 512, CV_8UC4);

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
			for (int i = 0; i < 512 * 424; i++)
			{
				// 0-255���ͼ��Ϊ����ʾ���ԣ�ֻȡ������ݵĵ�8λ
				UINT16 intensity = static_cast<UINT16>(depthData[i]);
				reinterpret_cast<UINT16*>(i_depth.data)[i] = intensity;
			}

			// ʵ����16λunsigned int����
			//hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_depth.data));
		}
		
		
		//������֡�Ͳ�ɫ֡
		hr = m_pDepthFrameReader->AcquireLatestFrame(&m_pDepthFrame);
		hr = m_pDepthFrameReader->AcquireLatestFrame(&m_pColorFrame);

		//����������ݺ���ɫ���ݵ�����
		UINT16 *depthData = new UINT16[424 * 512];
		hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, depthData);

		Mat i_rgb(1080, 1920, CV_8UC4);
		nColorBufferSize = 1920 * 1080 * 4;
		if (SUCCEEDED(hr))
			hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, i_rgb.data, ColorImageFormat::ColorImageFormat_Bgra);
		
		
		//�������ӳ����
		ICoordinateMapper*      m_pCoordinateMapper;
		hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		ColorSpacePoint* m_pColorCoordinates = new ColorSpacePoint[512 * 424];
		
		
		//���ͼӳ�䵽��ɫͼ
		HRESULT hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(512 * 424, depthData, 512 * 424, m_pColorCoordinates);
		Mat i_depthToRgb(424, 512, CV_8UC4);
		if (SUCCEEDED(hr))
		{
			for (int i = 0; i < 424 * 512; i++)
			{
				ColorSpacePoint p = m_pColorCoordinates[i];
				if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
				{
					int colorX = static_cast<int>(p.X + 0.5f);
					int colorY = static_cast<int>(p.Y + 0.5f);

					if ((colorX >= 0 && colorX < 1920) && (colorY >= 0 && colorY < 1080))
					{
						i_depthToRgb.data[i * 4] = i_rgb.data[(colorY * 1920 + colorX) * 4];
						i_depthToRgb.data[i * 4 + 1] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 1];
						i_depthToRgb.data[i * 4 + 2] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 2];
						i_depthToRgb.data[i * 4 + 3] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 3];
					}
				}
			}
		}
		//���ͼӳ�䵽����ռ�
		if (SUCCEEDED(hr))
		{
			HRESULT hr = m_pCoordinateMapper->MapDepthFrameToCameraSpace(512 * 424, depthData, 512 * 424, m_pCameraCoordinates);
		}
		//OpenGL��ʾ
		if (SUCCEEDED(hr))
		{
			for (int i = 0; i < 512 * 424; i++)
			{
				CameraSpacePoint p = m_pCameraCoordinates[i];
				if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity() && p.Z != -std::numeric_limits<float>::infinity())
				{
					float cameraX = static_cast<float>(p.X);
					float cameraY = static_cast<float>(p.Y);
					float cameraZ = static_cast<float>(p.Z);

					//cout << "x: " << cameraX << "y: " << cameraY << "z: " << cameraZ << endl;
					GLubyte *rgb = new GLubyte();
					rgb[2] = result.data[i * 4 + 0];
					rgb[1] = result.data[i * 4 + 1];
					rgb[0] = result.data[i * 4 + 2];
					// ��ʾ��
					glColor3ubv(rgb);
					glVertex3f(cameraX, -cameraY, cameraZ);
				}
			}
		}
		/*if (waitKey(1) == VK_ESCAPE)
			break;
		imshow("mosic", result);
		if (waitKey(1) == VK_ESCAPE)
			break;*/


		//Mat i_rgb_resize = i_rgb.clone();       // ��С���㿴
		//cv::resize(i_rgb_resize, i_rgb_resize, Size(512, 424));
		//// ��ʾ
		//imshow("rgb", i_rgb_resize);
		//if (waitKey(1) == VK_ESCAPE)
		//	break;
		//imshow("i_src_depth", i_src_depth);
		//if (waitKey(1) == VK_ESCAPE)
		//	break;

		// �ͷ���Դ
		SafeRelease(m_pColorFrame);
		SafeRelease(m_pDepthFrame);
		SafeRelease(m_pColorFrameReference);
		SafeRelease(m_pDepthFrameReference);
		SafeRelease(m_pMultiFrame);
	}
	// �رմ��ڣ��豸
	cv::destroyAllWindows();
	m_pKinectSensor->Close();
	std::system("pause");
	return 0;
}
