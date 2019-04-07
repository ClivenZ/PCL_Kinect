#include <Kinect.h>
#include "stdafx.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Dense>

//using Eigen::MatrixXd;
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

using namespace cv;
using namespace std;

//安全释放指针
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}
//坐标初始化
const double camera_factor = 1000;
const double camera_cx = 263.73696;
const double camera_cy = 201.72450;
const double camera_fx = 379.40726;
const double camera_fy = 378.54472;

//Kinect 获取 Depth 和 GDB 放入 Mat
int KinectForDepthAndGDB() {
//相机内参
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
	
// 获取Kinect设备
	IKinectSensor* m_pKinectSensor;
	HRESULT hr;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	IMultiSourceFrameReader* m_pMultiFrameReader = NULL;
	if (m_pKinectSensor)
	{
		hr = m_pKinectSensor->Open();
		if (SUCCEEDED(hr))
		{
			// 获取多数据源到读取器  
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
// 三个数据帧及引用
	IDepthFrameReference* m_pDepthFrameReference = NULL;
	IColorFrameReference* m_pColorFrameReference = NULL;
	IInfraredFrameReference* m_pInfraredFrameReference = NULL;
	IInfraredFrame* m_pInfraredFrame = NULL;
	IDepthFrame* m_pDepthFrame = NULL;
	IColorFrame* m_pColorFrame = NULL;
// 三个图片格式
	Mat i_rgb(1080, 1920, CV_8UC4);      //注意：这里必须为4通道的图，Kinect的数据只能以Bgra格式传出
	Mat i_depth(424, 512, CV_16UC1);
	Mat i_ir(424, 512, CV_16UC1);
	Mat result(424, 512, CV_8UC4);

	UINT16 *depthData = new UINT16[424 * 512];
	IMultiSourceFrame* m_pMultiFrame = nullptr;

	while (true) {
		// 获取新的一个多源数据帧
		hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);
		if (FAILED(hr) || !m_pMultiFrame)
		{
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
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_InfraredFrameReference(&m_pInfraredFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pInfraredFrameReference->AcquireFrame(&m_pInfraredFrame);
		
		// color拷贝到图片中
		UINT nColorBufferSize = 1920 * 1080 * 4;
		if (SUCCEEDED(hr))
			hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);

		// depth拷贝到图片中
		if (SUCCEEDED(hr))
		{
			hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, depthData);
			for (int i = 0; i < 512 * 424; i++)
			{
				// 0-255深度图，为了显示明显，只取深度数据的低8位
				UINT16 intensity = static_cast<UINT16>(depthData[i]);
				reinterpret_cast<UINT16*>(i_depth.data)[i] = intensity;
			}

			// 实际是16位unsigned int数据
			//hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_depth.data));
		}

		// infrared拷贝到图片中
		if (SUCCEEDED(hr))
		{
			hr = m_pInfraredFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_ir.data));
		}

		int i = 0;
		for (int row = 0; row < 424; row++)
		{
			for (int col = 0; col < 512; col++)
			{
				UINT16* p = (UINT16*)i_depth.data;
				UINT16 depthValue = static_cast<UINT16>(p[row * 512 + col]);
				//cout << "depthValue       " << depthValue << endl;
				if (depthValue != -std::numeric_limits<UINT16>::infinity() && depthValue != -std::numeric_limits<UINT16>::infinity() && depthValue != 0 && depthValue != 65535)
				{
					// 投影到彩色图上的坐标
					Eigen::Vector3d uv_depth(col, row, 1.0f);
					Eigen::Vector3d uv_color = (double)depthValue * intricdepth2RGB * uv_depth;//  / 1000.f +T / 1000;

					int X = static_cast<int>(uv_color[0] / uv_color[2]);
					int Y = static_cast<int>(uv_color[1] / uv_color[2]);
					//cout << "X:       " << X << "     Y:      " << Y << endl;
					if ((X >= 0 && X < 1920) && (Y >= 0 && Y < 1080))
					{
						//cout << "X:       " << X << "     Y:      " << Y << endl;
						result.data[i * 4] = i_rgb.data[4 * (Y * 1920 + X)];
						result.data[i * 4 + 1] = i_rgb.data[4 * (Y * 1920 + X) + 1];
						result.data[i * 4 + 2] = i_rgb.data[4 * (Y * 1920 + X) + 2];
						result.data[i * 4 + 3] = i_rgb.data[4 * (Y * 1920 + X) + 3];

					}
				}
				i++;
			}
		}
		// 显示
		//imshow("rgb", i_rgb);
		//if (waitKey(1) == VK_ESCAPE)
		//	break;
		imshow("depth", i_depth);
		if (waitKey(1) == VK_ESCAPE)
			break;
		imshow("mosic", result);
		if (waitKey(1) == VK_ESCAPE)
			break;
		/*imshow("ir", i_ir);
		if (waitKey(1) == VK_ESCAPE)
			break;*/

	}
}


int main() {

	KinectForDepthAndGDB();

}