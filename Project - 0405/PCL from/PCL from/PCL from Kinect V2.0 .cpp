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

//��ȫ�ͷ�ָ��
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}
//�����ʼ��
const double camera_factor = 1000;
const double camera_cx = 263.73696;
const double camera_cy = 201.72450;
const double camera_fx = 379.40726;
const double camera_fy = 378.54472;

//Kinect ��ȡ Depth �� GDB ���� Mat
int KinectForDepthAndGDB() {
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

	IMultiSourceFrameReader* m_pMultiFrameReader = NULL;
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
	IDepthFrameReference* m_pDepthFrameReference = NULL;
	IColorFrameReference* m_pColorFrameReference = NULL;
	IInfraredFrameReference* m_pInfraredFrameReference = NULL;
	IInfraredFrame* m_pInfraredFrame = NULL;
	IDepthFrame* m_pDepthFrame = NULL;
	IColorFrame* m_pColorFrame = NULL;
// ����ͼƬ��ʽ
	Mat i_rgb(1080, 1920, CV_8UC4);      //ע�⣺�������Ϊ4ͨ����ͼ��Kinect������ֻ����Bgra��ʽ����
	Mat i_depth(424, 512, CV_16UC1);
	Mat i_ir(424, 512, CV_16UC1);
	Mat result(424, 512, CV_8UC4);

	UINT16 *depthData = new UINT16[424 * 512];
	IMultiSourceFrame* m_pMultiFrame = nullptr;

	while (true) {
		// ��ȡ�µ�һ����Դ����֡
		hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);
		if (FAILED(hr) || !m_pMultiFrame)
		{
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
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_InfraredFrameReference(&m_pInfraredFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pInfraredFrameReference->AcquireFrame(&m_pInfraredFrame);
		
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

		// infrared������ͼƬ��
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
					// ͶӰ����ɫͼ�ϵ�����
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
		// ��ʾ
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