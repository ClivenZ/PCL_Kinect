// KinectFirst.cpp : �������̨Ӧ�ó������ڵ㡣
//

#include "stdafx.h"
#include <kinect.h>
#include <iostream>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  

using namespace cv;
using namespace std;

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

int _tmain(int argc, _TCHAR* argv[])
{
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
	Mat i_depth(424, 512, CV_8UC1);
	Mat i_Infrared(424, 512, CV_8UC1);
	Mat i_ir(424, 512, CV_16UC1);
	Mat i_depthToRgb(424, 512, CV_8UC4);

	UINT16 *depthData = new UINT16[424 * 512];
	UINT16 *InfraredData = new UINT16[424 * 512];
	CameraSpacePoint* m_pCameraCoordinates = new CameraSpacePoint[512 * 424];
	ColorSpacePoint* m_pColorCoordinates = new ColorSpacePoint[512 * 424];
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
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_InfraredFrameReference(&m_pInfraredFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pInfraredFrameReference->AcquireFrame(&m_pInfraredFrame);

		// color������ͼƬ��
		UINT nColorBufferSize = 1920 * 1080 * 4;
		if (SUCCEEDED(hr))
			hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);



		if (SUCCEEDED(hr))
		{
			hr = m_pInfraredFrame->CopyFrameDataToArray(424 * 512, InfraredData);
			for (int i = 0; i < 512 * 424; i++)
			{
				// 0-255���ͼ��Ϊ����ʾ���ԣ�ֻȡ������ݵĵ�8λ
				BYTE intensity = static_cast<BYTE>(InfraredData[i] % 256);
				reinterpret_cast<BYTE*>(i_Infrared.data)[i] = intensity;
			}
		}

		// depth������ͼƬ��
		if (SUCCEEDED(hr))
		{
			hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, depthData);
			for (int i = 0; i < 512 * 424; i++)
			{
				// 0-255���ͼ��Ϊ����ʾ���ԣ�ֻȡ������ݵĵ�8λ
				BYTE intensity = static_cast<BYTE>(depthData[i] % 256);
				reinterpret_cast<BYTE*>(i_depth.data)[i] = intensity;
			}
			ICoordinateMapper*      m_pCoordinateMapper = NULL;
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);

			HRESULT hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(512 * 424, depthData, 512 * 424, m_pColorCoordinates);


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
			imshow("rgb2depth", i_depthToRgb);
			if (waitKey(1) == VK_ESCAPE)
				break;
			// ��ʾ
			/*imshow("rgb", i_rgb);
			if (waitKey(1) == VK_ESCAPE)
			break;*/
			imshow("depth", i_depth);
			if (waitKey(1) == VK_ESCAPE)
				break;
			imshow("Infrared", i_Infrared);
			if (waitKey(1) == VK_ESCAPE)
				break;

			if (SUCCEEDED(hr))
			{
				HRESULT hr = m_pCoordinateMapper->MapDepthFrameToCameraSpace(512 * 424, depthData, 512 * 424, m_pCameraCoordinates);
			}
			if (SUCCEEDED(hr))
			{
				if (waitKey(100) == VK_SPACE)
				{
					SYSTEMTIME st;
					GetLocalTime(&st);
					char output_file[32];
					char output_RGB[32];
					char output_depth[32];
					char output_ir[32];
					sprintf_s(output_file, "%4d-%2d-%2d-%2d-%2d-%2d.txt", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
					sprintf_s(output_RGB, "%4d-%2d-%2d-%2d-%2d-%2d-rgb.png", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
					sprintf_s(output_depth, "%4d-%2d-%2d-%2d-%2d-%2d-depth.png", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
					sprintf_s(output_ir, "%4d-%2d-%2d-%2d-%2d-%2d-ir.png", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond);
					imwrite(output_RGB, i_rgb);
					imwrite(output_depth, i_depth);
					imwrite(output_ir, i_Infrared);
					FILE *file = fopen(output_file, "w");
					for (int i = 0; i < 512 * 424; i++)
					{
						CameraSpacePoint p = m_pCameraCoordinates[i];
						if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity() && p.Z != -std::numeric_limits<float>::infinity())
						{
							float cameraX = static_cast<float>(p.X);
							float cameraY = static_cast<float>(p.Y);
							float cameraZ = static_cast<float>(p.Z);
							if (file)
							{
								float b = i_depthToRgb.data[i * 4 + 0];
								float g = i_depthToRgb.data[i * 4 + 1];
								float r = i_depthToRgb.data[i * 4 + 2];
								fprintf(file, "%.4f %.4f %.4f %.4f %.4f %.4f\n", cameraX, cameraY, cameraZ, r, g, b);
							}
							//cout << "x: " << cameraX << "y: " << cameraY << "z: " << cameraZ << endl;
							//GLubyte *rgb = new GLubyte();
							//rgb[2] = i_depthToRgb.data[i * 4 + 0];
							//rgb[1] = i_depthToRgb.data[i * 4 + 1];
							//rgb[0] = i_depthToRgb.data[i * 4 + 2];
							//// ��ʾ��
							//glColor3ubv(rgb);
							//glVertex3f(cameraX, -cameraY, cameraZ);
						}
					}
					fclose(file);
					cout << "�ļ�����ɹ�" << endl;
				}

			}

		}



		// �ͷ���Դ
		SafeRelease(m_pColorFrame);
		SafeRelease(m_pDepthFrame);
		SafeRelease(m_pInfraredFrame);
		SafeRelease(m_pColorFrameReference);
		SafeRelease(m_pDepthFrameReference);
		SafeRelease(m_pInfraredFrameReference);
		SafeRelease(m_pMultiFrame);
	}
	// �رմ��ڣ��豸
	cv::destroyAllWindows();
	m_pKinectSensor->Close();
	std::system("pause");

	return 0;
}