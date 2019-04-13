#include <time.h>
#include "kinect.h"

#include <iostream>

#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  

#include <gl/glew.h>
#include <gl/GLU.h>
#include <GLFW/glfw3.h>

//pcl io��
#include <pcl/io/io.h>
//pcl io�� ��д.pcd �ļ�
#include <pcl/io/pcd_io.h>
//pcl ������
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <thread>
#include <mutex>
#include <queue>
#define GL_DISPLAY
#define SAVE_IMG


using namespace cv;
using namespace std;
using namespace pcl;

// ����ڲ�(����Ҫ���д����޸ģ�������)
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;



#pragma comment(lib, "opengl32.lib")
double yaw, pitch, lastX, lastY; int ml;
static void on_mouse_button(GLFWwindow * win, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT) ml = action == GLFW_PRESS;
}
static double clamp(double val, double lo, double hi) { return val < lo ? lo : val > hi ? hi : val; }
static void on_cursor_pos(GLFWwindow * win, double x, double y)
{
	if (ml)
	{
		yaw = clamp(yaw - (x - lastX), -120, 120);
		pitch = clamp(pitch + (y - lastY), -80, 80);
	}
	lastX = x;
	lastY = y;
}


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

int main()
{
#ifdef GL_DISPLAY
	/// [gl display]
	// Open a GLFW window to display our output
	int a = glfwInit();
	GLFWwindow * win = glfwCreateWindow(1024, 768, "gl_win", nullptr, nullptr);
	glfwSetCursorPosCallback(win, on_cursor_pos);
	glfwSetMouseButtonCallback(win, on_mouse_button);
	glfwMakeContextCurrent(win);
#endif
#pragma region ��ʼ���豸
	// ��ȡKinect�豸
	IKinectSensor* m_pKinectSensor;
	ICoordinateMapper*      m_pCoordinateMapper;
	CameraIntrinsics* m_pCameraIntrinsics = new CameraIntrinsics();
	HRESULT hr;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	IMultiSourceFrameReader* m_pMultiFrameReader;
	IBodyFrameSource* m_pBodyFrameSource;
//	IBodyFrameReader* m_pBodyFrameReader;
	if (m_pKinectSensor)
	{
		hr = m_pKinectSensor->Open();
		//Sleep(2000);
		if (SUCCEEDED(hr))
		{
			m_pKinectSensor->get_BodyFrameSource(&m_pBodyFrameSource);
			// ��ȡ������Դ����ȡ��  
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Color |
				FrameSourceTypes::FrameSourceTypes_Infrared |
				FrameSourceTypes::FrameSourceTypes_Depth,
				&m_pMultiFrameReader);
		}
	}
	if (SUCCEEDED(hr))
	{
		hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
	}
	if (!m_pKinectSensor || FAILED(hr))
	{
		return E_FAIL;
	}
	// ��������֡������
	IDepthFrameReference* m_pDepthFrameReference = NULL;
	IColorFrameReference* m_pColorFrameReference = NULL;
	IDepthFrame* m_pDepthFrame = NULL;
	IColorFrame* m_pColorFrame = NULL;
	// �ĸ���ͼƬ��ʽ
	Mat i_rgb(1080, 1920, CV_8UC4);      //ע�⣺�������Ϊ4ͨ����ͼ��Kinect������ֻ����Bgra��ʽ����
	Mat i_depth_raw(424, 512, CV_16UC1);
	Mat i_depthToRgb(424, 512, CV_8UC4);

	
	IMultiSourceFrame* m_pMultiFrame = NULL;

	DepthSpacePoint* m_pDepthCoordinates = NULL;
	ColorSpacePoint* m_pColorCoordinates = NULL;
	CameraSpacePoint* m_pCameraCoordinates = NULL;
	
#pragma endregion

	m_pColorCoordinates = new ColorSpacePoint[512 * 424];
	m_pCameraCoordinates = new CameraSpacePoint[512 * 424];
	UINT16 *depthData = new UINT16[424 * 512];
	BYTE *bgraData = new BYTE[1080 * 1920 * 4];

#pragma region ��ȡ����֡�߳�
	while (1)
	{
			
		//mutex1.lock();
		HRESULT hr = 0;
		// ��ȡ�µ�һ����Դ����֡
		hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);	// ���m_pMultiFrame��Ϊ�գ����Ὣ���ÿ�
		if (m_pMultiFrame == NULL)
		{
			//mutex1.unlock();
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
			hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, bgraData, ColorImageFormat::ColorImageFormat_Bgra);
		if (SUCCEEDED(hr))
			hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);
		// ���depth������
		UINT nDepthBufferSize = 424 * 512;
		if (SUCCEEDED(hr))
		{
			hr = m_pDepthFrame->CopyFrameDataToArray(nDepthBufferSize, reinterpret_cast<UINT16*>(depthData));
			hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_depth_raw.data));
		}
		// ���ͼӳ�䵽��ɫͼ��
		if (SUCCEEDED(hr))
		{
			HRESULT hr = m_pCoordinateMapper->MapDepthFrameToColorSpace(512 * 424, depthData, 512 * 424, m_pColorCoordinates);		// ע������Ĳ�ɫ����Ҫд��424*512������Ϊʲô������������Ϊ�˴�����һ������colorSpacePoints�Ĵ�С
		}

#ifdef GL_DISPLAY
		glfwPollEvents();
		// Set up a perspective transform in a space that we can rotate by clicking and dragging the mouse
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(60, (float)1280 / 960, 0.01f, 20.0f);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);
		glTranslatef(0, 0, +0.5f);
		glRotated(pitch, 1, 0, 0);
		glRotated(yaw, 0, 1, 0);
		glTranslatef(0, 0, -0.5f);

		// We will render our depth data as a set of points in 3D space
		glPointSize(2);
		glEnable(GL_DEPTH_TEST);
		glBegin(GL_POINTS);
#endif
		// ���ͼ��ռ�ӳ�䵽����ռ�
		if (SUCCEEDED(hr))
		{
			HRESULT hr = m_pCoordinateMapper->MapDepthFrameToCameraSpace(512 * 424, depthData, 512 * 424, m_pCameraCoordinates);
		}
		if (SUCCEEDED(hr))
		{
			for (int i = 0; i < 512 * 424; i++)
			{
				CameraSpacePoint p = m_pCameraCoordinates[i];
				ColorSpacePoint p_bgra = m_pColorCoordinates[i];
				int colorX = static_cast<int>(p_bgra.X + 0.5f);
				int colorY = static_cast<int>(p_bgra.Y + 0.5f);
				if ((colorX >= 0 && colorX < 1920) && (colorY >= 0 && colorY < 1080))
				{
					float bgra = ((float *)bgraData)[(colorY * 1920 + colorX)];
					//cout << bgra << endl;
					if (bgra != 0&&p.X >-0.2&&p.X<0.5 && p.Y>-1 && p.Y<1.0f && p.Z <1.5f && p.Z > 0.5f)
					{
						
#ifdef GL_DISPLAY
						GLubyte *rgb = new GLubyte();
						rgb[2] = bgraData[(colorY * 1920 + colorX) * 4];
						rgb[1] = bgraData[(colorY * 1920 + colorX) * 4+1];
						rgb[0] = bgraData[(colorY * 1920 + colorX) * 4+2];
						i_depthToRgb.data[i * 4] = i_rgb.data[(colorY * 1920 + colorX) * 4];
						i_depthToRgb.data[i * 4 + 1] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 1];
						i_depthToRgb.data[i * 4 + 2] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 2];
						i_depthToRgb.data[i * 4 + 3] = i_rgb.data[(colorY * 1920 + colorX) * 4 + 3];
						
						// ��ʾ��
						glColor3ubv(rgb);
						glVertex3f(-p.X, -p.Y, p.Z);
#endif
					}
				}
			}
		}

#pragma region ����ͼƬ
#ifdef SAVE_IMG
		imwrite("depth.jpg", i_depth_raw);
		imwrite("color.jpg", i_rgb);
		imwrite("depth2rgb.jpg", i_depthToRgb);
		cout << "����ɹ�" << endl;
#endif
#pragma endregion
#ifdef GL_DISPLAY
		glEnd();
		glfwSwapBuffers(win);
#endif
		////���� PCD �ļ�
		//cv::Mat color = cv::imread("depth.jpg");
		//cv::Mat depth = cv::imread("color.jpg");

		//int rowNumber = color.rows;
		//int colNumber = color.cols;

		//pcl::PointCloud<pcl::PointXYZRGBA> cloud;

		//cloud.height = rowNumber;
		//cloud.width = colNumber;
		//cloud.points.resize(cloud.width * cloud.height);

		//for (unsigned int u = 0; u < rowNumber; ++u)
		//{
		//	for (unsigned int v = 0; v < colNumber; ++v)
		//	{
		//		unsigned int num = u * colNumber + v;
		//		double Xw = 0, Yw = 0, Zw = 0;

		//		Zw = ((double)depth.at<uchar>(u, v)) / 255.0 * 10001.0;
		//		Xw = (u - camera_cx) * Zw / camera_fx;
		//		Yw = (v - camera_cy) * Zw / camera_fy;

		//		cloud.points[num].b = color.at<cv::Vec3b>(u, v)[0];
		//		cloud.points[num].g = color.at<cv::Vec3b>(u, v)[1];
		//		cloud.points[num].r = color.at<cv::Vec3b>(u, v)[2];

		//		cloud.points[num].x = Xw;
		//		cloud.points[num].y = Yw;
		//		cloud.points[num].z = Zw;
		//	}
		//}

		///**cloud = cloud;*/
		//pcl::io::savePCDFile("MyPCL.pcd", cloud);
		///*pcl::visualization::CloudViewer viewer("Cloud Viewer");

		//viewer.showCloud(cloud);

		//viewer.runOnVisualizationThreadOnce(viewerOneOff);

		//while (!viewer.wasStopped())
		//{
		//	user_data = 9;
		//}*/
		//printf("�����������\n");

		// �ͷ���Դ
		SafeRelease(m_pColorFrame);
		SafeRelease(m_pDepthFrame);
		SafeRelease(m_pColorFrameReference);
		SafeRelease(m_pDepthFrameReference);
		SafeRelease(m_pMultiFrame);

		//mutex1.unlock();
	}
#pragma endregion
	// �رմ��ڣ��豸
	cv::destroyAllWindows();
	SafeRelease(m_pCoordinateMapper);
	m_pKinectSensor->Close();
	std::system("pause");
	return 0;
}
