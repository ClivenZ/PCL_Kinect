#include <time.h>
#include "kinect.h"
#include <iostream>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>  

#include <gl/glew.h>
#include <gl/GLU.h>
#include <GLFW/glfw3.h>

//#include <math.h>

//#include <glut.h>
#include <thread>
#include <mutex>
//#include "PracticalSocket.h"
#include <queue>
#define GL_DISPLAY
//#define SAVE_IMG
using namespace cv;
using namespace std;

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
	//Mat i_depth_raw(424, 512, CV_16UC1);

	
	IMultiSourceFrame* m_pMultiFrame = NULL;

	DepthSpacePoint*        m_pDepthCoordinates = NULL;
	ColorSpacePoint*        m_pColorCoordinates = NULL;
	CameraSpacePoint*        m_pCameraCoordinates = NULL;
	
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

		// ���depth������
		UINT nDepthBufferSize = 424 * 512;
		if (SUCCEEDED(hr))
		{
			hr = m_pDepthFrame->CopyFrameDataToArray(nDepthBufferSize, reinterpret_cast<UINT16*>(depthData));
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
		imwrite("depth.png", i_depth_raw);
		imwrite("color.jpg", i_rgb);
		imwrite("depth2rgb.jpg", i_depthToRgb);
#endif
#pragma endregion
#ifdef GL_DISPLAY
		glEnd();
		glfwSwapBuffers(win);
#endif

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
