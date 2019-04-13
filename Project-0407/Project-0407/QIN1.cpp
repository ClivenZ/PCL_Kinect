#include <Kinect.h>
#include <opencv2\opencv.hpp>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/core/mat.inl.hpp>

using namespace cv;
using namespace std;


IKinectSensor* pSensor = nullptr;
ICoordinateMapper *pMapper = nullptr;
const int iDWidth = 512, iDHeight = 424;//深度图尺寸
const int iCWidth = 1920, iCHeight = 1080;//彩色图尺寸
CameraSpacePoint depth2xyz[iDWidth*iDHeight];
ColorSpacePoint depth2rgb[iCWidth*iCHeight];



void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(1.0, 1.0, 1.0);//设置背景颜色 
}


//启动Kinect
bool initKinect()
{
	if (FAILED(GetDefaultKinectSensor(&pSensor))) return false;
	if (pSensor)
	{
		pSensor->get_CoordinateMapper(&pMapper);
		pSensor->Open();
		cout << "已打开相机" << endl;
		return true;
	}
	else return false;
}
//获取深度帧
Mat DepthData()
{
	IDepthFrameSource* pFrameSource = nullptr;
	pSensor->get_DepthFrameSource(&pFrameSource);
	IDepthFrameReader* pFrameReader = nullptr;
	pFrameSource->OpenReader(&pFrameReader);
	IDepthFrame* pFrame = nullptr;
	Mat mDepthImg(iDHeight, iDWidth, CV_16UC1);
	while (true)
	{
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{

			pFrame->CopyFrameDataToArray(iDWidth * iDHeight, reinterpret_cast<UINT16*>(mDepthImg.data));
			cout << "已获取深度帧" << endl;
			pFrame->Release();
			return mDepthImg;
			break;
		}
	}
}
//获取彩色帧
Mat RGBData()
{
	IColorFrameSource* pFrameSource = nullptr;
	pSensor->get_ColorFrameSource(&pFrameSource);
	IColorFrameReader* pFrameReader = nullptr;
	pFrameSource->OpenReader(&pFrameReader);
	IColorFrame* pFrame = nullptr;
	Mat mColorImg(iCHeight, iCWidth, CV_8UC4);
	while (true)
	{
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{

			pFrame->CopyConvertedFrameDataToArray(iCWidth * iCHeight * 4, mColorImg.data, ColorImageFormat_Bgra);
			cout << "已获取彩色帧" << endl;
			pFrame->Release();
			return mColorImg;
			break;
		}
	}
}

void getPointCloudFromImage(Mat depthImage, Mat rgbImage, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out)
{

	pMapper->MapDepthFrameToCameraSpace(iDWidth*iDHeight, reinterpret_cast<UINT16*>(depthImage.data), iDWidth*iDHeight, depth2xyz);
	pMapper->MapDepthFrameToColorSpace(512 * 424, reinterpret_cast<UINT16*>(depthImage.data), 512 * 424, depth2rgb);

	cloud_out.height = 1;
	cloud_out.is_dense = 1;

	for (size_t i = 0; i < iDWidth; i++)
	{
		for (size_t j = 0; j < iDHeight; j++)
		{
			pcl::PointXYZRGB pointTemp;
			if (depth2xyz[i + j * iDWidth].Z > 0.5)
			{

				pointTemp.x = depth2xyz[i + j * iDWidth].X;
				pointTemp.y = depth2xyz[i + j * iDWidth].Y;
				pointTemp.z = depth2xyz[i + j * iDWidth].Z;
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

int PCLforPCD(void) {
	Mat depthImage = cv::imread("Depth.jpg");
	Mat rgbImage = cv::imread("GDB.jpg");
	

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
	getPointCloudFromImage(depthImage, rgbImage, *cloud_out);
	pcl::visualization::CloudViewer viewerG("Cloud Viewe");
	pcl::io::savePCDFileASCII("输出点云.pcd", *cloud_out);
	//pcl::PLYWriter writer;
	//writer.write("输出点云.ply", *cloud_out);
	viewerG.runOnVisualizationThreadOnce(viewerOneOff);
	viewerG.showCloud(cloud_out);
	while (true) if (cv::waitKey(30) == VK_ESCAPE) 
		break;
	return 1;
}

int main()
{
	initKinect();
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.runOnVisualizationThreadOnce(viewerOneOff);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	Mat mColorImg;
	Mat mDepthImg;
	while (cv::waitKey(30) != VK_ESCAPE)
	{
		mColorImg = RGBData();
		mDepthImg = DepthData();
		//imshow("RGB", mColorImg);
	
		pMapper->MapDepthFrameToColorSpace(iDHeight*iDWidth, reinterpret_cast<UINT16*>(mDepthImg.data), iDHeight*iDWidth, depth2rgb);//深度图到颜色的映射
		pMapper->MapDepthFrameToCameraSpace(iDHeight*iDWidth, reinterpret_cast<UINT16*>(mDepthImg.data), iDHeight*iDWidth, depth2xyz);//深度图到相机三维空间的映射
		//for (int i = 0; i < iDWidth*iDHeight; i++)
		//{
		//	cout << i << ":  " << "X=" << depth2rgb[i].X << ";  Y=" << depth2rgb[i].Y<<endl;
		//}

		float maxX = depth2xyz[0].X, maxY = depth2xyz[0].Y, maxZ = depth2xyz[0].Z;
		float minX = depth2xyz[0].X, minY = depth2xyz[0].Y, minZ = depth2xyz[0].Z;
		for (size_t i = 0; i < iDWidth; i++)
		{
			for (size_t j = 0; j < iDHeight; j++)
			{
				pcl::PointXYZRGBA pointTemp;
				if (depth2xyz[i + j * iDWidth].Z > 0.5&&depth2rgb[i + j * iDWidth].X < 1920 && depth2rgb[i + j * iDWidth].X>0 && depth2rgb[i + j * iDWidth].Y < 1080 && depth2rgb[i + j * iDWidth].Y>0)
				{
					pointTemp.x = -depth2xyz[i + j * iDWidth].X;
					if (depth2xyz[i + j * iDWidth].X > maxX) maxX = -depth2xyz[i + j * iDWidth].X;
					if (depth2xyz[i + j * iDWidth].X < minX) minX = -depth2xyz[i + j * iDWidth].X;
					pointTemp.y = depth2xyz[i + j * iDWidth].Y;
					if (depth2xyz[i + j * iDWidth].Y > maxY) maxY = depth2xyz[i + j * iDWidth].Y;
					if (depth2xyz[i + j * iDWidth].Y < minY) minY = depth2xyz[i + j * iDWidth].Y;
					pointTemp.z = depth2xyz[i + j * iDWidth].Z;
					if (depth2xyz[i + j * iDWidth].Z != 0.0)
					{
						if (depth2xyz[i + j * iDWidth].Z > maxZ) maxZ = depth2xyz[i + j * iDWidth].Z;
						if (depth2xyz[i + j * iDWidth].Z < minZ) minZ = depth2xyz[i + j * iDWidth].Z;
					}
					pointTemp.b = mColorImg.at<cv::Vec4b>(depth2rgb[i + j * iDWidth].Y, depth2rgb[i + j * iDWidth].X)[0];
					pointTemp.g = mColorImg.at<cv::Vec4b>(depth2rgb[i + j * iDWidth].Y, depth2rgb[i + j * iDWidth].X)[1];
					pointTemp.r = mColorImg.at<cv::Vec4b>(depth2rgb[i + j * iDWidth].Y, depth2rgb[i + j * iDWidth].X)[2];
					pointTemp.a = mColorImg.at<cv::Vec4b>(depth2rgb[i + j * iDWidth].Y, depth2rgb[i + j * iDWidth].X)[3];
					cloud->push_back(pointTemp);
				}

			}

		}
		cv::imwrite("Depth.jpg", mDepthImg);
		cv::imwrite("RGB.jpg", mColorImg);
		cout << "已保存图像" << endl;
		Mat depthImage = cv::imread("Depth.jpg");
		Mat rgbImage = cv::imread("GDB.jpg");


		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
		getPointCloudFromImage(depthImage, rgbImage, *cloud_out);
		pcl::visualization::CloudViewer viewerG("Cloud Viewe");
		pcl::io::savePCDFileASCII("输出点云.pcd", *cloud_out);
		//pcl::PLYWriter writer;
		//writer.write("输出点云.ply", *cloud_out);
		viewerG.runOnVisualizationThreadOnce(viewerOneOff);
		viewerG.showCloud(cloud_out);
		while (true) if (cv::waitKey(30) == VK_ESCAPE)
			break;
		
		
		
		//PCLforPCD();
		viewer.showCloud(cloud);
		mColorImg.release();
		mDepthImg.release();
		cloud->clear();
		waitKey(10);
	}
	
	return 0;
}
