#include "stdafx.h"
#include "Kinect.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "pcl/io/openni2/openni.h"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>

int fun1() {
	cv::Mat color = cv::imread("1.jpg");
	cv::Mat depth = cv::imread("2.jpg");

	int rowNumber = color.rows;
	int colNumber = color.cols;

	cloud_a.height = rowNumber;
	cloud_a.width = colNumber;
	cloud_a.points.resize(cloud_a.width * cloud_a.height);

	for (unsigned int u = 0; u < rowNumber; ++u)
	{
		for (unsigned int v = 0; v < colNumber; ++v)
		{
			unsigned int num = u * colNumber + v;
			double Xw = 0, Yw = 0, Zw = 0;

			Zw = ((double)depth.at<uchar>(u, v)) / 255.0 * 10001.0;
			Xw = (u - u0) * Zw / fx;
			Yw = (v - v0) * Zw / fy;

			cloud_a.points[num].b = color.at<cv::Vec3b>(u, v)[0];
			cloud_a.points[num].g = color.at<cv::Vec3b>(u, v)[1];
			cloud_a.points[num].r = color.at<cv::Vec3b>(u, v)[2];

			cloud_a.points[num].x = Xw;
			cloud_a.points[num].y = Yw;
			cloud_a.points[num].z = Zw;
		}
	}

	*cloud = cloud_a;
	pcl::io::savePCDFile("colorImage.pcd", *cloud);
	/*pcl::visualization::CloudViewer viewer("Cloud Viewer");

	viewer.showCloud(cloud);

	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	while (!viewer.wasStopped())
	{
		user_data = 9;
	}*/
	printf("点云生成完毕\n");
	return 0;
}