#include <pcl/visualization/cloud_viewer.h>
#include <iostream>//��׼C++���е�������������ͷ�ļ���
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>//pcd ��д����ص�ͷ�ļ���
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ���
#include<fstream>  
#include <string>  
#include <vector> 
#include <vtkObject.h>

using namespace std;

int main()
{
	typedef struct tagPOINT_3D
	{
		double x;  //mm world coordinate x  
		double y;  //mm world coordinate y  
		double z;  //mm world coordinate z  
		double r;
	}POINT_WORLD;
	vtkObject::GlobalWarningDisplayOff();
	/////����txt����  
	int number_Txt;
	FILE *fp_txt;
	tagPOINT_3D TxtPoint;
	vector<tagPOINT_3D> m_vTxtPoints;
	fp_txt = fopen("za.txt", "r");
	if (fp_txt)
	{
		while (fscanf(fp_txt, "%lf %lf %lf", &TxtPoint.x, &TxtPoint.y, &TxtPoint.z) != EOF)
		{
			m_vTxtPoints.push_back(TxtPoint);
		}
	}
	else
		cout << "txt���ݼ���ʧ�ܣ�" << endl;
	number_Txt = m_vTxtPoints.size();
	//pcl::PointCloud<pcl::PointXYZ> cloud;
	//����ʹ�á�PointXYZ������Ϊ�Һ�����ĵ�����Ϣ�ǰ�������ά���꣬ͬʱ���е�����Ϣ������rgb��ɫ��Ϣ�Ļ��߻��а���rgba��ɫ��ǿ����Ϣ��
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Fill in the cloud data  
	cloud->width = number_Txt;
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = m_vTxtPoints[i].x;
		cloud->points[i].y = m_vTxtPoints[i].y;
		cloud->points[i].z = m_vTxtPoints[i].z;
	}
	pcl::io::savePCDFileASCII("txt2pcd_bunny1.pcd", *cloud);
	std::cerr << "Saved " << cloud->points.size() << " data points to txt2pcd.pcd." << std::endl;

	//for (size_t i = 0; i < cloud.points.size(); ++i)
	//  std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

	//PCL Visualizer
	// Viewer  
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.addPointCloud(cloud);
	viewer.setBackgroundColor(0, 0, 0);
	viewer.spin();
	system("pause");
	return 0;

}
