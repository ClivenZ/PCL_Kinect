#include <iostream>
#include <iomanip>
#include <vector>

#include <GL/GLU.h>
#include <math.h>

using namespace xn;
using namespace std;

/* 点云数据格式 */
struct point_xyz
{
	float  X;
	float  Y;
	float  Z;
	float  R;
	float  G;
	float  B;

	point_xyz(XnPoint3D pos, XnRGB24Pixel color)
	{
		X = pos.X;
		Y = pos.Y;
		Z = pos.Z;
		R = (float)color.nRed / 255;
		G = (float)color.nGreen / 255;
		B = (float)color.nBlue / 255;
	}
};

/* 点云类 */
class point_cloud
{
private:
	Context            &context;
	DepthGenerator     &depth_generator;
	ImageGenerator     &image_generator;
	XnStatus           result_val;
	XnMapOutputMode    map_mode;

public:
	vector<point_xyz>  cloud_vector;        //存放一帧图像中的所有点云数据

	/* 构造函数\析构函数 */
	point_cloud(Context &contex, DepthGenerator &depthGenerator, ImageGenerator &imageGenerator)
		: context(contex), depth_generator(depthGenerator),
		image_generator(imageGenerator), result_val(XN_STATUS_OK) {}
	~point_cloud() { stop(); }

	/* 输出模式设置 */
	inline const XnMapOutputMode get_default_output_mode();
	void set_output_mode(const XnMapOutputMode &outputMode);

	/* 点云数据处理*/
	void        init();
	void        stop();
	void        updata();
	inline void clear();
	inline void print();
	inline int  size();

	/* 错误输出 */
	inline void printError();
};

inline const XnMapOutputMode point_cloud::get_default_output_mode()
{
	XnMapOutputMode outputMode = { XN_VGA_X_RES, XN_VGA_Y_RES, 30 };
	return outputMode;
}

void point_cloud::set_output_mode(const XnMapOutputMode &outputMode)
{
	map_mode.nFPS = outputMode.nFPS;
	map_mode.nXRes = outputMode.nXRes;
	map_mode.nYRes = outputMode.nYRes;
}

void point_cloud::init()
{
	result_val = context.Init();
	printError();

	/* 从文件中获取数据 */
	result_val = context.OpenFileRecording("tempRec.oni");
	printError();
	result_val = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth_generator);
	printError();
	result_val = context.FindExistingNode(XN_NODE_TYPE_IMAGE, image_generator);
	printError();

	/* 从设备中获取数据 */
	/*result_val = depth_generator.Create(context);
	printError();
	result_val = image_generator.Create(context);
	printError();
	// set output mode
	result_val = depth_generator.SetMapOutputMode(map_mode);
	printError();
	result_val = image_generator.SetMapOutputMode(map_mode);
	printError(); */

	/* 开始生成数据 */
	result_val = context.StartGeneratingAll();
	printError();
}

void point_cloud::stop()
{
	context.StopGeneratingAll();
	context.Shutdown();
}

inline void point_cloud::print()
{
	int i;
	cout.flags(ios::left);    //Left-aligned
	cout << "Point number: " << size() << endl;
	for (i = 0; i < size(); i++)
	{
		cout << "X:" << setw(10) << cloud_vector[i].X;
		cout << "Y:" << setw(10) << cloud_vector[i].Y;
		cout << "Z:" << setw(10) << cloud_vector[i].Z;
		cout << "R:" << setw(10) << cloud_vector[i].R;
		cout << "G:" << setw(10) << cloud_vector[i].G;
		cout << "B:" << setw(10) << cloud_vector[i].B << endl;
	}
}

inline int point_cloud::size()
{
	return cloud_vector.size();
}

inline void point_cloud::clear()
{
	cloud_vector.clear();
}

inline void point_cloud::printError()
{
	if (result_val != XN_STATUS_OK)
	{
		printf("Error: %s", xnGetStatusString(result_val));
		exit(-1);
	}
}

void point_cloud::updata()
{
	result_val = context.WaitNoneUpdateAll();
	/* 获得深度和图像数据 */
	const XnDepthPixel*  pDepth = depth_generator.GetDepthMap();
	const XnRGB24Pixel*  pImage = image_generator.GetRGB24ImageMap();

	/* 清零点云向量 */
	clear();

	/* 获得点的数量 */
	DepthMetaData   mDepthMD;
	depth_generator.GetMetaData(mDepthMD);
	unsigned int uPointNum = mDepthMD.FullXRes() * mDepthMD.FullYRes();

	/* 获得原始的点云数据 */
	XnPoint3D* pDepthPointSet = new XnPoint3D[uPointNum];
	unsigned int i, j, idxShift, idx;
	for (j = 0; j < mDepthMD.FullYRes(); ++j)
	{
		idxShift = j * mDepthMD.FullXRes();
		for (i = 0; i < mDepthMD.FullXRes(); ++i)
		{
			idx = idxShift + i;
			pDepthPointSet[idx].X = i;
			pDepthPointSet[idx].Y = j;
			pDepthPointSet[idx].Z = pDepth[idx];
		}
	}

	/* 将原始数据转换成真实的3D数据 */
	XnPoint3D* p3DPointSet = new XnPoint3D[uPointNum];
	depth_generator.ConvertProjectiveToRealWorld(uPointNum, pDepthPointSet, p3DPointSet);
	delete[] pDepthPointSet;

	/* 输出点云数据 */
	for (i = 0; i < uPointNum; ++i)
	{
		if (p3DPointSet[i].Z == 0)    // 跳过深度为0的数据
			continue;

		cloud_vector.push_back(point_xyz(p3DPointSet[i], pImage[i]));
	}
	delete[] p3DPointSet;

}

int main(void)
{
	XnStatus eResult = XN_STATUS_OK;
	int i = 0;

	// init
	Context mContext;
	eResult = mContext.Init();

	DepthGenerator mDepthGenerator;
	eResult = mDepthGenerator.Create(mContext);
	ImageGenerator mImageGenerator;
	eResult = mImageGenerator.Create(mContext);

	// set output mode
	XnMapOutputMode mapMode;
	mapMode.nXRes = XN_VGA_X_RES;
	mapMode.nYRes = XN_VGA_Y_RES;
	mapMode.nFPS = 30;
	eResult = mDepthGenerator.SetMapOutputMode(mapMode);
	eResult = mImageGenerator.SetMapOutputMode(mapMode);

	// start generating  
	eResult = mContext.StartGeneratingAll();

	// read data
	vector<SColorPoint3D> vPointCloud;
	while (!xnOSWasKeyboardHit())
	{
		eResult = mContext.WaitNoneUpdateAll();
		// get the depth map
		const XnDepthPixel*  pDepthMap = mDepthGenerator.GetDepthMap();

		// get the image map
		const XnRGB24Pixel*  pImageMap = mImageGenerator.GetRGB24ImageMap();

		// generate point cloud
		vPointCloud.clear();
		GeneratePointCloud(mDepthGenerator, pDepthMap, pImageMap, vPointCloud);

		// print point cloud
		cout.flags(ios::left);    //Left-aligned
		cout << "Point number: " << vPointCloud.size() << endl;
		for (i = 0; i < vPointCloud.size(); i++)
		{
			cout << setw(10) << i;
			cout << "X:" << setw(10) << vPointCloud[i].X;
			cout << "Y:" << setw(10) << vPointCloud[i].Y;
			cout << "Z:" << setw(10) << vPointCloud[i].Z;
			cout << "R:" << setw(10) << vPointCloud[i].R;
			cout << "G:" << setw(10) << vPointCloud[i].G;
			cout << "B:" << setw(10) << vPointCloud[i].B << endl;
		}
	}

	//stop
	mContext.StopGeneratingAll();
	mContext.Shutdown();

	return 0;
}
