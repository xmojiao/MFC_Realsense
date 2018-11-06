#pragma once
#include <vector>
#include <algorithm>
#include <iostream>
#include <string>
#include <math.h>
#include <fstream>
#include <sstream>              // Stringstreams

//control 
#include "globalVar.h"

////Realsense
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

//////
#include <opencv2/opencv.hpp>

/////////PCL
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件  
#include <pcl/io/ply_io.h> //PCL的PCD格式文件的输入输出头文件  
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件 
#include <pcl/visualization/cloud_viewer.h>  //点云显示库函数
#include <pcl/visualization/pcl_visualizer.h>//点云显示库函数


#include <pcl/sample_consensus/sac_model_plane.h>//Ransac
#include <pcl/sample_consensus/ransac.h>

#include <pcl/segmentation/sac_segmentation.h>//Segmentation
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>


#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>


using namespace std;
using namespace cv;

#define PI 3.141592654

#define MIN_NUM_FIT 20
#define _A_ 0
#define _B_ 1
#define _C_ 2
#define _D_ 3   //平面方程

#define _ROWS_ 720
#define _COLS_ 1280 //图像尺寸
#define CLOUD_POINT_NUM 921600  //点云总点数

#define ROWS_OF_SUB_BLOCK 72
#define COLS_OF_SUB_BLOCK 128   //分的块包含的点数

#define BLOCK_ROWS_NUM 5   //分块的行数
#define BLOCK_COLS_NUM 5    //分块的列数 
#define MIN_X 0
#define MAX_X 1
#define MIN_Y 2
#define MAX_Y 3
#define MIN_Z 4
#define MAX_Z 5


//单位m
class AlgorithmDetect
{
public:

	AlgorithmDetect();
	~AlgorithmDetect();

public:
	rs2::pointcloud pc;
	rs2::points points;
	rs2::pipeline pipe;

	float mBarrierHeightThresh;
	float mSideThresh;
	float mDistanceThresh;
	int mDownSampleRate;
	bool showBox;
	bool showPlane;

	void detect(vector<Point3f> vp3f);
	void startDetect();
	void startDetectFromFile();

	void sparseBlock(vector<Point3f> vp3fSrc, vector<vector<Point3f> > &vvp3fRes);

	void getBoundingBox(vector<Point3f> vp3fBlock, vector<float> &vpfVertex);
	float getNearestDis(vector<Point3f> vp3fBlock);

private:
	bool fitPlane(vector<Point3f> points, Vec4f &planePara);//点云拟合平面	
	void calDisPtToLine(float A, float B, float C, vector<Point2f> vP2fSrc, vector<float> vfRes);
	void calDisPtToPlane(Vec4f planePara, vector<Point3f> vP2fSrc, vector<float> &vfRes);
	float calDisPtToPlane(Vec4f planePara, Point3f pt3f);
	void calPtToPlaneFoot(Vec4f planeParam3D, vector<Point3f> pt3fFiltered, vector<Point3f> &vP3fFoot);
	void getPlaneCenter(vector<Point3f> points, Vec4f &planeParam3D, Point3f &center);	
public:
	float norm2(Point3f pt3f);
	Point3f cross(Point3f p1, Point3f p2);
	float dot(Point3f p1, Point3f p2);
	float dot(Point2f p1, Point2f p2);
	Point3f RMultiP3f(cv::Mat R1to2, Point3f p3f);
	float distance(Point2f p1, Point2f p2);
	float distance(Point3f p1, Point3f p2);
	void cvFitPlane(const CvMat* points, float* plane);//fitPlane调用该函数
};

