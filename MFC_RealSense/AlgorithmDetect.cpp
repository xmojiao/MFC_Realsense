//viewer.addLine(pcl::PointXYZ(coef[0], coef[1], -(coef[3]+ coef[0]* coef[0]+ coef[1] * coef[1])/ coef[2]), pcl::PointXYZ(coef[0], coef[1], coef[2]));
#include "stdafx.h"
#include "AlgorithmDetect.h"


AlgorithmDetect::AlgorithmDetect()
{
	mDistanceThresh = 2000.0f / 1000.0f;
	mSideThresh = 600.0f / 1000.0f;
	mBarrierHeightThresh = 80.0f / 1000.0f;
	mDownSampleRate = 0;
	showBox = false;
}


AlgorithmDetect::~AlgorithmDetect()
{
}

void AlgorithmDetect::startDetect()
{
	pipe.start();
	unsigned long long NUM = 0;
	pcl::visualization::PCLVisualizer viewer("cloud");
	viewer.setBackgroundColor(1.0f,1.0f,1.0f, 0);
	while (algoCtrl)
	{
		auto frames = pipe.wait_for_frames();//获取相机图像（rgb&depth）
											 //auto color = frames.get_color_frame();//获取rgb
		auto depth = frames.get_depth_frame();//获取depth深度图
		

		int height = depth.get_height();//720
		int width = depth.get_width();//1280
		points = pc.calculate(depth);
		auto vertices = points.get_vertices();              // get vertices   float x, y, z;
															//auto tex_coords = points.get_texture_coordinates(); // and texture coordinates  float u, v;
		cout << "Frame ID: " << NUM++ << endl;
		cout << "the size of source point cloud is : " << points.size() << endl;
		if (NUM == 20) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFile(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）  
			cloudFile->width = height * width;
			cloudFile->height = 1;
			cloudFile->is_dense = false;
			cloudFile->resize(cloudFile->width * cloudFile->height);
			for (size_t i = 0; i < cloudFile->size(); i++)
			{
				cloudFile->points[i].x = vertices[i].x;
				cloudFile->points[i].y = vertices[i].y;
				cloudFile->points[i].z = vertices[i].z;
			}
			pcl::io::savePCDFile("origin_data.pcd", *cloudFile);
		}

		vector<Point3f> vp3fFilter;
		for (int i = 0; i < points.size() - mDownSampleRate; i = i + mDownSampleRate)
		{
			if (vertices[i].x == 0 || vertices[i].y == 0 || vertices[i].z == 0)
				continue;

			if (norm2(Point3f(vertices[i].x, vertices[i].y, vertices[i].z)) > mDistanceThresh ||
				abs(vertices[i].x) > mSideThresh)
				continue;

			vp3fFilter.push_back(cv::Point3f(vertices[i].x, vertices[i].y, vertices[i].z));
		}
		cout << "the size of filtered point cloud is : " << vp3fFilter.size() << endl;
		if (vp3fFilter.size() < 100)
			continue;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）  
		cloud->width = vp3fFilter.size();
		cloud->height = 1;
		cloud->is_dense = false;
		cloud->resize(cloud->width * cloud->height);
		for (size_t i = 0; i < vp3fFilter.size(); i++)
		{
			cloud->points[i].x = vp3fFilter[i].x;
			cloud->points[i].y = vp3fFilter[i].y;
			cloud->points[i].z = vp3fFilter[i].z;
		}
		//平滑
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMls(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）  
		//pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
		//mls.setInputCloud(cloud);
		//pcl::search::KdTree<pcl::PointXYZ>::Ptr treeMls(new pcl::search::KdTree<pcl::PointXYZ>);
		//mls.setSearchMethod(treeMls);
		//mls.setPolynomialFit(true);
		//mls.setPolynomialOrder(4);
		//mls.setComputeNormals(false);
		//mls.setSearchRadius(8);
		//mls.setUpsamplingMethod(mls.SAMPLE_LOCAL_PLANE);
		//mls.setUpsamplingRadius(10);
		//mls.setUpsamplingStepSize(4.5);
		//mls.process(*cloudMls);

		//过滤
		/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilterd(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(cloudMls);
		sor.setMeanK(50);
		sor.setStddevMulThresh(0.5);
		sor.filter(*cloudFilterd);
		std::cout << "Cloud after filter:" << cloud->size() - cloudFilterd->size() << endl;*/

		/*if (NUM == 5)
		{
			std::string filename("text.pcb");
			pcl::PCDWriter writer;
			writer.write(filename, *cloud);
		}*/
	//	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	//	viewer.showCloud(cloud);
	//	while (!viewer.wasStopped())
	//	{
	//	}
	//}

		//使用ransac找到平面外和平面内的点
	    std:vector<int> inliers;
		pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
		ransac.setDistanceThreshold(mBarrierHeightThresh);
		ransac.computeModel();
		ransac.getInliers(inliers);

		Eigen::VectorXf coef = Eigen::VectorXf::Zero(4, 1);
		ransac.getModelCoefficients(coef);
		cout << coef[0] << " " << coef[1] << " " << coef[2] << " " << coef[3] << endl;

		vector<int> outLiers;
		size_t step = 0;
		for (size_t i = 0; i < cloud->size(); i++)
		{
			if (step >= inliers.size())
			{
				outLiers.push_back(i);
				//break;
				continue;
			}
				
			if (i != inliers[step])
			{
				outLiers.push_back(i);
			}
			else
			{
				step++;
			}
			//cout << "i " << i << endl;
			//cout << "step " << step << endl;
			//cout << "vp3fFilter.size() " << vp3fFilter.size() << endl;
		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr inlierCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr outLiersCloud(new pcl::PointCloud<pcl::PointXYZ>);
		inlierCloud->resize(inliers.size());
		outLiersCloud->resize(outLiers.size());
		pcl::copyPointCloud(*cloud, inliers, *inlierCloud);
		pcl::copyPointCloud(*cloud, outLiers, *outLiersCloud);

		cout <<"the num of inlier points is:  "<<inliers.size()<< endl;
		cout << "the num of outLiers points is:  " << outLiers.size() << endl;
		
		if (outLiers.size() == 0)
			continue;

		//聚类
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(outLiersCloud);
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(0.02); // 2cm
		ec.setMinClusterSize(100);
		ec.setMaxClusterSize(25000);
		ec.setSearchMethod(tree);
		ec.setInputCloud(outLiersCloud);
		ec.extract(cluster_indices);
		cout << "the number of cluster type is: " << cluster_indices.size() << endl;

		//统计字子点云像素点个数
		vector<vector<Point3f> > vvp3fSubSet;
		for (size_t i = 0; i < cluster_indices.size(); i++)
		{
			vector<Point3f> temp;
			for (size_t j = 0; j < cluster_indices[i].indices.size(); j++)
			{
				int index = cluster_indices[i].indices[j];
				float x = outLiersCloud->points[index].x;
				float y = outLiersCloud->points[index].y;
				float z = outLiersCloud->points[index].z;
				temp.push_back(Point3f(x, y, z));
			}
			vvp3fSubSet.push_back(temp);
			cout << "sub point cloud size is: " << temp.size() << endl;
		}
		//for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		//{
		//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		//	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		//		cloud_cluster->points.push_back(outLiersCloud->points[*pit]); //*
		//	cloud_cluster->width = cloud_cluster->points.size();
		//	cloud_cluster->height = 1;
		//	cloud_cluster->is_dense = true;
		//	std::cout << "PointCloud representing the Cluster: " 
		//		<< cloud_cluster->points.size() << " data points." << std::endl;
		//}

		//通过pcl的viewer显示
		viewer.removeAllPointClouds();
		viewer.removeAllShapes();

		string strCloudID = "Cloud";
		string strBoxID = "Box";
		vector<float> vfDis;
		for (size_t i = 0; i < cluster_indices.size(); i++)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*outLiersCloud, cluster_indices[i], *temp);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(temp, abs((51 * i)%255), abs((255-51 * i) % 255), abs((51 * i) % 255));
			strCloudID = strCloudID + to_string(i);
			strBoxID = strBoxID + to_string(i);
			viewer.addPointCloud(temp, source_cloud_color_handler, strCloudID);
			if (showBox)
			{
				vector<float> vbox;
				getBoundingBox(vvp3fSubSet[i], vbox);
				viewer.addCube(vbox[MIN_X], vbox[MAX_X], vbox[MIN_Y], vbox[MAX_Y],
					vbox[MIN_Z], vbox[MAX_Z], 255, 255, 255, strBoxID);
			}
			float D = getNearestDis(vvp3fSubSet[i]);
			cout << "the distance between robot and barrier " << i << " is " << D << endl;
			vfDis.push_back(D);
		}
		//viewer.removeAllPointClouds();
		//viewer.removeAllShapes();

		if (showPlane)
		{
			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
			coefficients->values.push_back(coef[0]);
			coefficients->values.push_back(coef[1]);
			coefficients->values.push_back(coef[2]);
			coefficients->values.push_back(coef[3]);
			viewer.addPlane(*coefficients, "cloud");
		}
		

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler1(inlierCloud, 255, 0, 0);
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler2(outLiersCloud, 0, 255, 0);
		
		viewer.addPointCloud(inlierCloud, source_cloud_color_handler1, "cloud1");
		//viewer.addPointCloud(outLiersCloud, source_cloud_color_handler2, "cloud");
		
		viewer.spinOnce(1, false);
	}
	pipe.stop();
	//algoCtrl = false;
}

void AlgorithmDetect::startDetectFromFile()
{
	pcl::PointCloud<pcl::PointXYZ> cloudFile;
	pcl::io::loadPCDFile("origin_data.pcd", cloudFile);
	pcl::visualization::PCLVisualizer viewer("cloud");
	viewer.setBackgroundColor(1.0f, 1.0f, 1.0f, 0);
	while (algoCtrl)
	{
		vector<pcl::PointXYZ> vp3fFilter;
		for (int i = 0; i < cloudFile.size() - mDownSampleRate; i = i + mDownSampleRate)
		{
			if (cloudFile[i].x == 0 || cloudFile[i].y == 0 || cloudFile[i].z == 0)
				continue;

			if (norm2(Point3f(cloudFile[i].x, cloudFile[i].y, cloudFile[i].z)) > mDistanceThresh ||
				abs(cloudFile[i].x) > mSideThresh)
				continue;

			vp3fFilter.push_back(cloudFile[i]);
		}
		cout << "the size of filtered point cloud is : " << vp3fFilter.size() << endl;
		if (vp3fFilter.size() < 100)
			continue;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）  
		cloud->width = vp3fFilter.size();
		cloud->height = 1;
		cloud->is_dense = false;
		cloud->resize(cloud->width * cloud->height);
		for (size_t i = 0; i < vp3fFilter.size(); i++)
		{
			cloud->points[i].x = vp3fFilter[i].x;
			cloud->points[i].y = vp3fFilter[i].y;
			cloud->points[i].z = vp3fFilter[i].z;
		}
		//平滑
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudMls(new pcl::PointCloud<pcl::PointXYZ>); // 创建点云（指针）  
		//pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
		//mls.setInputCloud(cloud);
		//pcl::search::KdTree<pcl::PointXYZ>::Ptr treeMls(new pcl::search::KdTree<pcl::PointXYZ>);
		//mls.setSearchMethod(treeMls);
		//mls.setPolynomialFit(true);
		//mls.setPolynomialOrder(4);
		//mls.setComputeNormals(false);
		//mls.setSearchRadius(8);
		//mls.setUpsamplingMethod(mls.SAMPLE_LOCAL_PLANE);
		//mls.setUpsamplingRadius(10);
		//mls.setUpsamplingStepSize(4.5);
		//mls.process(*cloudMls);

		//过滤
		/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilterd(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(cloudMls);
		sor.setMeanK(50);
		sor.setStddevMulThresh(0.5);
		sor.filter(*cloudFilterd);
		std::cout << "Cloud after filter:" << cloud->size() - cloudFilterd->size() << endl;*/

		/*if (NUM == 5)
		{
			std::string filename("text.pcb");
			pcl::PCDWriter writer;
			writer.write(filename, *cloud);
		}*/
		//	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
		//	viewer.showCloud(cloud);
		//	while (!viewer.wasStopped())
		//	{
		//	}
		//}

		//使用ransac找到平面外和平面内的点
		std:vector<int> inliers;
		pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));
		pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
		ransac.setDistanceThreshold(mBarrierHeightThresh);
		ransac.computeModel();
		ransac.getInliers(inliers);

		Eigen::VectorXf coef = Eigen::VectorXf::Zero(4, 1);
		ransac.getModelCoefficients(coef);
		cout << coef[0] << " " << coef[1] << " " << coef[2] << " " << coef[3] << endl;

		vector<int> outLiers;
		size_t step = 0;
		for (size_t i = 0; i < cloud->size(); i++)
		{
			if (step >= inliers.size())
			{
				outLiers.push_back(i);
				continue;
			}
			if (i != inliers[step])
			{
				outLiers.push_back(i);
			}
			else
			{
				step++;
			}
			//cout << "i " << i << endl;
			//cout << "step " << step << endl;
			//cout << "vp3fFilter.size() " << vp3fFilter.size() << endl;
		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr inlierCloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr outLiersCloud(new pcl::PointCloud<pcl::PointXYZ>);
		inlierCloud->resize(inliers.size());
		outLiersCloud->resize(outLiers.size());
		pcl::copyPointCloud(*cloud, inliers, *inlierCloud);
		pcl::copyPointCloud(*cloud, outLiers, *outLiersCloud);

		cout << "the num of inlier points is:  " << inliers.size() << endl;
		cout << "the num of outLiers points is:  " << outLiers.size() << endl;

		if (outLiers.size() == 0)
			continue;

		//聚类
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(outLiersCloud);
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(0.02); // 2cm
		ec.setMinClusterSize(100);
		ec.setMaxClusterSize(25000);
		ec.setSearchMethod(tree);
		ec.setInputCloud(outLiersCloud);
		ec.extract(cluster_indices);
		cout << "the number of cluster type is: " << cluster_indices.size() << endl;

		//统计字子点云像素点个数
		vector<vector<Point3f> > vvp3fSubSet;
		for (size_t i = 0; i < cluster_indices.size(); i++)
		{
			vector<Point3f> temp;
			for (size_t j = 0; j < cluster_indices[i].indices.size(); j++)
			{
				int index = cluster_indices[i].indices[j];
				float x = outLiersCloud->points[index].x;
				float y = outLiersCloud->points[index].y;
				float z = outLiersCloud->points[index].z;
				temp.push_back(Point3f(x, y, z));
			}
			vvp3fSubSet.push_back(temp);
			//cout << "sub point cloud size is: " << temp.size() << endl;
		}
		//for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		//{
		//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		//	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		//		cloud_cluster->points.push_back(outLiersCloud->points[*pit]); //*
		//	cloud_cluster->width = cloud_cluster->points.size();
		//	cloud_cluster->height = 1;
		//	cloud_cluster->is_dense = true;
		//	std::cout << "PointCloud representing the Cluster: " 
		//		<< cloud_cluster->points.size() << " data points." << std::endl;
		//}

		//通过pcl的viewer显示
		viewer.removeAllPointClouds();
		viewer.removeAllShapes();

		string strCloudID = "Cloud";
		string strBoxID = "Box";
		vector<float> vfDis;
		for (size_t i = 0; i < cluster_indices.size(); i++)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*outLiersCloud, cluster_indices[i], *temp);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(temp, abs((51 * i) % 255), abs((255 - 51 * i) % 255), abs((51 * i) % 255));
			strCloudID = strCloudID + to_string(i);
			strBoxID = strBoxID + to_string(i);
			viewer.addPointCloud(temp, source_cloud_color_handler, strCloudID);
			if (showBox)
			{
				vector<float> vbox;
				getBoundingBox(vvp3fSubSet[i], vbox);
				viewer.addCube(vbox[MIN_X], vbox[MAX_X], vbox[MIN_Y], vbox[MAX_Y],
					vbox[MIN_Z], vbox[MAX_Z], 255, 255, 255, strBoxID);
			}
			float D = getNearestDis(vvp3fSubSet[i]);
			//cout << "the distance between robot and barrier " << i << " is " << D << endl;
			vfDis.push_back(D);
		}
		//viewer.removeAllPointClouds();
		//viewer.removeAllShapes();

		if (showPlane)
		{
			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
			coefficients->values.push_back(coef[0]);
			coefficients->values.push_back(coef[1]);
			coefficients->values.push_back(coef[2]);
			coefficients->values.push_back(coef[3]);
			viewer.addPlane(*coefficients, "cloud");
		}


		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler1(inlierCloud, 255, 0, 0);
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler2(outLiersCloud, 0, 255, 0);
	
		viewer.addPointCloud(inlierCloud, source_cloud_color_handler1, "cloud1");
		//viewer.addPointCloud(outLiersCloud, source_cloud_color_handler2, "cloud");
		//viewer.add coef[0], coef[1], -(coef[3]+ coef[0]* coef[0]+ coef[1] * coef[1])/ coef[2])
		viewer.addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(1,0,0),"line1");
		viewer.addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0,1,0), "line2");
		viewer.addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0,0,1), "line3");
		float a = coef[0];float b = coef[1];float c = coef[2];float d = coef[3];
		float x0 = -a * d / (a * a + b * b + c * c);
		float y0 = -b * d / (a * a + b * b + c * c);
		float z0 = -c * d / (a * a + b * b + c * c);
		//Z轴： (0,0,0)-> (-ad/(a*a+b*b+c*c),-bd/(a*a+b*b+c*c),-cd/(a*a+b*b+c*c))
		//-coef[0] * coef[3] / (coef[0] * coef[0] + coef[1] * coef[1] + coef[2] * coef[2]), -coef[1] * coef[3] / (coef[0] * coef[0] + coef[1] * coef[1] + coef[2] * coef[2]), -coef[2] * coef[3] / (coef[0] * coef[0] + coef[1] * coef[1] + coef[2] * coef[2])
		viewer.addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(x0,y0,z0), "line4");
		//X轴： (-ad/(a*a+b*b+c*c),-bd/(a*a+b*b+c*c),-cd/(a*a+b*b+c*c)) -> (-ad/(a*a+b*b+c*c)+1,-bd/(a*a+b*b+c*c),-cd/(a*a+b*b+c*c))
		viewer.addLine(pcl::PointXYZ(x0,y0,z0), pcl::PointXYZ(x0+1, y0, z0),"line5");
		//Y轴： (-ad/(a*a+b*b+c*c),-bd/(a*a+b*b+c*c),-cd/(a*a+b*b+c*c)) -> (-ad/(a*a+b*b+c*c),-bd/(a*a+b*b+c*c)-1,-cd/(a*a+b*b+c*c)+1)
		viewer.addLine(pcl::PointXYZ(x0,y0,z0), pcl::PointXYZ(x0, y0-1, z0+1), "line6");
		Eigen::MatrixXf matrix_pc(4, 4);
		matrix_pc << x0, x0 + 1, x0, 0,
			y0, y0, y0 - 1, 0,
			z0, z0, z0 + 1, 0,
			1, 1, 1, 1;
		Eigen::MatrixXf matrix_pw(4, 4);
		matrix_pw << 0, 1, 0, 0,
			0, 0, sqrt(2), 0,
			0, 0, 0, sqrt(x0*x0 + y0 * y0 + z0 * z0),
			1, 1, 1, 1;
		Eigen::MatrixXf T_wc = matrix_pw * matrix_pc.inverse();
		cout << "T_wc" << T_wc << endl;
		cout << "T_cw" << T_wc.inverse()<< endl;
		Eigen::Vector4f pc1;
		pc1 << x0, y0, z0, 1;
		Eigen::Vector4f pc2;
		pc2 << x0 + 1, y0, z0, 1;
		Eigen::Vector4f pc3;
		pc3 << x0, y0 - 1, z0 + 1, 1;
		Eigen::Matrix <float, 4, 1> result = T_wc.cast<float>() * pc1;
		cout << "pw1" << result << endl;
		cout << "pw2"<<T_wc.cast<float>() * pc2 <<endl;
		cout << "pw3" << T_wc.cast<float>() * pc3 << endl;
	
		viewer.spinOnce(1, false);
	}
	//algoCtrl = false;
}

//------------------------------此处为分割线，以下函数未使用-----------------------
//---------------------------------------------------------------------------------
void AlgorithmDetect::getBoundingBox(vector<Point3f> vp3fBlock, vector<float> &vpfVertex)
{
	vpfVertex.clear();
	if (vp3fBlock.size() == 0)
		return;
	float minX = vp3fBlock[0].x, maxX = vp3fBlock[0].x;
	float minY = vp3fBlock[0].y, maxY = vp3fBlock[0].y;
	float minZ = vp3fBlock[0].z, maxZ = vp3fBlock[0].z;
	for (size_t i = 0; i!= vp3fBlock.size(); i++)
	{
		if (vp3fBlock[i].x < minX)
			minX = vp3fBlock[i].x;
		if (vp3fBlock[i].x > maxX)
			maxX = vp3fBlock[i].x;
		if (vp3fBlock[i].y < minY)
			minY = vp3fBlock[i].y;
		if (vp3fBlock[i].y > maxY)
			maxY = vp3fBlock[i].y;
		if (vp3fBlock[i].z < minZ)
			minZ = vp3fBlock[i].z;
		if (vp3fBlock[i].z > maxZ)
			maxZ = vp3fBlock[i].z;
	}
	vpfVertex.push_back(minX);
	vpfVertex.push_back(maxX);
	vpfVertex.push_back(minY);
	vpfVertex.push_back(maxY);
	vpfVertex.push_back(minZ);
	vpfVertex.push_back(maxZ);
}

float AlgorithmDetect::getNearestDis(vector<Point3f> vp3fBlock)
{
	vector<float> vfNorm;
	for (size_t i = 0; i != vp3fBlock.size(); i++)
	{
		vfNorm.push_back(norm2(vp3fBlock[i]));
	}
	vector<float>::iterator itMin = min_element(vfNorm.begin(), vfNorm.end());
	return (*itMin);
}

void AlgorithmDetect::detect(vector<Point3f> vp3f)
{

	vector<vector<Point3f> > vvp3fRes;
	sparseBlock(vp3f, vvp3fRes);

	vector<Vec4f> vPara;

	for (size_t i = 0; i < vvp3fRes.size(); i++)
	{
		Vec4f planePara;
		fitPlane(vvp3fRes[i], planePara);
		vPara.push_back(planePara);
	}

	fstream fs("planPara.txt", ios::out);
	if (!fs.is_open())
	{
		cout << "open txt file failed.." << endl;
	}
	else
	{
		for (size_t i = 0; i < vPara.size(); i++)
		{
			fs << vPara[i][_A_] << " " << vPara[i][_B_] << " " << vPara[i][_C_]<<" "<< vPara[i][_D_] << endl;
		}
		fs.close();
		cout << "save planPara.." << endl;
	}

	fstream fs1("fliterdata.txt", ios::out);
	if (!fs1.is_open())
	{
		cout << "open txt file failed.." << endl;
	}
	else
	{
		for (size_t i = 0; i < vvp3fRes.size(); i++)
		{
			for (size_t j = 0; j < vvp3fRes[i].size(); j++)
			{
				fs1 << vvp3fRes[i][j].x << " " << vvp3fRes[i][j].y << " " << vvp3fRes[i][j].z << endl;
			}
		}		
		fs1.close();
		cout << "save fliterdata.." << endl;
	}
	//break;
}
void AlgorithmDetect::sparseBlock(vector<Point3f> vp3fSrc, vector<vector<Point3f> > &vvp3fRes)
{
	for (size_t i = 0; i < vvp3fRes.size(); i++)
	{
		vvp3fRes[i].clear();
	}
	vvp3fRes.clear();

	if (vp3fSrc.size() != CLOUD_POINT_NUM)
	{
		cout << "err: point cloud is imcompleted.." << endl;
		return;
	}

	vector<Point3f> temp;

	for (size_t r = 0; r < BLOCK_ROWS_NUM; r++)
	{
		for (size_t c = 0; c < BLOCK_COLS_NUM; c++)
		{
			vector<Point3f> temp;

			for (size_t i = 0; i < ROWS_OF_SUB_BLOCK; i++)
			{
				for (size_t j = 0; j < COLS_OF_SUB_BLOCK; j++)
				{
					int h = i+r*ROWS_OF_SUB_BLOCK;
					int w = j+c*COLS_OF_SUB_BLOCK;				
					Point3f pt3 = vp3fSrc[h*_COLS_ + w];
					if (pt3.x==0 || pt3.y == 0 || pt3.z == 0)
						continue;
					if (norm2(pt3) > mDistanceThresh || abs(pt3.x) > mSideThresh)
						continue;
					temp.push_back( vp3fSrc[h*_COLS_ + w] );//Point3f pt3 = vp3fSrc[h*_COLS_ + w]
				}
			}
			vvp3fRes.push_back(temp);
		}
	}
}

bool AlgorithmDetect::fitPlane(vector<Point3f> vpt3f, Vec4f &planePara)
{
	if (vpt3f.size()<MIN_NUM_FIT)
	{
		for (int k = 0; k < 4; k++)
			planePara[k] = 0;
		return false;
	}
	CvMat*points_mat = cvCreateMat(vpt3f.size(), 3, CV_32FC1);//定义用来存储需要拟合点的矩阵
	for (int i = 0; i < vpt3f.size(); ++i)
	{
		points_mat->data.fl[i * 3 + 0] = vpt3f[i].x;//矩阵的值进行初始化 X的坐标值  
		points_mat->data.fl[i * 3 + 1] = vpt3f[i].y;//Y的坐标值  
		points_mat->data.fl[i * 3 + 2] = vpt3f[i].z; //Z的坐标值

	}
	float planeParam[4] = { 0 };//定义用来储存平面参数的数组   
	cvFitPlane(points_mat, planeParam);//调用方程
	for (int k = 0; k < 4; k++)
	{
		planePara[k] = planeParam[k];
	}
	return true;
}

void AlgorithmDetect::cvFitPlane(const CvMat* points, float* plane)
{
	int nrows = points->rows;
	int ncols = points->cols;
	int type = points->type;
	CvMat* centroid = cvCreateMat(1, ncols, type);
	cvSet(centroid, cvScalar(0));
	for (int c = 0; c < ncols; c++) {
		for (int r = 0; r < nrows; r++)
		{
			centroid->data.fl[c] += points->data.fl[ncols*r + c];
		}
		centroid->data.fl[c] /= nrows;
	}
	// Subtract geometric centroid from each point.  
	CvMat* points2 = cvCreateMat(nrows, ncols, type);
	for (int r = 0; r < nrows; r++)
		for (int c = 0; c < ncols; c++)
			points2->data.fl[ncols*r + c] = points->data.fl[ncols*r + c] - centroid->data.fl[c];
	// Evaluate SVD of covariance matrix.  
	CvMat* A = cvCreateMat(ncols, ncols, type);
	CvMat* W = cvCreateMat(ncols, ncols, type);
	CvMat* V = cvCreateMat(ncols, ncols, type);
	cvGEMM(points2, points, 1, NULL, 0, A, CV_GEMM_A_T);
	cvSVD(A, W, NULL, V, CV_SVD_V_T);
	// Assign plane coefficients by singular vector corresponding to smallest singular value.  
	plane[ncols] = 0;
	for (int c = 0; c < ncols; c++) {
		plane[c] = V->data.fl[ncols*(ncols - 1) + c];
		plane[ncols] += plane[c] * centroid->data.fl[c];
	}
	// Release allocated resources.  
	cvReleaseMat(&centroid);
	cvReleaseMat(&points2);
	cvReleaseMat(&A);
	cvReleaseMat(&W);
	cvReleaseMat(&V);
}

void AlgorithmDetect::getPlaneCenter(vector<Point3f> points, Vec4f &planeParam3D, Point3f &center)
{

	Vec4f v4fPlanePara0, v4fPlanePara1;
	fitPlane(points, v4fPlanePara0);
	v4fPlanePara0[3] = -v4fPlanePara0[3];
	vector<Point3f> pt3fFiltered;
	for (size_t i = 0; i != points.size(); i++)
	{
		double tempDis = calDisPtToPlane(v4fPlanePara0, points[i]);
		if (tempDis < 10)
		{
			pt3fFiltered.push_back(points[i]);
		}
	}
	fitPlane(pt3fFiltered, v4fPlanePara1);
	v4fPlanePara1[3] = -v4fPlanePara1[3];
	vector<Point3f> vP3fFoot;
	calPtToPlaneFoot(v4fPlanePara1, pt3fFiltered, vP3fFoot);

	/*double scale = 2;
	int padding = 10;
	vector<Point2f> vp2fXY;
	vector<int> vX;
	vector<int> vY;
	for (size_t i = 0; i != vP3fFoot.size(); i++)
	{
		Point2f p2fTemp;
		p2fTemp.x = scale*vP3fFoot[i].x;
		p2fTemp.y = scale*vP3fFoot[i].y;
		vp2fXY.push_back(p2fTemp);
		vX.push_back(p2fTemp.x);
		vY.push_back(p2fTemp.y);
	}

	vector<int>::iterator itXmin = min_element(vX.begin(), vX.end());
	vector<int>::iterator itXmax = max_element(vX.begin(), vX.end());
	vector<int>::iterator itYmin = min_element(vY.begin(), vY.end());
	vector<int>::iterator itYmax = max_element(vY.begin(), vY.end());
	int xmin = *itXmin;
	int xmax = *itXmax;
	int ymin = *itYmin;
	int ymax = *itYmax;
	Mat Img(ymax - ymin + 2 * padding, xmax - xmin + 2 * padding, CV_8U, Scalar(0));
	for (size_t i = 0; i != vX.size(); i++)
	{
		int tempRow = vY[i] - ymin + padding;
		int tempCol = vX[i] - xmin + padding;
		Img.at<char>(tempRow, tempCol) = 255;
	}
	int kernelSize = 5;
	Mat maskMat = getStructuringElement(MORPH_ELLIPSE, cv::Size(2 * kernelSize + 1, 2 * kernelSize + 1), Point(-1, -1));
	Mat imgMorph;
	morphologyEx(Img, imgMorph, MORPH_CLOSE, maskMat);

	vector<vector<Point> > contours;
	findContours(imgMorph, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	vector<int> vPNum;
	for (int i = 0; i != contours.size(); i++)
	{
		vPNum.push_back(contours[i].size());
	}
	vector<int>::iterator itNumMax = max_element(vPNum.begin(), vPNum.end());
	vector<Point> vPForFit = contours[itNumMax - vPNum.begin()];
	if (vPForFit.size() < 6)
		return;
	Mat pointsf;
	Mat(vPForFit).convertTo(pointsf, CV_32F);
	RotatedRect box = fitEllipse(pointsf);
	if (MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height) * 8)
		return;

	Point2f centerTemp = box.center;

	centerTemp.x = (centerTemp.x + xmin - padding) / scale;
	centerTemp.y = (centerTemp.y + ymin - padding) / scale;

	float resZ = -(v4fPlanePara1[0] * centerTemp.x + v4fPlanePara1[1] * centerTemp.y + v4fPlanePara1[3]) / v4fPlanePara1[2];

	planeParam3D = v4fPlanePara1;
	center.x = centerTemp.x;
	center.y = centerTemp.y;
	center.z = resZ;*/
}

void AlgorithmDetect::calDisPtToPlane(Vec4f planePara, vector<Point3f> vP2fSrc, vector<float> &vfRes)
{
	vfRes.clear();
	if (vP2fSrc.size() == 0)
		return;
	Point3f pp(planePara[_A_], planePara[_B_], planePara[_C_]);
	float norm2pt = sqrt(dot(pp, pp));
	for (size_t i = 0 ;i != vP2fSrc.size(); i++)
	{
		float dot1 = abs(dot(pp, vP2fSrc[i]) + planePara[_D_]);
		vfRes.push_back(dot1 / norm2pt);
	}
}

float AlgorithmDetect::calDisPtToPlane(Vec4f planePara, Point3f pt3f)
{
	Point3f pp(planePara[_A_], planePara[_B_], planePara[_C_]);
	float dot1 =abs( dot(pp, pt3f) + planePara[_D_]);
	float norm2pt = sqrt(dot(pp, pp));
	return dot1 / norm2pt;	
}

void AlgorithmDetect::calPtToPlaneFoot(Vec4f planeParam3D, vector<Point3f> pt3fFiltered, vector<Point3f> &vP3fFoot)
{
	vP3fFoot.clear();
	double A = planeParam3D[0];
	double B = planeParam3D[1];
	double C = planeParam3D[2];
	double D = planeParam3D[3];

	Mat Lx(3, 3, CV_64F);
	Mat H(4, 3, CV_64F);
	Mat Bx(4, 4, CV_64F);

	Lx.at<double>(0, 0) = 0; Lx.at<double>(0, 1) = -C; Lx.at<double>(0, 2) = B;
	Lx.at<double>(1, 0) = C; Lx.at<double>(1, 1) = 0; Lx.at<double>(1, 2) = -A;
	Lx.at<double>(2, 0) = -B; Lx.at<double>(2, 1) = A; Lx.at<double>(2, 2) = 0;

	H.at<double>(0, 0) = 0;  H.at<double>(0, 1) = -C;  H.at<double>(0, 2) = B;
	H.at<double>(1, 0) = C;  H.at<double>(1, 1) = 0;  H.at<double>(1, 2) = -A;
	H.at<double>(2, 0) = -B;  H.at<double>(2, 1) = A;  H.at<double>(2, 2) = 0;
	H.at<double>(3, 0) = A;  H.at<double>(3, 1) = B;  H.at<double>(3, 2) = C;

	Bx.at<double>(0, 0) = 0;  Bx.at<double>(0, 1) = -C;  Bx.at<double>(0, 2) = B; Bx.at<double>(0, 3) = 0;
	Bx.at<double>(1, 0) = C;  Bx.at<double>(1, 1) = 0;  Bx.at<double>(1, 2) = -A; Bx.at<double>(1, 3) = 0;
	Bx.at<double>(2, 0) = -B;  Bx.at<double>(2, 1) = A;  Bx.at<double>(2, 2) = 0; Bx.at<double>(2, 3) = 0;
	Bx.at<double>(3, 0) = 0;  Bx.at<double>(3, 1) = 0;  Bx.at<double>(3, 2) = 0; Bx.at<double>(3, 3) = -D;

	Mat HTrans, HHinv, HHinvHtrans;
	transpose(H, HTrans);
	Mat tempM = HTrans*H;
	HHinv = tempM.inv();
	HHinvHtrans = HHinv*HTrans;
	Mat Ax(3, 4, CV_64F);
	Ax = HHinvHtrans*Bx;

	for (size_t i = 0; i != pt3fFiltered.size(); i++)
	{
		Point3f pt3f;
		pt3f.x = Ax.at<double>(0, 0)*pt3fFiltered[i].x + Ax.at<double>(0, 1)*pt3fFiltered[i].y + Ax.at<double>(0, 2)*pt3fFiltered[i].z + Ax.at<double>(0, 3);
		pt3f.y = Ax.at<double>(1, 0)*pt3fFiltered[i].x + Ax.at<double>(1, 1)*pt3fFiltered[i].y + Ax.at<double>(1, 2)*pt3fFiltered[i].z + Ax.at<double>(1, 3);
		pt3f.z = Ax.at<double>(2, 0)*pt3fFiltered[i].x + Ax.at<double>(2, 1)*pt3fFiltered[i].y + Ax.at<double>(2, 2)*pt3fFiltered[i].z + Ax.at<double>(2, 3);
		vP3fFoot.push_back(pt3f);
	}
}

void AlgorithmDetect::calDisPtToLine(float A, float B, float C, vector<Point2f> vP2fSrc, vector<float> vfRes)
{
	vfRes.clear();
	float normLize = sqrt(pow(A, 2) + pow(B, 2));
	for (int i = 0; i != vP2fSrc.size(); i++)
	{
		float disTemp = (A*vP2fSrc[i].x + B*vP2fSrc[i].y + C) / normLize;
		disTemp = abs(disTemp);
		vfRes.push_back(disTemp);
	}
}

float AlgorithmDetect::norm2(Point3f pt3f)
{
	return sqrt(dot(pt3f, pt3f));
}
Point3f AlgorithmDetect::cross(Point3f p1, Point3f p2)
{
	Point3f p3fRes;
	p3fRes.z = p1.x * p2.y - p1.y * p2.x;
	p3fRes.x = (p1.y * p2.z - p1.z * p2.y) / p3fRes.z;
	p3fRes.y = (p1.z * p2.x - p1.x * p2.z) / p3fRes.z;
	p3fRes.z = 1;
	return p3fRes;
}
float AlgorithmDetect::dot(Point3f p1, Point3f p2)
{
	return p1.x*p2.x + p1.y*p2.y + p1.z*p2.z;
}
float AlgorithmDetect::dot(Point2f p1, Point2f p2)
{
	return p1.x*p2.x + p1.y*p2.y;
}
Point3f AlgorithmDetect::RMultiP3f(cv::Mat R1to2, Point3f p3f)
{
	double a11, a12, a13, a21, a22, a23, a31, a32, a33;
	a11 = R1to2.at<double>(0, 0); a12 = R1to2.at<double>(0, 1); a13 = R1to2.at<double>(0, 2);
	a21 = R1to2.at<double>(1, 0); a22 = R1to2.at<double>(1, 1); a23 = R1to2.at<double>(1, 2);
	a31 = R1to2.at<double>(2, 0); a32 = R1to2.at<double>(2, 1); a33 = R1to2.at<double>(2, 2);
	float x = a11*p3f.x + a12*p3f.y + a13*p3f.z;
	float y = a21*p3f.x + a22*p3f.y + a23*p3f.z;
	float z = a31*p3f.x + a32*p3f.y + a33*p3f.z;
	Point3f res(x, y, z);
	return res;
}
float AlgorithmDetect::distance(Point2f p1, Point2f p2)
{
	return (sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y)));
}
float AlgorithmDetect::distance(Point3f p1, Point3f p2)
{
	return (sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y)+ (p1.z - p2.z)*(p1.z - p2.z)));
}
