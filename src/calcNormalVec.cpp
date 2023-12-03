#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp> 
//#include <pcl/common/io.h> don't use this!! this is for another concatenateField function
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h> // PLY related 
#include <iostream>
#include <string>
#include <pcl/filters/filter.h> // remove관련 함수의 헤더
#include <pcl/features/normal_3d_omp.h>



using namespace std;

int main(){
string input_path="/root/catkin_ws/Dataset/ply_data/";
string output_path="/root/catkin_ws/Dataset/ADOP_dataset/";
string filename;
int countfile=0;
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_ply(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr accum_ply(new pcl::PointCloud<pcl::PointXYZRGB>);

//for calculating & combining point cloud
	while(1){	
		// Load Point Cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		filename = input_path + to_string(countfile) + ".ply";
		//filename=input_path+"combined.ply";
		if(pcl::io::loadPLYFile(filename, *cloud)==-1){
			cout << "Do not exist any file to load." << endl;
			break;
		}
		else
			cout << "success to load " << filename << endl;

		

		// Create the normal estimation class, and pass the input dataset to it
		cout << "Start to calculate the normal vector..." << endl;
		pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
		ne.setInputCloud (cloud);
		ne.setNumberOfThreads(15);
		ne.useSensorOriginAsViewPoint();
		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
		ne.setSearchMethod (tree);
		
		cout << "success to create an kdtree" << endl;

		// Output datasets
		// pcl::PointNormal - A point structure representing Eudlidean xyz coordinates, together with normal coordinates and the surface curvature estimate
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);


		// Compute Normal
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		ne.setRadiusSearch (0.05);   // Use all neighbors in a sphere of radius [0.03: 3cm --> 0.3: 30cm]
		cout << "success to set radius search" << endl;

		ne.compute (*normals);   // Compute the features
		cout << "success to compute the features" << endl;
		
		//generate point with normal + xyz value
		pcl::concatenateFields(*cloud, *normals, *cloud_with_normals); //field를 더하는 함수 
		cout << "success to concatenate" << endl;

		//ply파일들을 합쳐주는 작업
		*temp_cloud += *cloud_with_normals;

		countfile++;

	}

	// check & remove if normal has invalid value
	cout << "trying to remove Nan values" << endl;
	std::vector<int> indn;
  	pcl::removeNaNNormalsFromPointCloud(*temp_cloud, *temp_cloud, indn);
	cout << "Success to remove Nan values" << endl;

	
	//합친 ply파일을 저장하기
	pcl::io::savePLYFileBinary(output_path+"final_bin.ply", *temp_cloud);
	pcl::io::savePLYFileASCII(output_path+"final_asc.ply", *temp_cloud);

	cout<<"All Done"<<endl;
	return 0;

}



