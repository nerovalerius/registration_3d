#include	<ros/ros.h>	
#include	<pcl/point_cloud.h>	
#include	<pcl_conversions/pcl_conversions.h>	
#include	<sensor_msgs/PointCloud2.h>	
#include	<pcl/filters/statistical_outlier_removal.h>	
	



// THE MAIN
	
main(int	argc,	char**	argv) {	

    ros::init(argc,	argv,	"pcl_outlier_filter");	
    std::cout << "PCL Outlier Filter" << std::endl;

    if (argc < 2){
        std::cout << "Usage: " << "pcl_outlier_filter <pointcloud.pcd>" << std::endl;
        return -1;
    }

    // Variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>	cloud_filtered;



    // Read Cloud from File
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1){ 

        std::cout << "could not read " << argv[1] << std::endl;
        return (-1);
    }


    // Filtering
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ>	statFilter;	
    statFilter.setInputCloud(cloud);	
    statFilter.setMeanK(50);	
    statFilter.setStddevMulThresh(2.0);	
    statFilter.filter(cloud_filtered);	


    // Give the output file a proper name 
    std::string output_file = argv[1];
    output_file = output_file.substr (0,output_file.length()-4);
    output_file+="_outlier_filtered";
    output_file+=".pcd";

    // Save the PointClouds into a file
    pcl::io::savePCDFileASCII(output_file, cloud_filtered);


    std::cout << "finished - Pointclouds outlier filtered " << std::endl;

    return	0;	
}	

