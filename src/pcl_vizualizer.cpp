//      ___         _            _      __   __  _        _                  ____  ___  
//     / __|  ___  | |__   ___  | |_    \ \ / / (_)  ___ (_)  ___   _ _     |__ / |   \ 
//    | (__  / _ \ | '_ \ / _ \ |  _|    \ V /  | | (_-< | | / _ \ | ' \     |_ \ | |) |
//     \___| \___/ |_.__/ \___/  \__|     \_/   |_| /__/ |_| \___/ |_||_|   |___/ |___/ 
//                                                                                                                        
// Armin Niedermüller
//
// PCL_Viewer - View pcd files
//


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/make_shared.hpp>
#include <pcl/visualization/pcl_visualizer.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;


// visualizer
pcl::visualization::PCLVisualizer *p;
// iewport
int vp_1;



int main (int argc, char** argv){

    if(argc < 1){
        std::cout << "usage: pcl_viewer <file.pcd>";
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);


    // Read Cloud from File
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *source) == -1){ 

        std::cout << "could not read " << argv[1] << std::endl;
        return (-1);
    }

        // Read Cloud from File
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[2], *target) == -1){ 

        std::cout << "could not read " << argv[2] << std::endl;
        return (-1);
    }



    // Create a PCLVisualizer object
    p = new pcl::visualization::PCLVisualizer ("Pairwise Incremental Registration example");
    p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
    PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tgt_h (target, 255, 255, 255);
    PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_src_h (source, 255, 255, 255);


    p->createViewPort (0, 0, 1.0, 1.0, vp_1);
    p->addPointCloud (target, cloud_tgt_h, "target", vp_1);
    p->addPointCloud (source, cloud_src_h, "source", vp_1);


    // Set point size bigger
    p->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target");
    p->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source");

    p->spin();

    return (0);
}