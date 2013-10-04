#ifndef PCL_SEGMENTATION_REGION_GROWING_HSV_HPP_
#define PCL_SEGMENTATION_REGION_GROWING_HSV_HPP_

#include <QtGui>
#include "region_growing_hsv.h"
#include <algorithm>

bool comparePair (std::pair<float, int> i, std::pair<float, int> j)
{
    return (i.first < j.first);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
RegionGrowingHSV::RegionGrowingHSV () :
    searchDis_ (2),
    color_r2r_threshold_ (10.0f),
    hist_width(2),
    cfg_(true),
    edge_poss_(0.45),
    merge_min_(true),
    input_xdbhul_(0.45)
{
    min_pts_per_cluster_ = 10;
    max_pts_per_cluster_ = 500;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
RegionGrowingHSV::~RegionGrowingHSV ()
{
}

void RegionGrowingHSV::setInputXdbhl(unsigned int thresh)
{
    input_xdbhul_ = static_cast<float>(thresh)/50.0f;
}

void RegionGrowingHSV::setEdgePoss(float thresh)
{
    edge_poss_ = thresh;
    std::cout<<"edge_poss "<<edge_poss_<<std::endl;
}

void RegionGrowingHSV::chufenge(bool ischecked)
{
    cfg_ = ischecked;
}

void RegionGrowingHSV::mergeMinSizeRegion(bool ischecked)
{
    merge_min_ = ischecked;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float RegionGrowingHSV::getSearchLength () const
{
    return (searchDis_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RegionGrowingHSV::setSearchLength (unsigned int thresh)
{
    searchDis_ = thresh;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float RegionGrowingHSV::getRegionColorThreshold () const
{
    return color_r2r_threshold_;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RegionGrowingHSV::setRegionColorThreshold (float thresh)
{
    color_r2r_threshold_ = thresh;
    std::cout<<"color_r2r_threshold_ "<<color_r2r_threshold_<<std::endl;
}

void RegionGrowingHSV::setHistWidth(float width)
{
    hist_width = width;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RegionGrowingHSV::setMinClusterSize(unsigned int size)
{
    min_pts_per_cluster_ = size;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool RegionGrowingHSV::extract ()
{
    clusters_.clear ();
    point_neighbours_.clear ();
    point_labels_.clear ();
    num_pts_in_segment_.clear ();
    number_of_segments_ = 0;

    bool segmentation_is_possible = prepareForSegmentation ();
    if ( !segmentation_is_possible )
    {
        std::cout<<"segment is impossible"<<std::endl;
        return false;
    }
    std::cout<<"find point neighbours"<<std::endl;
    findPointNeighbours ();

//    std::vector<int> init_seed;
//    std::cout<<"calculate seed queue"<<std::endl;
//    computeInitalSeed(init_seed);
//    clusters_.clear();
//    std::vector<int> pa;
//    std::vector<int> pas;
//    for(int i=0;i<num_pts;i++ )
//    {
//        if( i<2000 )
//            pa.push_back(init_seed[i]);
//    }
//    clusters_.push_back(pa);
//    clusters_.push_back(pas);

    std::cout<<"apply region growing algorithm"<<std::endl;
    applySmoothRegionGrowingAlgorithm ();

    std::cout<<"apply region merging algorithm"<<std::endl;
    regionMerge();

    std::cout<<"end extract..."<<std::endl;
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool RegionGrowingHSV::prepareForSegmentation ()
{
    if ( num_pts == 0 )
    {
        QMessageBox::about(NULL,QObject::tr("¾¯¸æ"),QObject::tr("the point cloud is empty!"));
        std::cout<<"points equals 0"<<std::endl;
        return (false);
    }

    if ( searchDis_ < 1 || color_r2r_threshold_ < 0.0f || edge_poss_<0 || edge_poss_>1 )
    {
        return (false);
    }

    return (true);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void RegionGrowingHSV::setInputmesh(pcl::PolygonMesh& mesh)
{
    m_polymesh = mesh;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(m_polymesh.cloud,*cloud_rgb);
    num_pts = cloud_rgb->points.size();
    cloud_hsv_.resize(num_pts);
    is_seed_.resize(num_pts,false);

//    FILE *file;
//    file = fopen("D:\\test.txt","w");
    for(int i=0;i<num_pts;i++)
    {
        cloud_hsv_[i] = RGBToHSV(cloud_rgb->points[i].r,cloud_rgb->points[i].g,cloud_rgb->points[i].b);
        //std::cout<<i<<"  "<<cloud_hsv_[i]<<std::endl;
//        fprintf(file,"%f\n",cloud_hsv_[i]);
    }
//    fclose(file);
}

#endif    // PCL_SEGMENTATION_REGION_GROWING_HSV_HPP_
