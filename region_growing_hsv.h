#ifndef PCL_REGION_GROWING_HSV_H_
#define PCL_REGION_GROWING_HSV_H_

#include <pcl/PolygonMesh.h>
#include <queue>
#include <pcl/point_types.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkCellArray.h>
#include <vtkSmartPointer.h>
#include <vtkCleanPolyData.h>
#include <pcl/point_types_conversion.h>
#include <pcl/ros/conversions.h>

class RegionGrowingHSV
{
public:
    int min_pts_per_cluster_;
    int max_pts_per_cluster_;   // ∑¿÷π‘Îµ„«¯”Ú¿©…¢
    std::vector<std::vector<int> > point_neighbours_;
    std::vector<std::vector<int> > neighbours_for_possi_;
    std::vector<int> point_labels_;
    std::vector<int> num_pts_in_segment_;
    std::vector<std::vector<int> > clusters_;
    int number_of_segments_;
    pcl::PolygonMesh m_polymesh;
    std::vector<float> cloud_hsv_;
    std::vector<bool> is_seed_;
    float hist_width;
    std::vector<float> xdbhl_;
    std::vector<float > hist_proi_;
    std::vector<std::pair<int,float> > point_proi_;
    std::vector<float> possibility_;
    unsigned int searchDis_;
    float input_xdbhul_;
    bool cfg_;
    bool merge_min_;
    float edge_poss_;

public:

    RegionGrowingHSV ();

    virtual ~RegionGrowingHSV ();

    float getSearchLength () const;

    void setSearchLength (unsigned int thresh);

    void setEdgePoss(float thresh);

    void setInputXdbhl(unsigned int thresh);

    float getRegionColorThreshold () const;

    void setRegionColorThreshold (float thresh);

    void chufenge(bool ischecked);

    void mergeMinSizeRegion(bool ischecked);

    void applySmoothRegionGrowingAlgorithm();

    void growRegion(int initial_seed, int segment_number,std::vector<std::vector<int> > &Area_cluster);

    void setInputmesh(pcl::PolygonMesh &mesh);

    void setHistWidth(float width);

    void setMinClusterSize(unsigned int size);

    virtual bool extract();

    void findRegionNeighbours (std::vector< std::vector< std::pair<float, int> > >& neighbours_out,
                               std::vector< std::vector<int> >& regions_in);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getColoredCloud();

protected:

    void computeInitalSeed(std::vector<int>& seed);

    void calculatePointPossibility();

    void computexdbhl();

    int mesh2vtk (const pcl::PolygonMesh& mesh, vtkPolyData *poly_data);

    virtual bool prepareForSegmentation ();

    virtual void findPointNeighbours ();

    void applyRegionMergingAlgorithm ();

    void regionMerge();

    void findOldRegionNeighbours(std::vector<std::vector<int> >& nearby_region_id,
                                 std::vector< std::vector<int> > old_segment);

    void findNeighbours(int current_seed,std::vector<int>& seed_initial,std::vector<int>& neighbours,
                        std::vector<bool>& is_neighbour,vtkSmartPointer<vtkPolyData> m_polydata);

    void assembleRegions (std::vector<unsigned int>& num_pts_in_region, int num_regions);

    virtual bool validatePoint (int initial_seed, int point, int nghbr, bool& is_a_seed,
                                int segtment_number,std::vector<std::vector<int> > &Area_cluster);

    inline bool
    initCompute ()
    {
      // Check if input was set
      if (!m_polymesh.cloud.data.size())
        return (false);
      return (true);
    }

    inline float RGBToHSV(float r,float g,float b)
    {
        float minv = std::min(r,std::min(g,b));
        float maxv = std::max(r,std::max(g,b));
        if( minv==maxv )
            return 0;
        else if( maxv==r && g>=b )
        {
            return ( 60.0f*(g-b)/(maxv-minv) );
        }
        else if( maxv==r && g<b )
        {
            return ( 360.0f + 60.0f*(g-b)/(maxv-minv) );
        }
        else if( maxv==g )
        {
            return ( 120.0f + 60.0f *(b-r)/(maxv-minv) );
        }
        else        //(maxv==b )
        {
            return ( 240.0f + 60.0f *(r-g)/(maxv-minv) );
        }
    }


    inline bool
    deinitCompute ()
    {
      return (true);
    }

    int computeLowThreshold(int &num,const std::vector< std::vector<int> > hist);

    int computeHighThreshold(int &num,const std::vector< std::vector<int> > hist);

protected:

    int num_pts;

    /** \brief Thershold used in color test for points. */
    float color_p2p_threshold_;

    std::vector<std::pair<int,int> > color_threshold_;

    /** \brief Thershold used in color test for regions. */
    float color_r2r_threshold_;

};
#endif
