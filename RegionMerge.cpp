#include "region_growing_hsv.h"

bool compareSegmentSize(std::vector<int>& i,std::vector<int>& j)
{
    return (i.size()>j.size());
}

bool compareid(int &i,int &j)
{
    return (i<j);
}

bool compareisaac(std::pair<int,float>& i,std::pair<int,float>& j)
{
    return (i.second<j.second);
}

void RegionGrowingHSV::regionMerge()
{
    std::cout<<"old segment size is "<<number_of_segments_<<std::endl;
    bool is_end = false;
    std::vector< std::vector<int> > old_segment;
    std::vector< std::vector<int> > nearbyRegion;
    std::vector<int> region_belong;
    std::vector<float> segment_color;
    std::vector<int> isaac;
    int num_of_new_segment;
    if( cfg_ )
    {
        // merge regions with similar color
        while( !is_end )
        {
            std::cout<<"merge"<<number_of_segments_<<std::endl;
            is_end = true;
            old_segment.clear();
            segment_color.clear();
            old_segment.resize(number_of_segments_,isaac);
            segment_color.resize(number_of_segments_,0);

            // initial old_segment and segment_color
            for(int i=0;i<num_pts;i++)
            {
                int segment_index = point_labels_[i];
                old_segment[ segment_index ].push_back( i );
                segment_color[ segment_index ] += cloud_hsv_[ i ];
            }

            for(int i=0;i<number_of_segments_;i++)
            {
                segment_color[i] = static_cast<float>(segment_color[i]/static_cast<int>(old_segment[i].size()));
            }

            // find nearby region
            nearbyRegion.clear();
            nearbyRegion.resize(number_of_segments_,isaac);
            findOldRegionNeighbours(nearbyRegion,old_segment);

            // merge regions with color diff small than color_r2r_threshold_
            num_of_new_segment = 0;
            region_belong.clear();
            region_belong.resize(number_of_segments_,-1);

            for(int i=0;i<number_of_segments_;i++)
            {
                // 该区域已划分到新区域中
                if( region_belong[i]!=-1 )
                {
                    continue;
                }

                // 该区域未划分到新区域中,则遍历其周围区域寻找满足条件的区域合并
                std::vector<std::pair<int,float> > diff;
                std::pair<int,float> node;
                for(int j=0;j<nearbyRegion[i].size();j++)
                {
                    float color_diff = std::abs( segment_color[i]-segment_color[ nearbyRegion[i][j] ] );
                    node.first = j;
                    node.second = color_diff;
                    diff.push_back(node);
                }
                if(!diff.empty())
                {
                    std::sort(diff.begin(),diff.end(),compareisaac);
                    if( diff[0].second<color_r2r_threshold_ )	// 找到满足条件的区域
                    {
                        if( region_belong[ nearbyRegion[i][ diff[0].first ] ]!=-1 )
                        {
                            region_belong[i] = region_belong[ nearbyRegion[i][ diff[0].first ] ];
                        }
                        else
                        {
                            region_belong[i] = num_of_new_segment;
                            region_belong[ nearbyRegion[i][ diff[0].first ] ] = num_of_new_segment;
                            num_of_new_segment++;
                        }
                        is_end = false;
                    }
                }
                if( region_belong[i]==-1 )
                {
                    region_belong[i] = num_of_new_segment;
                    num_of_new_segment++;
                }
            }
            if( is_end==true )
                break;

            // update point_labels_ and number_of_segments_
            for(int i=0;i<number_of_segments_;i++)
            {
                int new_region_id = region_belong[i];
                for(int j=0;j<old_segment[i].size();j++)
                {
                    point_labels_[ old_segment[i][j] ] = new_region_id;
                }
            }
            number_of_segments_ = num_of_new_segment;
        }
    }
    if( merge_min_ )
    {
        // remove regions which has less than min_pts_per_cluster_ point
        is_end = false;
        while( !is_end )
        {
            std::cout<<"remove"<<number_of_segments_<<std::endl;
            is_end = true;
            old_segment.clear();
            old_segment.resize(number_of_segments_,isaac);
            segment_color.clear();
            segment_color.resize(number_of_segments_,0);

            // initial old_segment and segment_color
            for(int i=0;i<num_pts;i++)
            {
                int segment_index = point_labels_[i];
                old_segment[ segment_index ].push_back( i );
                segment_color[ segment_index ] += cloud_hsv_[ i ];
            }

            for(int i=0;i<number_of_segments_;i++)
            {
                segment_color[i] = static_cast<float>(segment_color[i]/static_cast<int>(old_segment[i].size()));
            }

            // find nearby region
            nearbyRegion.clear();
            nearbyRegion.resize(number_of_segments_,isaac);
            findOldRegionNeighbours(nearbyRegion,old_segment);

            // merge regions with color diff small than color_r2r_threshold_
            num_of_new_segment = 0;
            region_belong.clear();
            region_belong.resize(number_of_segments_,-1);

            for(int i=0;i<number_of_segments_;i++)
            {
                // 该区域已划分到新区域中
                if( region_belong[i]!=-1 )
                {
                    continue;
                }

                // 该区域大小满足要求
                if( old_segment[i].size()>min_pts_per_cluster_ )
                {
                    region_belong[i] = num_of_new_segment;
                    num_of_new_segment++;
                    continue;
                }

                // 该区域大小小于指定阈值
                if( nearbyRegion[i].size()<1 )
                {
                    std::cout<<"validate region"<<std::endl;
                    region_belong[i] = num_of_new_segment;
                    num_of_new_segment++;
                }
                else
                {
                    std::vector<std::pair<int,float> > diff;
                    std::pair<int,float> node;
                    for(int j=0;j<nearbyRegion[i].size();j++)
                    {
                        float color_diff = std::abs( segment_color[i]-segment_color[ nearbyRegion[i][j] ] );
                        node.first = j;
                        node.second = color_diff;
                        diff.push_back(node);
                    }
                    std::sort(diff.begin(),diff.end(),compareisaac);
                    if( region_belong[ nearbyRegion[i][diff[0].first] ]!=-1 )
                    {
                        region_belong[i] = region_belong[ nearbyRegion[i][diff[0].first] ];
                    }
                    else
                    {
                        region_belong[i] = num_of_new_segment;
                        region_belong[ nearbyRegion[i][diff[0].first] ] = num_of_new_segment;
                        num_of_new_segment++;
                    }
                    is_end = false;
                }
            }
            if( is_end==true )
                break;

            // update point_labels_ and number_of_segments_
            for(int i=0;i<number_of_segments_;i++)
            {
                int new_region_id = region_belong[i];
                for(int j=0;j<old_segment[i].size();j++)
                {
                    point_labels_[ old_segment[i][j] ] = new_region_id;
                }
            }
            number_of_segments_ = num_of_new_segment;
        }
    }
    // gain clusters_
    clusters_.clear();
    clusters_.resize(number_of_segments_,isaac);
    for(int i=0;i<num_pts;i++)
    {
        int id = point_labels_[i];
        clusters_[id].push_back(i);
    }
/*
    // 计算最终区域H均值
    segment_color.clear();
    segment_color.resize(clusters_.size(),0);
    for( int i=0;i<clusters_.size();i++)
    {
        for(int j=0;j<clusters_[i].size();j++)
        {
            segment_color[i] += cloud_hsv_[ clusters_[i][j] ];
        }
        segment_color[i] = static_cast<float>( segment_color[i]/static_cast<int>(clusters_[i].size()));
        std::cout<<i<<" region H average is "<<segment_color[i]<<std::endl;
    }
    std::cout<<"size of final segment "<<clusters_.size()<<std::endl;
    */
}
