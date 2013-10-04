#include "region_growing_hsv.h"

bool com (std::pair<int,float> i,std::pair<int,float> j)
{
    return (i.second>j.second);
}

void RegionGrowingHSV::applySmoothRegionGrowingAlgorithm()
{
    point_labels_.resize (num_pts, -1);
    std::vector<std::vector<int> > Area_cluster;
    std::vector<int> isaac;
    int seed = 0;

    int number_of_segments = 0;

    std::vector<int> init_seed;
    std::cout<<"calculate seed queue"<<std::endl;
    computeInitalSeed(init_seed);
    if( init_seed.size()!=num_pts )
    {
        std::cout<<"error"<<std::endl;
    }

    while (!init_seed.empty())
    {
        // select the seed point index
        seed = init_seed[0];
        init_seed.erase(init_seed.begin());
        if( point_labels_[seed]!=-1 )
            continue;

        Area_cluster.push_back(isaac);
        growRegion (seed, number_of_segments,Area_cluster);
        number_of_segments++;
    }
    number_of_segments_ = number_of_segments;
}

void RegionGrowingHSV::growRegion(int initial_seed, int segment_number,std::vector<std::vector<int> >& Area_cluster)
{
    std::queue<int> seeds;
    seeds.push (initial_seed);
    Area_cluster[segment_number].push_back(initial_seed);

    point_labels_[initial_seed] = segment_number;

    while (!seeds.empty ())
    {
        int curr_seed;
        curr_seed = seeds.front ();
        seeds.pop ();

        size_t i_nghbr = 0;
        while ( i_nghbr < point_neighbours_[curr_seed].size () )
        {
            int index = point_neighbours_[curr_seed][i_nghbr];
            if (point_labels_[index] != -1)
            {
                i_nghbr++;
                continue;
            }

            bool is_a_seed = false;
            bool belongs_to_segment = validatePoint (initial_seed, curr_seed, index, is_a_seed,segment_number,Area_cluster);

            if (belongs_to_segment == false)
            {
                i_nghbr++;
                continue;
            }

            Area_cluster[segment_number].push_back(index);
            point_labels_[index] = segment_number;

            if (is_a_seed)
            {
                seeds.push (index);
            }

            i_nghbr++;
        }// next neighbour
    }// next seed
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                 *************   initial_seed, current_seed, index, is_a_seed(false)
bool RegionGrowingHSV::validatePoint (int initial_seed, int point, int nghbr, bool& is_a_seed,
                                      int segment_number, std::vector<std::vector<int> > &Area_cluster)
{
    is_a_seed = true;

    // find current seed qujian
    int qjid = static_cast<int>(cloud_hsv_[point]/hist_width);
    std::pair<int,int> qujian;
    qujian = color_threshold_[qjid];
    //std::cout<<"point H "<<cloud_hsv_.points[point].h<<" qujian "<<qjid<<" "<<qujian.first<<" "<<qujian.second<<std::endl;

    // check the color difference bettwen the current seed
    if( cloud_hsv_[nghbr]<qujian.first || cloud_hsv_[nghbr]>qujian.second)
        return false;

    // chech the nghbr point if it is a seed point

    if(possibility_[nghbr]<edge_poss_)
        is_a_seed = false;

    return true;
}

void RegionGrowingHSV::computeInitalSeed(std::vector<int> &seed)
{
    int size_of_hist = static_cast<int>(360/hist_width)+1;
    std::vector< std::vector<int> > hist( size_of_hist );
    std::pair<int,int> qujian;
    color_threshold_.resize(size_of_hist,qujian);

    // 统计分布到hist中
    for(int i=0;i<cloud_hsv_.size();i++)
    {
        int index = static_cast<int>(cloud_hsv_[i]/hist_width);   // index 可能小于0->h为-1
        hist[index].push_back(i);
    }

    // 计算相对变化率
    std::cout<<" calculate xdbhl_ "<<std::endl;
    xdbhl_.clear();
    for(int i=0;i<hist.size()-1;i++)
    {
        float xd = std::abs(static_cast<float>(hist[i].size())-static_cast<float>(hist[i+1].size()));
        float fm = std::max(10.0f,(static_cast<float>(hist[i].size())+static_cast<float>(hist[i+1].size()))/2.0f);
        xdbhl_.push_back(static_cast<float>(xd/fm));
    }

    // calculate color_threshold_;
    std::cout<<" calculate color_threshold_ "<<std::endl;
    for(int i=0;i<hist.size();i++)
    {
        qujian.first = computeLowThreshold(i,hist)-1;
        qujian.second = computeHighThreshold(i,hist)+1;
        color_threshold_[i] = qujian;
        //std::cout<<"the "<<i<<" color_threshold "<<color_threshold_[i].first<<"  "<<color_threshold_[i].second<<"size "<<hist[i].size()<<std::endl;
    }

    // 计算优先级
    std::cout<<" calculate hist_proi_ "<<std::endl;
    hist_proi_.clear();
    for(int i=0;i<hist.size();i++)
    {
        float g = 0.0;
        if( i==0 )
        {
            g = std::pow( xdbhl_[i],2.0f );
        }
        else if( i==hist.size()-1 )
        {
            g = std::pow( xdbhl_[i-1],2.0f );
        }
        else
            g = std::pow( xdbhl_[i-1],2.0f ) + std::pow( xdbhl_[i],2.0f);

        this->hist_proi_.push_back(g);
    }

    // 计算点的优先级
    std::cout<<"calculatePointPossibility!!!"<<std::endl;
    calculatePointPossibility();
    point_proi_.clear();
    for(int i=0;i<hist_proi_.size();i++)
    {
        std::pair<int,float> g;
        float hist_poss = static_cast<float>(hist[i].size())/static_cast<float>(num_pts);
        for(int j=0;j<hist[i].size();j++)
        {
            int point_id = hist[i][j];
            g.first = point_id;
            g.second = possibility_[point_id] * hist_poss * (3.0f-hist_proi_[i]);
            point_proi_.push_back(g);
        }
    }

    // insert point id to the seed vector
    std::sort(point_proi_.begin(),point_proi_.end(),com);
    for(int i=0;i<point_proi_.size();i++)
    {
        seed.push_back(point_proi_[i].first);
    }
}


int RegionGrowingHSV::computeHighThreshold(int &num,const std::vector< std::vector<int> > hist)
{
    int size_of_hist = static_cast<int>(360/hist_width)+1;
    if( num==size_of_hist-1 )
        return 360;
    else
    {
        int j = num;
        while( j<size_of_hist-1 && xdbhl_[j]<input_xdbhul_ )    // 0.45
        {
            j++;
        }
        return ( (j+1) *hist_width );
    }
}

int RegionGrowingHSV::computeLowThreshold(int &num,const std::vector< std::vector<int> > hist)
{
    if( num==0 )
        return 0;
    else
    {
        int j = num;
        while( j>0 && xdbhl_[j-1]<input_xdbhul_ )   // 0.45f
        {
            j--;
        }
        return j*hist_width;
    }
}

void RegionGrowingHSV::calculatePointPossibility()
{
    possibility_.clear();
    possibility_.resize(num_pts,0);
    for(int id=0;id<num_pts;id++)
    {
        int qjid = static_cast<int>(cloud_hsv_[id]/hist_width);
        std::pair<int,int> qujian;
        qujian = color_threshold_[qjid];
        int inear_size = 0;
        for(int i=0;i<neighbours_for_possi_[id].size();i++)
        {
            int neighbour_id = neighbours_for_possi_[id][i];
            if( cloud_hsv_[neighbour_id]>=qujian.first && cloud_hsv_[neighbour_id]<=qujian.second )
                inear_size++;
        }
        possibility_[id] = static_cast<float>(inear_size)/static_cast<float>(neighbours_for_possi_[id].size());
    }
}
