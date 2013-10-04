#include "region_growing_hsv.h"
#include <pcl/ros/conversions.h>
#include <vtkFloatArray.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
////////////////////////////////////////////////////////////////////////////////////////////
void RegionGrowingHSV::findPointNeighbours ()
{
    // convert point cloud type to vtkPolyData
    vtkPolyData* m_polydata = vtkPolyData::New();
    int convertnum = mesh2vtk(m_polymesh,m_polydata);
    if( num_pts!=convertnum )
        return;

    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree =
            boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(m_polymesh.cloud,*cloud);
    tree->setInputCloud(cloud);
    std::vector<float> nearestDis;

    std::vector<int> neighbours;
    point_neighbours_.clear();
    neighbours_for_possi_.clear();
    point_neighbours_.resize (num_pts, neighbours);
    neighbours_for_possi_.resize(num_pts,neighbours);

    for (int i_point = 0; i_point < num_pts; i_point++)
    {
        std::vector<bool> is_neighbour(num_pts,false);
        std::vector<int> seed_initial;
        std::vector<int> seed;
        seed_initial.push_back(i_point);

        int time = 0;
        neighbours.clear ();

        while( !seed_initial.empty() && time<searchDis_ )
        {
            while( !seed_initial.empty() )      // 将seed_initial中的种子点copy到seed中
            {
                seed.swap(seed_initial);
                seed_initial.clear();
            }
            while(!seed.empty())
            {
                int current_seed = seed[0];
                seed.erase(seed.begin());
                is_neighbour[current_seed] = true;
                findNeighbours(current_seed,seed_initial,neighbours,is_neighbour,m_polydata);
            }
            time++;
            if(time==1)
            {
                if(neighbours.size()<3)
                {
                    neighbours.clear();
                    nearestDis.clear();
                    tree->nearestKSearch(cloud->points[i_point],10,neighbours,nearestDis);
                }
                point_neighbours_[i_point] = neighbours;
            }
        }
        //std::cout<<"size neighbours of the "<<i_point<<"'s point is "<<neighbours.size()<<std::endl;
        if(neighbours.size()<3)
        {
            neighbours.clear();
            nearestDis.clear();
            tree->nearestKSearch(cloud->points[i_point],10,neighbours,nearestDis);
        }
        neighbours_for_possi_[i_point].swap (neighbours);
    }
    std::cout<<"find point neighbours end..."<<std::endl;
}

/////////////////////////////////////////////////////////////////////////////////////
void RegionGrowingHSV::findNeighbours(int current_seed,std::vector<int>& seed_initial,std::vector<int>& neighbours,
                                      std::vector<bool>& is_neighbour,vtkSmartPointer<vtkPolyData> m_polydata)
{
    vtkIdList* list_c = vtkIdList::New();   // 保存索引点关联的所有面片
    m_polydata->GetPointCells(current_seed,list_c);

    // find connected points
    for( vtkIdType j=0;j<list_c->GetNumberOfIds();j++ ) // 遍历关联的所有面片
    {
        vtkIdList* list_p = vtkIdList::New();   // 保存一个面片的点集
        m_polydata->GetCellPoints(list_c->GetId(j),list_p);   // 获取单个面片的点集
        for( vtkIdType m=0;m<list_p->GetNumberOfIds();m++ )
        {
            int id = list_p->GetId(m);
            if(is_neighbour[id]==true)	// 已标记
                continue;
            neighbours.push_back(id);
            seed_initial.push_back(id);
            is_neighbour[id] = true;
        }
        list_p->Delete();
    }
    list_c->Delete();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr RegionGrowingHSV::getColoredCloud()
{
    // 随机得到颜色集合
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(m_polymesh.cloud,*cloud_rgb);

    srand(static_cast<unsigned int> (time(0)));
    std::vector<unsigned char> colors;
    for(size_t i_segment = 0;i_segment<clusters_.size();i_segment++ )
    {
        colors.push_back (static_cast<unsigned char> ((rand ()+45) % 256));
        colors.push_back (static_cast<unsigned char> ((rand ()+23) % 256));
        colors.push_back (static_cast<unsigned char> (rand () % 256));
    }

    std::vector<std::vector<int> >::iterator i_segment;
    int next_color = 0;
    for( i_segment = clusters_.begin();i_segment != clusters_.end(); i_segment++ )
    {
       std::vector<int>::iterator i_point;
       for( i_point = (*i_segment).begin();i_point != (*i_segment).end();i_point++)
       {
           int index = *i_point;
           cloud_rgb->points[index].r = colors[3*next_color];
           cloud_rgb->points[index].g = colors[3*next_color+1];
           cloud_rgb->points[index].b = colors[3*next_color+2];
       }
       next_color++;
    }
    return cloud_rgb;
}


void RegionGrowingHSV::findOldRegionNeighbours(std::vector<std::vector<int> > &nearby_region_id,
                                               std::vector< std::vector<int> > old_segment)
{
    std::vector<bool> has_reached;
    std::vector<int> region_id;
    for(int i=0;i<number_of_segments_;i++)
    {
        region_id.clear();
        has_reached.clear();
        has_reached.resize(number_of_segments_,false);
        for(int j=0;j<old_segment[i].size();j++)
        {
            int id = old_segment[i][j];    // 遍历old_segment[i]中所有点
            for(int s=0;s<point_neighbours_[id].size();s++)
            {
                int region_belong = point_labels_[ point_neighbours_[id][s] ];
                if( region_belong!=i )
                {
                    if( has_reached[region_belong]==false)
                    {
                        has_reached[region_belong] = true;
                        region_id.push_back(region_belong);
                    }
                }
            }
        }
        // 将附近区域按点云大小排序
        //std::sort(region_id.begin(),region_id.end(),compareid);     // old_segment序号小则点云大
        //std::cout<<"the "<<i<<" region neighbours size is "<<region_id.size()<<std::endl;
        nearby_region_id[i].swap(region_id);
    }
    std::cout<<"find region neighbours ends"<<std::endl;
}


int RegionGrowingHSV::mesh2vtk (const pcl::PolygonMesh& mesh, vtkPolyData *poly_data)
{
    int nr_points = mesh.cloud.width * mesh.cloud.height;
    int nr_polygons = static_cast<int> (mesh.polygons.size ());

    // reset vtkPolyData object
    //poly_data = vtkPolyData::New (); // OR poly_data->Reset();
    vtkSmartPointer<vtkPoints> vtk_mesh_points = vtkSmartPointer<vtkPoints>::New ();
    vtkSmartPointer<vtkCellArray> vtk_mesh_polygons = vtkSmartPointer<vtkCellArray>::New ();
    poly_data->SetPoints (vtk_mesh_points);

    // get field indices for x, y, z (as well as rgb and/or rgba)
    int idx_x = -1, idx_y = -1, idx_z = -1, idx_rgb = -1, idx_rgba = -1, idx_normal_x = -1, idx_normal_y = -1, idx_normal_z = -1;
    for (int d = 0; d < static_cast<int> (mesh.cloud.fields.size ()); ++d)
    {
        if (mesh.cloud.fields[d].name == "x") idx_x = d;
        else if (mesh.cloud.fields[d].name == "y") idx_y = d;
        else if (mesh.cloud.fields[d].name == "z") idx_z = d;
        else if (mesh.cloud.fields[d].name == "rgb") idx_rgb = d;
        else if (mesh.cloud.fields[d].name == "rgba") idx_rgba = d;
        else if (mesh.cloud.fields[d].name == "normal_x") idx_normal_x = d;
        else if (mesh.cloud.fields[d].name == "normal_y") idx_normal_y = d;
        else if (mesh.cloud.fields[d].name == "normal_z") idx_normal_z = d;
    }
    if ( ( idx_x == -1 ) || ( idx_y == -1 ) || ( idx_z == -1 ) )
        nr_points = 0;

    // copy point data
    vtk_mesh_points->SetNumberOfPoints (nr_points);
    if (nr_points > 0)
    {
        Eigen::Vector4f pt = Eigen::Vector4f::Zero ();
        Eigen::Array4i xyz_offset (mesh.cloud.fields[idx_x].offset, mesh.cloud.fields[idx_y].offset, mesh.cloud.fields[idx_z].offset, 0);
        for (vtkIdType cp = 0; cp < nr_points; ++cp, xyz_offset += mesh.cloud.point_step)
        {
            memcpy(&pt[0], &mesh.cloud.data[xyz_offset[0]], sizeof(float));
            memcpy(&pt[1], &mesh.cloud.data[xyz_offset[1]], sizeof(float));
            memcpy(&pt[2], &mesh.cloud.data[xyz_offset[2]], sizeof(float));
            vtk_mesh_points->InsertPoint(cp, pt[0], pt[1], pt[2]);
        }
    }

    // copy polygon data
    if (nr_polygons > 0)
    {
        for (int i = 0; i < nr_polygons; i++)
        {
            unsigned int nr_points_in_polygon = static_cast<unsigned int> (mesh.polygons[i].vertices.size ());
            vtk_mesh_polygons->InsertNextCell (nr_points_in_polygon);
            for (unsigned int j = 0; j < nr_points_in_polygon; j++)
                vtk_mesh_polygons->InsertCellPoint(mesh.polygons[i].vertices[j]);
        }
        poly_data->SetPolys (vtk_mesh_polygons);
    }

    // copy color information
    if (idx_rgb != -1 || idx_rgba != -1)
    {
        vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
        colors->SetNumberOfComponents (3);
        colors->SetName ("Colors");
        pcl::RGB rgb;
        int offset = (idx_rgb != -1) ? mesh.cloud.fields[idx_rgb].offset : mesh.cloud.fields[idx_rgba].offset;
        for (vtkIdType cp = 0; cp < nr_points; ++cp)
        {
            memcpy (&rgb, &mesh.cloud.data[cp * mesh.cloud.point_step + offset], sizeof (pcl::RGB));
            const unsigned char color[3] = {rgb.r, rgb.g, rgb.b};
            colors->InsertNextTupleValue (color);
        }
        poly_data->GetPointData ()->SetScalars (colors);
    }

    // copy normal information
    if ( ( idx_normal_x != -1 ) && ( idx_normal_y != -1 ) && ( idx_normal_z != -1 ) )
    {
        vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New ();
        normals->SetNumberOfComponents (3);
        float nx = 0.0f, ny = 0.0f, nz = 0.0f;
        for (vtkIdType cp = 0; cp < nr_points; ++cp)
        {
            memcpy (&nx, &mesh.cloud.data[cp*mesh.cloud.point_step+mesh.cloud.fields[idx_normal_x].offset], sizeof(float));
            memcpy (&ny, &mesh.cloud.data[cp*mesh.cloud.point_step+mesh.cloud.fields[idx_normal_y].offset], sizeof(float));
            memcpy (&nz, &mesh.cloud.data[cp*mesh.cloud.point_step+mesh.cloud.fields[idx_normal_z].offset], sizeof(float));
            const float normal[3] = {nx, ny, nz};
            normals->InsertNextTupleValue (normal);
        }
        poly_data->GetPointData()->SetNormals (normals);
    }
    std::cout<<"mesh2vtk end"<<"poly_data"<<poly_data->GetNumberOfPoints()<<std::endl;
    if (poly_data->GetPoints() == NULL)
        return (0);
    return (static_cast<int> (poly_data->GetPoints()->GetNumberOfPoints ()));
}
