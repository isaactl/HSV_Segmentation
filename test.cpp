///////////////////////////////////////////////////////////////////
//// ��old_segment��ֵ
//std::cout<<"the old region size is "<<number_of_segments_<<std::endl;
//std::vector< std::vector<int> > old_segment;
//std::vector<int> pts_in_segment;
//old_segment.resize(number_of_segments_,pts_in_segment);
//std::vector< float > segment_color(number_of_segments_,0);

//for (int i_point = 0; i_point < num_pts; i_point++)
//{
//    int segment_index = point_labels_[i_point];
//    old_segment[segment_index].push_back(i_point);
//    segment_color[segment_index] += cloud_hsv_[i_point];
//}

///////////////////////////////////////////////////////////////////
//// �ϲ���ɫ���������--����segment��H��ƽ��ֵ
//std::cout<<"calculate the H difference"<<std::endl;
//for(int i=0;i<number_of_segments_;i++)
//{
//    segment_color[i] = static_cast<float>(segment_color[i]/static_cast<int>(old_segment[i].size()));
//}

///////////////////////////////////////////////////////////////////
//// Ѱ�Ҹ�������
//std::cout<<"find the nearby region"<<std::endl;
//std::vector<std::vector<int> > nearbyRegion(number_of_segments_,pts_in_segment);    //���渽������ID
//findOldRegionNeighbours(nearbyRegion,old_segment);

///////////////////////////////////////////////////////////////////
//// ������ɫ��Ϣ���кϲ����ѻ��ֵ��������Ҳ�迼������� ���յõ�cluster_
//std::cout<<"start merge..."<<std::endl;
//std::vector< std::vector<int> > new_segment;
//std::vector<int> segment_region(number_of_segments_,-1);
//int num_of_new_segment = 0;
//for( int i=0;i<number_of_segments_;i++ )    // �ϲ���Hֵ�����С��������
//{
//    if(segment_region[i]!=-1)
//        continue;

//    int isaac = -1;
//    for(int j=0;j<nearbyRegion[i].size();j++)
//    {
//        float color_diff = std::abs(segment_color[i]-segment_color[nearbyRegion[i][j]]);
//        std::cout<<"color diff "<<segment_color[i]<<" "<<segment_color[nearbyRegion[i][j]]<<" "<<color_diff<<std::endl;
//        if( color_diff<color_r2r_threshold_ )
//        {
//            isaac = nearbyRegion[i][j];   // nearby region id
//            if(segment_region[isaac]!=-1)
//                segment_region[i] = segment_region[isaac];
//            else
//            {
//                segment_region[isaac] = num_of_new_segment;
//                segment_region[i] = num_of_new_segment;
//                num_of_new_segment++;
//            }
//        }
//    }
//    if(segment_region[i]==-1)   // �ҵ���������Ҫ�������
//    {
////            if(old_segment[i].size()>min_pts_per_cluster_)
////            {
//            segment_region[i] = num_of_new_segment;
//            num_of_new_segment++;
////            }
////            else                // С����ϲ����������
////            {

////            }
//    }
//}
//std::cout<<"merged use color difference,now try to merge small. the new segment size is "<<num_of_new_segment<<std::endl;

//std::vector<int> diii;
//new_segment.resize(num_of_new_segment,diii);
//for(int i=0;i<number_of_segments_;i++)
//{
////        if(segment_region[i]==-1)
////            break;
//    int new_segment_id = segment_region[i];
//    for(int j=0;j<old_segment[i].size();j++)
//    {
//        new_segment[new_segment_id].push_back(old_segment[i][j]);
//    }
//}

//clusters_.resize(num_of_new_segment);
//clusters_.swap(new_segment);
//std::vector<std::vector<int> >::iterator cluster_iter = clusters_.begin ();
//while (cluster_iter != clusters_.end ())
//{
//    if ((*cluster_iter).size() < min_pts_per_cluster_ )
//    {
//        cluster_iter = clusters_.erase (cluster_iter);
//    }
//    else
//        cluster_iter++;
//}
//std::cout<<"size of final segment "<<clusters_.size()<<std::endl;
////    // ����ǰ��ķָ���Ϣ�ϲ�����
////    std::cout<<"clusters_ size "<<num_of_new_segment<<std::endl;
////    std::vector<int> old_segment_in_new_segment;
////    clusters_.resize(num_of_new_segment,old_segment_in_new_segment);
////    for(int i=0;i<number_of_segments_;i++)
////    {
////        int segment_new_id = segment_region[i];
////        for(int s=0;s<old_segment[i].size();s++)
////        {
////            clusters_[segment_new_id].push_back(old_segment[i][s]);
////        }
////    }
