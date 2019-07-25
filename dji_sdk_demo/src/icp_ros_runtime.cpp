
#include <SphereX/spherex_pp.h>

#define PI 3.14159265

Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity ();
PointCloud::Ptr global_cloud (new PointCloud), cloud_inRadius (new PointCloud), cloud_inPlane (new PointCloud);
PointCloud::Ptr cloud_inObstacle (new PointCloud),cloud_outPlane (new PointCloud), cloud_plane_one(new PointCloud);

std::ofstream pcdfile_write;

ros::ServiceServer set_next_hop_server;

int counter=0;
SphereX hop;

int main (int argc, char** argv)
{

  ros::init(argc,argv,"icp_ros_runtime");
	ros::NodeHandle nh;
  ros::Subscriber pcdFileListSub = nh.subscribe("/pcd_file_string", 10, &storefilename_callback);
  ros::Subscriber hopStatusSub = nh.subscribe("spherex/hopStatus", 100, &hop_status_callback);
  set_next_hop_server      = nh.advertiseService("dji_sdk/set_next_hop_service",  &set_next_hop_callback);

  pcdfile_write.open ("pcd_file_list.txt");

  std::vector<PCD, Eigen::aligned_allocator<PCD> > data;


  while(ros::ok())
  {
      if(hop.hop_status==true)
      {
          ROS_INFO("in Hopped");

          loadData_from_txt(data);

          ROS_INFO("Missed Load data");

          if (data.empty ())
          {
            PCL_ERROR ("Syntax is: %s <source.pcd> <target.pcd> [*]", argv[0]);
            PCL_ERROR ("[*] - multiple files can be added. The registration results of (i, i+1) will be registered against (i+2), etc");
            return (-1);
          }
          PCL_INFO ("Loaded %d datasets.", (int)data.size ());

        	PointCloud::Ptr result (new PointCloud), source, target;
          Eigen::Matrix4f pairTransform;

          for (size_t i = 1; i < data.size (); ++i)
          {
            source = data[i-1].cloud;
            target = data[i].cloud;

            PointCloud::Ptr temp (new PointCloud);
            PCL_INFO ("Aligning %s (%d) with %s (%d).\n", data[i].f_name.c_str (), source->points.size (), data[i-1].f_name.c_str (), target->points.size ());
            pairAlign (source, target, temp, pairTransform, true);

            GlobalTransform *= pairTransform;

            pcl::transformPointCloud(*target, *result, GlobalTransform);

            *global_cloud += *result;
            std::cout << GlobalTransform << std::endl;
            std::cout << "Now lets go to the local pairtransform" << std::endl;
            std::cout << pairTransform << std::endl;

         }
           std::stringstream ss;
           ss << "1.pcd";
           pcl::io::savePCDFile (ss.str (), *global_cloud, true);
           counter = 0;
					 hop.icp_status = true;

       }
			    bool ifcompute = compute_next_hop();

       ros::spinOnce();
   }

}

// Functions Start from here

void storefilename_callback(const std_msgs::String& pcd_file_name)
{
  if(!hop.hop_status && !hop.icp_status)   //if hop_status is true it means hop is completed
  {
    if(counter % 2 == 0)
    {
    	std::stringstream ss;
    	ss << pcd_file_name.data << ".pcd";
      ROS_INFO("%s",ss.str().c_str());
    	pcdfile_write << ss.str()<<endl;
    }
    counter++;
  }
  if(counter >= 6)
  {
      hop.hop_status=true;
  }
  else hop.hop_status = false;
}



void hop_status_callback(const std_msgs::Bool& status)
{
  hop.hop_status = status.data;
}

bool set_next_hop_callback(dji_sdk::setNextHop::Request &req, dji_sdk::setNextHop::Response &res)
{
  if(req.ifpped)
  {
    res.d = hop.d;
    res.theta = hop.theta;
    res.phi = hop.phi;
    res.vset = true;
    ROS_INFO("Next hop Set Request Properly Adressed");
    return true;
  }
  else
  {
    ROS_INFO("Only accepts clients with valid requests");
    res.vset = false;
    return false;
  }
}

void loadData_from_txt (std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
  std::string extension (".pcd");
  int argc =0;
  std::string line;
  ROS_INFO("In LoadData Loop");
  // Suppose the first argument is the actual test model
  ifstream pcdfile_linecount ("pcd_file_list.txt");
  if (pcdfile_linecount.is_open())
  {
    ROS_INFO("counting the arguments");
    while(getline (pcdfile_linecount,line))
    {

      argc++;
    }
    ROS_INFO("%d",argc);
    pcdfile_linecount.close();
  }
  else
  {
    ROS_INFO("unable to open the file");
  }
  ROS_INFO("In LoadData Loop");
  ifstream pcdfile_read ("pcd_file_list.txt");


  for (int i = 1; i < argc; i++)
  {
    std::stringstream ss;
    if (pcdfile_read.is_open())
    {
    getline (pcdfile_read,line) ;
    ss << line;
    ROS_INFO("Loading Data %s", ss.str().c_str());

    }
    std::string fname = ss.str();
    // Needs to be at least 5: .plot
    ROS_INFO("fname %s", fname.c_str());
    if (fname.size () <= extension.size ())
      continue;

    std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);

    //check that the argument is a pcd file
    ROS_INFO("Is the compare output %d",fname.compare (fname.size () - extension.size (), extension.size (), extension) );
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the cloud and saves it into the global list of models
      ROS_INFO("Successfully Matched");
      PCD m;
      m.f_name = ss.str();
      pcl::io::loadPCDFile (ss.str(), *m.cloud);
      //remove NAN points from the cloud
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);

      models.push_back (m);
    }
  }
  pcdfile_read.close();
}

bool compute_next_hop()
{
  if(hop.hop_status && hop.icp_status)
  {

      double x_ = GlobalTransform(0, 3);
      double y_ = GlobalTransform(1, 3);
      double z_ = GlobalTransform(2, 3);
      double r = 3;

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>), cloud_inRadius_filtered(new pcl::PointCloud<pcl::PointXYZ>);
      cloud->width  = 300;
      cloud->height = 300;
      cloud->points.resize (cloud->width*cloud->height);
      int a = -3;
      int b = 3;
      int c = -3;
      int d = 3;
      for (size_t i = 0; i < cloud->points.size (); ++i)
      {

      cloud->points[i].x = (b-a)*(rand()/(RAND_MAX + 1.0f))+ a +x_;
      cloud->points[i].y = (d-c)*(rand()/(RAND_MAX + 1.0f))+ c +y_+;
      cloud->points[i].z = -0.6;

      }
      *global_cloud += *cloud;

      int number_of_indices = 70000 ;
      std::vector<float> k_radius(number_of_indices);
      std::vector<int> k_indices(number_of_indices);
      pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
      kdtree.setInputCloud (global_cloud);
      pcl::PointXYZ searchPoint;
      searchPoint.x = x_;
      searchPoint.y = y_;
      searchPoint.z = z_;
      hop.x = x_;
      hop.y = y_;
      hop.z = z_;
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
      kdtree.radiusSearch(searchPoint, r , k_indices, k_radius);
      inliers->indices = k_indices;
      pcl::ExtractIndices<pcl::PointXYZ> filter_pc ; // Initializing with true will allow us to extract the removed indices
      filter_pc.setInputCloud(global_cloud);
      filter_pc.setIndices(inliers);
      filter_pc.filter(*cloud_inRadius);
      std::stringstream ss1;
      ss1 << "2.pcd";
      pcl::io::savePCDFile (ss1.str (), *cloud_inRadius, true);

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices),outliers2 (new pcl::PointIndices) ;

      //Eigen::Vector3f axis_plane(0.0, 0.0, 1.0);
      //filters
      pcl::VoxelGrid<pcl::PointXYZ> clean;
      clean.setInputCloud(cloud_inRadius);
      clean.setLeafSize(0.03f, 0.03f, 0.03f);
      clean.filter(*cloud_inRadius_filtered);
      std::stringstream ss3;
      ss3 << "3.pcd";
      pcl::io::savePCDFile (ss3.str (), *cloud_inRadius_filtered, true);
      //filter_end
      //plane_segmenter
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.2);
      seg.setInputCloud(cloud_inRadius_filtered);
      seg.segment(*inliers2, *coefficients);
      pcl::ExtractIndices<pcl::PointXYZ> extract (true);
      extract.setInputCloud(cloud_inRadius_filtered);
      extract.setIndices(inliers2);
      extract.setNegative(false);
      extract.filter(*cloud_inPlane);
      //extract.setNegative(true);
      extract.getRemovedIndices(*outliers2);
      //extract.filter(outliers2);
      extract.setIndices(outliers2);
      extract.setNegative(false);
      extract.filter(*cloud_outPlane);
      std::stringstream ss4;
      ss4 << "4.pcd";
      pcl::io::savePCDFile (ss4.str (), *cloud_inPlane, true);
      std::stringstream ss5;
      ss5 << "5.pcd";
      pcl::io::savePCDFile (ss5.str (), *cloud_outPlane, true);
      //split_planes_with thickness
      double thickness_t = 0.1;
      double distance_del = 0.2;
      double z_min = -0.3; // later we will calculate from planes by averaging their zs
      double z_max = 3;    // later we will calculate this from planes by averaging
      int planes_number = (int)((z_max - z_min)/(thickness_t+ distance_del));
      //p.z>z_min+i*(distance_del) p.z<z_min+i*(distance_del)+thickness_t
      std::vector<pcl::PointIndices::Ptr, Eigen::aligned_allocator<pcl::PointIndices::Ptr> > plane_inliers_pointer;
      for(int i = 0; i < planes_number; i++)
      {
        pcl::PointIndices::Ptr inlier_segment (new pcl::PointIndices);
        for(int j=0; j < cloud_outPlane->points.size(); j++)
        {
          if(cloud_outPlane->points[j].z > (z_min+i*(distance_del)) && cloud_outPlane->points[j].z<(z_min+i*(distance_del)+thickness_t))
          {
           inlier_segment->indices.push_back(j);
          }

        }
        plane_inliers_pointer.push_back(inlier_segment);


      }

      //plane_inliers_pointer.at(0) = outliers2;
      //plane_inliers_pointer[0] = outliers2;
      cout << "Point Cloud " << 0 << "has got " << plane_inliers_pointer[2]->indices.size() << " Points" << endl;
      extract.setInputCloud(cloud_outPlane);
      extract.setIndices(plane_inliers_pointer[0]);
      cout<<"Size of Cloud 1 "<<plane_inliers_pointer[0]->indices.size()<<endl;
      extract.setNegative(false);
      extract.filter(*cloud_plane_one);
      std::stringstream ss6;
      ss6 << "6.pcd";
      pcl::io::savePCDFile (ss6.str (), *cloud_plane_one, true);
      PointCloud::Ptr cloud2 (new PointCloud), cloud3 (new PointCloud), cloud4 (new PointCloud), cloud5 (new PointCloud), cloud6 (new PointCloud), cloud7 (new PointCloud), cloud8 (new PointCloud);
      extract.setIndices(plane_inliers_pointer[1]);
      cout<<"Size of Cloud 2 "<<plane_inliers_pointer[1]->indices.size()<<endl;
      extract.setNegative(false);
      extract.filter(*cloud2);
      std::stringstream ss7;
      ss7 << "7.pcd";
      pcl::io::savePCDFile (ss7.str (), *cloud2, true);
      extract.setIndices(plane_inliers_pointer[2]);
      cout<<"Size of Cloud 3 "<<plane_inliers_pointer[2]->indices.size()<<endl;
      extract.setNegative(false);
      extract.filter(*cloud3);
      std::stringstream ss8;
      ss8 << "8.pcd";
      pcl::io::savePCDFile (ss8.str (), *cloud3, true);
      extract.setIndices(plane_inliers_pointer[3]);
      cout<<"Size of Cloud 4 "<<plane_inliers_pointer[3]->indices.size()<<endl;
      extract.setNegative(false);
      extract.filter(*cloud4);
      std::stringstream ss9;
      ss9 << "9.pcd";
      pcl::io::savePCDFile (ss9.str (), *cloud4, true);
      extract.setIndices(plane_inliers_pointer[4]);
      cout<<"Size of Cloud 5 "<<plane_inliers_pointer[4]->indices.size()<<endl;
      extract.setNegative(false);
      extract.filter(*cloud5);
      std::stringstream ss10;
      ss10 << "10.pcd";
      pcl::io::savePCDFile (ss10.str (), *cloud5, true);
      extract.setIndices(plane_inliers_pointer[5]);
      cout<<"Size of Cloud 6 "<<plane_inliers_pointer[5]->indices.size()<<endl;
      extract.setNegative(false);
      extract.filter(*cloud6);
      std::stringstream ss11;
      ss11 << "11.pcd";
      pcl::io::savePCDFile (ss11.str (), *cloud6, true);

      int numberOfSectors = 24;
      int anglePerSector = 360/numberOfSectors;
      std::vector<Plane, Eigen::aligned_allocator<Plane> > Planes;
      //plane_segmenter end
      for(int m = 0; m < planes_number; m++)
      {

        PointCloud::Ptr cloud_ofPlane (new PointCloud);
        pcl::ExtractIndices<pcl::PointXYZ> extract_as_cloud (true);
        extract_as_cloud.setInputCloud(cloud_outPlane);
        extract_as_cloud.setIndices(plane_inliers_pointer[m]);
        extract_as_cloud.setNegative(false);
        extract_as_cloud.filter(*cloud_ofPlane);
        cout<<"Size of Cloud  "<<m<<" "<<plane_inliers_pointer[m]->indices.size()<<endl;
        cout<<"size of a cloud"<<cloud_ofPlane->points.size()<<endl;
        std::vector<Sector, Eigen::aligned_allocator<Sector>> sectors;
        int sector_number_array[numberOfSectors];
        for(int k=0; k <= numberOfSectors; k++)
         {
            pcl::PointIndices::Ptr inliers_sector (new pcl::PointIndices);
            for(int j=0; j < cloud_ofPlane->points.size(); j++)
            {
              double theta = atan2(cloud_ofPlane->points[j].y, cloud_ofPlane->points[j].x)*180/PI;
              if(theta>=0) theta = theta;
              else theta = theta + 360;
              int sector_number = int(theta/anglePerSector);
              if(sector_number == k)
              {
              inliers_sector->indices.push_back(j);
              //cout<<"one point matched"<<k<<"in"<<"with"<<theta<<endl;
              }
            }
            Sector sector;
            sector.indices = inliers_sector;
            sector.sector_number = k;
            if(inliers_sector->indices.size() > 15) sector.indicate = false;
            else sector.indicate = true;
            sectors.push_back(sector);
            //cout<<"size of the sector"<<sector.indices->indices.size()<<endl;
            //cout<<"size of the computed"<<inliers_sector->indices.size()<<endl;

         }
         Plane plane;
         plane.sectors = sectors;
         Planes.push_back(plane);

          cout<<"size of sector "<<sectors[3].indices->indices.size()<<endl;






      }
      cout<<"total_number of indices in plane 0" << plane_inliers_pointer[0]->indices.size()<<endl;
      for(int i = 0; i < numberOfSectors; i++)
      {
      cout<<"size of indices in "<<i<<"th sector of each plane " << Planes[0].sectors[i].indices->indices.size() << " index is "<<Planes[0].sectors[i].indicate<<endl;
      }
      cout<<"total_number of indices in plane 0" << plane_inliers_pointer[1]->indices.size()<<endl;
      for(int i = 0; i < numberOfSectors; i++)
      {
      cout<<"size of indices in "<<i<<"th sector of each plane " << Planes[1].sectors[i].indices->indices.size()<<" index is "<<Planes[1].sectors[i].indicate<<endl;
      }
      cout<<"total_number of indices in plane 0" << plane_inliers_pointer[2]->indices.size()<<endl;
      for(int i = 0; i < numberOfSectors; i++)
      {
      cout<<"size of indices in "<<i<<"th sector of each plane " << Planes[2].sectors[i].indices->indices.size()<<" index is "<<Planes[2].sectors[i].indicate<<endl;
      }
      cout<<"total_number of indices in plane 0" << plane_inliers_pointer[3]->indices.size()<<endl;
      for(int i = 0; i < numberOfSectors; i++)
      {
      cout<<"size of indices in "<<i<<"th sector of each plane " << Planes[3].sectors[i].indices->indices.size()<<" index is "<<Planes[3].sectors[i].indicate<<endl;
      }
      cout<<"total_number of indices in plane 0" << plane_inliers_pointer[4]->indices.size()<<endl;
      for(int i = 0; i < numberOfSectors; i++)
      {
      cout<<"size of indices in "<<i<<"th sector of each plane " << Planes[4].sectors[i].indices->indices.size()<<" index is "<<Planes[4].sectors[i].indicate<<endl;
      }
      cout<<"total_number of indices in plane 0" << plane_inliers_pointer[5]->indices.size()<<endl;
      for(int i = 0; i < numberOfSectors; i++)
      {
      cout<<"size of indices in "<<i<<"th sector of each plane " << Planes[5].sectors[i].indices->indices.size()<<" index is "<<Planes[5].sectors[i].indicate<<endl;
      }
      cout<<"total_number of indices in plane 0" << plane_inliers_pointer[6]->indices.size()<<endl;
      for(int i = 0; i < numberOfSectors; i++)
      {
      cout<<"size of indices in "<<i<<"th sector of each plane " << Planes[6].sectors[i].indices->indices.size()<<" index is "<<Planes[6].sectors[i].indicate<<endl;
      }
      cout<<"total_number of indices in plane 0" << plane_inliers_pointer[7]->indices.size()<<endl;
      for(int i = 0; i < numberOfSectors; i++)
      {
      cout<<"size of indices in "<<i<<"th sector of each plane " << Planes[7].sectors[i].indices->indices.size()<<" index is "<<Planes[7].sectors[i].indicate<<endl;
      }

      std::vector<Candidate, Eigen::aligned_allocator<Candidate>> Candidates;
      std::vector<bool> bool_string;
      Candidate candidate;
      bool flag_start = false;
      for(int i=0; i < numberOfSectors; i++)
      {
        bool _indicate = true;
        for (int j=0 ; j < Planes.size() - 3; j++)
        {
          _indicate &= Planes[j].sectors[i].indicate;
        }
        bool_string.push_back(_indicate);
        if(bool_string[i])
        {
          if(flag_start == false)
          {
            candidate.begin = i;
            flag_start = true;
          }
        }
        else if(flag_start&&(!bool_string[i]))
        {
            candidate.end = i-1;
            flag_start = false;
            candidate.phi = (candidate.begin*anglePerSector + candidate.end*anglePerSector)/2;
            Candidates.push_back(candidate);
            Candidate candidate;
        }
      }

      for(int i= 0 ; i < Candidates.size(); i++)
      {
        cout<<"Candidate Begin for "<<i<<" is "<<Candidates[i].begin<<endl;
        cout<<"Candidate End for "<<i<<" is "<<Candidates[i].end<<endl;
        cout<<"Candidate End for "<<i<<" is "<<Candidates[i].phi<<endl;

      }
      cout<<hop.x<<", "<<hop.y<<endl;
      float preVecPhi = atan2((hop.y-hop.oy),(hop.x-hop.ox))*180/PI;
      if(preVecPhi>=0) hop.preVecPhi = preVecPhi;
      else hop.preVecPhi = preVecPhi + 360;
      float temp_phi;
      cout<<"Candidates size"<<Candidates.size()<<endl;
      for(int i = 0; i < Candidates.size(); i++)
      {
        if(i == 0)
        {
          temp_phi = abs(Candidates[i].phi - hop.preVecPhi);
          hop.phi = Candidates[i].phi - hop.preVecPhi;
          cout<<"hop phi is"<<hop.phi<<endl;
        }
        else{
          if(temp_phi > abs(Candidates[i].phi - hop.preVecPhi))
          {
            hop.phi = Candidates[i].phi - hop.preVecPhi;

          }
        }


      }
      hop.d = 1.5;
      hop.theta = 60;
      cout<<"hop phi is"<<hop.phi<<endl;
      hop.icp_status = true;



  }

}

void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = true)
{
  //
  // Downsample for consistency and speed
  // note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);

  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);

  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (10);
  for (int i = 0; i < 10; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

    prev = reg.getLastIncrementalTransformation ();

  }
  // Get the transformation from target to source
  targetToSource = Ti.inverse();
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  *output += *cloud_src;

  final_transform = targetToSource;
 }
