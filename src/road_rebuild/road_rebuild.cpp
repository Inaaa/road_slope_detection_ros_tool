#include "road_rebuild.hpp"
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>// 移动立方体算法
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/visualization/pcl_visualizer.h>//可视化
#include <boost/thread/thread.hpp>//多线程
#include <string.h>
#include <string>
#include <pcl/common/common_headers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/don.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <limits>



namespace road_slope_detection_ros_tool {

/**
 * Initialization
 */
RoadRebuild::RoadRebuild(const ros::NodeHandle& nhPrivate) : interface_{nhPrivate}, reconfigureServer_{nhPrivate} {

    interface_.fromParamServer();

    /*
     * Set up callbacks for subscribers and reconfigure.
     *
     * New subscribers can be created with "add_subscriber" in "cfg/RoadRebuild.if file.
     * Don't forget to register your callbacks here!
     */
    reconfigureServer_.setCallback(boost::bind(&RoadRebuild::reconfigureCallback, this, _1, _2));
    interface_.dummy_subscriber->registerCallback(&RoadRebuild::messageCallback, this);

    rosinterface_handler::showNodeInfo();

    /*
     * The preferred way of logging is to call the logging functions of the interface object.
     * These also work for nodelets and are better than using ROS_DEBUG (etc.) macros.
     */
    interface_.logDebug("Node initialized.");
}

/**
 * This callback is called whenever a new message arrives on the subscriber's topic.
 * To make this work, the callback has to be registered in the constructor above.
 */
void RoadRebuild::messageCallback(const Msg::ConstPtr& msg) {
    // Transform the message to the reference frame
    Msg msgTransformed;
    try {
        // Msg is just a std_msgs::Header so the transformation currently makes no sense:
        // msgTransformed = tfBuffer_.transform(*msg, interface_.reference_frame);
    } catch (tf2::TransformException& e) {
        interface_.logDebug("Failed to transform the message to " + interface_.reference_frame);
        return;
    }
    // TODO: do stuff to process the message
    Msg::Ptr newMsg = boost::make_shared<Msg>(*msg);
    interface_.dummy_publisher.publish(newMsg);
}

/**
 * This callback is called at startup or whenever a change was made in the dynamic_reconfigure window.
 * The parameter "level" is unused here. It is set to the number of changes in the config.
 * At startup, the callback is automatically called with level == std::numeric_limits<uint32_t>::max().
 */
void RoadRebuild::reconfigureCallback(const Interface::Config& config, uint32_t /*level*/) {
    interface_.fromConfig(config);
}


/**template<typename PointT>
void
CustomVoxelGrid<PointT>::applyFilter(PointCloud &output) {
    // Has the input dataset been set already?
    if (!input_) {
        PCL_WARN ("[pcl::%s::applyFilter] No input dataset given!\n", getClassName().c_str());
        output.width = output.height = 0;
        output.points.clear();
        return;
    }

    // Copy the header (and thus the frame_id) + allocate enough space for points
    output.height = 1;                    // downsampling breaks the organized structure
    output.is_dense = true;                 // we filter out invalid points

    Eigen::Vector4f min_p, max_p;
    // Get the minimum and maximum dimensions
    if (!filter_field_name_.empty()) // If we don't want to process the entire cloud...
        pcl::getMinMax3D<PointT>(input_, *indices_, filter_field_name_, static_cast<float> (filter_limit_min_),
                                 static_cast<float> (filter_limit_max_), min_p, max_p, filter_limit_negative_);
    else
        pcl::getMinMax3D<PointT>(*input_, *indices_, min_p, max_p);

    // Check that the leaf size is not too small, given the size of the data
    std::int64_t dx = static_cast<std::int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0]) + 1;
    std::int64_t dy = static_cast<std::int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1]) + 1;
    std::int64_t dz = static_cast<std::int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2]) + 1;

    if ((dx * dy * dz) > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())) {
        PCL_WARN("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.",
                 getClassName().c_str());
        output = *input_;
        return;
    }

    // Compute the minimum and maximum bounding box values
    min_b_[0] = static_cast<int> (std::floor(min_p[0] * inverse_leaf_size_[0]));
    max_b_[0] = static_cast<int> (std::floor(max_p[0] * inverse_leaf_size_[0]));
    min_b_[1] = static_cast<int> (std::floor(min_p[1] * inverse_leaf_size_[1]));
    max_b_[1] = static_cast<int> (std::floor(max_p[1] * inverse_leaf_size_[1]));
    min_b_[2] = static_cast<int> (std::floor(min_p[2] * inverse_leaf_size_[2]));
    max_b_[2] = static_cast<int> (std::floor(max_p[2] * inverse_leaf_size_[2]));

    // Compute the number of divisions needed along all axis
    div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
    div_b_[3] = 0;

    // Set up the division multiplier
    divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

    // Storage for mapping leaf and pointcloud indexes
    std::vector<cloud_point_index_idx> index_vector;
    index_vector.reserve(indices_->size());

    // If we don't want to process the entire cloud, but rather filter points far away from the viewpoint first...
    if (!filter_field_name_.empty()) {
        // Get the distance field index
        std::vector<pcl::PCLPointField> fields;
        int distance_idx = pcl::getFieldIndex<PointT>(filter_field_name_, fields);
        if (distance_idx == -1)
            PCL_WARN ("[pcl::%s::applyFilter] Invalid filter field name. Index is %d.\n", getClassName().c_str(),
                      distance_idx);

        // First pass: go over all points and insert them into the index_vector vector
        // with calculated idx. Points with the same idx value will contribute to the
        // same point of resulting CloudPoint
        for (std::vector<int>::const_iterator it = indices_->begin(); it != indices_->end(); ++it) {
            if (!input_->is_dense)
                // Check if the point is invalid
                if (!std::isfinite(input_->points[*it].x) ||
                    !std::isfinite(input_->points[*it].y) ||
                    !std::isfinite(input_->points[*it].z))
                    continue;

            // Get the distance value
            const std::uint8_t *pt_data = reinterpret_cast<const std::uint8_t *> (&input_->points[*it]);
            float distance_value = 0;
            memcpy(&distance_value, pt_data + fields[distance_idx].offset, sizeof(float));

            if (filter_limit_negative_) {
                // Use a threshold for cutting out points which inside the interval
                if ((distance_value < filter_limit_max_) && (distance_value > filter_limit_min_))
                    continue;
            } else {
                // Use a threshold for cutting out points which are too close/far away
                if ((distance_value > filter_limit_max_) || (distance_value < filter_limit_min_))
                    continue;
            }

            int ijk0 = static_cast<int> (std::floor(input_->points[*it].x * inverse_leaf_size_[0]) -
                                         static_cast<float> (min_b_[0]));
            int ijk1 = static_cast<int> (std::floor(input_->points[*it].y * inverse_leaf_size_[1]) -
                                         static_cast<float> (min_b_[1]));
            int ijk2 = static_cast<int> (std::floor(input_->points[*it].z * inverse_leaf_size_[2]) -
                                         static_cast<float> (min_b_[2]));

            // Compute the centroid leaf index
            int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
            index_vector.emplace_back(static_cast<unsigned int> (idx), *it);
        }
    }
        // No distance filtering, process all data
    else {
        // First pass: go over all points and insert them into the index_vector vector
        // with calculated idx. Points with the same idx value will contribute to the
        // same point of resulting CloudPoint
        for (std::vector<int>::const_iterator it = indices_->begin(); it != indices_->end(); ++it) {
            if (!input_->is_dense)
                // Check if the point is invalid
                if (!std::isfinite(input_->points[*it].x) ||
                    !std::isfinite(input_->points[*it].y) ||
                    !std::isfinite(input_->points[*it].z))
                    continue;

            int ijk0 = static_cast<int> (std::floor(input_->points[*it].x * inverse_leaf_size_[0]) -
                                         static_cast<float> (min_b_[0]));
            int ijk1 = static_cast<int> (std::floor(input_->points[*it].y * inverse_leaf_size_[1]) -
                                         static_cast<float> (min_b_[1]));
            int ijk2 = static_cast<int> (std::floor(input_->points[*it].z * inverse_leaf_size_[2]) -
                                         static_cast<float> (min_b_[2]));

            // Compute the centroid leaf index
            int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
            index_vector.emplace_back(static_cast<unsigned int> (idx), *it);
        }
    }

    // Second pass: sort the index_vector vector using value representing target cell as index
    // in effect all points belonging to the same output cell will be next to each other
    auto rightshift_func = [](const cloud_point_index_idx &x, const unsigned offset) { return x.idx >> offset; };
    boost::sort::spreadsort::integer_sort(index_vector.begin(), index_vector.end(), rightshift_func);

    // Third pass: count output cells
    // we need to skip all the same, adjacent idx values
    unsigned int total = 0;
    unsigned int index = 0;
    // first_and_last_indices_vector[i] represents the index in index_vector of the first point in
    // index_vector belonging to the voxel which corresponds to the i-th output point,
    // and of the first point not belonging to.
    std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
    // Worst case size
    first_and_last_indices_vector.reserve(index_vector.size());
    while (index < index_vector.size()) {
        unsigned int i = index + 1;
        while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx)
            ++i;
        if (i - index >= min_points_per_voxel_) {
            ++total;
            first_and_last_indices_vector.emplace_back(index, i);
        }
        index = i;
    }

    // Fourth pass: compute centroids, insert them into their final position
    output.points.resize(total);
    if (save_leaf_layout_) {
        try {
            // Resizing won't reset old elements to -1.  If leaf_layout_ has been used previously, it needs to be re-initialized to -1
            std::uint32_t new_layout_size = div_b_[0] * div_b_[1] * div_b_[2];
            //This is the number of elements that need to be re-initialized to -1
            std::uint32_t reinit_size = std::min(static_cast<unsigned int> (new_layout_size),
                                                 static_cast<unsigned int> (leaf_layout_.size()));
            for (std::uint32_t i = 0; i < reinit_size; i++) {
                leaf_layout_[i] = -1;
            }
            leaf_layout_.resize(new_layout_size, -1);
        }
        catch (std::bad_alloc &) {
            throw pcl::PCLException("VoxelGrid bin size is too low; impossible to allocate memory for layout",
                                    "voxel_grid.hpp", "applyFilter");
        }
        catch (std::length_error &) {
            throw pcl::PCLException("VoxelGrid bin size is too low; impossible to allocate memory for layout",
                                    "voxel_grid.hpp", "applyFilter");
        }
    }

    index = 0;
    for (const auto &cp : first_and_last_indices_vector) {
        // calculate centroid - sum values from all input points, that have the same idx value in index_vector array
        unsigned int first_index = cp.first;
        unsigned int last_index = cp.second;

        // index is centroid final position in resulting PointCloud
        if (save_leaf_layout_)
            leaf_layout_[index_vector[first_index].idx] = index;

// BEGIN CHANGE

        Eigen::Vector4f minimum = input_->points[index_vector[first_index].cloud_point_index].getVector4fMap();
        for (unsigned int li = first_index+1; li < last_index; ++li) {
            if (input_->points[index_vector[li].cloud_point_index].getVector4fMap()[2] < minimum[2]) {
                minimum = input_->points[index_vector[li].cloud_point_index].getVector4fMap();
            }
        }
        output.points[index].getVector4fMap() = minimum;

        ++index;

// END CHANGE
    }
    output.width = static_cast<std::uint32_t> (output.points.size());
}

**/


void
CustomVoxelGrid::applyFilter (PCLPointCloud2 &output)
{
  // If fields x/y/z are not present, we cannot downsample
  if (x_idx_ == -1 || y_idx_ == -1 || z_idx_ == -1)
  {
    //PCL_ERROR ("[pcl::%s::applyFilter] Input dataset doesn't have x-y-z coordinates!\n", getClassName ().c_str ());
    output.width = output.height = 0;
    output.data.clear ();
    return;
  }
  std::size_t nr_points  = input_->width * input_->height;

  // Copy the header (and thus the frame_id) + allocate enough space for points
  output.height         = 1;                    // downsampling breaks the organized structure
  if (downsample_all_data_)
  {
    output.fields       = input_->fields;
    output.point_step   = input_->point_step;
  }
  else
  {
    output.fields.resize (4);

    output.fields[0] = input_->fields[x_idx_];
    output.fields[0].offset = 0;

    output.fields[1] = input_->fields[y_idx_];
    output.fields[1].offset = 4;

    output.fields[2] = input_->fields[z_idx_];
    output.fields[2].offset = 8;

    output.point_step = 12;
  }
  output.is_bigendian = input_->is_bigendian;
  output.row_step     = input_->row_step;
  output.is_dense     = true;                 // we filter out invalid points

  Eigen::Vector4f min_p, max_p;
  // Get the minimum and maximum dimensions
  if (!filter_field_name_.empty ()) // If we don't want to process the entire cloud...
    getMinMax3D (input_, x_idx_, y_idx_, z_idx_, filter_field_name_,
                 static_cast<float> (filter_limit_min_),
                 static_cast<float> (filter_limit_max_), min_p, max_p, filter_limit_negative_);
  else
    getMinMax3D (input_, x_idx_, y_idx_, z_idx_, min_p, max_p);

  // Check that the leaf size is not too small, given the size of the data
  std::int64_t dx = static_cast<std::int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0])+1;
  std::int64_t dy = static_cast<std::int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1])+1;
  std::int64_t dz = static_cast<std::int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2])+1;

  if( (dx*dy*dz) > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()) )
  {
    //PCL_WARN("[pcl::%s::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.", getClassName().c_str());
    //output.width = output.height = 0;
    //output.data.clear();
    //return;
  }

  // Compute the minimum and maximum bounding box values
  min_b_[0] = static_cast<int> (std::floor (min_p[0] * inverse_leaf_size_[0]));
  max_b_[0] = static_cast<int> (std::floor (max_p[0] * inverse_leaf_size_[0]));
  min_b_[1] = static_cast<int> (std::floor (min_p[1] * inverse_leaf_size_[1]));
  max_b_[1] = static_cast<int> (std::floor (max_p[1] * inverse_leaf_size_[1]));
  min_b_[2] = static_cast<int> (std::floor (min_p[2] * inverse_leaf_size_[2]));
  max_b_[2] = static_cast<int> (std::floor (max_p[2] * inverse_leaf_size_[2]));

  // Compute the number of divisions needed along all axis
  div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
  div_b_[3] = 0;

  std::vector<cloud_point_index_idx> index_vector;
  index_vector.reserve (nr_points);

  // Create the first xyz_offset, and set up the division multiplier
  Array4size_t xyz_offset (input_->fields[x_idx_].offset,
                           input_->fields[y_idx_].offset,
                           input_->fields[z_idx_].offset,
                           0);
  divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);
  Eigen::Vector4f pt  = Eigen::Vector4f::Zero ();

  int centroid_size = 4;
  int rgba_index = -1;

  if (downsample_all_data_)
  {
    centroid_size = static_cast<int> (input_->fields.size ());

    // ---[ RGB special case
    // if the data contains "rgba" or "rgb", add an extra field for r/g/b in centroid
    for (int d = 0; d < centroid_size; ++d)
    {
      if (input_->fields[d].name == std::string ("rgba") || input_->fields[d].name == std::string ("rgb"))
      {
        rgba_index = d;
        centroid_size += 4;
        break;
      }
    }
  }

  // If we don't want to process the entire cloud, but rather filter points far away from the viewpoint first...
  if (!filter_field_name_.empty ())
  {
    // Get the distance field index
    int distance_idx = pcl::getFieldIndex (*input_, filter_field_name_);

    // @todo fixme
    if (input_->fields[distance_idx].datatype != pcl::PCLPointField::FLOAT32)
    {
      //PCL_ERROR ("[pcl::%s::applyFilter] Distance filtering requested, but distances are not float/double in the dataset! Only FLOAT32/FLOAT64 distances are supported right now.\n", getClassName ().c_str ());
      output.width = output.height = 0;
      output.data.clear ();
      return;
    }

    // First pass: go over all points and insert them into the index_vector vector
    // with calculated idx. Points with the same idx value will contribute to the
    // same point of resulting CloudPoint
    float distance_value = 0;
    for (std::size_t cp = 0; cp < nr_points; ++cp)
    {
      std::size_t point_offset = cp * input_->point_step;
      // Get the distance value
      memcpy (&distance_value, &input_->data[point_offset + input_->fields[distance_idx].offset], sizeof (float));

      if (filter_limit_negative_)
      {
        // Use a threshold for cutting out points which inside the interval
        if (distance_value < filter_limit_max_ && distance_value > filter_limit_min_)
        {
          xyz_offset += input_->point_step;
          continue;
        }
      }
      else
      {
        // Use a threshold for cutting out points which are too close/far away
        if (distance_value > filter_limit_max_ || distance_value < filter_limit_min_)
        {
          xyz_offset += input_->point_step;
          continue;
        }
      }

      // Unoptimized memcpys: assume fields x, y, z are in random order
      memcpy (&pt[0], &input_->data[xyz_offset[0]], sizeof (float));
      memcpy (&pt[1], &input_->data[xyz_offset[1]], sizeof (float));
      memcpy (&pt[2], &input_->data[xyz_offset[2]], sizeof (float));

      // Check if the point is invalid
      if (!std::isfinite (pt[0]) ||
          !std::isfinite (pt[1]) ||
          !std::isfinite (pt[2]))
      {
        xyz_offset += input_->point_step;
        continue;
      }

      int ijk0 = static_cast<int> (std::floor (pt[0] * inverse_leaf_size_[0]) - min_b_[0]);
      int ijk1 = static_cast<int> (std::floor (pt[1] * inverse_leaf_size_[1]) - min_b_[1]);
      int ijk2 = static_cast<int> (std::floor (pt[2] * inverse_leaf_size_[2]) - min_b_[2]);
      // Compute the centroid leaf index
      int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
      index_vector.emplace_back(idx, static_cast<unsigned int> (cp));

      xyz_offset += input_->point_step;
    }
  }
  // No distance filtering, process all data
  else
  {
    // First pass: go over all points and insert them into the right leaf
    for (std::size_t cp = 0; cp < nr_points; ++cp)
    {
      // Unoptimized memcpys: assume fields x, y, z are in random order
      memcpy (&pt[0], &input_->data[xyz_offset[0]], sizeof (float));
      memcpy (&pt[1], &input_->data[xyz_offset[1]], sizeof (float));
      memcpy (&pt[2], &input_->data[xyz_offset[2]], sizeof (float));

      // Check if the point is invalid
      if (!std::isfinite (pt[0]) ||
          !std::isfinite (pt[1]) ||
          !std::isfinite (pt[2]))
      {
        xyz_offset += input_->point_step;
        continue;
      }

      int ijk0 = static_cast<int> (std::floor (pt[0] * inverse_leaf_size_[0]) - min_b_[0]);
      int ijk1 = static_cast<int> (std::floor (pt[1] * inverse_leaf_size_[1]) - min_b_[1]);
      int ijk2 = static_cast<int> (std::floor (pt[2] * inverse_leaf_size_[2]) - min_b_[2]);
      // Compute the centroid leaf index
      int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
      index_vector.emplace_back(idx, static_cast<unsigned int> (cp));
      xyz_offset += input_->point_step;
    }
  }

  // Second pass: sort the index_vector vector using value representing target cell as index
  // in effect all points belonging to the same output cell will be next to each other
  auto rightshift_func = [](const cloud_point_index_idx &x, const unsigned offset) { return x.idx >> offset; };
  boost::sort::spreadsort::integer_sort(index_vector.begin(), index_vector.end(), rightshift_func);

  // Third pass: count output cells
  // we need to skip all the same, adjacenent idx values
  std::size_t total = 0;
  std::size_t index = 0;
  while (index < index_vector.size ())
  {
    std::size_t i = index + 1;
    while (i < index_vector.size () && index_vector[i].idx == index_vector[index].idx)
      ++i;
    ++total;
    index = i;
  }

  // Fourth pass: compute centroids, insert them into their final position
  output.width = std::uint32_t (total);
  output.row_step = output.point_step * output.width;
  output.data.resize (output.width * output.point_step);

  if (save_leaf_layout_)
  {
    try
    {
      // Resizing won't reset old elements to -1.  If leaf_layout_ has been used previously, it needs to be re-initialized to -1
      std::uint32_t new_layout_size = div_b_[0]*div_b_[1]*div_b_[2];
      //This is the number of elements that need to be re-initialized to -1
      std::uint32_t reinit_size = std::min (static_cast<unsigned int> (new_layout_size), static_cast<unsigned int> (leaf_layout_.size()));
      for (std::uint32_t i = 0; i < reinit_size; i++)
      {
        leaf_layout_[i] = -1;
      }
      leaf_layout_.resize (new_layout_size, -1);
    }
    catch (std::bad_alloc&)
    {
      throw pcl::PCLException("VoxelGrid bin size is too low; impossible to allocate memory for layout",
        "voxel_grid.cpp", "applyFilter");
    }
    catch (std::length_error&)
    {
      throw pcl::PCLException("VoxelGrid bin size is too low; impossible to allocate memory for layout",
        "voxel_grid.cpp", "applyFilter");
    }
  }

  // If we downsample each field, the {x,y,z}_idx_ offsets should correspond in input_ and output
  if (downsample_all_data_)
    xyz_offset = Array4size_t (output.fields[x_idx_].offset,
                               output.fields[y_idx_].offset,
                               output.fields[z_idx_].offset,
                               0);
  else
    // If not, we must have created a new xyzw cloud
    xyz_offset = Array4size_t (0, 4, 8, 12);

  index=0;
  Eigen::VectorXf centroid = Eigen::VectorXf::Zero (centroid_size);
  Eigen::VectorXf temporary = Eigen::VectorXf::Zero (centroid_size);

  for (std::size_t cp = 0; cp < index_vector.size ();)
  {
    std::size_t point_offset = index_vector[cp].cloud_point_index * input_->point_step;
    // Do we need to process all the fields?
    if (!downsample_all_data_)
    {
      memcpy (&pt[0], &input_->data[point_offset+input_->fields[x_idx_].offset], sizeof (float));
      memcpy (&pt[1], &input_->data[point_offset+input_->fields[y_idx_].offset], sizeof (float));
      memcpy (&pt[2], &input_->data[point_offset+input_->fields[z_idx_].offset], sizeof (float));
      centroid[0] = pt[0];
      centroid[1] = pt[1];
      centroid[2] = pt[2];
      centroid[3] = 0;
    }
    else
    {
	  PCL_ERROR("Unsupported operation", "CustomVoxelGrid");
      // ---[ RGB special case
      // fill extra r/g/b centroid field
      if (rgba_index >= 0)
      {
      }
      // Copy all the fields
      for (std::size_t d = 0; d < input_->fields.size (); ++d)
        memcpy (&centroid[d], &input_->data[point_offset + input_->fields[d].offset], field_sizes_[d]);
    }

    std::size_t i = cp + 1;
    while (i < index_vector.size () && index_vector[i].idx == index_vector[cp].idx)
    {
      std::size_t point_offset = index_vector[i].cloud_point_index * input_->point_step;
      if (!downsample_all_data_)
      {
        memcpy (&pt[0], &input_->data[point_offset+input_->fields[x_idx_].offset], sizeof (float));
        memcpy (&pt[1], &input_->data[point_offset+input_->fields[y_idx_].offset], sizeof (float));
        memcpy (&pt[2], &input_->data[point_offset+input_->fields[z_idx_].offset], sizeof (float));
	if (pt[1] < centroid[1])
	{
          centroid[0] = pt[0];
	  centroid[1] = pt[1];
	  centroid[2] = pt[2];
	}
      }
      else
      {
	  PCL_ERROR("Unsupported operation", "CustomVoxelGrid");
        // ---[ RGB special case
        // fill extra r/g/b centroid field
        if (rgba_index >= 0)
        {
        }
        // Copy all the fields
        for (std::size_t d = 0; d < input_->fields.size (); ++d)
          memcpy (&temporary[d], &input_->data[point_offset + input_->fields[d].offset], field_sizes_[d]);
	if (temporary[1] < centroid[1])
	{
	  centroid += temporary;
	}
      }
      ++i;
    }

	  // Save leaf layout information for fast access to cells relative to current position
    if (save_leaf_layout_)
      leaf_layout_[index_vector[cp].idx] = static_cast<int> (index);

    // Do we need to process all the fields?
    if (!downsample_all_data_)
    {
      // Copy the data
      memcpy (&output.data[xyz_offset[0]], &centroid[0], sizeof (float));
      memcpy (&output.data[xyz_offset[1]], &centroid[1], sizeof (float));
      memcpy (&output.data[xyz_offset[2]], &centroid[2], sizeof (float));
      xyz_offset += output.point_step;
    }
    else
    {
	  PCL_ERROR("Unsupported operation", "CustomVoxelGrid");
      std::size_t point_offset = index * output.point_step;
      // Copy all the fields
      for (std::size_t d = 0; d < output.fields.size (); ++d)
        memcpy (&output.data[point_offset + output.fields[d].offset], &centroid[d], field_sizes_[d]);

      // ---[ RGB special case
      // full extra r/g/b centroid field
      if (rgba_index >= 0)
      {
        float r = centroid[centroid_size-4], g = centroid[centroid_size-3], b = centroid[centroid_size-2], a = centroid[centroid_size-1];
        int rgb = (static_cast<int> (a) << 24) | (static_cast<int> (r) << 16) | (static_cast<int> (g) << 8) | static_cast<int> (b);
        memcpy (&output.data[point_offset + output.fields[rgba_index].offset], &rgb, sizeof (float));
      }
    }
    cp = i;
    ++index;
  }
}

void voxel_grid (pcl::PCLPointCloud2Ptr cloud,
                 pcl::PCLPointCloud2Ptr cloud_filtered)
{

    //std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
     //         << " data points (" << pcl::getFieldsList (*cloud) << ").";

    // Create the filtering object
    CustomVoxelGrid customVoxelGrid;
    customVoxelGrid.setInputCloud (cloud);
    customVoxelGrid.setLeafSize (0.01f, 0.01f, 2.0f);
    customVoxelGrid.getMinBoxCoordinates();
    customVoxelGrid.filter (*cloud_filtered);

    //std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
      //        << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")."<< std::endl;

    //pcl::PCDWriter writer;
    //writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered,
    //              Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

}







} // namespace road_slope_detection_ros_tool

