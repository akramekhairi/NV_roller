#include <mapper_emvs/data_loading.hpp>
#include <mapper_emvs/mapper_emvs.hpp>

#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <chrono>
#include <algorithm> // For std::min and std::max
#include <iostream>  // For std::cout
#include <iomanip>   // For std::fixed and std::setprecision
#include <fstream>   // For file output

// Input parameters
DEFINE_string(bag_filename, "input.bag", "Path to the rosbag");
DEFINE_string(event_topic, "/dvs/events", "Name of the event topic (default: /dvs/events)");
DEFINE_string(pose_topic, "/dvs/pose", "Name of the pose topic (default: /dvs/pose)");
DEFINE_string(camera_info_topic, "/dvs/camera_info", "Name of the camera info topic (default: /dvs/camera_info)");
DEFINE_int64(start_event_idx, 0, "The index of the first event to process (default: 0)");
DEFINE_int32(num_events, 25000, "The number of events to process (default: 25000)");

// Disparity Space Image (DSI) parameters. Section 5.2 in the IJCV paper.
DEFINE_int32(dimX, 0, "X dimension of the voxel grid (if 0, will use the X dim of the event camera) (default: 0)");
DEFINE_int32(dimY, 0, "Y dimension of the voxel grid (if 0, will use the Y dim of the event camera) (default: 0)");
DEFINE_int32(dimZ, 100, "Z dimension of the voxel grid (default: 100) must be <= 256");
DEFINE_double(fov_deg, 0.0, "Field of view of the DSI, in degrees (if < 10, will use the FoV of the event camera) (default: 0.0)");
DEFINE_double(min_depth, 0.3, "Min depth, in meters (default: 0.3)");
DEFINE_double(max_depth, 5.0, "Max depth, in meters (default: 5.0)");

// Depth map parameters (selection and noise removal). Section 5.2.3 in the IJCV paper.
DEFINE_int32(adaptive_threshold_kernel_size, 5, "Size of the Gaussian kernel used for adaptive thresholding. (default: 5)");
DEFINE_double(adaptive_threshold_c, 5., "A value in [0, 255]. The smaller the noisier and more dense reconstruction (default: 5.)");
DEFINE_int32(median_filter_size, 5, "Size of the median filter used to clean the depth map. (default: 5)");

// Point cloud parameters (noise removal). Section 5.2.4 in the IJCV paper.
DEFINE_double(radius_search, 0.05, "Size of the radius filter. (default: 0.05)");
DEFINE_int32(min_num_neighbors, 3, "Minimum number of points for the radius filter. (default: 3)");


int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  // Load all events, poses, and camera intrinsics from the rosbag
  sensor_msgs::CameraInfo camera_info_msg;
  std::vector<dvs_msgs::Event> all_events;
  std::map<ros::Time, geometry_utils::Transformation> all_poses;
  data_loading::parse_rosbag(FLAGS_bag_filename, all_events, all_poses, camera_info_msg,
                             FLAGS_event_topic, FLAGS_camera_info_topic, FLAGS_pose_topic);

  // Slice the events based on the command line arguments
  CHECK_GT(all_events.size(), 0) << "No events loaded from bag.";
  CHECK_GE(FLAGS_start_event_idx, 0) << "Start event index must be non-negative.";
  CHECK_GT(FLAGS_num_events, 0) << "Number of events must be positive.";
  const size_t start_idx = static_cast<size_t>(FLAGS_start_event_idx);
  const size_t end_idx = start_idx + static_cast<size_t>(FLAGS_num_events);
  CHECK_LT(start_idx, all_events.size()) << "Start event index is out of bounds.";
  CHECK_LE(end_idx, all_events.size()) << "End event index is out of bounds. Requested "
                                       << FLAGS_num_events << " events from index "
                                       << FLAGS_start_event_idx << " but only "
                                       << all_events.size() << " total events are available.";

  std::vector<dvs_msgs::Event> events_slice(all_events.begin() + start_idx, all_events.begin() + end_idx);
  
  ros::Time t_start = events_slice.front().ts;
  ros::Time t_end = events_slice.back().ts;
  
  image_geometry::PinholeCameraModel cam;
  cam.fromCameraInfo(camera_info_msg);
  
  LinearTrajectory trajectory = LinearTrajectory(all_poses);
  
  // This is the reference pose we want to save
  geometry_utils::Transformation T_w_rv;
  ros::Time t_ref = ros::Time(0.5 * (t_start.toSec() + t_end.toSec()));
  trajectory.getPoseAt(t_ref, T_w_rv);
  geometry_utils::Transformation T_rv_w = T_w_rv.inverse();
  
  EMVS::ShapeDSI dsi_shape(FLAGS_dimX, FLAGS_dimY, FLAGS_dimZ,
                           FLAGS_min_depth, FLAGS_max_depth,
                           FLAGS_fov_deg);
  EMVS::MapperEMVS mapper(cam, dsi_shape);
  
  mapper.evaluateDSI(events_slice, trajectory, T_rv_w);
  
  EMVS::OptionsDepthMap opts_depth_map;
  opts_depth_map.adaptive_threshold_kernel_size_ = FLAGS_adaptive_threshold_kernel_size;
  opts_depth_map.adaptive_threshold_c_ = FLAGS_adaptive_threshold_c;
  opts_depth_map.median_filter_size_ = FLAGS_median_filter_size;
  cv::Mat depth_map, confidence_map, semidense_mask;
  mapper.getDepthMapFromDSI(depth_map, confidence_map, semidense_mask, opts_depth_map);

  // ... (Average depth calculation and image saving code is unchanged) ...
  double sum_of_depths = 0.0;
  int valid_pixel_count = 0;
  for (int y = 0; y < depth_map.rows; ++y) {
    for (int x = 0; x < depth_map.cols; ++x) {
      if (semidense_mask.at<uchar>(y, x) > 0) {
        sum_of_depths += depth_map.at<float>(y, x);
        valid_pixel_count++;
      }
    }
  }
  double average_depth = (valid_pixel_count > 0) ? (sum_of_depths / valid_pixel_count) : 0.0;
  cv::imwrite("semidense_mask.png", 255 * semidense_mask);
  cv::Mat confidence_map_255;
  cv::normalize(confidence_map, confidence_map_255, 0, 255.0, cv::NORM_MINMAX, CV_32FC1);
  cv::imwrite("confidence_map.png", confidence_map_255);
  cv::Mat depthmap_8bit = cv::Mat::zeros(depth_map.rows, depth_map.cols, CV_8U);
  const float range = FLAGS_max_depth - FLAGS_min_depth;
  CHECK_GT(range, 0.0f) << "max_depth must be greater than min_depth.";
  for (int y = 0; y < depth_map.rows; ++y) {
    for (int x = 0; x < depth_map.cols; ++x) {
      if (semidense_mask.at<uchar>(y, x) > 0) {
        float depth = depth_map.at<float>(y, x);
        float clamped_depth = std::max((float)FLAGS_min_depth, std::min(depth, (float)FLAGS_max_depth));
        float normalized_val = (clamped_depth - FLAGS_min_depth) / range;
        depthmap_8bit.at<uchar>(y, x) = static_cast<uchar>(normalized_val * 255.0);
      }
    }
  }
  cv::imwrite("depth_map.png", depthmap_8bit);
  cv::Mat depthmap_color;
  cv::applyColorMap(depthmap_8bit, depthmap_color, cv::COLORMAP_RAINBOW);
  cv::Mat depth_on_canvas = cv::Mat(depth_map.rows, depth_map.cols, CV_8UC3, cv::Scalar(255,255,255));
  depthmap_color.copyTo(depth_on_canvas, semidense_mask);
  cv::imwrite("depth_colored.png", depth_on_canvas);

  // --- Convert to point cloud ---
  EMVS::OptionsPointCloud opts_pc;
  opts_pc.radius_search_ = FLAGS_radius_search;
  opts_pc.min_num_neighbors_ = FLAGS_min_num_neighbors;
  EMVS::PointCloud::Ptr pc (new EMVS::PointCloud);
  mapper.getPointcloud(depth_map, semidense_mask, opts_pc, pc);
  
  // Save point cloud to disk
  pcl::io::savePCDFileASCII ("pointcloud.pcd", *pc);
  LOG(INFO) << "Saved " << pc->points.size () << " data points to pointcloud.pcd";

  // --- NEW: Save the reference pose to a text file ---
  std::ofstream pose_file("pose.txt");
  if (pose_file.is_open())
  {
    pose_file << std::fixed << std::setprecision(8) << T_w_rv.getTransformationMatrix() << std::endl;
    pose_file.close();
    LOG(INFO) << "Saved reference camera pose to pose.txt";
  }
  else
  {
    LOG(ERROR) << "Unable to open pose.txt for writing.";
  }

  // --- Print the average depth to stdout for the Python script to capture ---
  google::LogToStderr();
  std::cout << std::fixed << std::setprecision(8) << average_depth << std::endl;

  return 0;
}