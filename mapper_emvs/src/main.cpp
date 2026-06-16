#include <mapper_emvs/data_loading.hpp>
#include <mapper_emvs/mapper_emvs.hpp>

#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <chrono>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

// Input parameters
DEFINE_string(bag_filename, "input.bag", "Path to the rosbag");
DEFINE_string(event_topic, "/capture_node/events", "Name of the event topic (default: /dvs/events)");
DEFINE_string(pose_topic, "/dvs/pose", "Name of the pose topic (default: /optitrack/davis)");
DEFINE_string(camera_info_topic, "/capture_node/camera_info", "Name of the camera info topic (default: /dvs/camera_info)");
DEFINE_double(start_time_s, 0.0, "Start time in seconds (default: 0.0)");
DEFINE_double(stop_time_s, 1000.0, "Stop time in seconds (default: 1000.0)");
DEFINE_double(speed, 0.2, "Speed of motion in y direction in meters/second (default: 0.2)");


// Disparity Space Image (DSI) parameters. Section 5.2 in the IJCV paper.
DEFINE_int32(dimX, 0, "X dimension of the voxel grid (if 0, will use the X dim of the event camera) (default: 0)");
DEFINE_int32(dimY, 0, "Y dimension of the voxel grid (if 0, will use the Y dim of the event camera) (default: 0)");
DEFINE_int32(dimZ, 100, "Z dimension of the voxel grid (default: 100) must be <= 256");
DEFINE_double(fov_deg, 0.0, "Field of view of the DSI, in degrees (if < 10, will use the FoV of the event camera) (default: 0.0)");
DEFINE_double(min_depth, 0.3, "Min depth, in meters (default: 0.3)");
DEFINE_double(max_depth, 0.6, "Max depth, in meters (default: 5.0)");

// Depth map parameters (selection and noise removal). Section 5.2.3 in the IJCV paper.
DEFINE_int32(adaptive_threshold_kernel_size, 5, "Size of the Gaussian kernel used for adaptive thresholding. (default: 5)");
DEFINE_double(adaptive_threshold_c, 5., "A value in [0, 255]. The smaller the noisier and more dense reconstruction (default: 5.)");
DEFINE_int32(median_filter_size, 5, "Size of the median filter used to clean the depth map. (default: 5)");

// Point cloud parameters (noise removal). Section 5.2.4 in the IJCV paper.
DEFINE_double(radius_search, 0.05, "Size of the radius filter. (default: 0.05)");
DEFINE_int32(min_num_neighbors, 3, "Minimum number of points for the radius filter. (default: 3)");


/*
 * Load a set of events and poses from a rosbag,
 * compute the disparity space image (DSI),
 * extract a depth map (and point cloud) from the DSI,
 * and save to disk.
 */

template<typename T>
cv::Mat warpToReference(const cv::Mat& src,
                       const geometry_utils::Transformation& T_rv_w_src,
                       const geometry_utils::Transformation& T_rv_w_target,
                       const image_geometry::PinholeCameraModel& cam,
                       T invalid_value,
                       bool is_binary_mask = false,
                       const cv::Mat& depth_map = cv::Mat());

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  // Load events, poses, and camera intrinsics from the rosbag
  sensor_msgs::CameraInfo camera_info_msg;
  std::vector<dvs_msgs::Event> events;
  std::map<ros::Time, geometry_utils::Transformation> poses;
  data_loading::parse_rosbag(FLAGS_bag_filename, events, camera_info_msg,
                             FLAGS_event_topic, FLAGS_camera_info_topic, FLAGS_start_time_s, FLAGS_stop_time_s);
  // Create a camera object from the loaded intrinsic parameters
  image_geometry::PinholeCameraModel cam;
  cam.fromCameraInfo(camera_info_msg);

    // Create two poses: one at t=0 (identity) and one at t=100
  geometry_utils::Transformation T_start;
  T_start.setIdentity(); // Identity transformation at t=0

  // Create end transformation with translation in y direction
  Eigen::Vector3d translation(0.0, -FLAGS_speed * (FLAGS_stop_time_s-FLAGS_start_time_s), 0.0);
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

  // Create the transformation using the constructor
  geometry_utils::Transformation T_end(translation, Eigen::Quaterniond(rotation));

  // Add poses to the map
  poses[ros::Time(FLAGS_start_time_s)] = T_start;
  poses[ros::Time(FLAGS_stop_time_s)] = T_end;


  // Use linear interpolation to compute the camera pose for each event
  LinearTrajectory trajectory = LinearTrajectory(poses);

  // Set the position of the reference view in the middle of the trajectory
  geometry_utils::Transformation T0_, T1_;
  ros::Time t0_, t1_;
  trajectory.getFirstControlPose(&T0_, &t0_);
  trajectory.getLastControlPose(&T1_, &t1_);
  geometry_utils::Transformation T_w_rv;
  trajectory.getPoseAt(ros::Time(0.5 * (t0_.toSec() + t1_.toSec())), T_w_rv);
  // Three reference views generated for start, mid, and end of time window
  geometry_utils::Transformation T_rv_w_mid = T_w_rv.inverse();
  geometry_utils::Transformation T_rv_w_start = T0_.inverse();
  geometry_utils::Transformation T_rv_w_end = T1_.inverse();

  // Initialize the DSI
  CHECK_LE(FLAGS_dimZ, 256) << "Number of depth planes should be <= 256";
  EMVS::ShapeDSI dsi_shape(FLAGS_dimX, FLAGS_dimY, FLAGS_dimZ,
                           FLAGS_min_depth, FLAGS_max_depth,
                           FLAGS_fov_deg);

  // Initialize mapper for all reference views
  EMVS::MapperEMVS mapper_mid(cam, dsi_shape);
  EMVS::MapperEMVS mapper_start(cam, dsi_shape);
  EMVS::MapperEMVS mapper_end(cam, dsi_shape);

  // 1. Back-project events into the DSI
  std::chrono::high_resolution_clock::time_point t_start_dsi = std::chrono::high_resolution_clock::now();
  mapper_mid.evaluateDSI(events, trajectory, T_rv_w_mid);
  mapper_start.evaluateDSI(events, trajectory, T_rv_w_start);
  mapper_end.evaluateDSI(events, trajectory, T_rv_w_end);
  std::chrono::high_resolution_clock::time_point t_end_dsi = std::chrono::high_resolution_clock::now();
  auto duration_dsi = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_dsi - t_start_dsi ).count();
  LOG(INFO) << "Time to evaluate DSI: " << duration_dsi << " milliseconds";
  LOG(INFO) << "Number of events processed: " << events.size() << " events";
  LOG(INFO) << "Number of events processed per second: " << static_cast<float>(events.size()) / (1000.f * static_cast<float>(duration_dsi)) << " Mev/s";

  LOG(INFO) << "Mean square = " << mapper_mid.dsi_.computeMeanSquare();

  // Write the DSI (3D voxel grid) to disk
  mapper_mid.dsi_.writeGridNpy("dsi.npy");


  // 2. Extract semi-dense depth map from DSI
  EMVS::OptionsDepthMap opts_depth_mid;
  opts_depth_mid.adaptive_threshold_kernel_size_ = FLAGS_adaptive_threshold_kernel_size;
  opts_depth_mid.adaptive_threshold_c_ = FLAGS_adaptive_threshold_c;
  opts_depth_mid.median_filter_size_ = FLAGS_median_filter_size;
  EMVS::OptionsDepthMap opts_depth_start;
  opts_depth_start.adaptive_threshold_kernel_size_ = FLAGS_adaptive_threshold_kernel_size;
  opts_depth_start.adaptive_threshold_c_ = FLAGS_adaptive_threshold_c;
  opts_depth_start.median_filter_size_ = FLAGS_median_filter_size;
  EMVS::OptionsDepthMap opts_depth_end;
  opts_depth_end.adaptive_threshold_kernel_size_ = FLAGS_adaptive_threshold_kernel_size;
  opts_depth_end.adaptive_threshold_c_ = FLAGS_adaptive_threshold_c;
  opts_depth_end.median_filter_size_ = FLAGS_median_filter_size;
  cv::Mat depth_mid, confidence_mid, semidense_mask_mid;
  cv::Mat depth_start, confidence_start, semidense_mask_start;
  cv::Mat depth_end, confidence_end, semidense_mask_end;
  mapper_mid.getDepthMapFromDSI(depth_mid, confidence_mid, semidense_mask_mid, opts_depth_mid);
  mapper_start.getDepthMapFromDSI(depth_start, confidence_start, semidense_mask_start, opts_depth_start);
  mapper_end.getDepthMapFromDSI(depth_end, confidence_end, semidense_mask_end, opts_depth_end);

  // Warp start and end depth products into the mid reference view.
  const cv::Mat depth_start_source = depth_start.clone();
  const cv::Mat depth_end_source = depth_end.clone();
  cv::Mat depth_start_warped = warpToReference<float>(depth_start, T_rv_w_start, T_rv_w_mid, cam, 0.0f, false, depth_start_source);
  depth_start = depth_start_warped;
  cv::Mat semidense_mask_start_warped = warpToReference<uchar>(semidense_mask_start, T_rv_w_start, T_rv_w_mid, cam, 0, true, depth_start_source);
  semidense_mask_start = semidense_mask_start_warped;
  cv::Mat confidence_start_warped = warpToReference<float>(confidence_start, T_rv_w_start, T_rv_w_mid, cam, 0.0f, false, depth_start_source);
  confidence_start = confidence_start_warped;

  cv::Mat depth_end_warped = warpToReference<float>(depth_end, T_rv_w_end, T_rv_w_mid, cam, 0.0f, false, depth_end_source);
  depth_end = depth_end_warped;
  cv::Mat semidense_mask_end_warped = warpToReference<uchar>(semidense_mask_end, T_rv_w_end, T_rv_w_mid, cam, 0, true, depth_end_source);
  semidense_mask_end = semidense_mask_end_warped;
  cv::Mat confidence_end_warped = warpToReference<float>(confidence_end, T_rv_w_end, T_rv_w_mid, cam, 0.0f, false, depth_end_source);
  confidence_end = confidence_end_warped;

  // 3. Fuse depth maps with weighted averaging
  cv::Mat fused_depth = cv::Mat::zeros(depth_mid.size(), CV_32F);
  for(int y = 0; y < depth_mid.rows; y++) {
    for(int x = 0; x < depth_mid.cols; x++) {
      float depth_m = depth_mid.at<float>(y, x);
      float depth_s = depth_start_warped.at<float>(y, x);
      float depth_e = depth_end_warped.at<float>(y, x);

      fused_depth.at<float>(y, x) = (3 * depth_m + 4 * depth_s + 2 * depth_e) / 3;
    }
  }
  
  // Save depth map, confidence map and semi-dense mask

  // Save semi-dense mask as an image
  cv::imwrite("semidense_mask_mid.png", 255 * semidense_mask_mid);
  cv::imwrite("semidense_mask_start.png", 255 * semidense_mask_start);
  cv::imwrite("semidense_mask_end.png", 255 * semidense_mask_end);

  // Save confidence map as an 8-bit image
  cv::Mat confidence_mid_255;
  cv::normalize(confidence_mid, confidence_mid_255, 0, 255.0, cv::NORM_MINMAX, CV_32FC1);
  cv::imwrite("confidence_mid.png", confidence_mid_255);
  cv::Mat confidence_start_255;
  cv::normalize(confidence_start, confidence_start_255, 0, 255.0, cv::NORM_MINMAX, CV_32FC1);
  cv::imwrite("confidence_start.png", confidence_start_255);
  cv::Mat confidence_end_255;
  cv::normalize(confidence_end, confidence_end_255, 0, 255.0, cv::NORM_MINMAX, CV_32FC1);
  cv::imwrite("confidence_end.png", confidence_end_255);

  // Normalize depth map using given min and max depth values
  cv::Mat depth_mid_255 = (depth_mid - dsi_shape.min_depth_) * (255.0 / (dsi_shape.max_depth_ - dsi_shape.min_depth_));
  cv::imwrite("depth_mid.png", depth_mid_255);
  cv::Mat depth_start_255 = (depth_start - dsi_shape.min_depth_) * (255.0 / (dsi_shape.max_depth_ - dsi_shape.min_depth_));
  cv::imwrite("depth_start.png", depth_start_255);
  cv::Mat depth_end_255 = (depth_end - dsi_shape.min_depth_) * (255.0 / (dsi_shape.max_depth_ - dsi_shape.min_depth_));
  cv::imwrite("depth_end.png", depth_end_255);

  // Save pseudo-colored depth map on white canvas
  cv::Mat depthmap_8bit, depthmap_color;
  depth_mid_255.convertTo(depthmap_8bit, CV_8U);
  cv::applyColorMap(depthmap_8bit, depthmap_color, cv::COLORMAP_RAINBOW);
  cv::Mat depth_on_canvas = cv::Mat(depth_mid.rows, depth_mid.cols, CV_8UC3, cv::Scalar(1,1,1)*255);
  depthmap_color.copyTo(depth_on_canvas, semidense_mask_mid);
  cv::imwrite("depth_colored.png", depth_on_canvas);

  // New code for inverted grayscale depth map
  cv::Mat inverted_depth_mid_255 = 255.0 - ((fused_depth - dsi_shape.min_depth_) * (255.0 / (dsi_shape.max_depth_ - dsi_shape.min_depth_)));
  cv::Mat inverted_depth_mid_8bit;
  inverted_depth_mid_255.convertTo(inverted_depth_mid_8bit, CV_8U);
  cv::Mat depth_on_black_mid = cv::Mat(depth_mid.rows, depth_mid.cols, CV_8U, cv::Scalar(0));
  inverted_depth_mid_8bit.copyTo(depth_on_black_mid, semidense_mask_mid);
  cv::imwrite("depth_grayscale_mid.png", depth_on_black_mid);

  cv::Mat inverted_depth_start_255 = 255.0 - ((depth_start - dsi_shape.min_depth_) * (255.0 / (dsi_shape.max_depth_ - dsi_shape.min_depth_)));
  cv::Mat inverted_depth_start_8bit;
  inverted_depth_start_255.convertTo(inverted_depth_start_8bit, CV_8U);
  cv::Mat depth_on_black_start = cv::Mat(depth_start.rows, depth_start.cols, CV_8U, cv::Scalar(0));
  inverted_depth_start_8bit.copyTo(depth_on_black_start, semidense_mask_start);
  cv::imwrite("depth_grayscale_start.png", depth_on_black_start);

  cv::Mat inverted_depth_end_255 = 255.0 - ((depth_end - dsi_shape.min_depth_) * (255.0 / (dsi_shape.max_depth_ - dsi_shape.min_depth_)));
  cv::Mat inverted_depth_end_8bit;
  inverted_depth_end_255.convertTo(inverted_depth_end_8bit, CV_8U);
  cv::Mat depth_on_black_end = cv::Mat(depth_end.rows, depth_end.cols, CV_8U, cv::Scalar(0));
  inverted_depth_end_8bit.copyTo(depth_on_black_end, semidense_mask_end);
  cv::imwrite("depth_grayscale_end.png", depth_on_black_end);


  // 4. Convert semi-dense depth map to point cloud
  EMVS::OptionsPointCloud opts_pc;
  opts_pc.radius_search_ = FLAGS_radius_search;
  opts_pc.min_num_neighbors_ = FLAGS_min_num_neighbors;
  
  EMVS::PointCloud::Ptr pc (new EMVS::PointCloud);
  mapper_mid.getPointcloud(depth_mid, semidense_mask_mid, opts_pc, pc);
  
  // Save point cloud to disk
  pcl::io::savePCDFileASCII ("pointcloud.pcd", *pc);
  LOG(INFO) << "Saved " << pc->points.size () << " data points to pointcloud.pcd";
  
  return 0;
}

template<typename T>
cv::Mat warpToReference(const cv::Mat& src,
                       const geometry_utils::Transformation& T_rv_w_src,
                       const geometry_utils::Transformation& T_rv_w_target,
                       const image_geometry::PinholeCameraModel& cam,
                       T invalid_value,
                       bool is_binary_mask,
                       const cv::Mat& depth_map)
{
  // Get camera intrinsics
  const double fx = cam.fx();
  const double fy = cam.fy();
  const double cx = cam.cx();
  const double cy = cam.cy();
  const int width = cam.fullResolution().width;
  const int height = cam.fullResolution().height;

  // Initialize warped result and depth buffer
  cv::Mat warped_result(height, width, src.type(), cv::Scalar(invalid_value));
  cv::Mat depth_buffer = cv::Mat::zeros(height, width, CV_32F);

  geometry_utils::Transformation T_target_src = T_rv_w_target * T_rv_w_src.inverse();
  Eigen::Matrix3d R = T_target_src.getRotationMatrix();
  Eigen::Vector3d t = T_target_src.getPosition();

  for(int y = 0; y < src.rows; ++y) {
    for(int x = 0; x < src.cols; ++x) {
      // Skip invalid pixels based on data type
      if(is_binary_mask) {
        if(src.at<uchar>(y, x) == 0) continue;
      } else {
        if(src.at<float>(y, x) <= 0) continue;
      }

      const float depth = depth_map.at<float>(y, x);
      if(depth <= 0) continue;

      // Back-project to 3D in source view
      Eigen::Vector3d p_src(
          (x - cx) * depth / fx,
          (y - cy) * depth / fy,
          depth);

      // Transform to target view coordinates
      Eigen::Vector3d p_target = R * p_src + t;
      if(p_target.z() <= 0) continue;

      // Project to target image plane
      const float u = (p_target.x() * fx / p_target.z()) + cx;
      const float v = (p_target.y() * fy / p_target.z()) + cy;

      if(u >= 0 && u < width && v >= 0 && v < height) {
        const int u_i = static_cast<int>(u);
        const int v_i = static_cast<int>(v);

        // Z-buffering with depth test
        float& current_depth = depth_buffer.at<float>(v_i, u_i);
        if(current_depth == 0 || p_target.z() < current_depth) {
          current_depth = p_target.z();
          warped_result.at<T>(v_i, u_i) = src.at<T>(y, x);
        }
      }
    }
  }

  // Post-processing
  if(!is_binary_mask) {
    // Median filtering for depth/confidence maps
    cv::medianBlur(warped_result, warped_result, 3);
  }

  if(is_binary_mask) {
    warped_result = (warped_result > 0);
    warped_result.convertTo(warped_result, CV_8U, 255);
  }

  return warped_result;
}

