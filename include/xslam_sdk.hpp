#pragma once

#include "slam_export.hpp"
#include <string>


/**
 * \defgroup xslam_sdk SDK interface
 * @{
 */

/**
 * @brief Available SDK Mode: to run the algorithm on the HOST PC or on the XVisio EDGE device using xslam_start_vo();
 */
enum SLAM_EXPORT xslam_sdk_mode
{
  SDK_HOST = 0,//default
  SDK_EDGE,
  //SDK_3DOF_EDGE,
  //SDK_3DOF_HOST
};

/**
 * @brief Available ToF framerates
 */
enum SLAM_EXPORT xslam_tof_framerate
{
  TOF_5Hz = 0,
  TOF_10Hz,
  TOF_15Hz,
  TOF_20Hz,
  TOF_25Hz,
  TOF_30Hz
};

/**
 * @brief Available ToF mode
 * default is TOF_LONG_RANGE
 */
enum SLAM_EXPORT xslam_tof_mode
{
  TOF_SHORT_RANGE = 0,
  TOF_LONG_RANGE,
};

/*
* @brief Available rgb resolutions
*/
enum SLAM_EXPORT xslam_rgb_resolution
{
   RGB_640x480 = 0,
   RGB_1280x720,
   RGB_1920x1080
};

/**
 * @brief RGB codec
 */
enum SLAM_EXPORT xslam_rgb_codec
{
  YUYV = 0,
  YUV420p,
  JPEG
};

/**
 * @brief RGB mode
 */
enum SLAM_EXPORT xslam_rgb_mode
{
  RGB_PREVIEW = 0,
  RGB_CAPTURE,
  RGB_VIDEO
};

/**
 * @brief IMU mode
 */
enum SLAM_EXPORT xslam_imu_mode
{
    IMU_MODE_0 = 0,
    IMU_MODE_1,
    IMU_MODE_2,
    IMU_MODE_3,
};

/**
 * @brief xslam possible status
 */
enum SLAM_EXPORT xslam_status
{
  // Failure
  failure = 0,

  // Success
  success,

  // Loading map failed: the specified map file was not found on the filesystem.
  map_not_found_on_filesystem,

  // the offline loop closure cannot be executed because the map is empty
  loop_closure_empty_map,

  // Loop-closure failed: VO algorithm not even started
  loop_closure_algo_uninitialized,
  // Loop-closure succeed: but some frames were not well localized so the map could be incomplete

  // The localization get lost during the map build, so it possible that the map is incomplete
  loop_closure_map_could_be_incomplete_because_lost_frames,

  // Loading sequence failed: the replay sequence cannot be loaded. 
  sequence_not_found,

  // xslam_set_option is used with an unsupported option
  not_supported,

  // xslam_set_option(never_lost) cannot be enabled when wheel odometry is activated
  cannot_use_never_lost_with_odometry,

  // bad filter history size
  history_size_bad_value,

  // read_fisheyes_calibration returned default parameter because the Fisheyes are not calibrated
  fisheye_camera_not_calibrated,

  // read_rgb_calibration returned default parameter because the RGB is not calibrated
  rgb_camera_not_calibrated,

  // read_tof_calibration returned default parameter because the ToF is not calibrated
  tof_camera_not_calibrated,

  // device not found when calling xslam_start_*
  no_device_found
};

/**
 * @brief List of possible option to be used with xslam_set_option
 */
enum SLAM_EXPORT xslam_option
{
  enable_6dof_filter, // by default
  disable_6dof_filter,

  enable_static_filter, // by default
  disable_static_filter,
  // enable_low_latency: disable by default
  enable_low_latency,
  disable_low_latency,

  // enable_only_imu_6fdof : output the 6 dof at 500hz instead of 600hz (600hz = 100hz (fisheye) + 500hz (IMU))
  enable_only_imu_6dof,
  disable_only_imu_6dof,

  // enable_only_frame_6fdof : output the 6 dof at 100hz instead of 600hz (one 6 dof for each frame localization)
  enable_only_frame_6dof,
  disable_only_frame_6dof,

  // enable_cpu_intensive_accurate_vslam: more accurate, but CPU intensive (may drop frame if low CPU), becarfull when using it with fast motion
  enable_cpu_intensive_accurate_vslam,
  enable_cpu_intermediate_accurate_vslam,
  disable_cpu_intensive_accurate_vslam,
  disable_cpu_intermediate_accurate_vslam,
  // never_lost : still return the 6dof with only the update of the orientation when the visual SLAM is lost
  // this option is enabled by default when xslam_odo_callback is not used, which means we are not using a robot
  // when a robot is used, another kind of "never_lost" mode is already used by default
  enable_never_lost,
  disable_never_lost,


  // enable/disable ToF depth image median filter
  // ToF median filter is disabled by default
  enable_tof_median_filter,
  disable_tof_median_filter,

  // Enable/disable online loop closure (cannot be used with xslam_start_cslam)
  enable_online_loop_closure,
  disable_online_loop_closure
};

// ============================================================================
// Structures
// ============================================================================


/**
 * @brief 2D point
 */
struct SLAM_EXPORT xslam_2d_point
{
  float x, y;
};


/**
 * @brief 3D point
 */
struct SLAM_EXPORT xslam_3d_point
{
  float x, y, z;
};


/**
 * @brief 2D pixel
 */
struct SLAM_EXPORT xslam_pixel
{
  short x, y;
};


/**
 * @brief Localization data supplied by the visual slam.
 */
struct SLAM_EXPORT xslam_pose
{
  /// Position in world coordinates
  float x,y,z;

  /// Orientation in world coordinates [deg]
  float pitch, yaw, roll;

  /// Rotation matrix as an alternative representation of the orientation
  /// in world coordinates. Storage is row major.
  float rotation[9];

  /// Timestamp of the pose in s
  float timestamp;

  /// Confidence level between 0 and 100
  uint8_t confidence;

  /// Timestamp on the device in µs
  long long deviceTimestamp;
};

/**
 * @brief Localization data supplied by the visual slam.
 */
struct SLAM_EXPORT xslam_pose_d
{
  /// Structure version
  const int version = 2;

  /// Position in world coordinates [m]
  double x, y, z;

  /// Orientation in world coordinates [deg]
  double pitch_deg, yaw_deg, roll_deg;

  /// Orientation in world coordinates using a rotation matrix (storage is row major)
  double rotation[9];

  /// Orientation in world coordinates using a quaternion [x,y,z,w]
  double quaternion[4];

  /// Confidence level between 0 and 100
  uint8_t confidence;

  /// Best estimate of the sensor's perception-timestamp in the xslam time frame [s]
  double xTimestamp;

  /// Timestamp of the host at data reception [s]
  double hostTimestamp;

  /// Timestamp of the device [s]
  double deviceTimestamp;

#ifdef __cplusplus
  xslam_pose_d():xTimestamp(-1){}
  xslam_pose_d(xslam_pose_d const& p):version(p.version)
  {
    x = p.x;
    y = p.y;
    z = p.z;
    pitch_deg = p.pitch_deg;
    yaw_deg = p.yaw_deg;
    roll_deg = p.roll_deg;

    for(int i = 0 ; i < 9 ; ++i)
    {
      rotation[i] = p.rotation[i];
    }

    for(int i = 0 ; i < 4 ; ++i)
    {
      quaternion[i] = p.quaternion[i];
    }

    confidence = p.confidence;

    xTimestamp = p.xTimestamp;
    hostTimestamp = p.hostTimestamp;
    deviceTimestamp = p.deviceTimestamp;
  }

  xslam_pose_d& operator=(xslam_pose_d const& p)
  {
    x = p.x;
    y = p.y;
    z = p.z;
    pitch_deg = p.pitch_deg;
    yaw_deg = p.yaw_deg;
    roll_deg = p.roll_deg;

    for(int i = 0 ; i < 9 ; ++i)
    {
      rotation[i] = p.rotation[i];
    }

    for(int i = 0 ; i < 4 ; ++i)
    {
      quaternion[i] = p.quaternion[i];
    }

    confidence = p.confidence;

    xTimestamp = p.xTimestamp;
    hostTimestamp = p.hostTimestamp;
    deviceTimestamp = p.deviceTimestamp;
    return *this;
  }
#endif
};

/**
 * @brief Localization data supplied by the visual slam.
 */
struct xslam_pose_quaternion
{
  /// Timestamp of the pose in s
  float timestamp;

  /// Position in world coordinates
  float x[3];

  /// Orientation in world coordinates using quaternion representation [x,y,z,w]
  float quaternion[4];

  /// Confidence level between 0 and 100
  char confidence;

  /// reserved memory
  double reserve;

  /// Timestamp on the device in µs
  long long deviceTimestamp;
};


/**
 * @brief General pose representation as a position vector and a rotation matrix.
 */
struct xslam_transform
{
  float T[3]; ///< Translation vector
  float R[9]; ///< Rotation matrix (row major)
};


/**
 * @brief Sparse map : Map generated by the SLAM algorithm
 */
struct xslam_sparse_map
{
  xslam_transform* poses = nullptr; ///< List of poses of the map in the world coordinate frame
  size_t poses_size = 0; ///< Number of poses of the map

  xslam_3d_point* points3d = nullptr; ///< List of 3D points of the map in the world coordinate frame
  size_t points3d_size = 0; ///< Number of 3D points of the map
};


// Pinhole Model (without distortion)
struct PinholeIntrinsic
{
  float K[6];
  /*
  K[0] : fx
  K[1] : fy
  K[2] : u0
  K[3] : v0
  K[4] : image width
  K[5] : image height
  */

};

// Polynomial Distortion Model
struct PDMIntrinsic
{
  float K[11];
/**

  Projection and raytrace formula can be found here:
  https://docs.opencv.org/3.4.0/d4/d94/tutorial_camera_calibration.html

  K[0] : fx
  K[1] : fy
  K[2] : u0
  K[3] : v0
  K[4] : k1
  K[5] : k2
  K[6] : p1
  K[7] : p2
  K[8] : k3
  K[9] : image width
  K[10] : image height
*/
};

// Unified camera model
struct UnifiedIntrinsic
{
  float K[7];
/**
  Projection and raytrace formula can be found here:
  1.  C. Geyer and K. Daniilidis, “A unifying theory for central panoramic systems and practical applications,” in Proc. 6th Eur. Conf. Comput. Vis.
II (ECCV’00), Jul. 26, 2000, pp. 445–461
  or
  2. "J.P. Barreto. General central projection systems, modeling, calibration and visual
servoing. Ph.D., University of Coimbra, 2003". Section 2.2.2.

  K[0] : fx
  K[1] : fy
  K[2] : u0
  K[3] : v0
  K[4] : xi
  K[5] : image width
  K[6] : image height

  More details,
  Projection:
    The simplest camera model is represented by projection relation:    p = 1/z K X
    where p=(u v)^T is an image point, X = (x y z)^T is a spatial point to be projected
    and K is a projection matrix: K = (fx 0 u0; 0 fy v0).

    The distortion model is added in the following manner.
    First we project all the points onto the unit sphere S 
        Qs = X / ||X|| = 1/rho (x y z)   where rho = sqrt(X^2+Y^2+Z^2)
    and then we apply the perspective projection with center (0 0 -xi)^T of Qs onto plan image
        p = 1/(z/rho + xi) K (x/rho  y/rho).

  Back-projection/raytrace:
    The normalized coordinate of a pixel is (x y 1)^1.
    We know that a line joining this normalized point and the projection center intersects the unit sphere 
    at a point Qs. This point is defined as
        Qs = (eta*x  eta*y  eta-xi)
    where scale factor    eta = (xi + sqrt(1 + (x^2+y^2)(1-xi^2))) / (x^2+y^2+1).
*/
};

using Extrinsic = xslam_transform;

struct FisheyeCalibration
{
  Extrinsic extrinsic;
  UnifiedIntrinsic intrinsic;
};

struct FisheyesCalibration
{
  FisheyeCalibration calibration[2];
};

struct RGBCalibration
{
  Extrinsic extrinsic;
  PDMIntrinsic intrinsic;
};

struct ToFCalibration
{
  Extrinsic extrinsic;
  PDMIntrinsic intrinsic;
};

using ToFCalibrationPDM = ToFCalibration;

/**
 * @brief Raw stereo images data and metadata.
 */
struct SLAM_EXPORT xslam_frame
{
  /// Left image
  unsigned char const* left = nullptr;

  /// Right image
  unsigned char const* right = nullptr;

  /// Size of the images
  int height, width;

  /// Timestamp of the frame in s
  float timestamp;

  /// Timestamp on the device in µs
  long long deviceTimestamp;

  /// Pixels format
  enum format
  {
    monochrome,
    //bayer,
    rgb
  } type;
};

/**
 * @brief Raw stereo images data and metadata using double precision timestamp.
 */
struct SLAM_EXPORT xslam_frame_d
{
  /// Left image
  unsigned char const* left = nullptr;

  /// Right image
  unsigned char const* right = nullptr;

  /// Size of the images
  int height, width;

  /// Timestamp of the frame in s
  /// Best estimate of the sensor's perception-timestamp in the xslam time frame [s]
  double xTimestamp;

  /// Timestamp of the host at data reception [s]
  double hostTimestamp;

  /// Timestamp on the device [s]
  double deviceTimestamp;

  /// Pixels format
  enum format
  {
    monochrome,
    //bayer,
    rgb
  } type;
};


/**
 * @brief Raw ToF depth data and metadata.
 */
struct SLAM_EXPORT xslam_tof
{
  /// Image size
  int height, width;

  /// Depth data (row major)
  float* data;

  /// Timestamp of the frame in s
  double timestamp;

  /// Timestamp on the device in µs
  long long deviceTimestamp;
};

/**
 * @brief Raw ToF depth data and metadata using double precision timestamp
 */
struct SLAM_EXPORT xslam_tof_d
{
  /// Image size
  int height, width;

  /// Depth data (row major)
  float* data;

  /// Timestamp of the frame in s
  /// Best estimate of the sensor's perception-timestamp in the xslam time frame [s]
  double xTimestamp;

  /// Timestamp of the host at data reception [s]
  double hostTimestamp;

  /// Timestamp on the device [s]
  double deviceTimestamp;
};


/**
 * @brief Raw ToF point cloud.
 */
struct SLAM_EXPORT xslam_cloud
{
  
  /// 3D points of the cloud 
  xslam_3d_point* points;

  /// Number of 3d points
  int size;

  /// Timestamp of the frame in s
  double timestamp;

  /// Timestamp on the device in µs
  long long deviceTimestamp;
};

/**
 * @brief Raw ToF infrared data and metadata.
 */
struct SLAM_EXPORT xslam_ir
{
  /// Image size
  int height, width;

  /// Infrared data (row major)
  short* data;

  /// Timestamp of the frame in s
  double timestamp;

  /// Timestamp on the device in µs
  long long deviceTimestamp;
};

/**
 * @brief Raw RGB data and metadata.
 */
struct SLAM_EXPORT xslam_rgb
{
  /// Image size
  int height, width;

  /// RGB data (row major)
  unsigned char* data;

  /// Timestamp of the frame in s
  double timestamp;

  /// Timestamp on the device in µs
  long long deviceTimestamp;

  /// Codec use to encode the image
  xslam_rgb_codec codec = YUYV;

  /// Data size
  int dataSize;
};

/**
 * @brief Raw stereo-depth data and metadata.
 */
struct SLAM_EXPORT xslam_stereo_depth
{
  /// Image size
  int height, width;

  /// Depth data (row major)
  float* data;

  /// Timestamp of the frame in s
  double timestamp;

  /// Timestamp on the device in µs
  long long deviceTimestamp;
};

/**
 * @brief Specification of the frame of reference of the slam.
 */
enum SLAM_EXPORT xslam_coordinate_frame
{
  coordinate_frame_absolute,
  coordinate_frame_relative
};

/**
 * @brief Specification of the frame coordinate system of the xslam_pose
 */
enum SLAM_EXPORT xslam_coordinate_system
{
  right_hand,//default
  left_hand
};


/**
 * @brief Raw IMU data and metadata (Inertial Measurement Unit).
 */
struct SLAM_EXPORT xslam_imu
{
  /// Timestamp of the frame in s
  float timestamp;

  /// Gyrometer data: radian/s
  float gyro[3];

  /// Accelerometer data: meter/s/s
  float accel[3];

  /// Magnetometer data
  float magn[3];

  /// Temperature in K (<0 if invalid)
  float temperature;

  /// Timestamp on the device in µs
  long long deviceTimestamp;
};


/**
 * @brief Raw IMU data and metadata usind double precision timestamp (Inertial Measurement Unit).
 */
struct SLAM_EXPORT xslam_imu_d
{
  /// Structure version
  const int version = 2;

  /// Gyrometer data
  double gyro[3];

  /// Accelerometer data
  double accel[3];

  /// Magnetometer data
  double magn[3];

  /// Temperature in K (<0 if invalid)
  double temperature;

  /// Best estimate of the sensor's perception-timestamp in the xslam time frame [s]
  double xTimestamp;

  /// Timestamp of the host at data reception [s]
  double hostTimestamp;

  /// Timestamp on the device [s]
  double deviceTimestamp;

#ifdef __cplusplus
  xslam_imu_d():temperature(0),xTimestamp(-1),hostTimestamp(-1),deviceTimestamp(-1){}
  xslam_imu_d(xslam_imu_d const& x):version(x.version)
  {
    for(int i = 0 ; i < 3 ; ++i)
    {
      gyro[i] = x.gyro[i];
      accel[i] = x.accel[i];
      magn[i] = x.magn[i];
    }
    temperature = x.temperature;
    xTimestamp = x.xTimestamp;
    hostTimestamp = x.hostTimestamp;
    deviceTimestamp = x.deviceTimestamp;
  }

  xslam_imu_d& operator=(xslam_imu_d const& x)
  {
    for(int i = 0 ; i < 3 ; ++i)
    {
      gyro[i] = x.gyro[i];
      accel[i] = x.accel[i];
      magn[i] = x.magn[i];
    }
    temperature = x.temperature;
    xTimestamp = x.xTimestamp;
    hostTimestamp = x.hostTimestamp;
    deviceTimestamp = x.deviceTimestamp;
    return *this;
  }
#endif
};


/**
 * @brief 2D occupancy grid output.
 *
 * The center of the grid is associated to the SLAM-world's origin.
 * The horizontal axis of the grid (X) is colinear with the SLAM's X axis.
 */
struct SLAM_EXPORT xslam_occupancy_grid_2d
{
  /// Occupancy grid as an image (row major)
  float const* grid = nullptr;

  /// Size of the grid (square)
  int size;

  /// [meter] Size of a pixel
  float pixel_size;
};


/**
 * @brief Stereo dense depth algo and gridding configuration.
 */
struct SLAM_EXPORT stereo_dense_depth_config
{
  /// Method used to generate the 3D point cloud from the images:
  /// 0 -> Fast direct interpolation (fdi)
  /// 1 -> Precise iterative interpolation (pii)
  /// 2 -> Precise feedback interpolation (pfi)
  int method;

  /// [0, +inf] (typ. 65 for fdi, 30.-40. for pii/pfi) The lower, the denser is
  /// the reconstructed 3D point cloud and the slower the algo.
  float canny_threshold;

  /// (recommanded) If this optimization is activated, the interpolated dense 3D
  /// point cloud (based on SLAM inliers) is refined using the epipolar constraint.
  bool fdi_optimize_point_cloud;

  /// (available if optimize_point_cloud) If activated, the dense 3D point cloud
  /// is smoothed based on a neighborhood of 5x5 pixels. Not cheap but usefull.
  bool fdi_smooth_point_cloud;

  /// [0-10] (typ. 2-3) Number of time the depth map is resampled. The precision
  /// is theoretically improved at each iteration, but empirically stagnates after
  /// a few ones.
  int pii_iterations;

  /// [4-32] (typ. 8-16) Smaller means denser depth sampling (and slower).
  int pfi_bucket_size;

  /// [meter] Distance between the camera and the floor.
  float camera_height;

  /// [meter] Distance between the top of the robot and the floor.
  float robot_height;

  /// [meter] Maximum height of obstacles the robot can move onto without trouble.
  float robot_crossing_capability;

  /// [meter] Side lenght of the square occupancy grid. The robot starts at its center.
  float map_size;

  /// [meter] Size of a pixel.
  float pixel_size;

  /// [0,1] (typ. 0.95) Confidence above which a pixel is definetly marked as
  /// occupied or free.
  float confidence_freezing_threshold;

  /// [0,1] (typ. 0.2) Confidence in the (un)detection of obstacle. The lower,
  /// the stronger the filter that denoises the occupancy grid.
  float detection_confidence;

  /// [0,1] (typ. 0.6) Confidence drop (at each iteration) where no detection occured.
  /// The higher, the stronger the filter that denoised the occupancy grid.
  float confidence_loss_rate;

  /// [meter] (typ. 2-3) Distance from the sensor after which detections are ignored.
  float detection_distance_limit;
};



/**
 * @brief Plane having equation: x n[0] + y n[1] + z n[2] - d = 0
 */
struct SLAM_EXPORT xslam_plane
{
  /// Unit vector normal to the plane in world coordinates
  float n[3];

  /// Signed distance between the plane and the origin of the world. The distance is
  /// signed according to the direction of the normale.
  float d;

  /// Array of 3D points in world coordinates lying on the plane that describes the
  /// polygon that borders the actually detected area.
  xslam_3d_point* border;

  /// Number of 3D points
  int border_size;
};


/**
 * @brief Surface robot odometry Input or Output.
 *
 * The odometry system of coordinates is the following (top view):
 *
 *           forward
 *            X ^
 *              |
 *              |  ^
 *      Y <-----O  | + yaw rate
 *               __/
 */
struct SLAM_EXPORT xslam_odo
{
  /// Linear speed forward in meter per second
  double linear_speed_x;

  /// Linear speed to the left in meter per second (only for 4 wheels robots)
  double linear_speed_y;

  /// Yaw rate
  double yaw_angular_velocity;

  /// Best estimate of the sensor's perception-timestamp in the xslam time frame [s]
  double xTimestamp;

  /// Timestamp on the device [s]
  double deviceTimestamp;
};


/**
 * @brief The xslam_lidar struct
 *
 * Using the odometry system of coordinates.
 */
struct SLAM_EXPORT xslam_lidar
{
  int size = 0;
  double const* angle = nullptr;
  double const* distance = nullptr;
  double const* quality = nullptr;

  double first_sensor_position_x = 0;
  double first_sensor_position_y = 0;
  double first_sensor_yaw = 0;
  double first_timestamp = 0;

  double last_sensor_position_x = 0;
  double last_sensor_position_y = 0;
  double last_sensor_yaw = 0;
  double last_timestamp = 0;

  float timestamp;
  float deviceTimestamp;
};


/**
 * @brief Audio data
 */
struct SLAM_EXPORT xslam_audio
{
    /// timestamp in s
    float timestamp;
    unsigned int size = 0;
    unsigned char *data = nullptr;

    /// Timestamp on the device in µs
    long long deviceTimestamp;
};


/**
 * @brief Config structure for the 2D & 3D occupancy grids
 *
 * Both configurations are merged because the 2D grid is extracted from the 3D one.
 */
struct SLAM_EXPORT xslam_sparse_grid_config
{
  /// [m] Size of a voxel/pixel
  float voxel_size = 0.05f;

  /// [0-1] Probability threshold over which a voxel/pixel is considered as occupied
  float occupied_if_proba_over = 0.5f;

  /// [m] Requested low height threshold for 2D grid
  float min_height = 0.0f;

  /// [m] Requested high height threshold for 2D grid
  float max_height = 0.2f;
};


/**
 * @brief 2D occupancy grid
 */
struct SLAM_EXPORT xslam_sparse_grid_2d
{
  /// Occupied "pixels"
  xslam_2d_point * pixels;

  /// Number of pixels
  int size;

  /// [m] Size of a pixel (more like a flat voxel)
  float pixel_size;

  /// [m] Requested low height threshold
  float min_height;

  /// [m] Requested high height threshold
  float max_height;
};


/**
 * @brief 3D occupancy grid
 */
struct SLAM_EXPORT xslam_sparse_grid_3d
{
  /// Occupied voxels
  xslam_3d_point const * voxels;

  /// Number of voxels
  int size;

  /// [m] Size of a voxel
  float voxel_size;
};


struct SLAM_EXPORT xslam_face
{
    xslam_3d_point vertices[3];
    xslam_3d_point normals[3];
};

struct SLAM_EXPORT xslam_mesh
{
    xslam_face* faces = nullptr;
    int size = 0;
};

/**
 * @brief 3D Stream status container
 */
struct SLAM_EXPORT xslam_stream_status
{
  struct stream
  {
    bool enable = false;
    int buffer_size = 0;
    int total = 0;
    int lost = 0;
    double frequency = 0;
    double last_timestamp = 0;
  };
  
  stream fisheye;
  stream imu;
  stream raw_imu;
  stream filter_imu;
  stream edge;
  stream rgb;
  stream tof;
  stream ir;
  stream cloud;
  stream sgbm;
};

struct SLAM_EXPORT xslam_callback_status
{
  std::string name;            // callback name
  int total = 0;               // total
  double time_in_callback = 0; // seconds
  double frequency = 0;        // Hz
  double last_timestamp = 0;   // seconds
};

/**
 * @brief Visual odometry stream status
 * The size of this structure can change in future version, more stream will be added
 */
struct SLAM_EXPORT xslam_vo_stream_status
{
  struct stream
  {
    int count = 0;
    double frequency = 0;
    double last_timestamp = 0;
    double mean_process_time = 0;
  };
  
  stream fisheye;
  stream imu;
  stream stereo_planes;
  stream tof;
  stream tof_planes;
  stream surface_reconstruction;
};


// ============================================================================
// Function definitions
// ============================================================================


/**
 * @brief Return the number of detected XVisio cameras.
 */
SLAM_EXPORT int xslam_camera_is_detected();

/**
 * @brief Wait for XVisio camera.
 */
SLAM_EXPORT void xslam_wait_for_camera();

/**
 * @brief Return device serial number as a string
 */
SLAM_EXPORT std::string xslam_device_serial_number();

/**
 * \defgroup xslam_versions Functions to retrieve softwares and modules current version
 * @return the version
 * @{
 */
SLAM_EXPORT std::string xslam_sdk_version();
SLAM_EXPORT std::string xslam_full_sdk_version();
SLAM_EXPORT std::string xslam_sdk_build_version();
SLAM_EXPORT std::string xslam_firmware_version();
SLAM_EXPORT std::string xslam_platform_version();
SLAM_EXPORT std::string xslam_drivers_version();

/** @} */

/**
 * @brief Displaying the version of the SDK, build and platform
 * @return Display on the terminal
 */
SLAM_EXPORT void xslam_disp_version();

/**
 * @brief Gives the time elapsed since the program has been lunched.
 * @return a duration in seconds.
 */
SLAM_EXPORT double xslam_elapsed_time();

/**
 * @brief Gives the host time when the device booted
 * @return a duration in seconds.
 */
SLAM_EXPORT double xslam_get_host_time_at_device_boot();

/**
 * @brief Gives the time the drivers would associate to a data received on the instant.
 * @return a duration in seconds.
 */
SLAM_EXPORT double xslam_get_drivers_time();


/**
 * @brief Gives the current host time
 * @return a duration in seconds.
 */
SLAM_EXPORT double xslam_host_time_now();


/**
 * @brief Set the folder that contains recorded raw sensors data to load.
 * @param path to the folder
 * @return a status code
 */
SLAM_EXPORT xslam_status xslam_set_replay_folder(const char *path);

/**
 * @brief Set the filepath to the record.
 * @param filepath to the record
 * @return a status code
 */
SLAM_EXPORT xslam_status xslam_set_replay_file(const char *filepath);

/**
 * @brief xslam_json_config Function to specify the path of a JSON file containing some configurations (synchronisation, extrinic odometer parameters ...).
 * @param filename path of the JSON config file to use
 */
SLAM_EXPORT void xslam_json_config(const char *filename);

/**
 * @brief Set the frame rate of the ToF sensor.
 * @param framerate code
 * @return a status code
 */
SLAM_EXPORT xslam_status xslam_set_tof_framerate(xslam_tof_framerate framerate);

/**
 * @brief Set the ToF exposure time
 * @param exposure time of the ToF in micro second
 * @return a status code
 */
SLAM_EXPORT bool xslam_set_tof_exposure_time(int exposure);



/**
 * @brief Set the ToF mode
 * @param xslam_tof_mode : short distance
 * @return a status code
 */
SLAM_EXPORT bool xslam_set_tof_mode(xslam_tof_mode tof_mode);



/**
 * @brief Set the resolution of the RGB sensor.
 * @param resolution code
 * @return a status code
 */
SLAM_EXPORT xslam_status xslam_set_rgb_resolution(xslam_rgb_resolution resolution);

/**
 * @brief Set the framerate of the RGB sensor.
 * @param framerate [6:30]
 * @return a status code
 */
SLAM_EXPORT xslam_status xslam_set_rgb_framerate(int framerate);

/**
 * @brief Set the exp compensation of the RGB sensor.
 * @param framerate [-9:9]
 * @return a status code
 */
SLAM_EXPORT xslam_status xslam_set_rgb_exp_compensation(int exp);

/**
 * @brief Set the awb of the RGB sensor.
 * @param awb [0:8]
 * @return a status code
 */
SLAM_EXPORT xslam_status xslam_set_rgb_awb(int awb);

/**
 * @brief Set the mode of the RGB sensor.
 * @param mode code
 * @return a status code
 */
SLAM_EXPORT xslam_status xslam_set_rgb_mode(xslam_rgb_mode mode);

/**
 * @brief Set the mode of the IMU sensor.
 * @param mode code
 * @return a status code
 */
SLAM_EXPORT xslam_status xslam_set_imu_mode(xslam_imu_mode mode);

/**
 * @brief Set the IMU synchronization
 * @param enabled enabled or disabled
 * @return a status code
 */
SLAM_EXPORT xslam_status xslam_set_imu_sync(bool enabled);

/**
 * @brief Set the stereo offset
 * @param offset in µs [-20000:60000] steps 1000
 * @return a status code
 */
SLAM_EXPORT xslam_status xslam_set_stereo_offset(int offset);

/**
 * @brief Set the edge prediction value
 * @param prediction in µs [0:60000] steps 1000
 * @return a status code
 */
SLAM_EXPORT xslam_status xslam_set_edge_prediction_offset(int prediction);

/**
 * @brief Save all available metadata about the stereo cameras, the ToF, etc.
 * @param path to the file to register to
 * @return a status code
 */
SLAM_EXPORT xslam_status xslam_save_info(const char *path);


/**
 * \defgroup xslam_callbacks Functions to set the callbacks handling events
 * @{
 */
SLAM_EXPORT void xslam_frame_callback(void (*f)(xslam_frame*));
SLAM_EXPORT void xslam_lost_callback(void (*f)(float));
SLAM_EXPORT void xslam_edge_lost_callback(void (*f)(float));
SLAM_EXPORT void xslam_imu_callback(void (*f)(xslam_imu*));
SLAM_EXPORT void xslam_imu_d_callback(void (*f)(xslam_imu_d*));
SLAM_EXPORT void xslam_raw_imu_callback(void (*f)(xslam_imu*));
SLAM_EXPORT void xslam_raw_imu_d_callback(void (*f)(xslam_imu_d*));
SLAM_EXPORT void xslam_odo_callback(void (*f)(xslam_odo*));
SLAM_EXPORT void xslam_lidar_callback(void (*f)(xslam_lidar *));

SLAM_EXPORT void xslam_set_coordinate_system(xslam_coordinate_system coordinate_system);

/**
 * @brief Get the 6dof filtered pose (if the filter option is not disabled by xslam_set_option(disable_6dof_filter) )
 */
SLAM_EXPORT void xslam_6dof_callback(void (*f)(xslam_pose*));
SLAM_EXPORT void xslam_6dof_quaternion_callback(void (*f)(xslam_pose_quaternion*));
SLAM_EXPORT void xslam_edge_6dof_callback(void (*f)(xslam_pose*));
SLAM_EXPORT void xslam_edge_6dof_quaternion_callback(void (*f)(xslam_pose_quaternion*));

/**
 * @brief Get the pose of the camera at timestamp offset.
 * @param pose of the camera at now + prediction_timestamp
 * @param prediction_timestamp time offset (in s) to add at current time to have the pose at
 * @return If failure, the given offset timestamp is invalid (too big or too small), else success.
 */
SLAM_EXPORT xslam_status xslam_get_pose(xslam_pose* pose, double prediction_timestamp);
SLAM_EXPORT xslam_status xslam_get_pose_d(xslam_pose_d* pose, double prediction_timestamp);
SLAM_EXPORT xslam_status xslam_get_pose_quaternion(xslam_pose_quaternion* pose, double prediction_timestamp);


/**
 * @brief Get the 6dof raw pose (independantly of xslam_set_option(disable_6dof_filter/enable_6dof_filter))
 */
SLAM_EXPORT void xslam_raw_6dof_callback(void (*f)(xslam_pose*));
SLAM_EXPORT void xslam_raw_6dof_quaternion_callback(void (*f)(xslam_pose_quaternion*));

SLAM_EXPORT void xslam_tof_callback(void (*f)(xslam_tof*));
SLAM_EXPORT void xslam_cloud_callback(void (*f)(xslam_cloud*));
SLAM_EXPORT void xslam_ir_callback(void (*f)(xslam_ir*));
SLAM_EXPORT void xslam_rgb_callback(void (*f)(xslam_rgb*));
SLAM_EXPORT void xslam_stereo_depth_callback(void (*f)(xslam_stereo_depth*));
SLAM_EXPORT void xslam_raw_stereo_depth_callback(void (*f)(xslam_stereo_depth*));

/**
 * @brief Get the planes computed using the stereo camera
 */
SLAM_EXPORT void xslam_planes_callback(void (*f)(xslam_plane*, int));

/**
 * @brief Get the planes computed using the ToF camera
 */
SLAM_EXPORT void xslam_planes_tof_callback(void (*f)(xslam_plane*, int));


/**
 * @brief Get the height of the ground in the world coordinate frame of the SLAM (elevation when calling xslam_start_vo is 0)
 * @param initial_ground_to_device_distance is the initial height in meter of the device compared to the plane
 * @param tolerance_m is a tolerance distance around the ground height:
 * Using a higher value for tolerance_m is better if the initial height is uncertained and it facilitate the association between 3D points and the ground
 * The drawback of a high value is a less accuracy height estimation of the ground
 * Note that the callback is called only when the ground plane is updated.
 * Usually, if the device is not moving or if no features are seen on the ground, the groud plane is not updated
 */
SLAM_EXPORT void xslam_ground_callback(void (*f)(xslam_plane*), double initial_ground_to_device_distance_m, double tolerance_m);


/**
 * @brief Reset the ground plane while the SLAM is running
 */
void xslam_reset_ground(double h, double var);


/**
 * @brief Grab the current 2D and associated 3D features.
 */
SLAM_EXPORT void xslam_2d_3d_points_callback(void (*f)(xslam_pixel const**,
                                                       xslam_3d_point const**,
                                                       size_t const* size,
                                                       size_t n_camera));

/**
 * @brief Grab the current 2D detected features
 */
SLAM_EXPORT void xslam_2d_points_callback(void (*f)(xslam_pixel const**,
                                                    size_t const* size,
                                                    size_t nb_device,
                                                    char const*));

/**
 * @brief Get the 3D occupancy grid
 */
SLAM_EXPORT void xslam_sparse_grid3d_callback(void (*f)(xslam_sparse_grid_3d), xslam_sparse_grid_config);

/**
 * @brief Get the 2D occupancy grid
 */
SLAM_EXPORT void xslam_sparse_grid2d_callback(void (*f)(xslam_sparse_grid_2d), xslam_sparse_grid_config);


/**
 * @brief Get the sparse 3D map
 */
SLAM_EXPORT void xslam_sparse_map_callback(void (*f)(xslam_sparse_map const*));

/**
 * @brief Get the memory size allocated for the map in bytes
 * Note that the memory reserved by default for the map at starting time is about 90 MBytes
 */
SLAM_EXPORT void xslam_sparse_map_memory_size(void (*f)(size_t bytes));




SLAM_EXPORT void xslam_raw_imu_start();
SLAM_EXPORT void xslam_raw_imu_stop();

#ifdef __cplusplus
#include <functional>
#include <memory>
#include <vector>
/**
 * \defgroup xslam_callbacks_cplusplus Alternative C++ functions to set the callbacks
 * @{
 */
SLAM_EXPORT void xslam_6dof_callback(std::function<void (xslam_pose*)>);
SLAM_EXPORT void xslam_6dof_d_callback(std::function<void (xslam_pose_d*)>);
SLAM_EXPORT void xslam_edge_6dof_callback(std::function<void (xslam_pose*)>);
SLAM_EXPORT void xslam_edge_6dof_quaternion_callback(std::function<void(xslam_pose_quaternion*)>);
SLAM_EXPORT void xslam_edge_6dof_d_callback(std::function<void (xslam_pose_d*)>);
SLAM_EXPORT void xslam_6dof_quaternion_callback(std::function<void (xslam_pose_quaternion*)>);
SLAM_EXPORT void xslam_raw_6dof_callback(std::function<void (xslam_pose*)>);
SLAM_EXPORT void xslam_raw_6dof_quaternion_callback(std::function<void (xslam_pose_quaternion*)>);
SLAM_EXPORT void xslam_lidar_6dof_callback(std::function<void (xslam_pose*)>);

SLAM_EXPORT void xslam_frame_callback(std::function<void (xslam_frame*)>);
SLAM_EXPORT void xslam_frame_d_callback(std::function<void (xslam_frame_d*)>);
SLAM_EXPORT void xslam_lost_callback(std::function<void (float)>);
SLAM_EXPORT void xslam_edge_lost_callback(std::function<void (float)>);
SLAM_EXPORT void xslam_imu_callback(std::function<void (xslam_imu*)>);
SLAM_EXPORT void xslam_imu_d_callback(std::function<void (xslam_imu_d*)>);
SLAM_EXPORT void xslam_raw_imu_callback(std::function<void (xslam_imu*)>);
SLAM_EXPORT void xslam_raw_imu_d_callback(std::function<void (xslam_imu_d*)>);
SLAM_EXPORT void xslam_stereo_grid2d_callback(std::function<void (xslam_occupancy_grid_2d*)>,
                                              stereo_dense_depth_config);
SLAM_EXPORT void xslam_lidar_grid2d_callback(std::function<void (xslam_occupancy_grid_2d*)>);
SLAM_EXPORT void xslam_tof_callback(std::function<void (xslam_tof*)>);
SLAM_EXPORT void xslam_tof_d_callback(std::function<void (xslam_tof_d*)>);
SLAM_EXPORT void xslam_cloud_callback(std::function<void (xslam_cloud*)>);
SLAM_EXPORT void xslam_ir_callback(std::function<void (xslam_ir*)>);
SLAM_EXPORT void xslam_rgb_callback(std::function<void (xslam_rgb*)>);
SLAM_EXPORT void xslam_odo_callback(std::function<void (xslam_odo*)>);
SLAM_EXPORT void xslam_lidar_callback(std::function<void(xslam_lidar *)> f);
SLAM_EXPORT void xslam_audio_callback(std::function<void (xslam_audio*)>);
SLAM_EXPORT void xslam_stereo_depth_callback(std::function<void (xslam_stereo_depth*)>);
SLAM_EXPORT void xslam_raw_stereo_depth_callback(std::function<void (xslam_stereo_depth*)>);

SLAM_EXPORT void xslam_2d_points_callback(std::function<void (xslam_pixel const**,
                                                              size_t const* size,
                                                              size_t nb_device,
                                                              char const*)>);
SLAM_EXPORT void xslam_2d_3d_points_callback(std::function<void (xslam_pixel const**,
                                                                 xslam_3d_point const**,
                                                                 size_t const* size,
                                                                 size_t n_camera)>);

SLAM_EXPORT void xslam_planes_callback(std::function<void (xslam_plane*, int)>);
SLAM_EXPORT void xslam_planes_tof_callback(std::function<void (xslam_plane*, int)>);
SLAM_EXPORT void xslam_ground_callback(std::function<void (xslam_plane*)>, double initial_ground_to_device_distance, double tolerance_m);

SLAM_EXPORT std::vector<xslam_2d_point> xslam_plane_to_image(xslam_pose xpose,
                                                             xslam_plane plane,
                                                             int icamera);

SLAM_EXPORT void xslam_mesh_callback(std::function<void (xslam_mesh*)>);
SLAM_EXPORT void xslam_current_mesh_callback(std::function<void (xslam_mesh*)>);

SLAM_EXPORT void xslam_sparse_map_callback(std::function<void(xslam_sparse_map const*)> map);

SLAM_EXPORT void xslam_sparse_grid3d_callback(std::function<void (xslam_sparse_grid_3d)>, xslam_sparse_grid_config);

/**
 * @brief Load a map from a binary file and apply a functor on it
 */
SLAM_EXPORT xslam_status xslam_load_map_from_file(std::string reference_map, std::function<void(xslam_sparse_map const&)> f);

SLAM_EXPORT void xslam_sparse_map_memory_size(std::function<void (size_t)> f);

SLAM_EXPORT xslam_status xslam_set_max_number_of_keyframes(int max_size, std::function<void()> restarting_map);

// check below documentation for slam to cslam switch API
SLAM_EXPORT void xslam_localized_on_reference_map(float reference_map_usage_threshold, std::function<void(float /* percentage */)>);
SLAM_EXPORT void xslam_lost_from_reference_map(float reference_map_usage_threshold, std::function<void(float /* percentage */)>);
SLAM_EXPORT void xslam_load_map_and_switch_to_cslam(const char *filename, std::function<void(int /* status of load map */)> done_callback);
SLAM_EXPORT void xslam_save_map_and_switch_to_cslam(const char *filename, std::function<void(int /* status of save map */, int /* map quality */)> done_callback);
SLAM_EXPORT void xslam_switch_to_cslam(std::function<void(int /* map quality */)> done_callback);

/**
 * @brief provide a callback which is called each time the loop closure causes a jump on the 6dof motion
 */
SLAM_EXPORT void xslam_loop_closure_jump(std::function<void(void)> f);

// Experimental
SLAM_EXPORT void xslam_optimize_share_map_from_file(std::string input_file, std::string output_file);
SLAM_EXPORT void xslam_optimize_share_map_from_buffer(std::shared_ptr<uint8_t> input, std::shared_ptr<uint8_t>& output);
/** @} */
#endif
/** @} */


/**
 * \defgroup xslam_control_functions Functions to start or stop the slam and other algos
 * @brief VO stands for Visual Odometry (slam from scratch). CSLAM stands for Constrained
 *        SLAM (slam with a preexisting map). EDGE designates the embedded slam algo.
 * @param filename of the map to load or save
 * @param reference frame to use
 * @return a status code
 * @{
 */
SLAM_EXPORT xslam_status xslam_start_camera();
SLAM_EXPORT xslam_status xslam_start_vo();
SLAM_EXPORT xslam_status xslam_save_map(char const *filename);
SLAM_EXPORT xslam_status xslam_loop_closure_and_save(const char *filename);
SLAM_EXPORT xslam_status xslam_start_cslam(const char *filename);
SLAM_EXPORT xslam_status xslam_start_cslam(const char *filename,
                                          xslam_coordinate_frame reference);
SLAM_EXPORT xslam_status xslam_start_edge_vo();
SLAM_EXPORT xslam_status xslam_stop();
SLAM_EXPORT xslam_status xslam_free();

/**
 * @brief xslam_clear_callbacks Stop the SDK process and clear all the user callbacks
 */
SLAM_EXPORT xslam_status xslam_clear_callbacks();


SLAM_EXPORT xslam_status xslam_set_option(xslam_option option);
/** @} */


/**
 * \defgroup xslam_cslam_function Functions to switch between SLAM to CSLAM (SLAM using a known map)
 * @{
 */
/**
 * @brief xslam_switch_to_cslam Non-blocking function to start optimization to improve the map quality and then use it as an immutable reference map.
 * @param done_callback When the switch is done the callback will be called. The input of the callback is the quality result (0-100) of the reference map.
 */
SLAM_EXPORT void xslam_switch_to_cslam(void (*done_callback)(int/*map quality*/));

/**
 * @brief xslam_save_map_and_switch_to_cslam Non-blocking function to start optimization to improve the map quality, save it into a binary file, then use it as an immutable reference map.
 * @param done_callback When the switch is done the callback will be called. The input of the callback are:
 *  int status  : 2 if file is written,  -1 means error while writing the map on the disk
 *  int quality : is the quality result (0-100) of the reference map.
 */
SLAM_EXPORT void xslam_save_map_and_switch_to_cslam(const char *filename, void (*done_callback)(int /* status of save map */, int /* map quality */));

/**
 * @brief xslam_load_map_and_switch_to_cslam Non-blocking function to load a reference map from file and use it as an immutable reference map.
 * @param done_callback When the switch is done the callback will be called. The input of the callback are:
 *  int status  : 2 if file is well loaded, -1 means the file does not exists and the given name is wrong, -2 means the file exists but cannot be loaded
 */
SLAM_EXPORT void xslam_load_map_and_switch_to_cslam(const char *filename, void(*done_callback)(int /* status of load map */));

/**
 * @brief xslam_lost_from_reference_map Call the callback if the SLAM uses a reference map and get lost to often from reference map.
 * @param reference_map_usage_threshold : threshold for the percentage [0.f,1.f] of reference map usage according to whole map (reference map and dynamic map). 
 * If SLAM is not currently using enough the reference map and the usage is below this threshold, then the callback is called with the current percentage [0.f,1.f] of
 * 3D points from the reference map.
 */
SLAM_EXPORT void xslam_lost_from_reference_map(float reference_map_usage_threshold, void(*done_callback)(float /* percentage */));

/**
 * @brief xslam_localized_on_reference_map Call the callback if the SLAM uses a reference map and is localized on the reference map.
 * @param reference_map_usage_threshold : threshold for the percentage [0.f,1.f] of reference map usage according to whole map (reference map and dynamic map). 
 * If SLAM is not currently using enough the reference map and the usage is below this threshold, then the callback is called with the current percentage [0.f,1.f] of 
 * 3D points from the reference map.
 */

SLAM_EXPORT void xslam_localized_on_reference_map(float reference_map_usage_threshold, std::function<void(float /* percentage */)>);
SLAM_EXPORT void xslam_localized_on_reference_map(float reference_map_usage_threshold, void(*done_callback)(float /* percentage */));
/** @} */

/**
 * Deprecated (because of ambiguous naming) function to reset edge slam.
 * Instead, use xslam_reset_edge_slam().
*/
SLAM_EXPORT xslam_status xslam_reset_slam();

/**
 * Function to reset edge slam, clear the map and re-initialize the 6dof
*/
SLAM_EXPORT xslam_status xslam_reset_edge_slam();


/**
 * @brief Restart a new map using the last pose as the initial pose: to be used when the system is lost to re-initiate the translation tracking.
          It only works with the vo mode, not with cslam.
          Returns xslam_status::failure if the cslam is running
 */

SLAM_EXPORT xslam_status xslam_online_restart_map();


/**
 * @brief Set a max number of keyframes (referenced poses on the map).
 * Once this maximum is reached, the map is cleared; there is no impact on the 6dof output (the coordinate frame is not reset)
 * Default maximum is set at 4096
 * Minimum value of max_size is 10
 */
SLAM_EXPORT xslam_status xslam_set_max_number_of_keyframes(int max_size, void (*restarting_map)(void));

/**
 * @brief Function to set the size of the history used to filter the poses, default is 0.05 second
 * Using a bigger value will result in a smoother motion but less accurate.
 */

SLAM_EXPORT xslam_status xslam_set_filter_history_size(double t);

/**
 * @brief Function to set the prediction (current_time + t) of the 6dof xslam_pose.
 * By default t=0 gives the filtered pose at the current time
 * To predict the pose in the future, t>0
 * To get a filtered pose in the past, t<0
 * t must be included in [-0.1,0.1]
 */

SLAM_EXPORT xslam_status xslam_set_filter_prediction(double t);


/**
 * @brief Function to enable/disable the 6dof filter
 */
SLAM_EXPORT xslam_status xslam_set_option(xslam_option option);


/**
 * @brief Function to reset the origin of the coordinate frame to the current pose
 */
xslam_status xslam_reset_origin();
/** @} */

/**
 * @brief Function to display a verbose explanation of a xslam_status instance
 */
SLAM_EXPORT void xslam_status_help(xslam_status status);
/** @} */

/**
 * @brief Function to display a verbose explanation of a xslam_status instance with a prefix message
 */
SLAM_EXPORT void xslam_status_help(std::string msg, xslam_status status);
/** @} */

/**
 * @brief Function to display the current status and options enabled of the SDK
 */
SLAM_EXPORT void xslam_show_sdk_status();
/** @} */

/* @brief get the status of the stream status
 *
 */
SLAM_EXPORT xslam_status xslam_get_stream_status(xslam_stream_status& );


/* @brief get the status of the visual odometry stream status
 *
 */
SLAM_EXPORT xslam_status xslam_get_vo_stream_status(xslam_vo_stream_status& );

/* @brief get the list of status of the callback
 *
 */
SLAM_EXPORT xslam_status xslam_get_callback_status(std::vector<xslam_callback_status>& list);

/**
 * \defgroup xslam_odometry Functions associated to odometry
 * @{
 *
 * @brief Function to add an odometry measurement.
 */
SLAM_EXPORT void xslam_input_odo(xslam_odo odo);
/** @} */


/**
 * \defgroup xslam_lidar Functions associated to lidar
 * @{
 *
 * @brief Function to add an lidar measurement.
 */
SLAM_EXPORT void xslam_input_lidar(xslam_lidar lidar);
/** @} */

/**
 * \defgroup xslam_lidar Functions associated to lidar
 * @{
 *
 * @brief Function to output the 6dof pose from the liar
 */
SLAM_EXPORT void xslam_lidar_6dof_callback(void (*f)(xslam_pose*));
/** @} */

/**
 * \defgroup xslam_hid Functions read/write on HID
 * @{
 */
SLAM_EXPORT bool xslam_hid_write( const unsigned char* data, unsigned int size );
SLAM_EXPORT bool xslam_hid_read( unsigned char* data, unsigned int size );
SLAM_EXPORT bool xslam_hid_write_read( const unsigned char *writeData, unsigned int writeSize, unsigned char *readData, unsigned int readSize );
SLAM_EXPORT bool xslam_hid_write_read_timeout( const unsigned char *writeData, unsigned int writeSize, unsigned char *readData, unsigned int readSize, unsigned int delaytime_ms = 0 );
SLAM_EXPORT bool xslam_hid_get_report( unsigned char *data );
/** @} */


SLAM_EXPORT bool xslam_set_stereo_distortion_mesh( unsigned char* data, float focal_distance_in_pixels );

/**
 * @brief Function to set the auto exposure
 * Settings to get a brighter image:
 * \param high_brightness 71
 * \param low_brightness 140
 * \param max_integration 50
 * \param damping_factor 0.2
 * To get a brighter image: xslam_set_auto_exposure(71,140,50,0.2);
 * To get an even brighter image, increase the low_brightness value
 */
SLAM_EXPORT xslam_status xslam_set_auto_exposure(int highBrightness, int lowBrightness, int maxIntegration, double dampingFactor);
SLAM_EXPORT xslam_status xslam_set_auto_exposure();
SLAM_EXPORT xslam_status xslam_set_gain( int gain );
SLAM_EXPORT xslam_status xslam_set_brightness( int brightness );
SLAM_EXPORT xslam_status xslam_set_gain_exposure(int gain, int exposure);


SLAM_EXPORT void xslam_parse_arguments(int argc, char *argv[]);

/**
 * @brief Function to set the debug level
 * \param level [0,9]
 * \param top_interval_ms refresh rate of the threads loading display
 */
SLAM_EXPORT void xslam_set_debug_level(int level);
SLAM_EXPORT void xslam_set_debug_level(int level, int top_interval_ms);
SLAM_EXPORT void xslam_set_log_file(const std::string& file_path, int log_level = 9);


/**
 * @brief Function to run the algorithms on the HOST PC or on the XVisio EDGE device using xslam_start_vo();
 * \param xslam_sdk_mode SDK_HOST(default), SDK_EDGE
 * void xslam_set_sdk_mode(xslam_sdk_mode mode);
 * then call: xslam_start_vo():
 */
SLAM_EXPORT void xslam_set_sdk_mode(xslam_sdk_mode mode);

/**
 * @brief Function to get the device to sleep.
 */
SLAM_EXPORT bool xslam_sleep_device();

/**
 * @brief Function to wake up the device.
 */
SLAM_EXPORT bool xslam_wake_up_device();

/**
 * @brief Function to skip the VSLAM processing
 */
SLAM_EXPORT void xslam_disable_vslam();

/**
 * @brief Function to re-enable the VSLAM processing if it has been paused with xslam_disable_vslam
 */
SLAM_EXPORT void xslam_enable_vslam();

/**
 * @brief Specify a configuration file to use
 * Settings to get a brighter image:
 * \param filename filename
 * To get a brighter image: xslam_set_auto_exposure(71,140,50,0.2);
 * To get an even brighter image, increase the low_brightness value
 *
 * Configuration file example:
 * # mode
 * #  0 : 6dof host
 * #  1 : 6dof edge
 * mode 0
 * 
 * # top time between each display of top, 0 means never
 * top 5000
 * 
 * # debug level
 * debug 0
 * 
 * # enable/disable motion filter; default is ON
 * filter 1
 * 
 * # filter prediction in second [-0.1,0.1]; default is 0
 * filter_prediction 0
 * 
 * # low_latency
 * low_latency 1
 * 
 */  
SLAM_EXPORT void xslam_set_configuration_file(std::string filename);


SLAM_EXPORT bool xslam_has_rgb();
SLAM_EXPORT bool xslam_has_tof();
SLAM_EXPORT bool xslam_has_audio();
SLAM_EXPORT bool xslam_has_speaker();

SLAM_EXPORT void xslam_rgb_stream_enable();
SLAM_EXPORT void xslam_rgb_stream_disable();
SLAM_EXPORT bool xslam_tof_stream_enable();
SLAM_EXPORT bool xslam_tof_stream_disable();
SLAM_EXPORT void xslam_audio_stream_enable();
SLAM_EXPORT void xslam_audio_stream_disable();
SLAM_EXPORT void xslam_speaker_stream_enable();
SLAM_EXPORT void xslam_speaker_stream_disable();

/** @} */
