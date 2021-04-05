The `klt_feature_tracker` repository is a ROS package that provides feature tracking capabilities.

This functionality is provided through a library `klt_feature_tracker` with a single function `trackFeatures()`.

To use this library in your ROS package, add the following to your CMakeLists.txt:
 - `find_package(klt_feature_tracker)`
 - `include_directories(${klt_feature_tracker_INCLUDE_DIRS})`
 - `target_link_libraries(my_node ${klt_feature_tracker_LIBRARIES})`

You can then use the function `trackFeatures(const cv::Mat &img_l, const cv::Mat &img_r, std::vector<cv::Point2f> &features_l, std::vector<cv::Point2f> &features_r, std::vector<int> &status, int stereo)`. The library is suitable for use with stereo images or monocular images.
For monocular applications, only the 'left' images and features are used.

Call the function with the following arguments:
 - `cv::Mat &img_l`: The left (or only) image as an OpenCV matrix.
 - `cv::Mat &img_r`: The right image as an OpenCV matrix. For monocular applications, this can be an empty image.
 - `std::vector<cv::Point2f> &features_l`: A vector of feature points tracked in the left image.
 - `std::vector<cv::Point2f> &features_r`: A vector of feature points tracked in the right image.
 - `std::vector<int> &status`: A vector used to communicate the status of each feature (see below).
 - `int stereo`: Select the stereo mode.
     - `stereo = 0`: Monocular mode: The right image will not be used and features_r will be meaningless.
     - `stereo = 1`: Semi-stereo mode: Newly initialized features will have both left and right image measurements, other features will only be tracked in the left image.
     - `stereo = 2`: Full-stereo mode: Features will always be tracked in both images.

The `status` vector is used to communicate with the library. It is used to request new features to be tracked and the tracker informs about the features state.

The size of `status` determines the number of features that are tracked.

The tracker interprets each entry of `status` as follows:
 - `status[i] = 0`: Feature `i` is inactive, do not track.
 - `status[i] = 1`: Feature `i` is active, track it according to `stereo`:
     - `stereo = 0` or `stereo = 1`: Track the feature over time in the left image.
     - `stereo = 2`: Track the feature over time in the left image, then track from left to right image.
 - `status[i] = 2`: Feature `i` is to be initialized according to `stereo`:
     - `stereo = 0`: Initialize a feature using only `img_l`.
     - `stereo = 1` or `stereo = 2`: Initialize a new feature and return left and right measurements.

The tracker writes each entry of `status` as follows:
 - `status[i] = 0`: Feature `i` is inactive, not tracked. This is either because `status[i]` was 0 when the `trackFeatures` was called, or the tracker was not able to track the feature in the new image(s).
 - `status[i] = 1`: Feature `i` is active and successfully tracked according to `stereo`:
     - `stereo = 0` or `stereo = 1`: `features_l[i]` is a meaninfull measurement.
     - `stereo = 2`: `features_l[i]` and `features_l[i]` are meaninfull measurements.
 - `status[i] = 2`: A new requested feature was successfully initialized according to `stereo`:
     - `stereo = 0`: `features_l[i]` is a meaninfull measurement.
     - `stereo = 1` or `stereo = 2`: `features_l[i]` and `features_l[i]` are meaninfull measurements.
     - Note that if you request a new feature at position `i` with `status[i] = 2`, it is not guaranteed that a new feature can successfully be initialized. In case of success, `status[i]` will be left at 2, in case of failure it will be 0.
