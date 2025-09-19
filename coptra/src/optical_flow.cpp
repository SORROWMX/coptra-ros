/*
 * Optical Flow node for PX4
 * Copyright (C) 2018 Package Authors
 *
 * Author: Package Author <author@example.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

#include <vector>
#include <cmath>
#include <algorithm>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <tf/transform_datatypes.h>
#include <tf2/exceptions.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_reconfigure/server.h>
#include <mavros_msgs/OpticalFlowRad.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <clover/FlowConfig.h>
#include <clover/OpticalFlowArduPilot.h>

using cv::Mat;

class OpticalFlow : public nodelet::Nodelet
{
public:
	OpticalFlow():
		camera_matrix_(3, 3, CV_64F)
	{}

private:
	bool enabled_;
	ros::Publisher flow_pub_, velo_pub_, shift_pub_;
	ros::Publisher optical_flow_ardupilot_pub_;  // Публикация под ArduPilot формат
	ros::Time prev_stamp_;
	std::string fcu_frame_id_, local_frame_id_;
	image_transport::CameraSubscriber img_sub_;
	image_transport::Publisher img_pub_;
	mavros_msgs::OpticalFlowRad flow_;
	int roi_px_;
	double roi_rad_;
	cv::Rect roi_;
	Mat hann_;
	Mat prev_, curr_;
	Mat camera_matrix_, dist_coeffs_;
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
	bool calc_flow_gyro_;
	float flow_gyro_default_;
	bool disable_on_vpe_;
	ros::Subscriber vpe_sub_;
	ros::Time last_vpe_time_;
	std::shared_ptr<dynamic_reconfigure::Server<clover::FlowConfig>> dyn_srv_;
	// OpenMV-like scaling and sensor id
	double scale_x_;
	double scale_y_;
	int sensor_id_;
	// Publishing controls
	bool publish_shift_;
	bool publish_debug_;
	// OpenMV-like image pipeline controls
	bool enable_resize_;
	int resize_width_;
	int resize_height_;
	bool use_hanning_;

	void onInit()
	{
		ros::NodeHandle& nh = getNodeHandle();
		ros::NodeHandle& nh_priv = getPrivateNodeHandle();
		image_transport::ImageTransport it(nh);
		image_transport::ImageTransport it_priv(nh_priv);

		tf_buffer_.reset(new tf2_ros::Buffer());
		tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_, nh));

		local_frame_id_ = nh.param<std::string>("mavros/local_position/tf/frame_id", "map");
		fcu_frame_id_ = nh.param<std::string>("mavros/local_position/tf/child_frame_id", "base_link");
		roi_px_ = nh_priv.param("roi", 128);
		roi_rad_ = nh_priv.param("roi_rad", 0.0);
		calc_flow_gyro_ = nh_priv.param("calc_flow_gyro", false);
		flow_gyro_default_ = nh_priv.param("flow_gyro_default", NAN);
		// OpenMV-like parameters
		scale_x_ = nh_priv.param("scale_x", 35.0);
		scale_y_ = nh_priv.param("scale_y", 53.0);
		sensor_id_ = nh_priv.param("sensor_id", 0);
		publish_shift_ = nh_priv.param("publish_shift", false);
		publish_debug_ = nh_priv.param("publish_debug", false);
		// OpenMV-like image pipeline params
		enable_resize_ = nh_priv.param("enable_resize", true);
		resize_width_ = nh_priv.param("resize_width", 64);
		resize_height_ = nh_priv.param("resize_height", 32);
		use_hanning_ = nh_priv.param("use_hanning", false);

		img_pub_ = it_priv.advertise("debug", 1);
		flow_pub_ = nh.advertise<mavros_msgs::OpticalFlowRad>("mavros/px4flow/raw/send", 1);
		velo_pub_ = nh_priv.advertise<geometry_msgs::TwistStamped>("angular_velocity", 1);
		shift_pub_ = nh_priv.advertise<geometry_msgs::Vector3Stamped>("shift", 1);
		optical_flow_ardupilot_pub_ = nh_priv.advertise<clover::OpticalFlowArduPilot>("optical_flow_ardupilot", 1);

		flow_.time_delta_distance_us = 0;
		flow_.distance = -1; // no distance sensor available
		flow_.temperature = 0;

		img_sub_ = it.subscribeCamera("image_raw", 1, &OpticalFlow::flow, this);

		disable_on_vpe_ = nh_priv.param("disable_on_vpe", false);
		if (disable_on_vpe_) {
			vpe_sub_ = nh.subscribe("mavros/vision_pose/pose", 1, &OpticalFlow::vpeCallback, this);
		}

		dyn_srv_ = std::make_shared<dynamic_reconfigure::Server<clover::FlowConfig>>(nh_priv);
		dynamic_reconfigure::Server<clover::FlowConfig>::CallbackType cb;

		cb = std::bind(&OpticalFlow::paramCallback, this, std::placeholders::_1, std::placeholders::_2);
		dyn_srv_->setCallback(cb);

		NODELET_INFO("Optical Flow initialized");
	}

	void parseCameraInfo(const sensor_msgs::CameraInfoConstPtr &cinfo) {
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				camera_matrix_.at<double>(i, j) = cinfo->K[3 * i + j];
			}
		}
		dist_coeffs_ = cv::Mat(cinfo->D, true);
	}

	void drawFlow(Mat& frame, double x, double y, double quality) const
	{
		double brightness = (1 - quality) * 25;;
		cv::Scalar color(brightness, brightness, brightness);
		double radius = std::sqrt(x * x + y * y);

		// draw a circle and line indicating the shift direction...
		cv::Point center(frame.cols >> 1, frame.rows >> 1);
		cv::circle(frame, center, (int)(radius*5), color, 3, cv::LINE_AA);
		cv::line(frame, center, cv::Point(center.x + (int)(x*5), center.y + (int)(y*5)), color, 3, cv::LINE_AA);
	}

	void flow(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cinfo)
	{
		if (!enabled_) return;

		if (disable_on_vpe_ &&
			!last_vpe_time_.isZero() &&
			(msg->header.stamp - last_vpe_time_).toSec() < 0.1) {
			return;
		}

		// OpenMV-like behavior: no undistort or camera model needed for dpix output
		// However, if ROI is specified in radians, we need camera intrinsics to compute ROI once

		auto img = cv_bridge::toCvShare(msg, "mono8")->image;

		if (!enable_resize_ && roi_.width == 0) { // ROI is not calculated (skip ROI if resize enabled)
			// Calculate ROI
			if (roi_rad_ != 0) {
				parseCameraInfo(cinfo);
				std::vector<cv::Point3f> object_points = {
					cv::Point3f(-sin(roi_rad_ / 2), -sin(roi_rad_ / 2), cos(roi_rad_ / 2)),
					cv::Point3f(sin(roi_rad_ / 2), sin(roi_rad_ / 2), cos(roi_rad_ / 2)),
				};

				std::vector<double> vec { 0, 0, 0 };
				std::vector<cv::Point2f> img_points;
				cv::projectPoints(object_points, vec, vec, camera_matrix_, dist_coeffs_, img_points);

				roi_ = cv::Rect(cv::Point2i(round(img_points[0].x), round(img_points[0].y)), 
					cv::Point2i(round(img_points[1].x), round(img_points[1].y)));

				ROS_INFO("ROI: %d %d - %d %d ", roi_.tl().x, roi_.tl().y, roi_.br().x, roi_.br().y);

			} else if (roi_px_ != 0) {
				roi_ = cv::Rect((msg->width / 2 - roi_px_ / 2), (msg->height / 2 - roi_px_ / 2), roi_px_, roi_px_);
			}
		}

		if (!enable_resize_ && roi_.width != 0) { // ROI is set and resize disabled
			// Apply ROI
			img = img(roi_);
		}

		if (enable_resize_) {
			cv::Mat small;
			cv::resize(img, small, cv::Size(resize_width_, resize_height_), 0, 0, cv::INTER_AREA);
			small.convertTo(curr_, CV_32F);
		} else {
			img.convertTo(curr_, CV_32F);
		}

		if (prev_.empty() || (msg->header.stamp - prev_stamp_).toSec() > 0.1
			|| (prev_.size() != curr_.size())) { // outdated previous frame or size changed
			prev_ = curr_.clone();
			prev_stamp_ = msg->header.stamp;
			if (use_hanning_) cv::createHanningWindow(hann_, curr_.size(), CV_32F);

		} else {
			double response;
			cv::Point2d shift = use_hanning_ ? cv::phaseCorrelate(prev_, curr_, hann_, &response)
										  : cv::phaseCorrelate(prev_, curr_, cv::Mat(), &response);

			// Publish raw shift in pixels
			geometry_msgs::Vector3Stamped shift_vec;
			shift_vec.header.stamp = msg->header.stamp;
			shift_vec.header.frame_id = msg->header.frame_id;
			shift_vec.vector.x = shift.x;
			shift_vec.vector.y = shift.y;
			if (publish_shift_) shift_pub_.publish(shift_vec);

			// OpenMV-like dpix output: scale and sign to match OpenMV script behavior
			// sub_pixel_x = int(-x_translation * scale_x), sub_pixel_y = int(y_translation * scale_y)
			auto clamp_int16 = [](double v) -> int16_t {
				if (v > 32767.0) return 32767;
				if (v < -32768.0) return -32768;
				return static_cast<int16_t>(std::lrint(v));
			};
			int16_t flow_x_dpix = clamp_int16(-shift.x * scale_x_);
			int16_t flow_y_dpix = clamp_int16( shift.y * scale_y_);
			uint8_t quality = static_cast<uint8_t>(std::min(255.0, std::max(0.0, response * 255.0)));

			// Publish fields exactly as OpenMV expects via ArduPilot-compatible message
			clover::OpticalFlowArduPilot optical_flow_ap_msg;
			optical_flow_ap_msg.header.stamp = msg->header.stamp;
			optical_flow_ap_msg.header.frame_id = msg->header.frame_id;
			// OpenMV does not send rates, set to zero
			optical_flow_ap_msg.flow_rate_x = 0.0f;
			optical_flow_ap_msg.flow_rate_y = 0.0f;
			// Pixel shifts (dpix)
			optical_flow_ap_msg.flow_x = flow_x_dpix;
			optical_flow_ap_msg.flow_y = flow_y_dpix;
			// Quality and sensor id
			optical_flow_ap_msg.quality = quality;
			optical_flow_ap_msg.sensor_id = static_cast<uint8_t>(sensor_id_);
			optical_flow_ardupilot_pub_.publish(optical_flow_ap_msg);

			// Update previous frame
			prev_ = curr_.clone();
			prev_stamp_ = msg->header.stamp;

			// Debug image
			if (publish_debug_ && img_pub_.getNumSubscribers() > 0) {
				// publish debug image
				cv::Mat dbg8;
				if (enable_resize_) {
					curr_.convertTo(dbg8, CV_8U);
				} else {
					dbg8 = img.clone();
				}
				drawFlow(dbg8, shift_vec.vector.x, shift_vec.vector.y, response);
				cv_bridge::CvImage out_msg;
				out_msg.header.frame_id = msg->header.frame_id;
				out_msg.header.stamp = msg->header.stamp;
				out_msg.encoding = sensor_msgs::image_encodings::MONO8;
				out_msg.image = dbg8;
				img_pub_.publish(out_msg.toImageMsg());
			}
		}
	}

	geometry_msgs::Vector3Stamped calcFlowGyro(const std::string& frame_id, const ros::Time& prev, const ros::Time& curr)
	{
		tf2::Quaternion prev_rot, curr_rot;
		tf2::fromMsg(tf_buffer_->lookupTransform(frame_id, local_frame_id_, prev).transform.rotation, prev_rot);
		tf2::fromMsg(tf_buffer_->lookupTransform(frame_id, local_frame_id_, curr, ros::Duration(0.1)).transform.rotation, curr_rot);

		geometry_msgs::Vector3Stamped flow;
		flow.header.frame_id = frame_id;
		flow.header.stamp = curr;
		// https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Quaternion_↔_angular_velocities
		auto diff = ((curr_rot - prev_rot) * prev_rot.inverse()) * 2.0f;
		flow.vector.x = -diff.x();
		flow.vector.y = -diff.y();
		flow.vector.z = -diff.z();

		return flow;
	}

	void paramCallback(clover::FlowConfig &config, uint32_t level)
	{
		enabled_ = config.enabled;
		if (!enabled_) {
			prev_ = Mat(); // clear previous frame
		}
	}

	void vpeCallback(const geometry_msgs::PoseStamped& vpe) {
		last_vpe_time_ = vpe.header.stamp;
	}
};

PLUGINLIB_EXPORT_CLASS(OpticalFlow, nodelet::Nodelet)
