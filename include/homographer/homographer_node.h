#ifndef STITCHTRON9000_EXTRACTOR_NODE_H
#define STITCHTRON9000_EXTRACTOR_NODE_H

#include <ros/ros.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include "stitchtron9000/Homography.h"
#include "stitchtron9000/KeyFrame.h"

namespace s9000 {
namespace homographer {

class HomographerNode {
	public:
		HomographerNode(const ros::NodeHandle& pnh);
		void print_homography() const;

	private:
		void keyframe_cb(const stitchtron9000::KeyFrame& msg);

		ros::NodeHandle pnh_; // node
		ros::Subscriber sub_keyframe_;  // subscriber to get keyframes
		ros::Publisher pub_homography_; // publisher to publish homography

		// data structures to store new keyframe
		stitchtron9000::Homography hom_curr_; // current homography matrix

		// data structures to store data from previous keyframe
		std::vector<cv::Point2f> points_prev_;	// previous feature coordinates
		std::vector<int> id_prev_;				// previous indices
};


} // end namespace extractor
} // end namespace s9000

#endif // STITCHTRON9000_HOMOGRAPHER_NODE_H 
