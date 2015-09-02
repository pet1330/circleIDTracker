#ifndef __FIND_CIRCLE_H__
#define __FIND_CIRCLE_H__

#include "CCircleDetect.h"
#include "CTransformation.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/QuadWord.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include "circle_detection/detection_results.h"
#include "circle_detection/detection_results_array.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Image.h>
#include <math.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <map>
#include <string>
#include <algorithm>
#include <vector>
#include <utility>

#define MAX_PATTERNS 20
#define NUM_CIRCLES 5

#define SEARCH_X	0
#define SEARCH_Y	0
#define SEARCH_Z	0.5

#define KNN_DIST_THRLD	0.05

/*
struct COMPARE_CIRCLE_POSITION {
	bool operator() (SSegment i,SSegment j) {
		return i.angleToRef<j.angleToRef;
	} //Compare two SSegment by their angleToRef
} compare_circle_pos;
*/

bool circlePositionCompare(const STrackedObject &i,const STrackedObject &j) {
	return i.segment.angleToRef<j.segment.angleToRef;
} //Compare two SSegment by their angleToRef

// pair<id,dist>
bool distCompare(const std::pair<int,double> &i, const std::pair<int,double> &j) {
	return i.second < j.second;
}

class FindCircle {
public:

    int defaultImageWidth;
    int defaultImageHeight;
    float circleDiameter;
    const static int numCommands = 4;
    

    void cameraInfoCallBack(const sensor_msgs::CameraInfo &msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void init(int argc, char* argv[]);
    FindCircle(void);
    ~FindCircle(void);

private:
    // Generate universally unique ID
    template<typename T>
    std::string num_to_str(T num) {std::stringstream ss; ss << num; return ss.str();}
    std::string startup_time_str;
    boost::uuids::uuid dns_namespace_uuid;
    std::string generateUUID(std::string time, int id);

    ros::NodeHandle *nh;
    image_transport::Publisher imdebug;
    tf::TransformListener* lookup;
    ros::Publisher pub, vis_pub;
    std::string im_topic, viz_topic, result_topic, debug_topic, cam_info;

    // Tracking Code
    std::clock_t start;
    CRawImage *image;
    CCircleDetect *detectorArray[MAX_PATTERNS];
    SSegment currentSegmentArray[MAX_PATTERNS];

    //3D transform code
    CTransformation *trans;
};

#endif
