#include "FindCircle.h"

#define ONE_ROUND_RADIAN (2*M_PI)
#define ANGLE_THRLD_RATE (0.1*ONE_ROUND_RADIAN)


std::string FindCircle::generateUUID(std::string time, int id) {
    boost::uuids::name_generator gen(dns_namespace_uuid);
    time += num_to_str<int>(id);
    return num_to_str<boost::uuids::uuid>(gen(time.c_str()));
}

// Compute the orientation (in radian) of the vector from start to end points. In range [0, 2PI)
double angleBetweenPts(const std::pair<float,float> &start, const std::pair<float,float> &end) {
	double del_x = end.first - start.first ;
	double del_y = end.second - start.second ;
	double rtn = atan2(del_y,del_x);
	return (rtn>=0)? rtn : rtn+ONE_ROUND_RADIAN; // Turn it to [0, 2PI)
}

std::pair<int,int> largestDistIndices(const std::vector<STrackedObject> &circles) {
	std::pair<int,int> rtn;
	double maxDist = -INFINITY;
	for (int i = 0; i < circles.size() - 1; i++) {
		for (int j = i + 1; j < circles.size(); j++) {
			int distX = std::pow((circles[j].segment.x - circles[i].segment.x), 2);
			int distY = std::pow((circles[j].segment.y - circles[i].segment.y), 2);
			double distSq = distX + distY;
			if (distSq > maxDist) {
				maxDist = distSq;
				rtn = std::pair<int,int>(i,j);
			}
		}
	}
	return rtn;
}

int nearest3DObjIx(const std::vector<STrackedObject> &circles, const float x, const float y, const float z) {
	int ix = -1;
    double minDist = INFINITY;
    for (int i = 0; i < circles.size(); i++) {
		double distX = std::pow((circles[i].x - x), 2);
		double distY = std::pow((circles[i].y - y), 2);
		double distZ = std::pow((circles[i].z - z), 2);
		double distSq = distX + distY + distZ;
		if (distSq < minDist) {
			ix = i;
			minDist = distSq;
		}
    }
    return ix;
}

std::vector<std::pair<int,double> > knn3DObjIx(const std::vector<STrackedObject> &circles, const float x, const float y, const float z) {
	std::vector<std::pair<int,double> > list;
    for (int i = 0; i < circles.size(); i++) {
		double distX = std::pow((circles[i].x - x), 2);
		double distY = std::pow((circles[i].y - y), 2);
		double distZ = std::pow((circles[i].z - z), 2);
		double distSq = distX + distY + distZ;
		std::cout << distSq << std::endl;
// 		if (distSq<KNN_DIST_THRLD)
		list.push_back(std::pair<int,double>(i,distSq));
    }
    std::sort(list.begin(),list.end(),distCompare);
	return list;
}

std::pair<float,float> midPoint(const std::vector<STrackedObject> &circles, const std::pair<int,int> &circleIx) {
	float middleX = ((circles[circleIx.first].segment.x + circles[circleIx.second].segment.x) / 2);
	float middleY = ((circles[circleIx.first].segment.y + circles[circleIx.second].segment.y) / 2);
	return std::pair<float,float>(middleX,middleY);
}

std::pair<float,float> midPoint(const std::vector<STrackedObject> &circles) {
	float sumX = 0.f;
	float sumY = 0.f;
	for (std::vector<STrackedObject>::const_iterator it=circles.begin(); it!=circles.end(); ++it) {
		sumX += it->segment.x;
		sumY += it->segment.y;
	}
	return std::pair<float,float>(sumX/circles.size(),sumY/circles.size());
}

void FindCircle::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (image->bpp != msg->step / msg->width || image->width != msg->width || image->height != msg->height) {
        delete image;
        ROS_DEBUG("Readjusting image format from %ix%i %ibpp, to %ix%i %ibpp.", image->width, image->height, image->bpp, msg->width, msg->height, msg->step / msg->width);
        image = new CRawImage(msg->width, msg->height, msg->step / msg->width);
    }

    memcpy(image->data, (void*) &msg->data[0], msg->step * msg->height);

    circle_detection::detection_results_array tracked_objects;
    tracked_objects.header = msg->header;
    visualization_msgs::MarkerArray marker_list;
	
	std::vector<STrackedObject> circles;
	
    //search image for circles
    for (int i = 0; i < MAX_PATTERNS; i++) {
        SSegment lastSegment = currentSegmentArray[i];
//         currentSegmentArray[i] = detectorArray[i]->findSegment(image, lastSegment);
        currentSegmentArray[i] = detectorArray[i]->findSegment(image, lastSegment);
        if (currentSegmentArray[i].valid) {
			STrackedObject sTrackedObject = trans->transform(currentSegmentArray[i]);
			// Filter error values
			if (isnan(sTrackedObject.x)) continue;
			if (isnan(sTrackedObject.y)) continue;
			if (isnan(sTrackedObject.z)) continue;
			if (isnan(sTrackedObject.roll)) continue;
			if (isnan(sTrackedObject.pitch)) continue;
			if (isnan(sTrackedObject.yaw)) continue;

			circles.push_back(sTrackedObject);
		}
	}
	
// 	std::cout << "found " << circles.size() << " circles\n";
	
	// Early rejection if the candidate circles are just not enough
	if (circles.size()<NUM_CIRCLES) {
		return;
	}
	
	// Search for the nearest circle to predefined center point (3D)
	const int nearestIx = nearest3DObjIx(circles,SEARCH_X,SEARCH_Y,SEARCH_Z);
	
	// Search for the knn (k=5) of the nearest point
	std::vector<std::pair<int,double> > knn = knn3DObjIx(circles,circles[nearestIx].x,circles[nearestIx].y,circles[nearestIx].z);

// 	Early rejection if the candidate circles are just not enough
// 	if (knn.size()<NUM_CIRCLES) {
// 		std::cout << "Not enough circles! " << knn.size() << std::endl;
// 		return;
// 	}
	
	std::vector<STrackedObject> knnCircles;
    for (int i = 0; i < NUM_CIRCLES; i++) {
		knnCircles.push_back(circles[knn[i].first]);
    }
	
	std::pair<int,int> firstAndThirdCircles = largestDistIndices(knnCircles); // The first and third circles
	std::pair<float,float> midXYFirstAndThird = midPoint(knnCircles, firstAndThirdCircles); // The middle point between first and third circles
	std::pair<float,float> midXYRef = midPoint(knnCircles); // The middle point of the 5 points
	double refAngle = angleBetweenPts(midXYFirstAndThird,midXYRef) - 0.5*M_PI ; // The reference line 
	if (refAngle<0) refAngle += ONE_ROUND_RADIAN; // Turn it to [0, 2PI)
//     std::cout << refAngle/M_PI*180 << std::endl;
	
	for (std::vector<STrackedObject>::iterator it=knnCircles.begin(); it!=knnCircles.end(); ++it) {
		double angleToRef = angleBetweenPts(
			std::pair<float,float>(it->segment.x,it->segment.y),midXYRef) - refAngle;
		it->segment.angleToRef = (angleToRef>=0)? angleToRef : angleToRef+ONE_ROUND_RADIAN;
	}
	
	// Sort the detected circles according to the angle to reference line
	std::sort(knnCircles.begin(),knnCircles.end(),circlePositionCompare);
	
	// Turn it back to [-PI, PI)
	const double angle_offset = refAngle>ONE_ROUND_RADIAN/2 ? refAngle-ONE_ROUND_RADIAN : refAngle ;

// 	/*
	// Sanity check: compare the second line's orientation
	// Get the angle from 3rd circle to 4th circle
	double angle_0to2 = angleBetweenPts(
		std::pair<float,float>((knnCircles.begin())->segment.x,(knnCircles.begin())->segment.y),
		std::pair<float,float>((knnCircles.begin()+2)->segment.x,(knnCircles.begin()+2)->segment.y)
	);
	double angle_4to3 = angleBetweenPts(
		std::pair<float,float>((knnCircles.begin()+4)->segment.x,(knnCircles.begin()+4)->segment.y),
		std::pair<float,float>((knnCircles.begin()+3)->segment.x,(knnCircles.begin()+3)->segment.y)
	);
	if (abs(angle_0to2-angle_4to3)>ANGLE_THRLD_RATE) {
		std::cout << "lines doesn't allign!" << std::endl;
		return;
	}
// 	*/
	
	int i =0;
	int personId = 0;
	for (std::vector<STrackedObject>::iterator it=knnCircles.begin(); it!=knnCircles.end(); ++it) {
		double angle = ((it->segment.angle-angle_offset)/M_PI+1)/2*numCommands;
		int digit = (int)(floor(angle+0.5)+2) % numCommands;
		personId += digit * pow(4,i);
		
// 		std::cout << i << ": " << it->segment.angleToRef/M_PI*180 << " " << it->segment.bwRatio << " " << digit << std::endl;
		
		// temp value to hold current detection
		circle_detection::detection_results objectsToAdd;

		// Convert to ROS standard Coordinate System
		objectsToAdd.pose.position.x = -(*it).y;
		objectsToAdd.pose.position.y = -(*it).z;
		objectsToAdd.pose.position.z = (*it).x;
		// Convert YPR to Quaternion
		tf::Quaternion q;
		q.setRPY((*it).roll, (*it).pitch, (*it).yaw);
		objectsToAdd.pose.orientation.x = q.getX();
		objectsToAdd.pose.orientation.y = q.getY();
		objectsToAdd.pose.orientation.z = q.getZ();
		objectsToAdd.pose.orientation.w = q.getW();

		// This needs to be replaced with a unique label as ratio is unreliable
		objectsToAdd.bwratio = (*it).bwratio;
		objectsToAdd.circleId = i++;
		objectsToAdd.digit = digit;

		// Hardcoded values need to be replaced
		objectsToAdd.objectsize.x = 3;
		objectsToAdd.objectsize.y = 3;
		objectsToAdd.objectsize.z = 0;

		tracked_objects.tracked_objects.push_back(objectsToAdd);		
	}

	tracked_objects.personId = personId;
	
	if(tracked_objects.tracked_objects.size()!=NUM_CIRCLES) {
		return;
    }

    if (tracked_objects.tracked_objects.size() > 0) {
		std::cout << "publish: " << personId <<std::endl;
		std::cout << "--" << std::endl;
        pub.publish(tracked_objects);
    }
    memcpy((void*) &msg->data[0], image->data, msg->step * msg->height);
    imdebug.publish(msg);
}

void FindCircle::cameraInfoCallBack(const sensor_msgs::CameraInfo &msg) {
    trans->updateParams(msg.K[2], msg.K[5], msg.K[0], msg.K[4]);
}

FindCircle::FindCircle(void) {
    nh = new ros::NodeHandle("~");
    defaultImageWidth = 640;
    defaultImageHeight = 480;
    circleDiameter = 0.049;
    startup_time_str = num_to_str<double>(ros::Time::now().toSec());
}

FindCircle::~FindCircle(void) {
    delete image;
    for (int i = 0; i < MAX_PATTERNS; i++) delete detectorArray[i];
// 	delete detector;
    delete trans;
}

void FindCircle::init(int argc, char* argv[]) {
    if (!nh->getParam("image_input_topic", this->im_topic)) {
        throw std::invalid_argument("image_input_topic not defined");
    }

    if (!nh->getParam("image_output_topic", this->debug_topic)) {
        throw std::invalid_argument("image_output_topic not defined");
    }

    if (!nh->getParam("camera_info", this->cam_info)) {
        throw std::invalid_argument("camera_info not defined");
    }

    if (!nh->getParam("results_topic", this->result_topic)) {
        throw std::invalid_argument("results_topic not defined");
    }

    if (!nh->getParam("marker_topic", this->viz_topic)) {
        throw std::invalid_argument("marker_topic not defined");
    }

    image_transport::ImageTransport it(*nh);
    nh->subscribe(this->cam_info, 1, &FindCircle::cameraInfoCallBack, this);
    image = new CRawImage(defaultImageWidth, defaultImageHeight, 4);
    trans = new CTransformation(circleDiameter, nh);
    for (int i = 0; i < MAX_PATTERNS; i++) {
        detectorArray[i] = new CCircleDetect(defaultImageWidth, defaultImageHeight);
    }
//     detector = new CCircleDetect(defaultImageWidth, defaultImageHeight);

    image->getSaveNumber();
    image_transport::Subscriber subim = it.subscribe(this->im_topic, 1, &FindCircle::imageCallback, this);

    imdebug = it.advertise(this->debug_topic, 1);
    pub = nh->advertise<circle_detection::detection_results_array>(this->result_topic, 0);
    vis_pub = nh->advertise<visualization_msgs::MarkerArray>(this->viz_topic, 0);
    lookup = new tf::TransformListener();
    ROS_DEBUG("Server running");
    ros::spin();
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "circle_detector", ros::init_options::AnonymousName);

    FindCircle *detector = new FindCircle();

    //Attempt to start detector
    detector->init(argc, argv);
    //Clean up
    detector->~FindCircle();
    return 0;

}
