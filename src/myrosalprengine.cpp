/**
 * @file myrosalprengine.cpp
 * @author Sukrut Rao (cs15btech11036@iith.ac.in)
 * @brief Implements subscriber node to perform license plate recognition
 *        using OpenALPR
 * @version 0.0.1
 * @date 2019-05-05
 * 
 * @copyright Copyright (c) 2019, Sukrut Rao. All rights reserved.
 * This source code is licensed under the BSD-style license found in the
 * LICENSE file in the root directory of this source tree. 
 * 
 */

#include <unistd.h>
#include <fstream>
#include <sstream>
#include <thread>
#include <string>
#include <cmath>
#include <alpr.h>
#include <cstdlib>
#include <assert.h>

#include <experimental/filesystem>
#include <sys/time.h>
#include <csignal>
#include <ros/xmlrpc_manager.h>

#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>

/**
 * @brief Stores data for a single license plate
 * 
 */
struct MyPlate {

	std::vector<float> confidences;
	std::vector<std::string> characters;
	float top_confidence;
	std::string top_plate;
	int requested_topn;
	int bbox[4];

	/**
	 * @brief Initializes member variables to empty values
	 * 
	 */
	MyPlate();

	/**
	 * @brief Stores the bbox in (top_left_x, top_left_y, width, height) format
	 *        given coordinates in a alpr::AlprCoordinate array
	 * 
	 * @param coordinates	Array of alpr::AlprCoordinate containing coordinates 
	 * @param size	Size of array, should always be 4
	 */
	void set_bbox(alpr::AlprCoordinate *coordinates, int size);

	/**
	 * @brief Writes plate data to file in JSON format
	 * 
	 * @param file_obj	std::ofstream file object to write to
	 */
	void toJSONFile(std::ofstream &file_obj);
};

MyPlate::MyPlate() {
	confidences.clear();
	characters.clear();
	top_confidence = 0.0;
	top_plate = "";
	requested_topn = 0;
	bbox[0] = bbox[1] = bbox[2] = bbox[3] = 0;
}

void MyPlate::set_bbox(alpr::AlprCoordinate *coordinates, int size) {
	assert(size == 4 && "Size must be equal to 4");
	bbox[0] = std::min(coordinates[0].x, coordinates[3].x);
	bbox[1] = std::min(coordinates[0].y, coordinates[1].y);
	bbox[2] = std::max(coordinates[1].x, coordinates[2].x)
				- std::min(coordinates[0].x, coordinates[3].x);
	bbox[3] = std::max(coordinates[2].y, coordinates[3].y)
				- std::min(coordinates[0].y, coordinates[1].y);
}

void MyPlate::toJSONFile(std::ofstream &file_obj) {
	file_obj << "\t\t\t{\n";
	file_obj << "\t\t\t\t\"candidates\": [\n";
	for (int i = 0; i < confidences.size(); i++) {
		file_obj << "\t\t\t\t\t{\n";
		file_obj << "\t\t\t\t\t\t\"confidence\": " << confidences[i] << ",\n";
		file_obj << "\t\t\t\t\t\t\"plate\": \"" << characters[i] << "\"\n";
		file_obj << "\t\t\t\t\t}";
		if (i != confidences.size()-1)
			file_obj << ",";
		file_obj << "\n";
	}
	file_obj << "\t\t\t\t],\n";
	file_obj << "\t\t\t\t\"bbox\": [";
	for (int i = 0; i < 4; i++) {
		file_obj << bbox[i];
		if (i != 3)
			file_obj << ", ";
	}
	file_obj << "],\n";
	file_obj << "\t\t\t\t\"confidence\": " << top_confidence << ",\n";
	file_obj << "\t\t\t\t\"plate\": \"" << top_plate << "\",\n";
	file_obj << "\t\t\t\t\"requested_topn\": " << requested_topn << "\n";
	file_obj << "\t\t\t}";
}

/**
 * @brief Stores data for a single image
 * 
 */
struct MyResult {
	std::string name;
	std::vector<MyPlate> plates;

	/**
	 * @brief Initializes member variables
	 * 
	 */
	MyResult();

	/**
	 * @brief Construct object and initializes with given arguments
	 * 
	 * @param name	Image frame ID
	 * @param plates Vector of license plates detected in the image
	 */
	MyResult(std::string name, std::vector<MyPlate> plates);

	/**
	 * @brief Writes plate data to file in JSON format
	 * 
	 * @param file_obj	std::ofstream file object to write to
	 */
	void toJSONFile(std::ofstream &file_obj);
};

MyResult::MyResult() {
	name = "";
	plates.clear();
}

MyResult::MyResult(std::string name, std::vector<MyPlate> plates) {
	this->name = name;
	this->plates = plates;
}

void MyResult::toJSONFile(std::ofstream &file_obj) {
	file_obj << "\t{\n";
	file_obj << "\t\t\"name\": \"" << name << "\",\n";
	file_obj << "\t\t\"results\": [\n";
	for (int i = 0; i < plates.size(); i++) {
		plates[i].toJSONFile(file_obj);
		if (i != plates.size()-1)
			file_obj << "\t\t\t,\n";
	}
	file_obj << "\n\t\t]\n\t}";
}

/**
 * @brief Stores data for a single image
 * 
 */
MyResult output;

/**
 * @brief Path of the output JSON file
 * 
 */
std::string output_path;

/**
 * @brief Stores output JSON file object
 * 
 */
std::ofstream file_obj;

/**
 * @brief Flag to check if any image has been received
 * 
 */
bool first_image;

/**
 * @brief Flag to check if a kill signal has been received
 * 
 * Either SIGINT and rosnode kill 
 */
sig_atomic_t volatile request_shutdown = 0;

/**
 * @brief Signal handler on receiving SIGINT signal
 * 
 * @param sig	The signal received
 */
void signal_handler(int sig) {
	request_shutdown = 1;
}

/**
 * @brief Handler to handle rosnode kill
 * 
 * Reference: https://answers.ros.org/question/27655/what-is-the-correct-way-to-do-stuff-before-a-node-is-shutdown/
 * 
 * @param params
 * @param result 
 */
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
	request_shutdown = 1;
}

/**
 * @brief Callback that processes image on receiving from publisher
 * 
 * @param msg	The message received from the publisher
 */
void myrosalprengineCallback(const sensor_msgs::ImageConstPtr& msg)
{
	if (!first_image) {
		file_obj << ",\n";
	} else {
		first_image = false;
	}
	std::string img_id = msg->header.frame_id.c_str();
	
	cv::Mat img = cv_bridge::toCvShare(msg, "mono8")->image;
	std::vector<alpr::AlprRegionOfInterest> regionsOfInterest;
	alpr::AlprRegionOfInterest fullFrame(0,0, img.cols, img.rows);
	regionsOfInterest.push_back(fullFrame);
	
	alpr::Alpr openalpr("us");
	openalpr.setTopN(10);

 	if (!openalpr.isLoaded()) {
		std::cerr << "Error loading OpenALPR" << std::endl;
		return;
	}

	alpr::AlprResults results = openalpr.recognize(img.data, img.step/img.cols, img.cols, img.rows, regionsOfInterest);
	std::vector<MyPlate> plate_outputs;

	for (int i = 0; i < results.plates.size(); i++) {
		MyPlate myplate;
		alpr::AlprPlateResult plate = results.plates[i];
		myplate.requested_topn = plate.requested_topn;
		myplate.set_bbox(plate.plate_points, 4);
		myplate.top_plate = plate.bestPlate.characters;
		myplate.top_confidence = plate.bestPlate.overall_confidence;
		for (int j = 0; j < plate.topNPlates.size(); j++) {
			myplate.confidences.push_back(plate.topNPlates[j].overall_confidence);
			myplate.characters.push_back(plate.topNPlates[j].characters);
		}
		plate_outputs.push_back(myplate);
	}

	output = MyResult(img_id, plate_outputs);
	output.toJSONFile(file_obj);
}
 
int main(int argc, char *argv[]) {
	
	if (argc != 3) {
		std::cout << "Usage: " << argv[0] << " --result <result_json_path>" << std::endl;
		return 1;
	}

	output_path = std::string(argv[2]);
	file_obj.open(output_path, std::ofstream::out);
	file_obj << "[\n";

	first_image = true;

	ros::init(argc, argv, "myrosalprengine");
	ROS_INFO("[myrosalprengine] Node started.");
	ros::NodeHandle nh;

	MyResult result;
	std::signal(SIGINT, signal_handler);
	std::signal(SIGTERM, signal_handler);
	ros::XMLRPCManager::instance()->unbind("shutdown");
  	ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("cv_camera/image_raw", 1, myrosalprengineCallback);

	while(!request_shutdown) {
		ros::spinOnce();
	}

	if(file_obj.is_open()) {
		file_obj << "\n]\n";
		file_obj.close();
	}
	
	ros::shutdown();
	return 0;
}