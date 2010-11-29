/*Localization Program
 *Author: Brian Chung
 *Methodology: Ugly, non-OOP, but easy to use
 */

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <stdlib.h>
#include <vector>
#include <string.h>
#include <math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "visualization_msgs/Marker.h"
#include <string.h>
#include "wifi_sniffer.h"
#include "wifi_sniffer/WifiSniff.h"
#include "wifi_sniffer/WifiScan.h"

#include "posePDF.h"
#include "pose.h"

#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>




////////////////////////////////////////////////////////////////////////
//Class Definitions
class wifiSniff {
    private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
		ar & mac;
		ar & ssid;
		ar & signal_level;
		ar & signal_noise;
    }
	std::string mac;
	std::string ssid;
	int signal_level;
	int signal_noise;

	public:
	wifiSniff() {}
	~wifiSniff() {}
	wifiSniff(std::string Mac, std::string Ssid, int Signal_level, int Signal_noise) {
		mac = Mac;
		ssid = Ssid;
		signal_level = Signal_level;
		signal_noise = Signal_noise;
	}
	std::string getMac() { return mac; }
	std::string getssid() { return ssid; }
	int getLevel() { return signal_level; }
	int getNoise() { return signal_noise; }
};
void deleteSniff(wifiSniff *sniff) { delete sniff; }

class wifiScan {
	private:
	
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & everySniff;
		ar & x;
		ar & y;
	}
	float x;
	float y;
	
	public:
	std::vector<wifiSniff*> everySniff;	
	wifiScan() {};
	~wifiScan() { for_each(everySniff.begin(), everySniff.end(), deleteSniff); }
	void addWifiSniff(wifiSniff *sniff) { everySniff.push_back(sniff); }
	wifiScan(float X, float Y) { x=X; y=Y; }
	float getX() { return x; }
	float getY() { return y; }

};
void deleteScan(wifiScan *scan) { delete scan; }

class scanSet {
	private:

	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & everyScan;
	}

	public:
	std::vector<wifiScan*> everyScan;	
	scanSet() {};
	~scanSet() { for_each(everyScan.begin(),everyScan.end(), deleteScan); }
	void addScanSet(wifiScan *scan) { everyScan.push_back(scan); }
	int getSize() { return everyScan.size(); }
	wifiScan* at(int i) { return everyScan.at(i); }
};

void saveScanSet(scanSet *set, const std::string &filename) {
	std::ofstream ofs(filename.c_str());
	boost::archive::text_oarchive oa(ofs);
	oa << *set;
	ofs.close();
}

scanSet *openScanSet(const std::string &filename) {
	scanSet *set = new scanSet();
	std::ifstream ifs(filename.c_str(), std::ios::binary);
	boost::archive::text_iarchive ia(ifs);
	ia >> *set;
	ifs.close();
	return set;
}



////////////////////////////////////////////////////////////////////////
//Global Variables
tf::TransformListener* tfListener = NULL;
tf::TransformBroadcaster* tfBroadcaster = NULL;
tf::Transform transformer; //transform we will use for /map to /odom
ros::Publisher point_pub;
visualization_msgs::Marker mark;
visualization_msgs::Marker arrowMark;

scanSet *scanSetI = new scanSet();
wifiScan *tempScan = new wifiScan();
PosePDF *mainPosePDF = NULL;
std::string fileName;
bool pubFlag = false;
bool initFlag = false;
bool wifiSignalReceived = false;

////////////////////////////////////////////////////////////////////////
//Callback functions

/* Wifi Scan Callback
 * Updates vector of scans with new scan
 * Uses the transform between /base_footprint and /map to
 * Obtain robot location and update vector with that thing
 */
void wifiCallback(const wifi_sniffer::WifiScan::ConstPtr& wifiMsg)
{
    ROS_INFO("Got WifiScan");
	//Save the information of the scan into a vector
	tempScan = new wifiScan(0.0,0.0);
	
    for(unsigned int i = 0; i < wifiMsg->sniffs.size(); i++) {
		wifiSniff *tempSniff = new wifiSniff(wifiMsg->sniffs.at(i).mac,
											wifiMsg->sniffs.at(i).ssid,
											wifiMsg->sniffs.at(i).signal_level,
											wifiMsg->sniffs.at(i).signal_noise);
		tempScan->addWifiSniff(tempSniff);
	}
	wifiSignalReceived = true;
}

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg)
{

	//Set robot pose to wherever initialposecallback is
	float x = poseMsg->pose.pose.position.x;
	float y = poseMsg->pose.pose.position.y;
	float theta = atan2(y, x);
	
	//broadcast map to odom transform
	transformer.setOrigin( tf::Vector3(x, y, 0.0) );
	transformer.setRotation( tf::Quaternion(poseMsg->pose.pose.orientation.x, 
    					poseMsg->pose.pose.orientation.y, 
    					poseMsg->pose.pose.orientation.z,
    					poseMsg->pose.pose.orientation.w));
	tfBroadcaster->sendTransform(tf::StampedTransform(transformer, ros::Time::now(), "/map", "/odom"));    
	
	//X, Y, theta, number of poses, isUniformDist, 
	if(mainPosePDF != NULL) {
		delete mainPosePDF;
		mainPosePDF = NULL;
	}
	mainPosePDF = new PosePDF(x, y, theta, 5000, false);
	initFlag = true;
	pubFlag = true;
}


//********************************************************************//
//Main Function
int main(int argc, char **argv)
{

	//initalize ros
    ros::init(argc, argv, "wifiMap");
	
	if(argc != 2) {
		printf("Error: Scan file required \n \n");
		return -1;
	}
	else {
		fileName = argv[1];
	}
	
    ros::NodeHandle node;
    
    //Transform broadcaster and listener
    tf::TransformBroadcaster _tfBroadcaster;
    tf::TransformListener _tfListener;
    tfListener = &_tfListener;
    tfBroadcaster = &_tfBroadcaster;    
    
    //Subscribe to topics and set up callbacks
	ros::Subscriber wifiSubscriber = node.subscribe("/wifi_scan_usb", 2, wifiCallback);
    ros::Subscriber subscriber = node.subscribe("/initialpose", 1000, initialPoseCallback);
    
    //Set up publishers
    point_pub = node.advertise<visualization_msgs::Marker>("point_marker", 1000, true);
 
	//Initialize the marker to use in publishing
	mark.header.frame_id = "/map";
	mark.header.stamp = ros::Time::now();
	mark.ns = "wifiLocations";
	mark.action = visualization_msgs::Marker::ADD;
	mark.pose.orientation.w = 1.0;
	mark.id = 0;
	mark.type = visualization_msgs::Marker::POINTS;
	mark.color.r = 1.0;
	mark.color.b = 0.5;
	mark.color.a = 1.0;
	mark.scale.x = 0.2;
	mark.scale.y = 0.2;
	  
	arrowMark.header.frame_id = "/map";
	arrowMark.header.stamp = ros::Time::now();
	arrowMark.ns = "poses";
	arrowMark.action = visualization_msgs::Marker::ADD;
	arrowMark.pose.orientation.w = 1.0;
	arrowMark.id = 0;
	arrowMark.type = visualization_msgs::Marker::POINTS;
	arrowMark.color.r = 1.0;
	arrowMark.color.b = 1.0;
	arrowMark.color.a = 1.0;
	arrowMark.scale.x = 0.02;
	arrowMark.scale.y = 0.02; 
	
    //Set running rate to 30hz:
    ros::Rate rate(30.0);
	
	//Load the file
	scanSetI = openScanSet(fileName);
	
	//Publish all the points u have to work off of
	for(int a = 0; a < scanSetI->getSize(); a++) {
		ROS_INFO("X: %f Y: %f", scanSetI->at(a)->getX(), scanSetI->at(a)->getY());
		
		geometry_msgs::Point point;
		point.x = scanSetI->at(a)->getX();
		point.y = scanSetI->at(a)->getY();
		point.z = 0.0;
		mark.points.push_back(point);
		
	}
	point_pub.publish(mark);
	
	float oldX, oldY, newX, newY;
	oldX = 0.0;
	oldY = 0.0;
	newX = 0.0;
	newY = 0.0;
		
	//Need to transform between /base_footprint and /map
	geometry_msgs::PoseStamped input_pose;
	geometry_msgs::PoseStamped output_pose;
	
	//Setting up Input Pose
	//It is set to robot 0,0.When transformed to odom, = how much moved
	input_pose.header.frame_id = "/base_footprint";
	input_pose.header.stamp = ros::Time(0);
	input_pose.pose.position.x = 0.0;
	input_pose.pose.position.y = 0.0;
	input_pose.pose.position.z = 0.0;
	input_pose.pose.orientation.x = 0.0;
	input_pose.pose.orientation.y = 0.0;
	input_pose.pose.orientation.z = 0.0;
	input_pose.pose.orientation.w = 1.0;
	ros::Time tim = ros::Time(0);
   
    while (node.ok())
    {
		try{
			tfListener->waitForTransform("/base_footprint","/odom", tim, ros::Duration(1.0));
			tfListener->transformPose("/odom", input_pose, output_pose);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s", ex.what());
			exit(1);
		}
		
		newX = output_pose.pose.position.x;
		newY = output_pose.pose.position.y;
		
		float dist = sqrt((newX - oldX)*(newX-oldX) + (newY - oldY)*(newY-oldY));
		
		//float thetaChange = atan2(newY,newX) - atan2(oldY,oldX); NEED NEW way to obtain theta change
		//Figure out if it has moved enough or received signal, if it has, do stuff here
		
		if((dist > .3)&& initFlag) {
			//Reset distances

			////////////////////////////////////////////////////////////
			//Need to call move
			float deltaX = newX - oldX;
			float deltaY = newY - oldY;
  
			
			//move all the poses
			mainPosePDF->move(deltaX, deltaY, atan2(oldY,oldX), atan2(newY,newX));
 			
 			oldX = newX;
			oldY = newY;      
			pubFlag = true;
			ROS_INFO("MOVE COMPLETED");
			ROS_INFO("Pose Size: %d", mainPosePDF->getPoseArray().size());
		}

  	 	if(wifiSignalReceived && initFlag) {
			////////////////////////////////////////////////////////////
			//Need to do mumbo jumbo with wifisignal
			ROS_INFO("Entered into wifi measurement model");
			for(unsigned int a = 0; a < mainPosePDF->getPoseArray().size(); a++) {
				//For each pose
				float scanDist = 1000000.0;
				int index = 0;
				
				//Find closest one
				for(unsigned int b = 0; b < scanSetI->everyScan.size(); b++) {
					//for each scan
					float tempDist = ((newX - scanSetI->everyScan.at(b)->getX())*(newX - scanSetI->everyScan.at(b)->getX()) +
									(newY - scanSetI->everyScan.at(b)->getY())*(newY - scanSetI->everyScan.at(b)->getY()));
					if (tempDist < scanDist) {
						scanDist = tempDist;
						index = b;
					}			
				}
				//With closest one, get likelihood
				float likelihood = 1.0;
				for(unsigned int c = 0; c < tempScan->everySniff.size(); c++) {
					if(tempScan->everySniff.at(c)->getLevel() != 0) {
						unsigned int d = 0;
						bool macFound = false;
						while(d < scanSetI->everyScan.at(index)->everySniff.size() && !macFound) {
							if(scanSetI->everyScan.at(index)->everySniff.at(d)->getssid() == tempScan->everySniff.at(c)->getssid()) {
								likelihood = likelihood * (scanSetI->everyScan.at(index)->everySniff.at(d)->getLevel() - tempScan->everySniff.at(c)->getLevel());
								macFound = true;
							}	
							d++;
						}
					}							
				}
				//Set likelihood to probability of that thing
				mainPosePDF->poseSet.at(a)->setProbability(likelihood);
			}
			//Now resample
			mainPosePDF->resampling();
			wifiSignalReceived = false;
			pubFlag = true;
		}
		
		//UPDATE MAP TO ODOM TRANSFORM HERE!!!!!
		/*
		 * 
		 * 
		 * 
		 * 
		 * 
		 * 
		 * 
		 */
		
		
  	 	//Publish Poses
		if(pubFlag && initFlag) {
			std::vector<Pose*> temp = mainPosePDF->getPoseArray();
			while(arrowMark.points.size() > 0) {
				arrowMark.points.pop_back();
			}
			for (unsigned int a = 0; a < temp.size(); a++) {
				geometry_msgs::Point point;
				point.x = temp.at(a)->getX();
				point.y = temp.at(a)->getY();
				point.z = 0.0;
				arrowMark.points.push_back(point);
			}
			point_pub.publish(arrowMark);
			pubFlag = false;
		}
  	 
  	    //check for incoming messages
  	    ros::spinOnce();
  	    rate.sleep();
    }

    return 0;
}



