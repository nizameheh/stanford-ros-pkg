/*Mapper Program
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
#include <geometry_msgs/Twist.h>
#include "visualization_msgs/Marker.h"
#include <string.h>
#include "wifi_sniffer.h"
#include "wifi_sniffer/WifiSniff.h"
#include "wifi_sniffer/WifiScan.h"


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
	std::vector<wifiSniff*> everySniff;	
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
	std::vector<wifiScan*> everyScan;	
	friend class boost::serialization::access;
	template<class Archive>
	void serialize(Archive & ar, const unsigned int version)
	{
		ar & everyScan;
	}

	public:

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
tf::Transform transformer; //transform we will use for /map to /odom
ros::Publisher point_pub;
visualization_msgs::Marker mark;

int counter = 0;
scanSet *scanSetI = new scanSet();
std::string fileName;

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

    //Need to transform between /base_footprint and /map
    geometry_msgs::PoseStamped input_pose;
    geometry_msgs::PoseStamped output_pose;
    
    //Setting up Input Pose
    //It is set to robot 0,0. When transformed to map, = point of wifiscan
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
    
    //Get the wifipoint, Saved into output_pose
   	try{
		tfListener->waitForTransform("/base_footprint","/map", tim, ros::Duration(1.0));
        tfListener->transformPose("/map", input_pose, output_pose);
        ROS_INFO("Transformed");
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        exit(1);
    }
    
    //Save the information of the scan into a vector
	wifiScan *tempScan = new wifiScan(output_pose.pose.position.x,output_pose.pose.position.y);
	
    for(unsigned int i = 0; i < wifiMsg->sniffs.size(); i++) {
		wifiSniff *tempSniff = new wifiSniff(wifiMsg->sniffs.at(i).mac,
											wifiMsg->sniffs.at(i).ssid,
											wifiMsg->sniffs.at(i).signal_level,
											wifiMsg->sniffs.at(i).signal_noise);
		tempScan->addWifiSniff(tempSniff);
	}
	//Need to push tempScan onto a scanSet
	scanSetI->addScanSet(tempScan);
	
	
	//Set up point to push onto publisher stack
	geometry_msgs::Point point;
	point.x = output_pose.pose.position.x;
	point.y = output_pose.pose.position.y;
	point.z = 0.0;
    mark.points.push_back(point);
	point_pub.publish(mark);
	
	counter = 0;
}




//****************************************************************************//
//Main Function
int main(int argc, char **argv)
{

	//initalize ros
    ros::init(argc, argv, "wifiMap");
	if(argc != 2) {
		printf("Error: Specify file to save to \n \n");
		return -1;
	}
	else {
		fileName = argv[1];
	}
	
    ros::NodeHandle node;
    
    //Transform listener
    tf::TransformListener _tfListener;
    tfListener = &_tfListener;    
    
    //Subscribe to topics and set up callbacks
	ros::Subscriber wifiSubscriber = node.subscribe("/wifi_scan_usb", 2, wifiCallback);
    
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
	
	//Load the file
	try {
		scanSetI = openScanSet(fileName);
	}
	catch(std::string what) {
		ROS_INFO("File is new");
	}
	
	//Publish all the points you have saved so far
	for(int a = 0; a < scanSetI->getSize(); a++) {
		ROS_INFO("X: %f Y: %f", scanSetI->at(a)->getX(), scanSetI->at(a)->getY());
		
		geometry_msgs::Point point;
		point.x = scanSetI->at(a)->getX();
		point.y = scanSetI->at(a)->getY();
		point.z = 0.0;
		mark.points.push_back(point);
	}
	
	point_pub.publish(mark);
	
    //Set running rate to 30hz:
    ros::Rate rate(30.0);



    while (node.ok())
    {
  	 
     
  	    if(counter == 600) {
	
			
			ROS_INFO("Counter hit 600. File saved.i.e. Should have all data before now");
			
			saveScanSet(scanSetI, fileName);


			for(int a = 0; a < scanSetI->getSize(); a++) {
				//oa << scanSetInstance.everyScan.at(a);
				ROS_INFO("X: %f Y: %f", scanSetI->at(a)->getX(), scanSetI->at(a)->getY());
				

				geometry_msgs::Point point;
				point.x = scanSetI->at(a)->getX();
				point.y = scanSetI->at(a)->getY();
				point.z = 0.0;
				mark.points.push_back(point);
				
			}
			
			point_pub.publish(mark);
			
			//reset counter so you're not writing again and again
			counter = 0;			

		}
		else counter++;
  	    
  	    
  	    //check for incoming messages
  	    ros::spinOnce();
  	    rate.sleep();
    }

    return 0;
}



