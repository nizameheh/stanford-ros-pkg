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
#include <nav_msgs/OccupancyGrid.h>
#include "visualization_msgs/Marker.h"
#include <string.h>
#include "wifi_sniffer.h"
#include "Publisher.h"
#include "wifi_sniffer/WifiSniff.h"
#include "wifi_sniffer/WifiScan.h"

#include "posePDF.h"
#include "pose.h"

#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>
#include "sniff.cpp"


void wifiCallback(const wifi_sniffer::WifiScan::ConstPtr& wifiMsg);
void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& poseMsg);
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& mapMsg);
bool isFreeInMap(float mX, float mY);
float getProbabilityOfOneScan(float diff);
Pose getBestPose();
void getNearestNeighbor(int &index, int poseIndex);

////////////////////////////////////////////////////////////////////////
//Global Variables
tf::TransformListener* tfListener = NULL;
tf::TransformBroadcaster* tfBroadcaster = NULL;
tf::Transform transformer; //transform we will use for /map to /odom
ros::Publisher point_pub;
visualization_msgs::Marker mark;
visualization_msgs::Marker arrowMark;
visualization_msgs::Marker nN;

boost::shared_ptr<nav_msgs::OccupancyGrid const> PosePDF::myMap;

scanSet *scanSetI = new scanSet();
wifiScan *tempScan = new wifiScan();
PosePDF *mainPosePDF = NULL;
std::string fileName;
bool pubFlag = false;
bool initFlag = false;
bool mapSaved = false;
bool mapLoadedFirst = false;
bool wifiSignalReceived = false;
double newTheta; double oldTheta;


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
	ros::Subscriber wifiSubscriber = node.subscribe("/wifi_scan", 2, wifiCallback);
    ros::Subscriber subscriber = node.subscribe("/initialpose", 1000, initialPoseCallback);
    ros::Subscriber mapSubscriber = node.subscribe("/map",1,mapCallback);
    
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
	mark.color.g = 1.0;
	mark.color.r = 1.0;
	mark.color.a = 1.0;
	mark.scale.x = 0.1;
	mark.scale.y = 0.1;
	
	
    //Set running rate to 30hz:
    ros::Rate rate(30.0);
	
	//Load the file
	scanSetI = openScanSet(fileName);
	
	//Publish all the points u have to work off of
	for(int a = 0; a < scanSetI->getSize(); a++) {
		//ROS_INFO("X: %f Y: %f", scanSetI->at(a)->getX(), scanSetI->at(a)->getY());
		
		geometry_msgs::Point point;
		point.x = scanSetI->at(a)->getX();
		point.y = scanSetI->at(a)->getY();
		point.z = 0.0;
		mark.points.push_back(point);
	}
	
	int count =0;
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
	
	//Transform just for getting newTheta
	try{
		tfListener->waitForTransform("/base_footprint","/odom", tim, ros::Duration(1.0));
		tfListener->transformPose("/odom", input_pose, output_pose);
		oldTheta = tf::getYaw(output_pose.pose.orientation);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s", ex.what());
		exit(1);
	}
	
    
    while (node.ok())
    {
		//If we have it set to uniform mode
		if(mapLoadedFirst && ConfigFileLoader::getInstance()->getValue("isInitialDistributionUniform")) {
			
			mainPosePDF = new PosePDF(0.0, 0.0, 0.0, 10000, true);
			//ROS_INFO("creating uniform pose no issue");
			pubFlag = true;
			initFlag = true;
			mapLoadedFirst = false;
		}
		
		try{
			tfListener->waitForTransform("/base_footprint","/odom", tim, ros::Duration(1.0));
			tfListener->transformPose("/odom", input_pose, output_pose);
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s", ex.what());
			exit(1);
		}
		
		newTheta = tf::getYaw(output_pose.pose.orientation);	
		newX = output_pose.pose.position.x;
		newY = output_pose.pose.position.y;
		
		float dist = sqrt((newX - oldX)*(newX-oldX) + (newY - oldY)*(newY-oldY));


		if((dist > .2 || (newTheta-oldTheta > 1.0))&& initFlag) {
			//ROS_INFO("Entered into move");

			////////////////////////////////////////////////////////////
			//Need to call move
			float deltaX = newX - oldX;
			float deltaY = newY - oldY;
  
			mainPosePDF->move(deltaX, deltaY, oldTheta, newTheta);
			
			//mainPosePDF->resampling();
			mainPosePDF->kldSampling();
			//Publisher::publishPDF(point_pub, mainPosePDF->getPoseArray(), "/map");
			
			
			oldTheta = newTheta;
 			oldX = newX;
			oldY = newY;      
			pubFlag = true;
		}

  	 	if(wifiSignalReceived && initFlag) {
			////////////////////////////////////////////////////////////
			//ROS_INFO("Entered into wifi algorithm");
			
			//Initialization variable for resampling
			mainPosePDF->countForResampling = 0.0; 
			
			for(unsigned int a = 0; a < mainPosePDF->poseSet.size(); a++) {
				int index = 0;
				
				getNearestNeighbor(index, a);
				
				
				//With closest one, get likelihood
				float wholeScanProb = 1.0;
				int numMatched = 0;

				for(unsigned int c = 0; c < tempScan->everySniff.size(); c++) {
					if(tempScan->everySniff.at(c)->getLevel() != 0) {
						unsigned int d = 0;
						bool macFound = false;
						while(d < scanSetI->everyScan.at(index)->everySniff.size() && !macFound) {
							
							if((scanSetI->everyScan.at(index)->everySniff.at(d)->getMac() == tempScan->everySniff.at(c)->getMac()) &&
								scanSetI->everyScan.at(index)->everySniff.at(d)->getLevel() != 0)
							{
								float difference =  fabs(scanSetI->everyScan.at(index)->everySniff.at(d)->getLevel() - tempScan->everySniff.at(c)->getLevel());
														  //(scanSetI->everyScan.at(index)->everySniff.at(d)->getLevel() - tempScan->everySniff.at(c)->getLevel());
								
								//here doing it based on a 'bimodal' gaussian
								//ROS_INFO("Difference for %d is: %f",d, difference);
								//if(difference < ConfigFileLoader::getInstance()->getValue("sensorModelGaussianStdDev")) {
									float probability = getProbabilityOfOneScan(difference);
									//ROS_INFO("probability: %f", probability);
									//(drand48() >.2)? probability: probability*=.8;
									//update pose probability
									wholeScanProb = wholeScanProb * probability;
									macFound = true;
								//}
								//if(macFound) numMatched++;
							}	
							d++;
							
						}
					
					//problem with macnotfound!?
					
					//if(!macFound)ROS_INFO("Pose: %d wholeScanProb: %f", a, wholeScanProb);
					//float rSN = (drand48() >.2)? 1.0: .8;
					if(!macFound) wholeScanProb = wholeScanProb * ConfigFileLoader::getInstance()->getValue("sensorModelUniformCoef");
					}							
				}
				//Set likelihood to probability of that thing
				
				//printf("Pose: %d wholeScanProb: %f numMatched: %d\n", a, wholeScanProb, numMatched);
				mainPosePDF->poseSet.at(a)->setProbability(wholeScanProb);
				mainPosePDF->countForResampling += wholeScanProb;

			}
			
			//Renormalize weights!
			for(unsigned int n = 0; n < mainPosePDF->poseSet.size(); n++) {
				mainPosePDF->poseSet.at(n)->setProbability(mainPosePDF->poseSet.at(n)->getProbability()
															/mainPosePDF->countForResampling);
			}
			
			Publisher::publishPDF(point_pub, mainPosePDF->getPoseArray(), "/map");
			mainPosePDF->kldSampling();

			

			
			
			
			wifiSignalReceived = false;
			//pubFlag = true; <-the reason for this is bc want to publish only when robot moved enough for capturing time and position. really crappy
		}

  	 	//Publish Poses
		if(initFlag && pubFlag) {
			Pose bestPose = getBestPose();
			printf("%f,%f,%d\n",bestPose.getX(),bestPose.getY(),count);
			Publisher::publishPoint(point_pub, &bestPose, "/map", "Best Pose", 0);
			count++;
			pubFlag = false;
		}

  	    //check for incoming messages
  	    ros::spinOnce();
  	    rate.sleep();
    }

    return 0;
}










////////////////////////////////////////////////////////////////////////
//Callback functions

/* Wifi Scan Callback
 * Updates vector of scans with new scan
 * Uses the transform between /base_footprint and /map to
 * Obtain robot location and update vector with that thing
 */
void wifiCallback(const wifi_sniffer::WifiScan::ConstPtr& wifiMsg)
{
    //printf("Got WifiScan\n");
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
	
	
	mainPosePDF = new PosePDF(x, y, tf::getYaw(poseMsg->pose.pose.orientation), ConfigFileLoader::getInstance()->getValue("maxNumberOfSamples"), false); //Doesn't matter hwat u put for bool
	//printf("mainposepdf created in initialposecallback\n");
	initFlag = true;
	pubFlag = true;
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& mapMsg) {
	if(!mapSaved) {
		PosePDF::myMap = mapMsg;
		//printf("Map saved\n");
		mapSaved = true;
		mapLoadedFirst = true;
		
		//Recorded wifiscans SOULDNT BE LIKE THIS!
		point_pub.publish(mark);
	}
}

float getProbabilityOfOneScan(float diff) {
   float d = diff;

   /* Params */
   static float mStdDev = ConfigFileLoader::getInstance()->getValue("sensorModelGaussianStdDev"); // in m, the std-dev of the gaussian
   static float uniformMax = ConfigFileLoader::getInstance()->getValue("sensorModelUniformCoef"); /* the weight of the uniform (ie, the value returned
   when the gaussian is 0 */
   
   // if (d> 9.0)
       // return uniformMax;
    
   static float gaussianMax = ConfigFileLoader::getInstance()->getValue("sensorModelGaussianCoef"); /* the weight of the gaussian, ie the value returned
   for d == 0 if there was no uniform distribution */
   

   float mVar = mStdDev * mStdDev; // variance, in m
   //float gaussianConstant = 1/sqrt(2*M_PI*mVar);
   float laplacianProb = .5*gaussianMax * exp(-d/gaussianMax);
   float gaussianProb = gaussianMax * exp(-.5 * d * d/ mVar);

   return uniformMax + gaussianProb;
}


Pose getBestPose() {
	
	float xBest = 0.0; float yBest = 0.0; float theta = 0.0;
	float coses = 0.0; float sines = 0.0;
	
	for(unsigned int j = 0; j < mainPosePDF->poseSet.size(); j++) {
		/*if (mainPosePDF->poseSet.at(j)->getProbability() > .6/mainPosePDF->poseSet.size()) */{
			xBest += mainPosePDF->poseSet.at(j)->getX()*mainPosePDF->poseSet.at(j)->getProbability();
			yBest += mainPosePDF->poseSet.at(j)->getY()*mainPosePDF->poseSet.at(j)->getProbability();
			theta += mainPosePDF->poseSet.at(j)->getTheta()*mainPosePDF->poseSet.at(j)->getProbability();
			coses += mainPosePDF->poseSet.at(j)->getProbability() * cos( mainPosePDF->poseSet.at(j)->getTheta());
			sines += mainPosePDF->poseSet.at(j)->getProbability() * sin( mainPosePDF->poseSet.at(j)->getTheta());
		}
	}	    
	theta = atan2(sines,coses);
	return Pose(xBest,yBest,theta,1);
}


void getNearestNeighbor(int &index, int poseIndex) {
	//For each pose
	float scanDist = 1000000.0;
	
	//Find closest one -Nearest Neighbor Approach
	for(unsigned int b = 0; b < scanSetI->everyScan.size(); b++) {
		//for each scan
		float tempDist = ((mainPosePDF->poseSet.at(poseIndex)->getX() - scanSetI->everyScan.at(b)->getX())*(mainPosePDF->poseSet.at(poseIndex)->getX() - scanSetI->everyScan.at(b)->getX()) +
						(mainPosePDF->poseSet.at(poseIndex)->getY() - scanSetI->everyScan.at(b)->getY())*(mainPosePDF->poseSet.at(poseIndex)->getY() - scanSetI->everyScan.at(b)->getY()));
		if (tempDist < scanDist) {
			scanDist = tempDist;
			index = b;
		}
	}
}
