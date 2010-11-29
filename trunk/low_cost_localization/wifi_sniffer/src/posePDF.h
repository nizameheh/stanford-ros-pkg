/*
 * Project 3
 * Team Win: Arne Bech, Brian Chung, Robert Kanter, Pierre Kreitmann
 */

#ifndef POSE_PDF_H
#define POSE_PDF_H


#include "pose.h"
#include "ConfigFileLoader.h"
#include <vector>
//#include "sensor_msgs/LaserScan.h"
#include <laser_geometry/laser_geometry.h>

class PosePDF
{
	private:
     //Stores all of the current Poses
    	Pose selectedPose;             //The "best" Pose
        Pose randomPoseChooser();
		float normalSample(float variance);
		void normalSample(float & x, float & y);
        bool isFreeInMap(float x, float y);
        void init(float x, float y, float theta, int numOfPoses, bool uniformDistribution, float stdDevX, float stdDevY, float stdDevTheta);
	
	public:
	    std::vector<Pose*> poseSet;
		PosePDF();
		PosePDF(float x, float y, float theta, int numOfPoses, bool uniformDistribution);
		PosePDF(float x, float y, float theta, int numOfPoses, bool uniformDistribution, float stdDevX, float stdDevY, float stdDevTheta);
		~PosePDF();
		PosePDF(std::vector<Pose*> poseSet);
		void move(float x, float y, float oldTheta, float newTheta);
		std::vector<Pose*> getPoseArray();
		void resampling();
		Pose* getSelectedPose();
		void setSelectedPose(const Pose & p);
};
    
#endif
