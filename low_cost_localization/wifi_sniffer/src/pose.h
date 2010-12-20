
#ifndef POSE_H
#define POSE_H

#include "ConfigFileLoader.h"

/*
 * Encapsulates information about a Pose
 */
class Pose
{
    private:
	    float x;            //The x coordinate
	    float y;            //The y coordinate
	    float theta;        //The angle
	    float probability; //Probability and weight are interchangable terms
		float normalSample(float variance);
		void normalSample(float & x, float & y);
		float angleMPiPi(float theta);
		
	public:
	    Pose(float x, float y, float theta, float prob);
	    Pose(const Pose & pose);
	    Pose();
	    static Pose * createRandomPose(float width, float height, float prob);
		void setPose(float X, float Y, float Theta, float Probability);
		/* deltaX and deltaY are the differences between the new values and the old values given by the odometry for x and y.
		 * oldOdomTheta is the old angular value and newOdomTheta the new angular value given by the odometry
		 */
		void move(float deltaX, float deltaY, float oldOdomTheta, float newOdomTheta);
		float getX();
		float getY();
		float getTheta();
		float getProbability(); 
		void setProbability(float prob);
		
};

#endif
