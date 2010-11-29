/*
 * Project 3
 * Team Win: Arne Bech, Brian Chung, Robert Kanter, Pierre Kreitmann
 */


#include "pose.h"
#include "math.h"
#include <stdlib.h>


/*
 * Sets x, y, theta, and probability for this Pose
 */
void Pose::setPose(float X, float Y, float Theta, float Probability) {
	probability = Probability;
	x = X;
	y = Y;
	theta = Theta;
}

Pose * Pose::createRandomPose(float width, float height, float prob) {
   Pose * result = new Pose(width*drand48(), height*drand48(), 2*M_PI*drand48(), prob);
   return result;
}

Pose::Pose()
{
   setPose(0,0,0,0);
}

/*
 * This Constructor creates a Pose from the passed in arguments
 */
Pose::Pose(float x, float y, float theta, float prob)
{
   setPose(x, y, theta, prob);
}

/*
 * This constructor creates a Pose from the passed in Pose
 */
Pose::Pose(const Pose & newPose)
{
    setPose(newPose.x, newPose.y, newPose.theta, newPose.probability);
}

//get x coord
float Pose::getX() {
	return x;
}

//get y coord
float Pose::getY() {
	return y;
}

//get theta
float Pose::getTheta() {
	return theta;
}

//get weight/probability
float Pose::getProbability() {
	return probability;
}

//set probability/weight
void Pose::setProbability(float prob) {
   probability = prob;
}
      

/* Updates one point based on the odometry readings. 
 * deltaTheta must be given in radians, in any interval.
 */
void Pose::move(float deltaX, float deltaY, float oldOdomTheta, float newOdomTheta)
{
    //TODO: a few more comments
   float deltaTheta = angleMPiPi(newOdomTheta-oldOdomTheta);
   
   /* (see Thrun, table 5.6, p.136) */
   float deltaRot1 = angleMPiPi(atan2(deltaY, deltaX) - oldOdomTheta); // [-Pi,Pi]
   float deltaTrans = sqrt(deltaX * deltaX + deltaY * deltaY);
   float deltaRot2 = angleMPiPi(deltaTheta - deltaRot1); // [-Pi, Pi]
   
   static float alpha1 = ConfigFileLoader::getInstance()->getValue("alpha1");
   static float alpha2 = ConfigFileLoader::getInstance()->getValue("alpha2");
   static float alpha3 = ConfigFileLoader::getInstance()->getValue("alpha3");
   static float alpha4 = ConfigFileLoader::getInstance()->getValue("alpha4");
   
   /* Sample the noises */
   float errorRot1 = normalSample(alpha1 * fabs(deltaRot1));
   float deltaHatRot1 = deltaRot1 - errorRot1;
   
   float errorTrans = normalSample(alpha3 * deltaTrans + alpha4 * fabs(deltaRot1));
   float deltaHatTrans = deltaTrans - errorTrans; 
   
   float errorRot2 = normalSample(alpha1 * fabs(deltaRot2) + alpha2 * deltaTrans); 
   float deltaHatRot2 = deltaRot2 - errorRot2;
   
   /* Compute the new values */
   x += deltaHatTrans * cos(theta + deltaHatRot1);
   y += deltaHatTrans * sin(theta + deltaHatRot1);
   theta = angleMPiPi(theta + deltaHatRot1 + deltaHatRot2);
}

void Pose::normalSample(float & x, float & y) {
   float s = 2;
   while (s>=1 || s==0) {
      x = 2*drand48() -1;
      y = 2*drand48() -1;
      s = x*x + y*y;
   }
   float multCoef = sqrt(-2*log(s)/s);
   x *= multCoef;
   y *= multCoef;
}

float Pose::normalSample(float variance) {
   float x,y;
   normalSample(x,y);
   return x * sqrt(variance);
}
/*
 * Makes sure that theta is between -PI and PI
 */

float Pose::angleMPiPi(float theta) {
   while (theta >= M_PI)
   {
      theta -= 2* M_PI;
   } 
   while (theta < -M_PI)
   {
      theta += 2 * M_PI;
   }
   
   return theta;
}

