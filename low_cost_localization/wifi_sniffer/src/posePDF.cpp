#include "posePDF.h"
#include <time.h>
#include <vector>
#include "pose.h"
#include <math.h>

/* Constructor,
 * Takes pose, number of poses to create, and the distribution type
 * Then initializes a set of poses
 */

PosePDF::PosePDF() { }

PosePDF::PosePDF(float x, float y, float theta, int numOfPoses, bool uniformDistribution)
{
    static float stdDevX = ConfigFileLoader::getInstance()->getValue("stdDevX");
    static float stdDevY = ConfigFileLoader::getInstance()->getValue("stdDevY");
    static float stdDevTheta = ConfigFileLoader::getInstance()->getValue("stdDevTheta");
    init(x, y, theta, numOfPoses, uniformDistribution, stdDevX, stdDevY, stdDevTheta);
}

/*
 * This constructor lets us change the stdDevX, stdDevY, and stdDevTheta during runtime
 */
PosePDF::PosePDF(float x, float y, float theta, int numOfPoses, bool uniformDistribution, float stdDevX, float stdDevY, float stdDevTheta)
{
    init(x, y, theta, numOfPoses, uniformDistribution, stdDevX, stdDevY, stdDevTheta);
}

/*
 * Apparently, C++ constructors can't call other constructors, so we have to put the constructor code in here
 * to be able to use it for both constructors.
 */
void PosePDF::init(float x, float y, float theta, int numOfPoses, bool uniformDistribution, float stdDevX, float stdDevY, float stdDevTheta)
{
    //For Uniform Distributions    
    if (uniformDistribution) {
        //Returns uniform distribution of points from -Range/2 to Range/2 for x,y,theta
        
        ROS_INFO("Initalizing posePDF with uniform distribution");
        
        //point probability (i.e particle weight);
        float probability = 1.0 / (float) numOfPoses;
        //over number of poses we want to create
        //for (int i = 0; i < numOfPoses; i++) {
        int i = 0;
        while (i < numOfPoses) {
   
            //compute delta positions, the range is fetched from the configuration file
            // so the final deltas will be between -range/2 to +range/2
            static float xRange = ConfigFileLoader::getInstance()->getValue("XRange");
            float dx = xRange * ( (float)rand() / (float)RAND_MAX );
            dx = dx - xRange/2;
            
            
            static float yRange = ConfigFileLoader::getInstance()->getValue("YRange");
            float dy = yRange * ( (float)rand() / (float)RAND_MAX );
            dy = dy - yRange/2;
            
            static float thetaRange = ConfigFileLoader::getInstance()->getValue("thetaRange");
            float dtheta = thetaRange * ( (float)rand() /(float)RAND_MAX );
            dtheta = dtheta - thetaRange/2;
            
            //make absolute position
            float newX = x + dx;
            float newY = y + dy;
            float newTheta = theta + dtheta;
            

            
			//create a pose and store it in the pose set
			Pose *aPose = new Pose(newX, newY, newTheta, probability);
			poseSet.push_back(aPose);
			i++;



        }    
    } else {
        // for a normal distribution
        ROS_INFO("Initalizing posePDF with normal distribution");
        
        //point weight
       float probability = 1 / (float)numOfPoses;
       
       //for all poses
       for (int i = 0; i < numOfPoses; i++) {
         
          //get delta coordinates
          float dx, dy;
          
          //get deltas from normal distribution
          //normalsample selects from gaussian and stores them in dx, dy
          PosePDF::normalSample(dx,dy);

          
          dx = dx * stdDevX;
          dy = dy * stdDevY;

          //get delta theta from normal distribution
          float dTheta = PosePDF::normalSample(1.0);

          dTheta = dTheta * stdDevTheta;
          
          //add Pose to the pose set with absolute coordinates
          Pose *aPose = new Pose(x + dx, y+dy, theta+dTheta, probability);
          poseSet.push_back(aPose);
       }
    }
}

//Constructor, uses existing pose set
PosePDF::PosePDF(std::vector<Pose*> poseSet) {
   this->poseSet = poseSet;

}

//Destructor
PosePDF::~PosePDF()
{
    for (unsigned int i = 0; i < poseSet.size(); i++)
    {
    	delete poseSet.at(i);
    }
}

//returns vector/array of poses
std::vector<Pose*> PosePDF::getPoseArray() {
   return poseSet;
}

//update all poses with the differences of position from the odometry
void PosePDF::move(float x, float y, float oldTheta, float newTheta) {
    
    //for all poses
    for (unsigned int i = 0; i < poseSet.size(); i++)
    {
        //move pose
        (poseSet.at(i))->move(x, y, oldTheta, newTheta);
    }
	ROS_INFO("posePDF.cpp//move successful");
//    std::vector<Pose*> newPoseSet;

//    poseSet = newPoseSet;
    
    //recalculate probabilities
    float prob = 1.0/(float)poseSet.size();
    for(unsigned int i = 0; i < poseSet.size(); i++)
    {
        poseSet.at(i)->setProbability(prob);
    } 
    
    
}



/* Resamples the pose array with weights into a pose array with uniform weigths
*/
void PosePDF::resampling() {
  //number of poses
  unsigned int m = poseSet.size();
  
  //get start position
  float randomFactor = drand48() / m;
  
  //get increment amount
  float increment = 1.0/m;
  
  //index of current pose
  unsigned int currentIndex = 0;
  
  //sum of all the weight up to the current pose
  float currentWeight = 0.0;
  
  //vector for storing our new resamples poses
  std::vector<Pose *> newPoseSet;
  
  //for m points
  for (unsigned int i = 0; i < m; i++) {
	float position = i * increment + randomFactor;
   
   //make sure we're sampling the right point
	while (position > currentWeight) {

        
        //update current total pose weight
		currentWeight += poseSet.at(currentIndex)->getProbability(); //probability means weight
		
		//update current pose index
     	currentIndex++;
     	currentIndex = currentIndex % m; //make sure we stay within the bounds
	}

	//ROS_INFO("We're on index: %i and looking at point %i", i, currentIndex);
	
	//Add the resampled pose to a set
	Pose *tempPose = new Pose(poseSet.at(currentIndex)->getX(),
					poseSet.at(currentIndex)->getY(),
					poseSet.at(currentIndex)->getTheta(),
					1/m);

	newPoseSet.push_back(tempPose);
  
  
  }
  
  //delete old poses
  for (unsigned int i = 0; i < poseSet.size(); i++)
  {
  	delete poseSet.at(i);
  }
  
  //use the resampled pose set
  poseSet.clear();
  poseSet = newPoseSet;
  
}


//Returns the selected pose. Note: integrateLaserWeights() must be called first
Pose* PosePDF::getSelectedPose()
{
    return &selectedPose;
}

//set the selected pose
void PosePDF::setSelectedPose(const Pose & p)
{
    selectedPose = p;
}


/* stores two normally sampled numbers in x and y (uses the Marsaglia polar method)
 * Variance is 1
 */
void PosePDF::normalSample(float & x, float & y) {
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

float PosePDF::normalSample(float variance) {
   float x,y;
   PosePDF::normalSample(x,y);
   return x * sqrt(variance);
}

