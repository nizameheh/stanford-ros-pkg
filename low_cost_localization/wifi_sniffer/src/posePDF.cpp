#include "posePDF.h"
#include <time.h>
#include <vector>
#include "pose.h"
#include <math.h>


/* for binNumber */
// spatial resolution for the bins
#define BIN_SP_RESOL (.1)
// number of bins in the three dimensions
#define BIN_NX (long(1 + 53.6 / BIN_SP_RESOL))
#define BIN_NY (long(1 + 53 / BIN_SP_RESOL))
#define BIN_NTHETA (24)
// angular resolution for the bins
#define BIN_ANG_RESOL (2 * M_PI/BIN_NTHETA)

/* STATIC VARIABLE WEIRDNESS */
boost::shared_ptr<nav_msgs::OccupancyGrid const> map;

/* Constructor,
 * Takes pose, number of poses to create, and the distribution type
 * Then initializes a set of poses
 */
PosePDF::PosePDF() { }



/*
void PosePDF::_xy2rowcol(long x, long y, long& row, long &col) const
{
	long orow=myMap->info.origin.position.x, ocol=myMap->info.origin.position.y;

	row = orow - y;
	col = ocol + x;
}

bool PosePDF::_withinBounds(long x, long y) const
{
	long row,col;
	_xy2rowcol(x,y,row,col);
	if(row>=0 && col>=0 && row<myMap->info.width && col<myMap->info.height) return true;
	return false;
}
float PosePDF::operator()(long x, long y) const
{
	if(_withinBounds(x,y)){ 
		long row,col;
		_xy2rowcol(x,y,row,col);
		return myMap->data[myMap->info.width*x + y];
	}
	return -42;
}
*/
PosePDF::PosePDF(float x, float y, float theta, int numOfPoses, bool uniformDistribution)
{
    static float stdDevX = ConfigFileLoader::getInstance()->getValue("stdDevX");
    static float stdDevY = ConfigFileLoader::getInstance()->getValue("stdDevY");
    static float stdDevTheta = ConfigFileLoader::getInstance()->getValue("stdDevTheta");
    static bool uDistribution = ConfigFileLoader::getInstance()->getValue("isInitialDistributionUniform");
    init(x, y, theta, numOfPoses, uDistribution, stdDevX, stdDevY, stdDevTheta);
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
            float dx = xRange * drand48();
            dx = dx - xRange/2;
            
            
            static float yRange = ConfigFileLoader::getInstance()->getValue("YRange");
            float dy = yRange * drand48();
            dy = dy - yRange/2;
            
            static float thetaRange = ConfigFileLoader::getInstance()->getValue("thetaRange");
            float dtheta = thetaRange * drand48();
            dtheta = dtheta - thetaRange/2;
            
            //make absolute position
            float newX = x + dx;
            float newY = y + dy;
            float newTheta = theta + dtheta;
            //ROS_INFO("no trouble so far, right before is in map");

            if(isFreeInMap(newX,newY)) {
				//create a pose and store it in the pose set
				Pose *aPose = new Pose(newX, newY, newTheta, probability);
				poseSet.push_back(aPose);
				i++;
			}
        }    
    } else {
        // for a normal distribution
        //ROS_INFO("Initalizing posePDF with normal distribution");
        
        //point weight
       float probability = 1.0 / (float)numOfPoses;
       
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

    std::vector<Pose*> newPoseSet;

    // erase poses that are inside the walls
    for(unsigned int i = 0; i < poseSet.size(); i++)
    {
        if (isFreeInMap(poseSet.at(i)->getX(), poseSet.at(i)->getY()))
        {
            newPoseSet.push_back(poseSet.at(i));
        }
        else {
            delete poseSet.at(i);
            poseSet.at(i) = NULL;

        }
    }
    poseSet = newPoseSet;
    
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

	//ROS_INFO("Current weight: %f", currentWeight);
	
	//Add the resampled pose to a set
	Pose *tempPose = new Pose(poseSet.at(currentIndex)->getX(),
					poseSet.at(currentIndex)->getY(),
					poseSet.at(currentIndex)->getTheta(),
					1.0/m);
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


bool PosePDF::isFreeInMap(float mX, float mY)
{
   int pxX = mX / myMap->info.resolution;
   int pxY = mY / myMap->info.resolution;
   //ROS_INFO("right before lookup value pxX: %d pxY: %d", pxX, pxY);
   float mapValue = myMap->data[myMap->info.width*pxY + pxX];
   if (mapValue == -1 || mapValue == 100) {
      return false;
   }
   else {
      return true;
   }
 
}  











//Helper to calculate the binNumber for the hashmap in kldSampling()
long PosePDF::binNumber(float x, float y, float theta) {
   long myNx = long(x / BIN_SP_RESOL);
   long myNy = long(y / BIN_ANG_RESOL);
   long myNTheta = long(theta);
   
   return myNTheta * (BIN_NX * BIN_NY) + myNy * BIN_NX + myNx;
}


/* Helper function for KLD sampling
 * Chooses rand from 0-1, and increments along our poseSet till we reach that pt
 * Assumes poseSet is[shouldbe] normalized to 1 
 */
Pose PosePDF::randomPoseChooser() {
   float r = drand48();
   unsigned int i = 0;
   while (r>=0 && i < poseSet.size()) {
       
      r -= poseSet.at(i)->getProbability();
      i++; 
   }
   
   
   return *(poseSet.at(i - 1));

}  

/* KLD Sampling
 * Use this function after resampling motion/sensor models to obtain
 * clustering of pose set into bins of certain size
 * Grid size: 50cm x 50cm x 15 deg
 * Delta: .01   [1 - delta = .99]
 * Z_1-delta: .8389 //Obtained from statistical table
 * Epsilon: .05
 */
void PosePDF::kldSampling() {
	float epsilon = .05;
	float z_1d = .8389;
	int m = 0;
	int k = 0;
	float m_x = 10000000; //Ideally infinity
	std::tr1::unordered_map<long, bool> bins;
	//keep the load factor at 0.5 to make it faster!
	bins.max_load_factor(0.5);
	std::vector<Pose *> newPoseSet;
	
	static float maxNumberOfSamples = 2500;//ConfigFileLoader::getInstance()->getValue("maxNumberOfSamples");
	while(m < m_x && m < maxNumberOfSamples) {
		//Choose random pose
		Pose weightedRandomPose = randomPoseChooser();
		
		//Create new set of poses
		Pose *tempPose = new Pose(weightedRandomPose);
									
		newPoseSet.push_back(tempPose);
		
		//Do bin checking thing
		if(bins.find(	binNumber(weightedRandomPose.getX(), 
						weightedRandomPose.getY(), 
						weightedRandomPose.getTheta())) == bins.end()) {
			k = k + 1;
			bins.insert(std::pair<long, bool>(binNumber(weightedRandomPose.getX(), 
											weightedRandomPose.getY(), 
											weightedRandomPose.getTheta()), true)); //bool value doesn't matter
			//--Set b to non empty
		
			if(k > 1)
			{ /* see Thrun, p.264 */
				float firstTerm = (k-1.0)/(2.0*epsilon);
				float secondTerm = pow((1.0 - (2.0/(9.0*(k-1.0))) + sqrt((2.0/(9.0*(k-1.0)))*z_1d)),3);
				m_x =  firstTerm*secondTerm;
			}
		}

		m++;
	}
	
	//ROS_INFO("Number of particles (KLD-resample): %d", m);
	
	//delete old poses
  	for (unsigned int i = 0; i < poseSet.size(); i++)
  	{
  	    delete poseSet.at(i);
  	}
	poseSet.clear();
  	poseSet = newPoseSet;
  	
  	
  	//Resize probabilities	
  	long nPoses = poseSet.size();
  	for (long i = 0; i < nPoses; i++) {
  	   poseSet.at(i)->setProbability(1.0 / nPoses);
  	}
}
