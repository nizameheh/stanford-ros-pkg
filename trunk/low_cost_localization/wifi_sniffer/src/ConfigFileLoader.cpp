/*
 * Project 3
 * Team Win: Arne Bech, Brian Chung, Robert Kanter, Pierre Kreitmann
 */

#include "ConfigFileLoader.h"

//Need because of C++'s stupid way of doing static variables
ConfigFileLoader* ConfigFileLoader::cfl_instance = NULL;
std::string ConfigFileLoader::cfl_filename = "part2_config.txt";


/*
 * Deletes the singleton instance of cfl_instance and switches the filename.
 * The next time getInstance() is called, it will load the new file
 */
void ConfigFileLoader::setFilename(std::string filename)
{
    //delete the cfl_instance
    if(!cfl_instance)
    {
        delete cfl_instance;
        cfl_instance = NULL;
    }
    
    //set the filename
    cfl_filename = filename;
}

/*
 * Returns an instance of the ConfigFileLoader singleton
 */
ConfigFileLoader* ConfigFileLoader::getInstance()
{
    //We try loading the file from the current directory, and then from the 
    //parent directory before giving up
    if(!cfl_instance)
    {
        try
        {
            cfl_instance = new ConfigFileLoader(cfl_filename);
        }
        catch(...)
        {
            try
            {
                cfl_instance = new ConfigFileLoader("./../" + cfl_filename);
            }
            catch(...)
            {
                printf("Could not open config file\n");
                exit(1);
            }
        }
    }

    return cfl_instance;
}

/*
 * The constructor opens the file and parses the keys and values into the
 * hashmap.  
 */
ConfigFileLoader::ConfigFileLoader(std::string filename)
{
    //keep the load factor at 0.5 to make it faster!
    dataMap.max_load_factor(0.5);

    //Try to read the file
    std::ifstream myFile;
    myFile.open (filename.c_str());
    if(myFile.is_open())
    {
        while(myFile.good())
        {
            //Each line is of the form "key = value" (without quotes)
            //where key is a string and value is a float
            //The dummy variable is to take care of the =
            std::string key, dummy;
            float value;
            myFile >> key >> dummy >> value;
            
            //Add the key and value to the hashmap
            dataMap.insert(std::pair<std::string,float>(key, value));
        }
    }
    else
    {
        //If there was a problem, throw an exception
        throw "error";
    }
    myFile.close();
}

/*
 * Destructor
 */
ConfigFileLoader::~ConfigFileLoader()
{
    
}

/*
 * Returns true if the key is in the hashmap, otherwise false
 */
bool ConfigFileLoader::hasKey(std::string key)
{
    //If the key isn't in the hasmap, it find() returns an iterator that is 
    //"after" the end of the hashmap, which is where end() points to, so we
    //just compare them
    return (dataMap.find(key) != dataMap.end());
}

/*
 * Returns the value associated with the key.  If the key isn't in the hashmap,
 * this throws an exception.  
 */
float ConfigFileLoader::getValue(std::string key)
{
    //Try to find the key in the hashmap and return its value
    std::tr1::unordered_map<std::string, float>::iterator it;
    it = dataMap.find(key);
    if(it == dataMap.end())
    {
        std::cerr << "Key does not exist" << std::endl;
        throw "Key does not exist";
    }
    return it->second;
}

/*
 * Returns a vector containing all of the keys from the hashmap (order is
 * not guaranteed)
 */
std::vector<std::string> ConfigFileLoader::getAllKeys()
{
    std::vector<std::string> vec;
    for(std::tr1::unordered_map<std::string, float>::iterator it = dataMap.begin(); it != dataMap.end(); it++)
    {
        vec.push_back(it->first);
    }
    return vec;
}



