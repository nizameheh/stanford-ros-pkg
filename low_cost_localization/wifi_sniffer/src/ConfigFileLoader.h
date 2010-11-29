/*
 * Project 3
 * Team Win: Arne Bech, Brian Chung, Robert Kanter, Pierre Kreitmann
 */

#ifndef CONFIG_FILE_LOADER_H
#define CONFIG_FILE_LOADER_H

#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <vector>
#include <tr1/unordered_map>

/*
 * tr1/unordered_map is (some future proposed implementation of) a hash map
 * This should be faster than using the regular stl map
 *      The function names are the same
 */

/*
 * Provides a singleton for loading and managing the config file.  It uses
 * a hashmap with keys and values.  All of the keys are strings and all of
 * the values are floats
 */
class ConfigFileLoader
{
    private:
        //hashmap that stores the keys and values
        std::tr1::unordered_map<std::string, float> dataMap;
        //constructor
        ConfigFileLoader(std::string filename);
        //singleton instance
        static ConfigFileLoader* cfl_instance;
        //filename to load
        static std::string cfl_filename;
        
    public:
        ~ConfigFileLoader();
        bool hasKey(std::string key);
        float getValue(std::string key);
        std::vector<std::string> getAllKeys();
        static ConfigFileLoader* getInstance();
        static void setFilename(std::string filename);
};


#endif
