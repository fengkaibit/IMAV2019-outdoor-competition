#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <fstream>
#include <map>
#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/package.h>

using namespace std;

class ParameterReader
{
public:
    ParameterReader(string filename=ros::package::getPath("landmark_detection")+"/config/landmark_parameters.txt")
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            cout<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1);
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string, string> data;
};

#endif // __CONFIG_H__
