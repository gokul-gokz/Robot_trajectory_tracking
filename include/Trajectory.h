
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <fstream>
#include <iostream>

/**
  * @brief Function to extract the trajectory waypoints from the file
  * @param location of the file
  *@return vector of the trajectory waypoints
  */
std::vector<std::vector<float> > trajectory_waypoints( const char input_file[])
{
    std::ifstream input;
    std::vector<std::vector<float> > Waypoints;
    std::vector<float> w;
    int counter;
    float data;

    //open the file
    input.open(input_file);
    bool flag=true;
    std::cout << "Reading from the file" <<std::endl;

    //Read till the end of the file
    while(!input.eof()) {

        if (input.bad()) {
           std::cout << "Unable to read the file" << std::endl;
            flag=false;

        } else if (input.fail() and !input.eof()) {
            std::cout << "Invalid_input" << std::endl;
            flag=false;

        }
        if(flag) {
            //read the data and push it into the vector
            input >> data;
            w.push_back(data);

            counter++;
            //Push each line as individual vector containing 3 joint angles and 1 time stamp
            if (counter>=4)
            {
                Waypoints.push_back(w);
                w.clear();
                counter=0;

            }
        }
    }
    input.close();

    return Waypoints;
}

#endif
