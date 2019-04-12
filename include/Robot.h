#include<iostream>
#include<vector>
#include <Eigen/Dense>


class Robot
{
private:
    int no_of_joints;
    float *link_lengths=NULL;
    float *joint_offset=NULL;
    float *link_offset=NULL;
    float *joint_angle=NULL;

public:
    Robot(int n_joints,float l_length[],float j_offset[], float l_offset[], float j_angle[]);

    std::vector<float> Inverse_kinematics(float cartesian_coordinates[6]);

    std::vector<float> Forward_Kinematics(float joint_angles[]);

    void print();

};