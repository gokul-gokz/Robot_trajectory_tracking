#include<iostream>
#include<vector>
#include<Eigen/Dense>


class Robot
{
private:
     int no_of_joints;
     Eigen::MatrixXd Dh_params;

public:
    Robot(int n_joints,Eigen::MatrixXd m1);

    std::vector<float> Inverse_kinematics(float cartesian_coordinates[6]);

    std::vector<float> Forward_Kinematics(float joint_angles[]);

    void print();

};