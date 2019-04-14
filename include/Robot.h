#include<iostream>
#include<vector>
#include<Eigen/Dense>


class Robot
{
private:
     int no_of_joints;
     Eigen::MatrixXd Dh_params;
     float *joint_min_limits;
     float *joint_max_limits;

public:
    Robot(int n_joints,Eigen::MatrixXd m1,float j_min_limits[],float j_max_limits[]);
    ~Robot();
    std::vector<float> Inverse_kinematics(float cartesian_coordinates[6]);

    std::vector<float> Forward_Kinematics(float joint_angles[]);

    void print();

};