#include<iostream>
#include<vector>
#include<Eigen/Dense>


class Robot
{
private:
     int no_of_joints;
     Eigen::MatrixXd Dh_params;
     Eigen::MatrixXd HT_BasetoEF;
     float *joint_min_limits;
     float *joint_max_limits;
     // temporary fix
    std::vector<std::vector<float> > IK_solutions;
    std::vector<std::vector<float> > valid_ik_solutions;


public:
    Robot(int n_joints,Eigen::MatrixXd m1,float j_min_limits[],float j_max_limits[]);
    ~Robot();
    std::vector<float> Forward_Kinematics(float joint_angles[]);
    void Inverse_kinematics(float cartesian_positions[]);
    void validate_IK_solutions();




    void print();

};