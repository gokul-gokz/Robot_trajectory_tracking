#ifndef ROBOT_H
#define ROBOT_H

//CPP
#include<iostream>

//Additional
#include<vector>
#include<Eigen/Dense>



/**
 * @brief Class implementation of Robot
 */

class Robot
{
private:
    /**Variable to store number of joints*/
     int no_of_joints;

    /**DH matrix*/
     Eigen::MatrixXd Dh_params;

    /**FK homogeneous TF matrix*/
     Eigen::MatrixXd HT_BasetoEF;

    /**Store the joint limits*/
     float *joint_min_limits;
     float *joint_max_limits;

    /**All Ik solutions are stored in this variable*/
     std::vector<std::vector<float> > IK_solutions;

    /**Valid IK solutions based on joint limit*/
     std::vector<std::vector<float> > valid_ik_solutions;

     /**Singularity point */
     bool singularity_pt;



public:

    /**
  * @brief Constructor
  * @param number of joints
  * @param goal DH_matrix
  * @param j_min_limits
  * @param j_max_limits
    */
    Robot(int n_joints,Eigen::MatrixXd m1,float j_min_limits[],float j_max_limits[]);



    /** Destructor */
    ~Robot();



        /**
    * @brief Function to calculate the forward kinematic
    * @param joint angles
    * @return cartesian coodinates
      */
    std::vector<float> Forward_Kinematics(float joint_angles[]);


    /**
   * @brief Function to calculate the Inverse kinematics, update the member variable IK_solutions
   * @param Cartesian coordiantes(x,y,z)
   * @return void
     */
    void Inverse_kinematics(float cartesian_positions[]);

    /**
   * @brief Function to validate the Ik solution calculated by Inverse kinematics, extracts the solutions from member variable
     * IK solutions and stores the validated IK solution in the valid_ik_solution variable
   * @return void
     */
    void validate_IK_solutions();

    /**
   * @brief Function to calculate the closest IK solution which has the minimum joint angle changes
   * @param current position in terms of joint angles(radians)
   * @param goal position interms of cartesian coordinates
   * @return void
     */
    std::vector<float> closest_IK_solutions(float current_position[],float goal_position[]);


    /**
   * @brief Function to print the valid IK solutions
        */
    void print();

};

#endif