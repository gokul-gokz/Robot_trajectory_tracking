#include<Robot.h>
#include <Trajectory.h>
#include <Connection.h>
using namespace std;

int main() {
    // Vector for extracting the waypoints
    std::vector<std::vector<float> > Waypoints;
    std::vector<unsigned char > commanded_position;
    std::vector<unsigned char > current_position;

    bool is_running = true;

    //Assign the joint limits
    float j_min_limits[3] = {-M_PI, -M_PI / 2, -M_PI};
    float j_max_limits[3] = {M_PI, M_PI / 2, M_PI};

    //Location of the file which has the trajectory
    char input_file[60] = "/home/gokul/Robot_Ik_trajectory_tracking/input.txt";
    float t = 0;
    Waypoints = trajectory_waypoints(input_file);

    //Create an Eigen matrix to store the DH parameters
    Eigen::MatrixXd m1(3, 4);
    m1 << M_PI / 2, 10, 0, 0, 0, 5, 0, 0, 0, 5, 0, 0;

    //Initialize the robot with DH param and joint limits
    Robot R1(3, m1, j_min_limits, j_max_limits);

    //Create a connection with packet data size 3(three angles)
    Connection s1(3);

    //Command the robot to the start point of the trajectory
    float current[] = {0, 0, 0};

    //Extract the start point of trajectory
    float start_waypoint[] = {Waypoints[0][0], Waypoints[0][1], Waypoints[0][2]};
    std::cout<<start_waypoint[0]<<std::endl;
    std::cout<<start_waypoint[1]<<std::endl;
    std::cout<<start_waypoint[2]<<std::endl;

    //Initialize joint angle increment
    float joint_angle_increment[] = {0, 0, 0};

    //Find the joint angles for the start position
    std::vector<float> p1 = R1.closest_IK_solutions(current,start_waypoint) ;

    //Assign the start position
    current[0]=p1[0];
    current[1]=p1[1];
    current[2]=p1[2];

    //Joint space trajectory control
    //Follow the trajectory
    for (int i = 1; i < Waypoints.size(); i++) {
        float t_prev = t;
        float t = Waypoints[i][3];
        float time_step = (t - t_prev) * 50;
        float next_waypoint[] = {Waypoints[i][0], Waypoints[i][1], Waypoints[i][2]};

        //Extract the joint angles of the waypoints
        std::vector<float> p1 = R1.closest_IK_solutions(current, next_waypoint);

        //Find the joint angle increment based on the time between 2 waypoints
        //Assuming that the robot is always able to go to the commanded position in the specific time
        joint_angle_increment[0] = (p1[0] - current[0]) / time_step;
        joint_angle_increment[1] = (p1[1] - current[1]) / time_step;
        joint_angle_increment[2] = (p1[2] - current[2]) / time_step;

        for(int j=0;j<time_step;j++) {

            // Receive the current position from the robot
            if(s1.receive(current_position))
                std::cout<<"Current position received successfully"<<std::endl;
            else
            {
                std::cout<<"Unable to read the current position"<<std::endl;
                break;
            }

            //Increment the current position
            current[0] = current[0] + joint_angle_increment[0];
            current[1] = current[1] + joint_angle_increment[1];
            current[2] = current[2] + joint_angle_increment[2];

            commanded_position.push_back(current[0]);
            commanded_position.push_back(current[1]);
            commanded_position.push_back(current[2]);

            std::cout << "Waypoint " << i << ":Joint_angles" << std::endl;
            std::cout << current[0] << std::endl;
            std::cout << current[1] << std::endl;
            std::cout << current[2] << std::endl;

            //Send the calculated joint angle through the socket
            if(s1.send(commanded_position))
            {
                cout<<"Commanded successfully"<<std::endl;
            } else
            {
                cout<<"Socket communication failure"<<std::endl;
            }

        }
    }

    return 0;

}
