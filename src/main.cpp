#include<Robot.h>
#include <fstream>

using namespace std;

std::vector<std::vector<float> > trajectory_waypoints( const char input_file[])
{
    std::ifstream input;
    std::vector<std::vector<float> > Waypoints;
    std::vector<float> w;
    int counter;
    float data;
    input.open(input_file);
    bool flag=true;
    cout << "Reading from the file" << endl;
    while(!input.eof()) {

        if (input.bad()) {
            cout << "Unable to read the file" << endl;
            flag=false;

        } else if (input.fail() and !input.eof()) {
            cout << "Invalid_input" << endl;
            flag=false;

        }
        if(flag) {

            input >> data;
            w.push_back(data);
            //cout << w[counter] << endl;
            counter++;
            if (counter>=4)
            {
                Waypoints.push_back(w);
                w.clear();
                counter=0;

            }
        }
    }
   input.close();

    for (int i=0;i<7;i++)
    {
        for (int j=0;j<4;j++)
        {
            cout<<Waypoints[i][j]<<endl;
        }
    }
    return Waypoints;
}



int main()
{
    std::vector<std::vector<float> > Waypoints;
    std::vector<float> angle_diff;
    float j_min_limits[3]={-M_PI,-M_PI/2,-M_PI};
    float j_max_limits[3]={M_PI,M_PI/2,M_PI};
    char input_file[60]="/home/gokul/Robot_Ik_trajectory_tracking/input.txt";
    float t=0;
    float time_step;
    Waypoints=trajectory_waypoints(input_file);
    Eigen::MatrixXd m1(3,4);
    m1<<90,10,0,0,0,5,0,0,0,5,0,0;
    Robot R1(3,m1,j_min_limits,j_max_limits);
    float j_angles[2]={1,2};
    R1.Forward_Kinematics(j_angles);
    R1.print();
    while(1)
    {
        for (int i=0;i<Waypoints.size()-1;i++)
        {
            float t_prev=t;
            float t=Waypoints[i][3];
            float time_step=(t-t_prev)*30;
            std::vector<float> p1=R1.Inverse_kinematics(Waypoints[i]);
            std::vector<float> p2=R1.Inverse_kinematics(Waypoints[i+1]);
            for (int i=0;i<p1.size();i++)
            {
                angle_diff.push_back(p1[i]-p2[i]);
            }




        }
    }
    return 0;
}
