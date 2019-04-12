#include<Robot.h>

Robot:: Robot(int n_joints,Eigen::MatrixXd m1)
{
    no_of_joints=n_joints;
    Dh_params.resize(no_of_joints,4);
    Dh_params=m1;
}

void Robot::print() {
    for (int i=0;i<no_of_joints;i++)
    {
        for (int j=0;j<4;j++)
            std::cout<<Dh_params(i,j)<<std::endl;
    }
}
int main()
{
    Eigen::MatrixXd m1(2,4);
    m1<<1,1,1,1,12,2,2,1;
    Robot R1(2,m1);
    R1.print();
    return 0;
}