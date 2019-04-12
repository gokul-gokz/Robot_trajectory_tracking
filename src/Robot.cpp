#include<Robot.h>
#include<math.h>

Robot:: Robot(int n_joints,Eigen::MatrixXd m1)
{
    no_of_joints=n_joints;
    Dh_params.resize(no_of_joints,4);
    Dh_params=m1;
}

std::vector<float> Robot::Forward_Kinematics(float joint_angles[])
{
    Eigen::MatrixXd Base_to_EFF(4,4);
    for (int i=0;i<no_of_joints;i++)
    {
        Eigen::MatrixXd T(4,4);
        T(0,0)=cos(joint_angles[i]);
        T(0,1)=-sin(joint_angles[i]);
        T(0,2)=0;
        T(0,3)=Dh_params(0,1);
        T(1,0)=sin(joint_angles[i])*cos(Dh_params(i,0));
        T(1,1)=cos(joint_angles[i])*cos(Dh_params(i,0));
        T(1,2)=-sin(Dh_params(i,0));
        T(1,3)=-sin(Dh_params(i,0))*Dh_params(i,2);
        T(2,0)=sin(joint_angles[i])*sin(Dh_params(i,0));
        T(2,1)=cos(joint_angles[i]*sin(Dh_params(i,0)));
        T(2,2)=cos(Dh_params(i,0));
        T(2,3)=cos(Dh_params(i,0))*Dh_params(i,2);
        T(3,0)=0;
        T(3,1)=0;
        T(3,2)=0;
        T(3,3)=1;

        if(i==0)
        {
            Base_to_EFF=T;
        }
        else
        {
            Base_to_EFF=Base_to_EFF*T;
        }
        std::cout<<Base_to_EFF<<std::endl;
    }

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
    m1<<1,1,1,1,1,1,1,1;
    Robot R1(2,m1);
    float j_angles[2]={1,2};
    R1.Forward_Kinematics(j_angles);
    R1.print();
    return 0;
}