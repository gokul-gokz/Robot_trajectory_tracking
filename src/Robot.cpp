#include<Robot.h>
#include<math.h>

Robot:: Robot(int n_joints,Eigen::MatrixXd m1, float j_min_limits[], float j_max_limits[])
{
    no_of_joints=n_joints;
    Dh_params.resize(no_of_joints,4);
    Dh_params=m1;
    joint_min_limits=new float[n_joints];
    joint_max_limits=new float[n_joints];
    for(int i=0;i<n_joints;i++)
    {
        joint_min_limits[i]=j_min_limits[i];
        joint_max_limits[i]=j_max_limits[i];
    }
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

Robot::~Robot()
{
    delete[] joint_max_limits;
    delete[] joint_min_limits;
}

//std::vector<float> Inverse_kinematics(float cartesian_coordinates[6])
//{
//    return NULL;
//}

void Robot::print() {
    for (int i=0;i<no_of_joints;i++)
    {
        for (int j=0;j<4;j++)
            std::cout<<Dh_params(i,j)<<std::endl;
    }
}
