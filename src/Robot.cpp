#include<Robot.h>
#include<math.h>

Robot:: Robot(int n_joints,Eigen::MatrixXd m1, float j_min_limits[], float j_max_limits[])
{
    no_of_joints=n_joints;
    Dh_params.resize(no_of_joints,4);\
    HT_BasetoEF.resize(4,4);
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
    //Eigen::MatrixXd Base_to_EFF(4,4);
    for (int i=0;i<no_of_joints;i++)
    {
        Eigen::MatrixXd T(4,4);
        T(0,0)=cos(joint_angles[i]);
        T(0,1)=-sin(joint_angles[i])*cos(Dh_params(i,0));
        T(0,2)=sin(joint_angles[i])*sin(Dh_params(i,0));
        T(0,3)=Dh_params(i,1)*cos(joint_angles[i]);
        T(1,0)=sin(joint_angles[i]);
        T(1,1)=cos(joint_angles[i])*cos(Dh_params(i,0));
        T(1,2)=-cos(joint_angles[i])*sin(Dh_params(i,0));
        T(1,3)=Dh_params(i,1)*sin(joint_angles[i]);
        T(2,0)=0;
        T(2,1)=sin(Dh_params(i,0));
        T(2,2)=cos(Dh_params(i,0));
        T(2,3)=Dh_params(i,2);
        T(3,0)=0;
        T(3,1)=0;
        T(3,2)=0;
        T(3,3)=1;

        if(i==0)
        {
            HT_BasetoEF=T;
        }
        else
        {
            HT_BasetoEF=HT_BasetoEF*T;
        }
        std::cout<<HT_BasetoEF<<std::endl;
    }

}

Robot::~Robot()
{
    delete[] joint_max_limits;
    delete[] joint_min_limits;
}

void Robot::Inverse_kinematics(float cartesian_positions[])
{
    float theta1,theta2,theta3,D;
    float a1=Dh_params(0,1);
    float a2=Dh_params(1,1);
    float a3=Dh_params(2,1);
    float alpha1=Dh_params(0,0);
    float alpha2=Dh_params(1,0);
    float alpha3=Dh_params(2,0);
    float x=cartesian_positions[0];
    float y=cartesian_positions[1];
    float z=cartesian_positions[2];
    std::vector<float> IK_solution;
    theta1=atan2(HT_BasetoEF(1,3),HT_BasetoEF(0,3));
    D=(pow(x-a1*cos(theta1),2)+pow(y-a1*sin(theta1),2)+pow(z,2)-pow(a2,2)-pow(a3,2))/(2*a2*a3);
    theta3=atan2(-sqrt(1-pow(D,2)),D);
    theta2=atan2(z,sqrt(pow(x-a1*cos(theta1),2)+pow(y-a1*sin(theta1),2)))-atan2(a3*sin(theta3),a2+a3*cos(theta3));
    std::cout<<"theta1="<<theta1<<std::endl;
    std::cout<<"theta2="<<theta2<<std::endl;
    std::cout<<"theta3="<<theta3<<std::endl;
    IK_solution.push_back(1);
    IK_solution.push_back(theta2);
    IK_solution.push_back(theta3);
    // IK_solution;
}

void Robot::print() {
    for (int i=0;i<no_of_joints;i++)
    {
        for (int j=0;j<4;j++)
            std::cout<<Dh_params(i,j)<<std::endl;
    }
}
