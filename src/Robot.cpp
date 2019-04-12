#include<Robot.h>

Robot:: Robot(int n_joints,float l_length[],float j_offset[], float l_offset[], float j_angle[])
{
    no_of_joints=n_joints;
    link_lengths=new float[n_joints];
    joint_offset=new float[n_joints];
    link_offset=new float[n_joints];
    joint_angle=new float[n_joints];
    for (int i=0;i<n_joints;i++)
    {
        link_lengths[i]=l_length[i];
        joint_offset[i]=j_offset[i];
        link_offset[i]=l_offset[i];
        joint_angle[i]=j_angle[i];
    }

}

void Robot::print() {
    for (int i=0;i<no_of_joints;i++)
    {
        std::cout<<"Link_length=";
        std::cout<<link_lengths[i]<<std::endl;

    }
}
int main()
{
    float a[2]={2,2};
    Robot R1(2,a,a,a,a);
    R1.print();
    return 0;
}