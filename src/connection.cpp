#include "Connection.h"
#include "iostream"
#include <stdlib.h>

Connection::Connection(int data_s)
{
    //Initialize the socket status and data size of teh buffer
    data_size=data_s;
    socket_status=true;
    socket_open=false;

    std::cout<<"Socket connection established"<<std::endl;
}

Connection::~Connection()
{
    //Destroy the connection
    socket_status=false;
    std::cout<<"Socket connection destroyed"<<std::endl;
}

int Connection::open()
{
    //open the connection if the socket status is good
    if(socket_status) {
        socket_open = true;
        return 1;
    }
    else
        return 0;
}

int Connection::close()
{
    //Close the connection
    socket_open=false;
    return 1;
}

int Connection::send(std::vector<unsigned char> &data)
{
    //Function to send the data to the robot (simulated)
    int success=0;
    if(open())
    {
        success= close();

    }
    return success;

}

int Connection::receive(std::vector<unsigned char> &data)
{
    //Function to receive data from the robot
    int success=0;
    if(open())
    {
        for(int i=0;i<data_size;i++)
        {
            buffer_data.clear();

            //buffer_data.push_back((float)data[i]);
        }

        success= close();

    }
    return success;
}
