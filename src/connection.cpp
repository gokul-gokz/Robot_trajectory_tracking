#include "Connection.h"
#include "iostream"

Connection::Connection()
{
    socket_status=true;
    socket_open=false;
    std::cout<<"Socket connection established";
}

Connection::~Connection()
{
    socket_status=false;
    std::cout<<"Socket connection destroyed";
}

int Connection::open()
{
    if(socket_status) {
        socket_open = true;
        return 1;
    }
    else
        return 0;
}

int Connection::close()
{
    socket_open=false;
    return 1;
}

int Connection::send(std::vector<unsigned char> &data)
{
    int success=0;
    if(open())
    {
        std::cout<<data[0];
        success= close();

    }
    return success;

}

int Connection::receive(std::vector<unsigned char> &data)
{
    int success=0;
    if(open())
    {
        std::vector<float> data;
        success= close();

    }
    return success;
}
