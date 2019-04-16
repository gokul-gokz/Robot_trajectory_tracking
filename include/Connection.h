#include <vector>
class Connection
{
    private:
        bool socket_status;
        bool socket_open;

    public:
        Connection();
        ~Connection();
	
        int open();
        int close();

        int send(std::vector<unsigned char> &data);


	//send data to the robot. use explicit pointer convertion

        int receive(std::vector<unsigned char> &data);
	//receive state of the robot. record to data. use explicit pointer convertion
};
