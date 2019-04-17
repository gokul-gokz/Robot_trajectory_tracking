#ifndef CONNECTION_H
#define CONNECTION_H


#include <vector>

/**
 * @brief Class implementation of connection
 */
class Connection {
private:
    /** Variable to check whether the connection is bad or good*/
    bool socket_status;

    /** Variable to check whether the socket is open or not*/
    bool socket_open;

    /** Variable to store the current position received from the robot*/
    std::vector<float> buffer_data;

    /**Variable to modify the data size of the buffer*/
    int data_size;

public:

    /**
    * @brief Constructor which initialize the connection class with buffer data size
    */
    Connection(int data_s);

    /**
    * @brief Destructor
    */
    ~Connection();

    /**
   * @brief Function to open the socket
     *@return wheteher the socket opening is success or not
   */
    int open();


        /**
    * @brief Function to close the socket
    *@return wheteher the socket closing is success or not
    */
    int close();

        /**
    * @brief Function to send data through the socket
    * @param data to be sent
    *@return sending the data is success or not
    */
    int send(std::vector<unsigned char> &data);


    /**
    * @brief Function to receive data through the socket and update the connection class buffer data with the received data
    * @param received data
    *@return whether the data is successfull received or not
    */
    int receive(std::vector<unsigned char> &data);
};

#endif