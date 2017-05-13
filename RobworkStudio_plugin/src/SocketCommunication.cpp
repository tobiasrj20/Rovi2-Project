#include "SocketCommunication.hpp"

void SocketCommunication::serverThread(){

    std::cout << "Socket server thread started" << std::endl;

    int port = 50000;
    int socket_fd, connection_fd;
    struct sockaddr_in server_address, client_address;
    socklen_t client_address_length = sizeof(client_address);
    // Create socket
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0)
        error("ERROR opening socket");

    // Setup server address
    bzero((char *) &server_address, sizeof(server_address));
    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = INADDR_ANY;
    server_address.sin_port = htons(port);

    // Bind the socket to the address
    if (bind(socket_fd, (struct sockaddr *) &server_address, sizeof(server_address)) < 0)
        error("ERROR on binding");

        // Begin listening for connections
        listen(socket_fd,1);

        connection_fd = accept(socket_fd, (struct sockaddr *) &client_address, &client_address_length);
        if (connection_fd < 0)
            error("ERROR on accept of connection");

        while(!stop) {

            // Receive 100-byte packet
            int receivedBytes;
            for(receivedBytes = 0; receivedBytes < 100; ) {

                int status = recv(connection_fd,
                                   buffer+receivedBytes,
                                   sizeof(buffer)-receivedBytes, 0);

                if ( status > 0 ) {
                    receivedBytes += status;
                    //printf("Bytes received: %d\n", status);
                }
                else {
                    //printf("Connection failed with : %d\n", iResult);
                    //break;
                }
            }
            if(receivedBytes >= 100) {
                //for(int i = 0 ; i < 100 ; i++ ){
                 //     cout << buffer[i] ;
               // }

                pthread_mutex_lock( &mtx );
                    strcpy(out_buffer,buffer);
                    data_ready = true;
                pthread_mutex_unlock( &mtx );

            } else {
                //break;
            }
        }

    close(connection_fd);
    close(socket_fd);
}

void SocketCommunication::runServerThread(){
    stop = false;
    std::thread *first = new std::thread(&SocketCommunication::serverThread, this);
}

void SocketCommunication::stopThreads(){
    stop = true;
}

bool SocketCommunication::createClient(int port)
{
       struct sockaddr_in server_address;
       struct hostent *server;

       // Parse input
       server = gethostbyname("127.0.0.1");
       //port = 112;

       if (server == NULL) {
           fprintf(stderr,"ERROR, no such host\n");
           exit(0);
       }

       // Open socket
       socket_fd = socket(AF_INET, SOCK_STREAM, 0);
       if (socket_fd < 0)
           std::cout << ("ERROR opening socket");

       // Setup address
       bzero((char *) &server_address, sizeof(server_address));
       server_address.sin_family = AF_INET;
       bcopy((char *)server->h_addr, (char *)&server_address.sin_addr.s_addr, server->h_length);
       server_address.sin_port = htons(port);

       // Connect the socket to the address
       if (connect(socket_fd,(struct sockaddr *)&server_address,sizeof(server_address)) < 0)
           std::cout << ("ERROR connecting");

       return 0;
}

int SocketCommunication::sendM(string message){
    int status;
    //FILE* tmpf = tmpfile();
    bzero(buffer,100);
    strncpy(buffer, message.c_str(), sizeof(buffer));
    // Send the message
    status = write(socket_fd,buffer,strlen(buffer));
    if (status < 0)
         std::cout << ("ERROR writing to socket");
    return 0;
}

void SocketCommunication::disconnect() {

}

bool SocketCommunication::dataReady(){
    bool return_bool = false;
    pthread_mutex_lock( &mtx );
        if(data_ready){
            return_bool = true;
            data_ready = false;
        }
    pthread_mutex_unlock( &mtx );
    return return_bool;
}

void SocketCommunication::receive(string& bffr) {
    pthread_mutex_lock( &mtx );
    bffr = buffer;
    pthread_mutex_unlock( &mtx );
}

void SocketCommunication::error(string msg)
{
    cout << msg << endl;
    exit(1);
}
