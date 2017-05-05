#include "SocketCommunication.hpp"

bool SocketCommunication::connect(int port)
{
    int socket_fd, connection_fd;
    char buffer[256];
    struct sockaddr_in server_address, client_address;
    socklen_t client_address_length = sizeof(client_address);
    int status;

    // Check that a port was provided
    if (port <= 0) {
     error("ERROR, no port provided\n");
    }

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
    listen(socket_fd,5);

    // Obs maybe this should be made non blocking thread: http://stackoverflow.com/questions/12936788/accept-in-a-thread-to-avoid-blocking-socket
    // Block until a connection is established
    connection_fd = accept(socket_fd, (struct sockaddr *) &client_address, &client_address_length);
    if (connection_fd < 0)
        error("ERROR on accept of connection");

    // Read received packet
    bzero(buffer,256);
    status = read(connection_fd,buffer,255);
    if (status < 0)
        error("ERROR reading from socket");
    printf("Here is the message: %s\n",buffer);

     // Send reply
    status = write(connection_fd,"I got your message",18);
    if (status < 0)
        error("ERROR writing to socket");

    // Close socket connection
    close(connection_fd);
    close(socket_fd);
    return 0;
}

bool SocketCommunication::create_client(int port)
{
    /*
    int socket_fd, port, status;
    struct sockaddr_in server_address;
    struct hostent *server;
    char buffer[256];

    // Check that host and port was provided
    if (port <= 0) {
       //fprintf(stderr,"usage %s hostname port\n", argv[0]);
       exit(0);
    }

    // Parse input
    port = atoi(argv[2]);
    server = gethostbyname(argv[1]);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }

    // Open socket
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0)
        error("ERROR opening socket");

    // Setup address
    bzero((char *) &server_address, sizeof(server_address));
    server_address.sin_family = AF_INET;
    bcopy((char *)server->h_addr, (char *)&server_address.sin_addr.s_addr, server->h_length);
    server_address.sin_port = htons(port);

    // Connect the socket to the address
    if (connect(socket_fd,(struct sockaddr *)&server_address,sizeof(server_address)) < 0)
        error("ERROR connecting");

    // Block on input from user
    printf("Please enter the message: ");
    bzero(buffer,256);
    fgets(buffer,255,stdin);

    // Send the message
    status = write(socket_fd,buffer,strlen(buffer));
    if (status < 0)
         error("ERROR writing to socket");

    // Block on reply from the server
    bzero(buffer,256);
    status = read(socket_fd,buffer,255);
    if (status < 0)
         error("ERROR reading from socket");

    // Print the reply
    printf("%s\n",buffer);

    return 0;*/
}

void SocketCommunication::disconnect() {

}


void SocketCommunication::receive() {

}

void SocketCommunication::error(string msg)
{
    cout << msg << endl;
    exit(1);
}
