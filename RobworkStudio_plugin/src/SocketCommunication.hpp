#pragma once
#include <iostream>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

using namespace std;

class SocketCommunication
{
    public:
        bool connect(int port);
        void disconnect();
        void receive();
        bool create_client(int port);
        void error(string msg);
    private:
};
