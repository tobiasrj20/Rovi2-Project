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
#include <netdb.h>
#include <thread>
#include <mutex>
#include <pthread.h>
#include <boost/lockfree/spsc_queue.hpp>
#include <sstream>

using namespace std;

class SocketCommunication
{
    public:
        int socket_fd;
        int sendM(string message);
        void disconnect();
        void receive(string &bffr);
        bool createClient(int port);
        void error(string msg);
        void serverThread();
        void runServerThread();
        bool dataReady();

        char buffer[100];
        char out_buffer[100];
        pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
        bool data_ready = false;

    private:


};
