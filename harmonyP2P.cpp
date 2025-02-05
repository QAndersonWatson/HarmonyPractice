#include <iostream> // Allows you to display output on the screen and read input from the keyboard
#include <thread> 
#include <cstring> // Allows you to perform tasks on arrays and C-style strings
#include <chrono>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sstream>
#include "research_interface.h"
#include "overrides.h"

#define PORT 12346
#define BUFFER_SIZE 2048
#define HARMONY_IP "192.168.2.1"
