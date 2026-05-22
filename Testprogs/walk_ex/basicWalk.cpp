#include <cmath>
#include <signal.h>
#include <unistd.h>

#include <unitree/robot/go2/sport/sport_client.hpp>

bool stopped = false;

void sigint_handler(int sig)
{
    if (sig == SIGINT)
    {
    stopped = true;
    }
}


int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);


    unitree::robot::go2::SportClient sport_client;
    sport_client.SetTimeout(10.0f); // Set timeout
    sport_client.Init(); // Initialize sport client

    // double t = 0; // Program running time
    // double dt = 0.01; // Control step size

    //basic forward
    sport_client.Move(0.0, 0.0, 2);   
    sleep(1);
    sport_client.Move(0.0, 0.0, 0.0);
}