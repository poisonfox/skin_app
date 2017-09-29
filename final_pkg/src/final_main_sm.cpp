#include <signal.h>
#include <ros/ros.h>

#include "final_statemachine.h"


void termination_handler(int signum);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rcsm1_node");
    ros::AsyncSpinner spinner(1);
    struct sigaction action;


    /* Set up the structure to specify the action */
    action.sa_handler = termination_handler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    sigaction(SIGINT, &action, NULL);
    Statmachine* ssm = new Statmachine();

    spinner.start();
    ssm->process();
    spinner.stop();
    delete ssm;


    return 0;
}

void termination_handler(int signum)
{
    ros::shutdown();
}
