#ifndef FINAL_STATEMACHINE_H
#define FINAL_STATEMACHINE_H

#include <ros/ros.h>
#include <luh_youbot_controller_api/controller_api.h>

//messages
#include "tld_msgs/BoundingBox.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/Path.h>
#include "std_msgs/Bool.h"
//#include <costmap_2d/costmap_2d_ros.h>
//#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <angles/angles.h>
#include <sensor_msgs/JointState.h>
//#include <base_local_planner/world_model.h>
//#include <base_local_planner/costmap_model.h>


class Statmachine
{
public:

    enum states {
        startState =0,
        searchState,
        getState,
        moveToTableState,
        stackState,
        endState,
        stealState

    };

     Statmachine();
     ~Statmachine();

    void process();
    void shutdown();


protected:

    void moveBase(double xdist, double ydist,double zdist);
    void Gripper(float width);
    //void followCube();
    void getCube();
    void gripCube(float x, float y);
    void gripCubeSteal();
    void stack();
    void stealCube();


    //callbacks
    void tracker0(const tld_msgs::BoundingBox &cambox0);
    void tracker1(const tld_msgs::BoundingBox &cambox1);
    void tracker2(const tld_msgs::BoundingBox &cambox2);
    void tracker3(const tld_msgs::BoundingBox &cambox3);
    void tracker4(const tld_msgs::BoundingBox &cambox4);
    void tracker5(const tld_msgs::BoundingBox &cambox5);
    void tracker6(const tld_msgs::BoundingBox &cambox6);
    void tracker7(const tld_msgs::BoundingBox &cambox7);
    void tracker8(const tld_msgs::BoundingBox &cambox8);

    void thetaCallback(const std_msgs::String &thetaString);
    void scanValues(const sensor_msgs::LaserScan &laser);
    void scanValues_back(const sensor_msgs::LaserScan &laserback);
    void SchiriCallback(const std_msgs::Bool &Schiri);
    void HindernisCallback(const nav_msgs::Path &Hindernis);
    void amclCallback(const geometry_msgs::PoseWithCovarianceStamped &amcl);
    void GripperIstCallback(const sensor_msgs::JointState &gripper_ist);

    //functions
    float Kollisionsvermeidung(float y);
    void roughOrientation();
    void stackCubes();
    void MoveToTable();
    void MoveToTableSteal();
    void simpleGoal(float x, float y, float z, float w);

    states m_NextState;

    double m_Dist;

    ros::NodeHandle n;

    //subscriber
    ros::Subscriber theta_subscriber;
    ros::Subscriber trackerSubscriber0;
    ros::Subscriber trackerSubscriber1;
    ros::Subscriber trackerSubscriber2;
    ros::Subscriber trackerSubscriber3;
    ros::Subscriber trackerSubscriber4;
    ros::Subscriber trackerSubscriber5;
    ros::Subscriber trackerSubscriber6;
    ros::Subscriber trackerSubscriber7;
    ros::Subscriber trackerSubscriber8;

    ros::Subscriber laserTrackerFront;
    ros::Subscriber laserTrackerBack;
    ros::Subscriber Schiri_subscriber;
    ros::Subscriber Hindernis_subschriber;
    ros::Subscriber amclSubscriber;
    ros::Subscriber GripperIstSubscriber;



    //publisher
    ros::Publisher simpleGoalPublisher;


    double distCubeImageMiddle;

    youbot_api::YoubotArm m_Arm;
    youbot_api::YoubotBase m_Base;
    youbot_api::YoubotGripper m_Gripper;

    double m_CurrentTheta;

    bool accessTheta;
    int phase;
    int cubePhase;
    int stealPhase;
    int stealCounter;
    int orientation;
    int counter1;
    int counter2;
    ros::Time begin;
    bool cubeFound;
    bool newData;
    bool schiriGo;
    int row1;
    int row2;
    int stackCounter;
    bool table_right;
    bool cubeGrip;
    int winkel;

    float grip_width;
    double secs2;

    float hin_x;
    float hin_y;
    float hin_z;
    float driveX;
    float driveY;

    double poseX;
    double poseY;
    double poseZ;

    float cam_X0;
    float cam_Y0;
    float cam_X1;
    float cam_Y1;
    float cam_X2;
    float cam_Y2;
    float cam_X3;
    float cam_Y3;
    float cam_X4;
    float cam_Y4;
    float cam_X5;
    float cam_Y5;
    float cam_X6;
    float cam_Y6;
    float cam_X7;
    float cam_Y7;
    float cam_X8;
    float cam_Y8;

    int cubeGround;
    int cubeMiddle;
    int cubeTop;

    float theta;

    double distFront;
    double distBack;

    std::vector<float> laserDatenX;
    std::vector<float> laserDatenY;
    std::vector<float> laserDatenX_B;
    std::vector<float> laserDatenY_B;
    std::vector<float> laserDaten_init;

    //float pose_1[4];


    int m_ImageHeight;
    int m_ImageWidth;


};

#endif // FINAL_STATEMACHINE_H
