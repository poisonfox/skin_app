#include "final_statemachine.h"
#include "luh_youbot_controller_api/controller_api.h"
#include "luh_youbot_controller_api/youbot_arm.h"
#include "luh_youbot_controller_api/youbot_gripper.h"
#include "math.h"
//#include <costmap_2d/costmap_2d_ros.h>
//#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <angles/angles.h>
#include <sensor_msgs/JointState.h>
//#include <base_local_planner/world_model.h>
//#include <base_local_planner/costmap_model.h>

using std::string;
namespace yapi = youbot_api ;

Statmachine::Statmachine()
{
    m_NextState = startState;

    m_Dist= 0.05;
    distCubeImageMiddle = 0.0;

    ros::NodeHandle np("~");
    m_Arm.init(n);
    m_Base.init(n);
    m_Gripper.init(n);

    //publisher
    simpleGoalPublisher = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);

    //subscriber
    theta_subscriber = n.subscribe("/theta",10, &Statmachine::thetaCallback, this);
    trackerSubscriber0 = n.subscribe("/tld_tracked_object1",10, &Statmachine::tracker0, this);
    trackerSubscriber1 = n.subscribe("/tld_tracked_object2",10, &Statmachine::tracker1, this);
    trackerSubscriber2 = n.subscribe("/tld_tracked_object3",10, &Statmachine::tracker2, this);
    trackerSubscriber3 = n.subscribe("/tld_tracked_object4",10, &Statmachine::tracker3, this);
    trackerSubscriber4 = n.subscribe("/tld_tracked_object5",10, &Statmachine::tracker4, this);
    trackerSubscriber5 = n.subscribe("/tld_tracked_object6",10, &Statmachine::tracker5, this);
    trackerSubscriber6 = n.subscribe("/tld_tracked_object7",10, &Statmachine::tracker6, this);
    trackerSubscriber7 = n.subscribe("/tld_tracked_object8",10, &Statmachine::tracker7, this);
    trackerSubscriber8 = n.subscribe("/tld_tracked_object9",10, &Statmachine::tracker8, this);
    amclSubscriber = n.subscribe("amcl_pose",10, &Statmachine::amclCallback, this);
    Schiri_subscriber = n.subscribe("/cd3_referee/start",10, &Statmachine::SchiriCallback, this);
    GripperIstSubscriber = n.subscribe("/arm_1/joint_states",101, &Statmachine::GripperIstCallback,this);
    laserTrackerFront = n.subscribe("/scan", 10, &Statmachine::scanValues, this);
    laserTrackerBack = n.subscribe("/scan_back", 10, &Statmachine::scanValues_back, this);


    accessTheta=false;

    m_ImageHeight = 480;
    m_ImageWidth = 640;
    row1 = 0;                   //vor CD ändern!!!  auf 0
    counter1 = 0;
    row2 = 0;          //vor CD ändern!!!   auf 0
    counter2 = 0;
    poseX =0;
    poseY =0;
    poseZ =0;
    stackCounter =0;
    stealCounter =0;
    winkel = 0;             //vor auf 0 ändern!!!
    cubeFound =false;
    schiriGo = false;
    cubeGrip = false;

    Gripper(0.059);
    cubePhase=1;            //vor CD ändern!!! auf 1
    stealPhase=0;
}


Statmachine::~Statmachine()
{

}


void Statmachine::GripperIstCallback(const sensor_msgs::JointState &gripper_ist){

        grip_width = gripper_ist.position[6];
}

void Statmachine::amclCallback(const geometry_msgs::PoseWithCovarianceStamped &amcl){

    poseX = amcl.pose.pose.position.x;
    poseY = amcl.pose.pose.position.y;
    poseZ = amcl.pose.pose.orientation.z;
}

void Statmachine::tracker0(const tld_msgs::BoundingBox &cambox0){


        cam_X0 = cambox0.x + cambox0.width/2;
        cam_Y0 = cambox0.y + cambox0.height/2;
        if (cambox0.x >1 && cam_X0 > 2 && cambox0.confidence > 0.70){
            secs2 = ros::Time::now().toSec() - begin.toSec();
            newData = true;
            cubeFound = true;
        }

}

void Statmachine::tracker1(const tld_msgs::BoundingBox &cambox1){

        cam_X1 = cambox1.x + cambox1.width/2;
        cam_Y1 = cambox1.y + cambox1.height/2;
        if (cambox1.x >1 && cam_X1 > 2 && cambox1.confidence > 0.70){
            secs2 = ros::Time::now().toSec() - begin.toSec();
            newData = true;
            cubeFound = true;
        }

}

void Statmachine::tracker2(const tld_msgs::BoundingBox &cambox2){

        cam_X2 = cambox2.x + cambox2.width/2;
        cam_Y2 = cambox2.y + cambox2.height/2;
        if (cambox2.x >1 && cam_X2 > 2 && cambox2.confidence > 0.70){
            secs2 = ros::Time::now().toSec() - begin.toSec();
            newData = true;
            cubeFound = true;
        }

}

void Statmachine::tracker3(const tld_msgs::BoundingBox &cambox3){

        cam_X3 = cambox3.x + cambox3.width/2;
        cam_Y3 = cambox3.y + cambox3.height/2;
        if (cambox3.x >1 && cam_X3 > 2 && cambox3.confidence > 0.70){
            secs2 = ros::Time::now().toSec() - begin.toSec();
            newData = true;
            cubeFound = true;
        }

}

void Statmachine::tracker4(const tld_msgs::BoundingBox &cambox4){

        cam_X4 = cambox4.x + cambox4.width/2;
        cam_Y4 = cambox4.y + cambox4.height/2;
        if (cambox4.x >1 && cam_X4 > 2 && cambox4.confidence > 0.70){
            secs2 = ros::Time::now().toSec() - begin.toSec();
            newData = true;
            cubeFound = true;
        }

}

void Statmachine::tracker5(const tld_msgs::BoundingBox &cambox5){

        cam_X5 = cambox5.x + cambox5.width/2;
        cam_Y5 = cambox5.y + cambox5.height/2;
        if (cambox5.x >1 && cam_X5 > 2 && cambox5.confidence > 0.70){
            secs2 = ros::Time::now().toSec() - begin.toSec();
            newData = true;
            cubeFound = true;
        }

}

void Statmachine::tracker6(const tld_msgs::BoundingBox &cambox6){

        cam_X6 = cambox6.x + cambox6.width/2;
        cam_Y6 = cambox6.y + cambox6.height/2;
        if (cambox6.x >1 && cam_X6 > 2 && cambox6.confidence > 0.70){
            secs2 = ros::Time::now().toSec() - begin.toSec();
            newData = true;
            cubeFound = true;
        }

}

void Statmachine::tracker7(const tld_msgs::BoundingBox &cambox7){

        cam_X7 = cambox7.x + cambox7.width/2;
        cam_Y7 = cambox7.y + cambox7.height/2;
        if (cambox7.x >1 && cam_X7 > 2 && cambox7.confidence > 0.70){
            secs2 = ros::Time::now().toSec() - begin.toSec();
            newData = true;
            cubeFound = true;
        }

}

void Statmachine::tracker8(const tld_msgs::BoundingBox &cambox8){

        cam_X8 = cambox8.x + cambox8.width/2;
        cam_Y8 = cambox8.y + cambox8.height/2;
        if (cambox8.x >1 && cam_X8 > 2 && cambox8.confidence > 0.70){
            secs2 = ros::Time::now().toSec() - begin.toSec();
            newData = true;
            cubeFound = true;
        }

}

void Statmachine::SchiriCallback(const std_msgs::Bool &Schiri){

    ROS_INFO("Goooo!!!");
    schiriGo = true;
}

void Statmachine::thetaCallback(const std_msgs::String &thetaString){

    if(!accessTheta)
    {
        std::string thetaS = thetaString.data.c_str();
        theta = -::atof(thetaS.c_str());
        m_CurrentTheta = (double)-theta;
    }
}


void Statmachine::scanValues(const sensor_msgs::LaserScan &laser)
{

    const float angle_min= -1.65056335926;
    const float angle_max= 1.64442741871;
    const float angle_increment= 0.00613592332229;
    laserDatenX = std::vector<float>();
    laserDatenY = std::vector<float>();

    for (unsigned int i=0; i<laser.ranges.size(); i++)
    {
        if(i == laser.ranges.size()/2.0){
            distFront = laser.ranges[i]*sin(angle_min + angle_increment*i);
        }
        laserDatenY.push_back(laser.ranges[i]*sin(angle_min + angle_increment*i));
        laserDatenX.push_back(laser.ranges[i]*cos(angle_min + angle_increment*i));
    }

}

void Statmachine::scanValues_back(const sensor_msgs::LaserScan &laserback)
{
    laserDatenX_B = std::vector<float>();
    laserDatenY_B = std::vector<float>();

    const float angle_min= -1.65056335926;
    const float angle_max= 1.64442741871;
    const float angle_increment= 0.00613592332229;

    for (unsigned int i=0; i<laserback.ranges.size(); i++)
    {
        //       if (laserback.ranges[i]>0.02 && laserback.ranges[i]<6.0){

        laserDatenY_B.push_back(laserback.ranges[i]*sin(angle_min + angle_increment*i));
        laserDatenX_B.push_back(laserback.ranges[i]*cos(angle_min + angle_increment*i));
        //      }
    }

}

void Statmachine::process()
{

    while(m_NextState != endState && ros::ok())
    {
        switch(m_NextState)
        {
        case startState:
        {
            m_Arm.moveToPose("FINAL_WINDSCHNITTIG");
            m_Arm.waitForCurrentAction();
            while (m_Arm.isBusy()){

            }
            //init or whatever
      //      ROS_INFO("In State Start");
            if(schiriGo){
                m_NextState = searchState;
            }
            break;
        }

        case searchState:
        {
            ROS_INFO("In State Search");
            if (row1==5 && row2==5 && cubePhase< 10){
                row1 =1;
            }
            else if (row1==5 && row2==5){ //+ Zeitbedingung
                m_NextState = stealState;
                break;
            }

            roughOrientation();
            m_Arm.moveToPose("FINAL_SEARCH");
            m_Arm.waitForCurrentAction();
            while (m_Arm.isBusy()){

            }

            while(!cubeFound && ros::ok() && row1 <5)
            {
                ros::spinOnce();

                if ((cubePhase > 3 || row2 ==5) && row1<5){         //hinteres Depot

                    counter1 ++;
                    float y = Kollisionsvermeidung(0.1);
                    if (y !=0){
                        moveBase(0,y,0);
                        usleep((500000));
                    }

                    else{
                        counter1 =0;
                        row1 ++;
                        ROS_INFO("ahhh, anderer Youbot!");
                        roughOrientation();
                    }
                    if (counter1 > 19)
                    {
                        ROS_INFO("row1 ++...");
                        counter1 =0;
                        row1 ++;
                        roughOrientation();
                    }
                }

                else if(row2 !=5){                  //vorderes Depot
                    counter2 ++;
                    float y = Kollisionsvermeidung(-0.1);
                    if (y !=0){
                        moveBase(0,y,0);
                        usleep((500000));
                    }

                    else{
                        counter2 =0;
                        row2 ++;
                        ROS_INFO("ahhh, anderer Youbot!");
                        roughOrientation();
                    }
                    if (counter2 > 10)
                    {
                        ROS_INFO("row1 ++...");
                        counter2 =0;
                        row2 ++;
                        roughOrientation();
                    }
                }
            }

            m_NextState = getState;
            break;
        }
        case getState:
        {
            ROS_INFO("In State Get");

            getCube();

            if (m_NextState != searchState){
                if ((cubePhase-1)%3==0 && cubeGrip){

                    m_NextState = moveToTableState;
                }
                 else {
                     m_NextState = searchState;
                 }

                ROS_INFO("fertig");
            }
            break;
        }
        case moveToTableState:
        {
            MoveToTable();
            m_NextState = stackState;
            break;
        }

        case stackState:
        {
            stack();
            usleep(400000);
            if ((cubePhase + stealPhase<12) && (row1 <5 || row2 <5)){
                 m_NextState = searchState;
            }
            else{
                m_NextState = stealState;
            }
            break;
        }

        case stealState:
        {
            MoveToTableSteal();

            stealCube();
            stealCounter++;
            if ((cubePhase+stealPhase)%3==1 && cubeGrip){
                m_NextState = stackState;
            }
            cubeGrip = false;
            break;
        }
        case endState:
        {
            break;
        }

    }
}
}


void Statmachine::roughOrientation()
{
    if ((cubePhase > 3 || row2 ==5) && row1<5){
        simpleGoal(11.8 + row1*0.1, 10.6 + counter1*0.1, 0.0188, 0.9998);
    }
    else if(row2 !=5){
        simpleGoal(9.08 - row2*0.1, 11.167 + counter2*0.1, 1, -0.022);
    }
    // 1.1 Daten in einen bzw zwei Vektor(en) packen
    std::vector<float> orthoX;
    std::vector<float> orthoY;
    std::vector<float> sideX;
    std::vector<float> sideY;
    orthoX.clear();
    orthoY.clear();

    usleep(200000);
    ros::spinOnce();
    for (unsigned int i=0; i<laserDatenX.size();i++)
    {
        if(laserDatenX[i]>0.05 && fabs(laserDatenY[i])<0.2){
            orthoX.push_back(laserDatenX[i]);
            orthoY.push_back(laserDatenY[i]);
        }
    }
    if (poseX > 10){
    // 1.2 Mittelwert befreien
    double x_ =0;
    double y_ =0;
    for (unsigned int i=0; i<orthoX.size();i++)
    {
        x_= x_ + orthoX[i];
        y_= y_ + orthoY[i];
    }
    x_ = x_/orthoX.size();
    y_ = y_/orthoX.size();

    for (unsigned int i=0; i<orthoX.size();i++)
    {
        orthoX[i] = orthoX[i] - x_;
        orthoY[i] = orthoY[i] - y_;
    }

    // 1.3 alpha_ berechnen

    float alpha_=0;
    for (unsigned int i=0; i<orthoX.size();i++)
    {
        alpha_ = alpha_ + atan (orthoX[i]/orthoY[i]);
    }
    alpha_ = alpha_/orthoX.size();

    // 1.4 drehen

    moveBase(0,0,-alpha_);
    }

    cubeFound = false;
}

void Statmachine::moveBase(double xdist, double ydist, double zdist)
{
    m_Base.move(xdist,ydist, zdist);
    m_Base.waitForCurrentAction();
    while(m_Base.isBusy()){

    }

}


void Statmachine::getCube()
{

    usleep(500000);
    ros::spinOnce();

    float x=0;
    float y=0;
    float delta = 30;
    float min =1000000000000;
    bool poseOK = false;

    while(!poseOK && ros::ok())
    {
        ros::spinOnce();

        float x=0;
        float y=0;

        double center_y = 308;
        double center_x = 330;


           if ((cam_X0-center_x)*(cam_X0-center_x)+(cam_Y0-center_y)*(cam_Y0-center_y)<min && cam_X0>2)
             {
                x=cam_X0;
                y=cam_Y0;
             }
           if ((cam_X1-center_x)*(cam_X1-center_x)+(cam_Y1-center_y)*(cam_Y1-center_y)<min && cam_X1>2)
           {
                x=cam_X1;
                y=cam_Y1;
           }
           if ((cam_X2-center_x)*(cam_X2-center_x)+(cam_Y2-center_y)*(cam_Y2-center_y)<min && cam_X2>2)
           {
                x=cam_X2;
                y=cam_Y2;
           }
           if ((cam_X3-center_x)*(cam_X3-center_x)+(cam_Y3-center_y)*(cam_Y3-center_y)<min && cam_X3>2)
           {
                x=cam_X3;
                y=cam_Y3;
           }
           if ((cam_X4-center_x)*(cam_X4-center_x)+(cam_Y4-center_y)*(cam_Y4-center_y)<min && cam_X4>2)
           {
                x=cam_X4;
                y=cam_Y4;
           }
           if ((cam_X5-center_x)*(cam_X5-center_x)+(cam_Y5-center_y)*(cam_Y5-center_y)<min && cam_X5>2)
           {
                x=cam_X5;
                y=cam_Y5;
           }
           if ((cam_X6-center_x)*(cam_X6-center_x)+(cam_Y6-center_y)*(cam_Y6-center_y)<min && cam_X6>2)
           {
                x=cam_X6;
                y=cam_Y6;
           }
           if ((cam_X7-center_x)*(cam_X7-center_x)+(cam_Y7-center_y)*(cam_Y7-center_y)<min && cam_X7>2)
           {
                x=cam_X7;
                y=cam_Y7;
           }
           if ((cam_X8-center_x)*(cam_X8-center_x)+(cam_Y8-center_y)*(cam_Y8-center_y)<min && cam_X8>2)
           {
                x=cam_X8;
                y=cam_Y8;
           }


        ROS_INFO("%f,%f",x,y);

        m_Base.abortCurrentAction();

        double diffX = center_x - x;
        double diffY = center_y - y;

        usleep(500000);
        if (diffX > -delta && diffX < delta && diffY > -2*delta && diffY < 2*delta )
        {
            m_Base.abortCurrentAction();
            poseOK = true;
            ROS_INFO("Ziel erreicht");
            gripCube(diffY,diffX);

            break;
        }
        else{

            if(diffY>100){
                diffY = 100;
            }
            if(diffY<-100){
                diffY = -100;
            }

            if(diffX>100){
                diffX = 100;
            }
            if(diffX<-100){
                diffX = -100;
            }

            if (x>2){
                moveBase( diffY*0.0004, diffX*0.0005, 0.0);
                while (m_Base.isBusy()){
                }
                usleep(700000);
            }

        }
        if(x<2){
            cubeFound = false;
            m_NextState = searchState;
            poseOK = true;
        }
    }

}

void Statmachine::gripCube(float x, float y){
   // usleep(500000);
    cubeGrip = false;
    ros::spinOnce();
    ROS_INFO("Entfernung x = %f",x);
    ROS_INFO("Entfernung y = %f",y);
    luh_youbot_kinematics::JointPosition cuPos;
    float q1 = 0.585 - y *0.001/0.225;

    ROS_INFO("Winkel q1 = %f",q1);
    cuPos.setQ1(q1);
    cuPos.setQ5(theta-y*0.001/0.225);

    if (x>15){
        cuPos.setQ2(1.131);
        cuPos.setQ3(1.4416);
        cuPos.setQ4(0.555);
        ROS_INFO("Boden");
    }
    else if (x<-10){
        cuPos.setQ2(1.0228);
        cuPos.setQ3(1.8256);
        cuPos.setQ4(0.268);
        ROS_INFO("Boden2");
    }
    else{
        cuPos.setQ2(1.0804);
        cuPos.setQ3(1.627);
        cuPos.setQ4(0.4608);
        ROS_INFO("Boden1,5");
    }
    luh_youbot_kinematics::CartesianPosition cuPos_cart = cuPos.toCartesian();

    m_Arm.moveToPose(cuPos, yapi::MotionMode::INTERPOLATE);
    m_Arm.waitForCurrentAction();
    while (m_Arm.isBusy()) {
    }
    Gripper(0.005);
  //  usleep(500000);
    cuPos.setQ2(0.9);

    cuPos_cart = cuPos.toCartesian();
    m_Arm.moveToPose(cuPos, yapi::MotionMode::INTERPOLATE);
    m_Arm.waitForCurrentAction();
    while (m_Arm.isBusy()) {
    }

    ROS_INFO("%f",grip_width);

    if (grip_width>0.0022){
        cubeGrip = true;
        switch (cubePhase%3) {
        case 1:
            m_Arm.moveToPose("FINAL_LOSE_CUBE1");
            break;
        case 2:
            m_Arm.moveToPose("FINAL_LOSE_CUBE2");
            break;
        case 0:
            m_Arm.moveToPose("FINAL_LOSE_CUBE3");
            break;
        default:
            break;
        }
        cubePhase++;
        m_Arm.waitForCurrentAction();
        while (m_Arm.isBusy()) {


        }
     //   usleep(500000);
    }
    Gripper(0.059);

    if(cubePhase%3==1 && cubeGrip)
    {
        m_Arm.moveToPose("FINAL_WINDSCHNITTIG");
    }
    else{
        m_Arm.moveToPose("FINAL_SEARCH");
    }

    m_Arm.waitForCurrentAction();
    cubeFound = false;
}

void Statmachine::MoveToTable(){

    //ros::spinOnce();
    //  if (Zeit >50)

    float pose_1[4]= {11.42, 11.65614, 0.998544, 0.05393};
    float pose_2[4]= {11.0964, 11.0, 0.9238, 0.38299};
    float pose_3[4]= {10.2545, 10.74, 0.7,0.7733};
    float pose_4[4]= {9.57446, 11.11, 0.32705, 0.945007};
    float pose_5[4]= {9.3342, 11.71366, -0.0297, 0.99956};
    float pose_6[4]= {9.5259,12.445, -0.4033, 0.91506};
    float pose_7[4]= {10.3006, 12.7656, -0.6,0.7461};
    float pose_8[4]= {11.0429,12.4798,-0.8708, 0.4916};

    int pose;

    switch (cubePhase + stealPhase) {
    case 4:
        pose = 4;
        break;
    case 7:
        pose =3;
        break;
    case 10:
        pose = 2;
        break;
    default:
        pose =3;
    }

    pose = pose + winkel;
    if (pose>8){
        pose = pose - 8;
    }

    switch (pose) {
    case 1:
        simpleGoal(pose_1[0], pose_1[1], pose_1[2], pose_1[3]);
        break;
    case 2:
        simpleGoal(pose_2[0], pose_2[1], pose_2[2], pose_2[3]);
        break;
    case 3:
        simpleGoal(pose_3[0], pose_3[1], pose_3[2], pose_3[3]);
        break;
    case 4:
        simpleGoal(pose_4[0], pose_4[1], pose_4[2], pose_4[3]);
        break;
    case 5:
        simpleGoal(pose_5[0], pose_5[1], pose_5[2], pose_5[3]);
        break;
    case 6:
        simpleGoal(pose_6[0], pose_6[1], pose_6[2], pose_6[3]);
        break;
    case 7:
        simpleGoal(pose_7[0], pose_7[1], pose_7[2], pose_7[3]);
        break;
    case 8:
        simpleGoal(pose_8[0], pose_8[1], pose_8[2], pose_8[3]);
        break;
    default:
        break;
    }

    usleep(400000);
    ros::spinOnce();
    std::vector<float> orthoX;

    for (unsigned int i=0; i<laserDatenX.size();i++)
    {
        if(laserDatenX[i]>0.05 && laserDatenX[i]<0.60 && fabs(laserDatenY[i])<0.08){
            orthoX.push_back(laserDatenX[i]);
        }
    }

    double x_ =0;
    for (unsigned int i=0; i<orthoX.size();i++)
    {
        x_= x_ + orthoX[i];
    }
    x_ = x_/orthoX.size();
    moveBase(x_-0.12,0,0);

    m_Arm.moveToPose("FINAL_SEARCH");
    m_Arm.waitForCurrentAction();
    while (m_Arm.isBusy()) {
    }

    // Schauen, ob da ein Würfel ist

    float min =1000;
    bool poseOK = false;

    while(!poseOK && ros::ok())
    {
        usleep(400000);
        float x=0;
        ros::spinOnce();

        double center_x = 310;


           if (fabs(cam_X0-center_x)<min && cam_X0>2)
             {
                x=cam_X0;
             }
           if (fabs(cam_X1-center_x)<min && cam_X1>2)
           {
                x=cam_X1;
           }
           if (fabs(cam_X2-center_x)<min && cam_X2>2)
           {
                x=cam_X2;
           }
           if (fabs(cam_X3-center_x)<min && cam_X3>2)
           {
                x=cam_X3;
           }
           if (fabs(cam_X4-center_x)<min && cam_X4>2)
           {
                x=cam_X4;
           }
           if (fabs(cam_X5-center_x)<min && cam_X5>2)
           {
                x=cam_X5;
           }
           if (fabs(cam_X6-center_x)<min && cam_X6>2)
           {
                x=cam_X6;
           }
           if (fabs(cam_X7-center_x)<min && cam_X7>2)
           {
                x=cam_X7;
           }
           if (fabs(cam_X8-center_x)<min && cam_X8>2)
           {
                x=cam_X8;
           }

           m_Base.abortCurrentAction();

           double diffX = center_x - x;
           float delta = 250;
           ROS_INFO("diffX = %f",diffX);

           if (diffX < -delta || diffX > delta)
           {
               poseOK = true;
               ROS_INFO("Ziel erreicht");
               usleep(200000);
               ros::spinOnce();
               for (unsigned int i=0; i<laserDatenX.size();i++)
               {
                   if(laserDatenX[i]>0.05 && laserDatenX[i]<0.60 && fabs(laserDatenY[i])<0.08){
                       orthoX.push_back(laserDatenX[i]);
                   }
               }

               double x_ =0;
               for (unsigned int i=0; i<orthoX.size();i++)
               {
                   x_= x_ + orthoX[i];
               }
               x_ = x_/orthoX.size();
               moveBase(x_-0.09,0,0);
               break;
           }
           else{
               if (diffX>0){
                    moveBase(0, -0.05, 0.6);
               }
               else{
                   moveBase(0, 0.05, -0.6);
               }
               while (m_Base.isBusy()){
               }
               usleep(300000);
            }
    }
}


void Statmachine::stack()
{

    m_Arm.moveToPose("FINAL_GRAP_CUBE1");
    m_Arm.waitForCurrentAction();
    Gripper(0.005);

    luh_youbot_kinematics::JointPosition cuPos;
    cuPos.setQ1(0.57596);
    cuPos.setQ2(0.80634);
    cuPos.setQ3(1.21999);
    cuPos.setQ4(1.12574);
    cuPos.setQ5(0);
    m_Arm.moveToPose(cuPos, yapi::MotionMode::INTERPOLATE);
    m_Arm.waitForCurrentAction();
    while (m_Arm.isBusy()) {
    }
    Gripper(0.059);
    m_Arm.moveToPose("FINAL_GRAP_CUBE2");
    m_Arm.waitForCurrentAction();
    Gripper(0.005);
    m_Arm.moveToPose("FINAL_STACK_2");
    m_Arm.waitForCurrentAction();
    Gripper(0.059);
    m_Arm.moveToPose("FINAL_GRAP_CUBE3");
    m_Arm.waitForCurrentAction();
    Gripper(0.005);
    m_Arm.moveToPose("ARM_UP");
    m_Arm.waitForCurrentAction();
    cuPos.setQ1(0.57596);
    cuPos.setQ2(0.663);
    cuPos.setQ3(0.9721);
    cuPos.setQ4(1.5);
    cuPos.setQ5(0);
    m_Arm.moveToPose(cuPos, yapi::MotionMode::INTERPOLATE);
    m_Arm.waitForCurrentAction();
    while (m_Arm.isBusy()) {

    }
    m_Arm.waitForCurrentAction();
    Gripper(0.059);
    m_Arm.moveToPose("FINAL_WINDSCHNITTIG");
    m_Arm.waitForCurrentAction();

}


void Statmachine::Gripper(float width){
    m_Gripper.setWidth(width,false);
    m_Gripper.waitForCurrentAction();
}

void Statmachine::simpleGoal(float x, float y, float z, float w){

    geometry_msgs::PoseStamped Pose;
    Pose.header.frame_id="map";
    Pose.pose.position.z = 0;
    Pose.pose.orientation.x = 0;
    Pose.pose.orientation.y = 0;
    Pose.pose.position.x = x;
    Pose.pose.position.y = y;
    Pose.pose.orientation.z = z;
    Pose.pose.orientation.w = w;

    simpleGoalPublisher.publish(Pose);
    m_Base.waitForCurrentAction();
    int i=0;
    bool poseOK = true;

    double secs = ros::Time::now().toSec();
    while (fabs(Pose.pose.position.x-poseX)>0.16 || fabs(Pose.pose.position.y-poseY)>0.16 || fabs(Pose.pose.orientation.z -poseZ)>0.18){
        if (i==1){
            ROS_INFO("fahre zu simple Goal");
        }
        if(ros::Time::now().toSec()-secs >35){
            poseOK = false;
            break;
        }
        i++;
    }
    usleep(500000);
    if(!poseOK){
        Pose.pose.position.x = Pose.pose.position.x-0.15;
        simpleGoalPublisher.publish(Pose);
        m_Base.waitForCurrentAction();
        int i=0;
        poseOK = true;

        secs = ros::Time::now().toSec();
        while (fabs(Pose.pose.position.x-poseX)>0.20 || fabs(Pose.pose.position.y-poseY)>0.20 || fabs(Pose.pose.orientation.z -poseZ)>0.20){
            if (i==1){
                ROS_INFO("fahre zu simple Goal");
            }
            if(ros::Time::now().toSec()-secs >10){
                poseOK = false;
                break;
            }
            i++;
        }
        if(!poseOK){
            Pose.pose.position.y = Pose.pose.position.y-0.15;
            simpleGoalPublisher.publish(Pose);
            m_Base.waitForCurrentAction();
            int i=0;
            poseOK = true;

            secs = ros::Time::now().toSec();
            while (fabs(Pose.pose.position.x-poseX)>0.20 || fabs(Pose.pose.position.y-poseY)>0.20 || fabs(Pose.pose.orientation.z -poseZ)>0.20){
                if (i==1){
                    ROS_INFO("fahre zu simple Goal");
                }
                if(ros::Time::now().toSec()-secs >10){
                    poseOK = false;
                    break;
                }
                i++;
            }
            if(!poseOK){
                Pose.pose.position.x = Pose.pose.position.x+0.3;
                simpleGoalPublisher.publish(Pose);
                m_Base.waitForCurrentAction();
                int i=0;
                poseOK = true;

                secs = ros::Time::now().toSec();
                while (fabs(Pose.pose.position.x-poseX)>0.20 || fabs(Pose.pose.position.y-poseY)>0.20 || fabs(Pose.pose.orientation.z -poseZ)>0.20){
                    if (i==1){
                        ROS_INFO("fahre zu simple Goal");
                    }
                    if(ros::Time::now().toSec()-secs >10){
                        poseOK = false;
                        break;
                    }
                    i++;
                }
                if(!poseOK){
                    Pose.pose.position.y = Pose.pose.position.y+0.3;
                    simpleGoalPublisher.publish(Pose);
                    m_Base.waitForCurrentAction();
                    int i=0;
                    poseOK = true;

                    secs = ros::Time::now().toSec();
                    while (fabs(Pose.pose.position.x-poseX)>0.20 || fabs(Pose.pose.position.y-poseY)>0.20 || fabs(Pose.pose.orientation.z -poseZ)>0.20){
                        if (i==1){
                            ROS_INFO("fahre zu simple Goal");
                        }
                        if(ros::Time::now().toSec()-secs >10){
                            poseOK = false;
                            break;
                        }
                        i++;
                    }
                }
            }
        }
    }
}

float Statmachine::Kollisionsvermeidung(float y){

    int counterYRF = 0;
    int counterYLF = 0;
    int counterYRB = 0;
    int counterYLB = 0;

    for (unsigned int i=0; i<laserDatenX.size();i++)
    {
        if (i==1)
        {
        }
        // Y-Links_Front Vermeidung
        if(laserDatenX[i]<0.07 && laserDatenY[i]<0.35 && laserDatenY[i]>0 && y >0){
            counterYLF++;
            if(counterYLF>2){
                y=0;
            }
        }
        // Y-Rechts_Front Vermeidung
        if(laserDatenX[i]<0.07 && laserDatenY[i]>-0.35 && laserDatenY[i]<0 && y <0){
            counterYRF++;
            if(counterYRF>2){
                y=0;
            }
        }

        // Y-Rechts_Back Vermeidung
        if(laserDatenX_B[i]<0.07 && laserDatenY_B[i]<0.35 &&  laserDatenY_B[i]>0 && y <0){
            counterYRB++;
            if(counterYRB>2){
                y=0;
            }
        }

        // Y-Links_Back Vermeidung
        if(laserDatenX_B[i]<0.07 && laserDatenY_B[i]>-0.35 &&  laserDatenY_B[i]< 0 && y >0){
            counterYLB++;
            if(counterYLB>2){
                y=0;
            }
        }
    }
    return y;
}

void Statmachine:: MoveToTableSteal(){
    float pose_1[4]= {11.41078, 11.65614, 0.998544, 0.05393};
    float pose_2[4]= {11.0964, 11.0, 0.9238, 0.38299};
    float pose_3[4]= {10.2545, 10.74, 0.7,0.7733};
    float pose_4[4]= {9.57446, 11.11, 0.32705, 0.945007};
    float pose_5[4]= {9.3342, 11.71366, -0.0297, 0.99956};
    float pose_6[4]= {9.5259,12.445, -0.4033, 0.91506};
    float pose_7[4]= {10.3006, 12.7656, -0.6,0.7461};
    float pose_8[4]= {11.0429,12.4798,-0.8708, 0.4916};


    stealCounter = stealCounter %3;
    int pose=6 +stealCounter + winkel;

    if (pose>8){
        pose = pose - 8;
    }

    switch (pose) {
    case 1:
        simpleGoal(pose_1[0], pose_1[1], pose_1[2], pose_1[3]);
        break;
    case 2:
        simpleGoal(pose_2[0], pose_2[1], pose_2[2], pose_2[3]);
        break;
    case 3:
        simpleGoal(pose_3[0], pose_3[1], pose_3[2], pose_3[3]);
        break;
    case 4:
        simpleGoal(pose_4[0], pose_4[1], pose_4[2], pose_4[3]);
        break;
    case 5:
        simpleGoal(pose_5[0], pose_5[1], pose_5[2], pose_5[3]);
        break;
    case 6:
        simpleGoal(pose_6[0], pose_6[1], pose_6[2], pose_6[3]);
        break;
    case 7:
        simpleGoal(pose_7[0], pose_7[1], pose_7[2], pose_7[3]);
        break;
    case 8:
        simpleGoal(pose_8[0], pose_8[1], pose_8[2], pose_8[3]);
        break;
    default:
        break;
    }

   // usleep(400000);
    ros::spinOnce();
    std::vector<float> orthoX;

    for (unsigned int i=0; i<laserDatenX.size();i++)
    {
        if(laserDatenX[i]>0.05 && laserDatenX[i]<0.60 && fabs(laserDatenY[i])<0.08){
            orthoX.push_back(laserDatenX[i]);
        }
    }

    double x_ =0;
    for (unsigned int i=0; i<orthoX.size();i++)
    {
        x_= x_ + orthoX[i];
    }
    x_ = x_/orthoX.size();
    moveBase(x_-0.35,0,0);

    m_Arm.moveToPose("ARM_UP");
    m_Arm.waitForCurrentAction();
    while (m_Arm.isBusy()) {
    }

}

void Statmachine::stealCube(){

    ROS_INFO("In stealState");

    luh_youbot_kinematics::JointPosition cuPos;
    cuPos.setQ1(0.576);                   //Klauen2
    cuPos.setQ2(1.2636);
    cuPos.setQ3(1.8623);
    cuPos.setQ4(-1.2165);
    cuPos.setQ5(0);
    m_Arm.moveToPose(cuPos, yapi::MotionMode::INTERPOLATE);
    m_Arm.waitForCurrentAction();
    while (m_Arm.isBusy()) {
    }
    usleep(500000);
    ros::spinOnce();

    int cube =1;

    float delta = 15;
    float min =10000000;
    bool poseOK = false;
    int i=0;

    while(!poseOK && ros::ok())
    {
        ros::spinOnce();

        double center_y = 390;
        double center_x = 378;
        float x=0;
        float y=0;


           if ((cam_X0-center_x)*(cam_X0-center_x)+(cam_Y0-center_y)*(cam_Y0-center_y)<min && cam_X0>2)
             {
                x=cam_X0;
                y=cam_Y0;
           }
           if ((cam_X1-center_x)*(cam_X1-center_x)+(cam_Y1-center_y)*(cam_Y1-center_y)<min && cam_X1>2)
           {
                x=cam_X1;
                y=cam_Y1;
           }
           if ((cam_X2-center_x)*(cam_X2-center_x)+(cam_Y2-center_y)*(cam_Y2-center_y)<min && cam_X2>2)
           {
                x=cam_X2;
                y=cam_Y2;
           }
           if ((cam_X3-center_x)*(cam_X3-center_x)+(cam_Y3-center_y)*(cam_Y3-center_y)<min && cam_X3>2)
           {
                x=cam_X3;
                y=cam_Y3;
           }
           if ((cam_X4-center_x)*(cam_X4-center_x)+(cam_Y4-center_y)*(cam_Y4-center_y)<min && cam_X4>2)
           {
                x=cam_X4;
                y=cam_Y4;
           }
           if ((cam_X5-center_x)*(cam_X5-center_x)+(cam_Y5-center_y)*(cam_Y5-center_y)<min && cam_X5>2)
           {
                x=cam_X5;
                y=cam_Y5;
           }
           if ((cam_X6-center_x)*(cam_X6-center_x)+(cam_Y6-center_y)*(cam_Y6-center_y)<min && cam_X6>2)
           {
                x=cam_X6;
                y=cam_Y6;
           }
           if ((cam_X7-center_x)*(cam_X7-center_x)+(cam_Y7-center_y)*(cam_Y7-center_y)<min && cam_X7>2)
           {
                x=cam_X7;
                y=cam_Y7;
           }
           if ((cam_X8-center_x)*(cam_X8-center_x)+(cam_Y8-center_y)*(cam_Y8-center_y)<min && cam_X8>2)
           {
                x=cam_X8;
                y=cam_Y8;
           }

        ROS_INFO("%f,%f",x,y);

        m_Base.abortCurrentAction();

        double diffX = center_x - x;
        double diffY = center_y - y;
        if (x!=0){
            cubeFound = true;
            usleep(500000);
            if (diffX > -2*delta && diffX < 2*delta && diffY > -delta && diffY < delta )
            {
                m_Base.abortCurrentAction();
                poseOK = true;
                ROS_INFO("Ziel erreicht");
                gripCubeSteal();

                break;
            }
            else{

                if(diffY>100){
                    diffY = 100;
                }
                if(diffY<-100){
                    diffY = -100;
                }

                if(diffX>100){
                    diffX = 100;
                }
                if(diffX<-100){
                    diffX = -100;
                }
                float x_dist;

                // Kollisionsvermeidung bei Klauen
                if (diffY*0.0009>distFront){
                    x_dist = diffY*0.0007;
                }
                else{
                    x_dist = 0;
                }
                moveBase( x_dist, diffX*0.0008, -diffX*0.0010);
                while (m_Base.isBusy()){
                }
                usleep(700000);
            }


        }
        else{
            if(i<5){
            float y = Kollisionsvermeidung(0.05);
            moveBase( 0, y, -0.06);
            if (y ==0){
                i=5;
            }
            i++;
            }
            else{
                poseOK = true;
                cuPos.setQ1(0);                   //Klauen2
                cuPos.setQ2(0);
                cuPos.setQ3(0);
                cuPos.setQ4(0);
                cuPos.setQ5(0);
                m_Arm.moveToPose(cuPos, yapi::MotionMode::INTERPOLATE);
                m_Arm.waitForCurrentAction();
                m_Arm.moveToPose("FINAL_WINDSCHNITTIG");
                 m_Arm.waitForCurrentAction();
            }
        }
    }

}
void Statmachine::gripCubeSteal(){

    luh_youbot_kinematics::JointPosition cuPos;
    cuPos.setQ1(0.576);                   //Klauen
    cuPos.setQ2(1.46084);
    cuPos.setQ3(1.31947);
    cuPos.setQ4(-0.9006);
    cuPos.setQ5(0);
    m_Arm.moveToPose(cuPos, yapi::MotionMode::INTERPOLATE);
    m_Arm.waitForCurrentAction();
    while (m_Arm.isBusy()) {
    }
    Gripper(0.005);
  //  usleep(500000);
    cuPos.setQ1(0.576);                   //Klauen3
    cuPos.setQ2(1.4608);
    cuPos.setQ3(1.0332);
    cuPos.setQ4(-0.6685);
    cuPos.setQ5(0);
    m_Arm.moveToPose(cuPos, yapi::MotionMode::INTERPOLATE);
    m_Arm.waitForCurrentAction();
    while (m_Arm.isBusy()) {
    }
    ROS_INFO("%f",grip_width);
    cubeGrip = false;
    if (grip_width>0.0022){
        cubeGrip = true ;
        cuPos.setQ1(0.576);                   //Klauen4
        cuPos.setQ2(1.4608);
        cuPos.setQ3(0.5777);
        cuPos.setQ4(0.1344);
        cuPos.setQ5(0);
        m_Arm.moveToPose(cuPos, yapi::MotionMode::INTERPOLATE);
        m_Arm.waitForCurrentAction();
        while (m_Arm.isBusy()) {
        }

        m_Arm.moveToPose("ARM_UP");
        m_Arm.waitForCurrentAction();
        switch ((cubePhase + stealPhase)%3) {
        case 1:
            m_Arm.moveToPose("FINAL_LOSE_CUBE1");
            break;
        case 2:
            m_Arm.moveToPose("FINAL_LOSE_CUBE2");
            break;
        case 0:
            m_Arm.moveToPose("FINAL_LOSE_CUBE3");
            break;
        default:
            break;

            stealPhase++;
            m_Arm.waitForCurrentAction();
            while (m_Arm.isBusy()) {

            }
        }
     //   usleep(500000);
    }
    else{
        Gripper(0.059);
        stealCube();
    }
    Gripper(0.059);

    if((cubePhase+stealPhase)%3==1 && cubeGrip)
    {
        m_Arm.moveToPose("FINAL_WINDSCHNITTIG");
    }
    else{
        m_Arm.moveToPose("FINAL_SEARCH");
    }

    m_Arm.waitForCurrentAction();
    cubeFound = false;
}
