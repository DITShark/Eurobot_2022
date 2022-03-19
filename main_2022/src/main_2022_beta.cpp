#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <main_2022/Mission_srv.h>

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <fstream>
#include <ctime>

using namespace std;

enum Status
{
    STRATEGY = 0,
    READY,
    RUN,
    FINISH
};

enum Mode
{
    EMERGENCY = 0,
    NORMAL
};

// Class Define

class mission
{
private:
    double x;
    double y;
    double z;
    double w;
    char missionType;

#if 0
    /*
    Mission Type List:

        'X' for No Mission
        '0' for Non stop Point

    */
#endif

public:
    mission(double x, double y, double z, double w, char missionType)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;
        this->missionType = missionType;
    };
    void update(double x, double y, double z, double w, char missionType)
    {
        if (x != -1)
        {
            this->x = x;
        }
        if (y != -1)
        {
            this->y = y;
        }
        if (z != -1)
        {
            this->z = z;
        }
        if (w != -1)
        {
            this->w = w;
        }
        if (missionType != -1)
        {
            this->missionType = missionType;
        }
    }
    double get_x()
    {
        return x;
    }
    double get_y()
    {
        return y;
    }
    double get_z()
    {
        return z;
    }
    double get_w()
    {
        return w;
    }
    char get_missionType()
    {
        return missionType;
    }
};

// Program Adjustment

const bool OPEN_POSITION_ADJUSTMENT = false;
const bool MISSION_NODE_NOEXIST = true;

// Adjustment Variable Define

const double INI_X_PURPLE = 0;
const double INI_Y_PURPLE = 0;
const double INI_Z_PURPLE = 0;
const double INI_W_PURPLE = 0;

const double INI_X_YELLOW = 0.832;
const double INI_Y_YELLOW = 0.257;
const double INI_Z_YELLOW = -0.263;
const double INI_W_YELLOW = 0.9646;

const double POSITION_CORRECTION_ERROR = 10;

// Variable Define

int side_state; // 1 for yellow , 2 for purple
int run_state;

int mission_num = 0;
int goal_num = 0;
int now_Status = 0;
int now_Mode = 1;

bool moving = false;
bool doing = false;
bool position_correction = false;

double position_x;
double position_y;
double orientation_z;
double orientation_w;

geometry_msgs::PoseStamped next_target;
geometry_msgs::Pose2D next_correction;

vector<mission> mission_List;

// Function Define

#if OPEN_POSITION_ADJUSTMENT

void checkPosError(mission real, mission target)
{
    double x_correction = 0;
    double y_correction = 0;
    double theta_correction = 0;
    if (abs(real.get_x() - target.get_x()) < POSITION_CORRECTION_ERROR)
    {
        x_correction = target.get_x() - real.get_x();
    }
    if (abs(real.get_y() - target.get_y()) < POSITION_CORRECTION_ERROR)
    {
        y_correction = target.get_y() - real.get_y();
    }
    if (abs(real.get_theta() - target.get_theta()) < POSITION_CORRECTION_ERROR)
    {
        theta_correction = target.get_theta() - real.get_theta();
    }
    if (x_correction != 0 && y_correction != 0 && theta_correction != 0)
    {
        position_correction = true;
        next_correction.x = x_correction;
        next_correction.y = y_correction;
        next_correction.theta = theta_correction;
        _positionCorrection.publish(next_correction);
    }
}

#endif

// Node Handling Class Define

class mainProgram
{
public:
    // Callback Function Define

    void position_callback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        position_x = msg->pose.pose.position.x;
        position_y = msg->pose.pose.position.y;
        orientation_z = msg->pose.pose.orientation.z;
        orientation_w = msg->pose.pose.orientation.w;
    }

    void emergency_callback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data)
        {
            now_Mode = 0;
            std_msgs::Bool publisher;
            publisher.data = true;
            _StopOrNot.publish(publisher);
        }
        else
        {
            now_Mode = 1;
            std_msgs::Bool publisher;
            publisher.data = false;
            _StopOrNot.publish(publisher);
        }
    }

    void moving_callback(const std_msgs::Bool::ConstPtr &msg)
    {
        clock_t start = clock();
        while (clock() - start < 1500000 && MISSION_NODE_NOEXIST)
        {
        }
        if (msg->data && moving && now_Mode)
        {
            if (mission_num == mission_List.size() - 1 && mission_List[mission_num].get_missionType() == 'X')
            {
                now_Status++;
            }
            else if (mission_num == mission_List.size() - 1 && MISSION_NODE_NOEXIST)
            {
                now_Status++;
            }
            else
            {

#if OPEN_POSITION_ADJUSTMENT
                checkPosError(new mission(x, y, theta, 'X'), mission_List[mission_num]);
#endif
                if (!position_correction)
                {
                    moving = false;
                    if (mission_List[mission_num].get_missionType() == 'X' || MISSION_NODE_NOEXIST)
                    {
                        mission_num++;
                        goal_num++;
                        if (mission_List[mission_num].get_missionType() != '0')
                        {
                            if (MISSION_NODE_NOEXIST)
                            {
                                while (mission_List[mission_num].get_missionType() != '0')
                                {
                                    if (mission_num == mission_List.size() - 1)
                                    {
                                        now_Status++;
                                        break;
                                    }
                                    mission_num++;
                                    goal_num++;
                                }
                                if (now_Status == 2)
                                {
                                    while (mission_List[goal_num].get_missionType() == '0')
                                    {
                                        goal_num++;
                                    }
                                    next_target.pose.position.x = mission_List[goal_num].get_x();
                                    next_target.pose.position.y = mission_List[goal_num].get_y();
                                    next_target.pose.orientation.z = mission_List[goal_num].get_z();
                                    next_target.pose.orientation.w = mission_List[goal_num].get_w();
                                    _target.publish(next_target);
                                    moving = true;
                                }
                            }
                            else
                            {
                                doing = true;
                            }
                        }
                        else
                        {
                            while (mission_List[goal_num].get_missionType() == '0')
                            {
                                goal_num++;
                            }
                            next_target.pose.position.x = mission_List[goal_num].get_x();
                            next_target.pose.position.y = mission_List[goal_num].get_y();
                            next_target.pose.orientation.z = mission_List[goal_num].get_z();
                            next_target.pose.orientation.w = mission_List[goal_num].get_w();
                            _target.publish(next_target);
                            moving = true;
                        }
                    }
                    else
                    {
                        doing = true;
                    }
                }
            }
        }
    }

    void resistance_callback(const std_msgs::Int32::ConstPtr &msg)
    {
    }

    bool givePath_callback(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res)
    {
        res.plan.poses.clear();
        while (1)
        {
            geometry_msgs::PoseStamped next;
            next.pose.position.x = mission_List[mission_num].get_x();
            next.pose.position.y = mission_List[mission_num].get_y();
            next.pose.orientation.z = mission_List[mission_num].get_z();
            next.pose.orientation.w = mission_List[mission_num].get_w();
            res.plan.poses.push_back(next);
            if (mission_List[mission_num].get_missionType() == '0')
            {
                mission_num++;
            }
            else
            {
                break;
            }
        }
        return true;
    }

#if OPEN_POSITION_ADJUSTMENT

    void correction_callback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data && position_correction && OPEN_POSITION_ADJUSTMENT)
        {
            mission_num++;
            position_correction = false;
            moving = false;
        }
    }

#endif

    ros::NodeHandle nh;

    // ROS Topics Publishers
    ros::Publisher _target = nh.advertise<geometry_msgs::PoseStamped>("target", 1000); // Publish goal to controller
    ros::Publisher _StopOrNot = nh.advertise<std_msgs::Bool>("Stopornot", 1000);       // Publish emergency state to controller

    // ROS Topics Subscribers
    ros::Subscriber _globalFilter = nh.subscribe<nav_msgs::Odometry>("global_filter", 1000, &mainProgram::position_callback, this); // Get position from localization
    ros::Subscriber _emergency = nh.subscribe<std_msgs::Bool>("emergency", 1000, &mainProgram::emergency_callback, this);           // Get emergency state from lidar
    ros::Subscriber _FinishOrNot = nh.subscribe<std_msgs::Bool>("Finishornot", 1000, &mainProgram::moving_callback, this);          // Get finish moving state from controller
    ros::Subscriber _resistance = nh.subscribe<std_msgs::Int32>("resistance", 1000, &mainProgram::resistance_callback, this);       // Get resistor from mission

    // ROS Service Server
    ros::ServiceServer _MissionPath = nh.advertiseService("MissionPath", &mainProgram::givePath_callback, this); // Path giving Service

    // ROS Service Client
    ros::ServiceClient _mission = nh.serviceClient<main_2022::Mission_srv>("Mission"); // Mission Giving Service

#if OPEN_POSITION_ADJUSTMENT

    ros::Publisher _positionCorrection = nh.advertise<geometry_msgs::Pose2D>("positionCorrection", 1000);
    ros::Subscriber _correctionFinish = nh.subscribe<std_msgs::Bool>("CorrectionFinish", 1000, correction_callback);

#endif
};

// Main Program

int main(int argc, char **argv)
{
    // ROS initial
    ros::init(argc, argv, "Main_Node_beta");

    // Node Handling Class Initialize

    mainProgram mainClass;

    // Main Nide Update Frequency

    ros::Rate rate(200);

    // Script Reading
    ifstream inFile;
    inFile.open("/home/ubuntu/eurobot_ros_ws/scriptBig.csv");
    if (inFile.fail())
    {
        cout << "Could Not Open File\n";
    }
    string value;
    string line;
    string field;
    double next_x;
    double next_y;
    double next_z;
    double next_w;
    char next_m;
    while (getline(inFile, line))
    {
        istringstream sin(line);
        getline(sin, field, ',');
        next_x = atof(field.c_str());
        // cout << next_x << " ";

        getline(sin, field, ',');
        next_y = atof(field.c_str());
        // cout << next_y << " ";

        getline(sin, field, ',');
        next_z = atof(field.c_str());
        // cout << next_z << " ";

        getline(sin, field, ',');
        next_w = atof(field.c_str());
        // cout << next_w << " ";

        getline(sin, field);
        const char *cstr = field.c_str();
        char b = *cstr;
        next_m = b;
        // cout << b << endl;
        mission nextMission(next_x, next_y, next_z, next_w, next_m);
        mission_List.push_back(nextMission);
    }
    // mission_List.push_back(new mission(100,100,0,'X')));

    while (ros::ok())
    {
        switch (now_Mode)
        {
        case NORMAL:
            switch (now_Status)
            {
            case STRATEGY:
                mainClass.nh.getParam("/side_state", side_state);
                if (side_state == '1')
                {
                    position_x = INI_X_YELLOW;
                    position_y = INI_Y_YELLOW;
                    orientation_z = INI_Z_YELLOW;
                    orientation_w = INI_W_YELLOW;
                }
                else if (side_state == '2')
                {
                    position_x = INI_X_PURPLE;
                    position_y = INI_Y_PURPLE;
                    orientation_z = INI_Z_PURPLE;
                    orientation_w = INI_W_PURPLE;
                }
                now_Status++;
                break;

            case READY:
                mainClass.nh.getParam("/run_state", run_state);
                if (run_state)
                {
                    now_Status++;
                    if (mission_List.size() == 0)
                    {
                        now_Status++;
                    }
                    else
                    {
                        while (mission_List[goal_num].get_missionType() == '0')
                        {
                            goal_num++;
                        }
                        next_target.pose.position.x = mission_List[goal_num].get_x();
                        next_target.pose.position.y = mission_List[goal_num].get_y();
                        next_target.pose.orientation.z = mission_List[goal_num].get_z();
                        next_target.pose.orientation.w = mission_List[goal_num].get_w();
                        mainClass._target.publish(next_target);
                        moving = true;
                    }
                }
                else
                {
                    // ROS_INFO("Waiting Now...");
                }
                break;

            case RUN:
                if (moving && !doing)
                {
                    ROS_INFO("Moving Now...");
                    ROS_INFO("Position at x:[%f], y:[%f], z:[%f], w:[%f]", position_x, position_y, orientation_z, orientation_w);
                }
                else if (doing && !moving)
                {
                    ROS_INFO("Doing Mission Now...");
                    main_2022::Mission_srv next;
                    next.request.mission = mission_List[mission_num].get_missionType();
                    while (mainClass._mission.call(next))
                    {
                        int finishType;
                        finishType = next.response.missionary;
                        if (finishType)
                        {
                            doing = false;
                            break;
                        }
                    }
                    if (mission_num == mission_List.size() - 1)
                    {
                        now_Status++;
                    }
                    else
                    {
                        mission_num++;
                        goal_num++;
                        if (mission_List[mission_num].get_missionType() != '0')
                        {
                            doing = true;
                        }
                        else
                        {
                            while (mission_List[goal_num].get_missionType() == '0')
                            {
                                goal_num++;
                            }
                            next_target.pose.position.x = mission_List[goal_num].get_x();
                            next_target.pose.position.y = mission_List[goal_num].get_y();
                            next_target.pose.orientation.z = mission_List[goal_num].get_z();
                            next_target.pose.orientation.w = mission_List[goal_num].get_w();
                            mainClass._target.publish(next_target);
                            moving = true;
                        }
                    }
                }

                break;

            case FINISH:
                ROS_INFO("Finish All Mission");
                break;
            }
            break;

        case EMERGENCY:
            ROS_INFO("Emergency State");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}