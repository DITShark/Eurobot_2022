#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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

class randomSample
{
private:
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;

public:
    randomSample()
    {
        this->x = -1;
        this->y = -1;
        this->z = -1;
        this->roll = -1;
        this->pitch = -1;
        this->yaw = -1;
    }

    randomSample(double x, double y, double z, double roll, double pitch, double yaw)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->roll = roll;
        this->pitch = pitch;
        this->yaw = yaw;
    };

    void update(double x, double y, double z, double roll, double pitch, double yaw)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->roll = roll;
        this->pitch = pitch;
        this->yaw = yaw;
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
    double get_roll()
    {
        return roll;
    }
    double get_pitch()
    {
        return pitch;
    }
    double get_yaw()
    {
        return yaw;
    }
};

class mission
{
private:
    double x;
    double y;
    double z;
    double w;
    char missionType;

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
const bool RANDOM_SAMPLE = false;

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
double mission_waitTime;
int runWhichScript; // 0 for big , 1 for small

int mission_num = 0;
int goal_num = 0;
int now_Status = 0;
int now_Mode = 1;

bool moving = false;
bool doing = false;
bool position_correction = false;
bool finishMission = false;

double position_x;
double position_y;
double orientation_z;
double orientation_w;
double startMissionTime;

geometry_msgs::PoseStamped next_target;
geometry_msgs::Pose2D next_correction;

vector<mission> mission_List;
vector<int> missionTime_correct_Type;
vector<int> missionTime_correct_Num;

randomSample randomBlue;
randomSample randomGreen;
randomSample randomRed;

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

void correctMissionTime(char missionC) // Create Rules for mission Wait Time
{
    for (size_t i = 0; i < missionTime_correct_Type.size(); i++)
    {
        if (missionC == missionTime_correct_Type[i])
        {
            mission_waitTime = missionTime_correct_Num[i];
            break;
        }
    }
}

// Node Handling Class Define

class mainProgram
{
public:
    // Callback Function Define

    void position_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
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
        if (msg->data && moving)
        {
            if (goal_num == mission_List.size() - 1 && mission_List[goal_num].get_missionType() == 'X')
            {
                now_Status++;
            }
            else
            {

#if OPEN_POSITION_ADJUSTMENT
                checkPosError(new mission(x, y, theta, 'X'), mission_List[goal_num]);
#endif
                if (!position_correction)
                {
                    moving = false;
                    if (mission_List[goal_num].get_missionType() == 'X')
                    {
                        mission_num++;
                        goal_num++;
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
                        ROS_INFO("1 Moving to x:[%f] y:[%f]", mission_List[goal_num].get_x(), mission_List[goal_num].get_y());
                        cout << endl;
                    }
                    else
                    {
                        doing = true;
                        std_msgs::Char mm;
                        mm.data = mission_List[goal_num].get_missionType();
                        _arm.publish(mm);
                        ROS_INFO("Doing Mission Now... [ %c ]", mission_List[goal_num].get_missionType());
                        cout << endl;
                        startMissionTime = ros::Time::now().toSec();
                        nh.getParam("/mission_waitTime", mission_waitTime);
                        correctMissionTime(mission_List[goal_num].get_missionType());
                    }
                }
            }
        }
    }

    void cameraInfo_callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
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
    ros::Publisher _arm = nh.advertise<std_msgs::Char>("arm_go_where", 1000);          // Publish mission to mission
    ros::Publisher _time = nh.advertise<std_msgs::Float32>("total_Time", 1000);        // Publish total Time

    // ROS Topics Subscribers
    // ros::Subscriber _globalFilter = nh.subscribe<nav_msgs::Odometry>("global_filter", 1000, &mainProgram::position_callback, this);               // Get position from localization
    ros::Subscriber _globalFilter = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("ekf_pose", 1000, &mainProgram::position_callback, this); // Get position from localization Lu
    ros::Subscriber _haveObsatcles = nh.subscribe<std_msgs::Bool>("have_obstacles", 1000, &mainProgram::emergency_callback, this);                   // Get emergency state from lidar
    ros::Subscriber _FinishOrNot = nh.subscribe<std_msgs::Bool>("Finishornot", 1000, &mainProgram::moving_callback, this);                           // Get finish moving state from controller
    ros::Subscriber _cameraInfo = nh.subscribe<std_msgs::Float32MultiArray>("Sample_position", 1000, &mainProgram::cameraInfo_callback, this);

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
    ros::Time initialTime = ros::Time::now();
    mainClass.nh.getParam("/runWhichScript", runWhichScript);

    // Main Node Update Frequency

    ros::Rate rate(200);

    ifstream inFile;
    string value;
    string line;
    string field;
    int waitCount = 0;

    while (ros::ok())
    {
        switch (now_Mode)
        {
        case NORMAL:
            switch (now_Status)
            {
            case STRATEGY:

                mainClass.nh.getParam("/side_state", side_state);

                if (side_state == 1)
                {
                    position_x = INI_X_YELLOW;
                    position_y = INI_Y_YELLOW;
                    orientation_z = INI_Z_YELLOW;
                    orientation_w = INI_W_YELLOW;
                }
                else if (side_state == 2)
                {
                    position_x = INI_X_PURPLE;
                    position_y = INI_Y_PURPLE;
                    orientation_z = INI_Z_PURPLE;
                    orientation_w = INI_W_PURPLE;
                }

                // Script Reading

                if (!runWhichScript)
                {
                    inFile.open("/home/ubuntu/Eurobot2022_ws/scriptBig.csv");
                    // inFile.open("/home/sharkkk/Eurobot_2022_ws/scriptBig.csv");
                }
                else
                {
                    inFile.open("/home/ubuntu/Eurobot2022_ws/scriptSmall.csv");
                }
                if (inFile.fail())
                {
                    cout << "Could Not Open File\n";
                }
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
                    if (side_state == 1)
                    {
                        mission nextMission(next_x, next_y, next_z, next_w, next_m);
                        mission_List.push_back(nextMission);
                    }
                    else if (side_state == 2)
                    {
                        mission nextMission(next_x, 3 - next_y, -next_z, next_w, next_m);
                        mission_List.push_back(nextMission);
                    }
                }

                mainClass.nh.getParam("/mission_waitTime", mission_waitTime);
                mainClass.nh.param("/missionTime_correct_Type", missionTime_correct_Type, missionTime_correct_Type);
                mainClass.nh.param("/missionTime_correct_Num", missionTime_correct_Num, missionTime_correct_Num);

                cout << endl;
                for (size_t i = 0; i < missionTime_correct_Type.size(); i++)
                {
                    ROS_INFO("Mission [%c] Correct to %d secs", missionTime_correct_Type[i], missionTime_correct_Num[i]);
                }
                cout << endl;

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
                        initialTime = ros::Time::now();
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
                        ROS_INFO("2 Moving to x:[%f] y:[%f]", mission_List[goal_num].get_x(), mission_List[goal_num].get_y());
                        cout << endl;
                    }
                }
                else
                {
                    if (waitCount++ > 50)
                    {
                        ROS_INFO("Waiting Now...");
                        cout << endl;
                        waitCount = 0;
                    }
                }
                break;

            case RUN:

                if (moving && !doing)
                {
                    // ROS_INFO("Moving Now...");
                    // ROS_INFO("Position at x:[%f], y:[%f], z:[%f], w:[%f]", position_x, position_y, orientation_z, orientation_w);
                }
                else if (doing && !moving && MISSION_NODE_NOEXIST)
                {
                    if (ros::Time::now().toSec() - startMissionTime < mission_waitTime && MISSION_NODE_NOEXIST)
                    {
                        // cout << "Doing Mission Now... [ " << mission_List[goal_num].get_missionType() << " ]" << endl;
                    }
                    else
                    {
                        doing = false;
                        if (goal_num == mission_List.size() - 1)
                        {
                            now_Status++;
                        }
                        else
                        {
                            mission_num++;
                            goal_num++;
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
                            ROS_INFO("3 Moving to x:[%f] y:[%f]", mission_List[goal_num].get_x(), mission_List[goal_num].get_y());
                            cout << endl;
                        }
                    }
                }
                else if (doing && !moving)
                {
                    // cout << "Doing Mission Now... [ " << mission_List[goal_num].get_missionType() << " ]" << endl;
                    main_2022::Mission_srv next;
                    next.request.mission = mission_List[goal_num].get_missionType();
                    while (mainClass._mission.call(next))
                    {
                        cout << "-" << endl;
                        int finishType;
                        finishType = next.response.missionary;
                        if (finishType)
                        {
                            doing = false;
                            break;
                        }
                    }
                    if (goal_num == mission_List.size() - 1)
                    {
                        now_Status++;
                    }
                    else
                    {
                        mission_num++;
                        goal_num++;
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
                        ROS_INFO("4 Moving to x:[%f] y:[%f]", mission_List[goal_num].get_x(), mission_List[goal_num].get_y());
                        cout << endl;
                    }
                }
                break;

            case FINISH:
                if (!finishMission)
                {
                    std_msgs::Float32 tt;
                    tt.data = ros::Time::now().toSec() - initialTime.toSec();
                    cout << "Mission Time: " << tt.data << endl;
                    mainClass._time.publish(tt);
                    ROS_INFO("Finish All Mission");
                    finishMission = true;
                }

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