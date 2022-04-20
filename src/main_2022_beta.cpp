#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
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
#include <ros/package.h>
#include <std_srvs/Empty.h>

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
    double w;

public:
    randomSample(double x, double y, double z, double w)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;
    };

    void update(double x, double y, double z, double w)
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
};

class Path
{
private:
    double x;
    double y;
    double z;
    double w;
    char missionType;

public:
    Path(double x, double y, double z, double w, char missionType)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;
        this->missionType = missionType;
    };

    void update(randomSample update)
    {
        this->x = update.get_x();
        this->y = update.get_y();
        this->z = update.get_z();
        this->w = update.get_w();
    }

    void updateMission(char update)
    {
        this->missionType = update;
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

class missionPoint
{
private:
    double x;
    double y;
    double z;
    double w;
    char missionType;
    int point;
    int missionOrder;

public:
    missionPoint(double x, double y, double z, double w, char missionType, int point, int missionOrder)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;
        this->missionType = missionType;
        this->point = point;
        this->missionOrder = missionOrder;
    };
    void changeMissionType(char newMission)
    {
        missionType = newMission;
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
    int get_point()
    {
        return point;
    }
    int get_missionOrder()
    {
        return missionOrder;
    }
};

// Program Adjustment

const bool RANDOM_SAMPLE = true;

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
int run_state = 0;
double mission_waitTime;

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

vector<Path> path_List;
vector<missionPoint> mission_List;
vector<int> missionTime_correct_Type;
vector<double> missionTime_correct_Num;

randomSample random_blue(1.265, 0.975, 0.237214, 0.971457);
randomSample random_green(1.475, 0.865, 0.9659258, 0.258819);
randomSample random_red(1.475, 1.085, -0.7071068, 0.7071068);

bool do_random_blue = true;
bool do_random_green = true;
bool do_random_red = true;

// Function Define

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

void updateRandomRoute(Path *updateM)
{
    double smallestX = 2;
    int updateWhich = 0;
    if (do_random_blue && smallestX > random_blue.get_x())
    {
        smallestX = random_blue.get_x();
        updateWhich = 1;
    }
    if (do_random_green && smallestX > random_green.get_x())
    {
        smallestX = random_green.get_x();
        updateWhich = 2;
    }
    if (do_random_red && smallestX > random_red.get_x())
    {
        smallestX = random_red.get_x();
        updateWhich = 3;
    }

    if (updateWhich == 1)
    {
        do_random_blue = false;
        updateM->update(random_blue);
        updateM->updateMission('B');
    }
    else if (updateWhich == 2)
    {
        do_random_green = false;
        updateM->update(random_green);
        updateM->updateMission('G');
    }
    else if (updateWhich == 3)
    {
        do_random_red = false;
        updateM->update(random_red);
        updateM->updateMission('R');
    }
    else
    {
        while (path_List[goal_num].get_missionType() == 'Z')
        {
            mission_num++;
            goal_num++;
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
        // if (msg->data)
        // {
        //     now_Mode = 0;
        //     std_msgs::Bool publisher;
        //     publisher.data = true;
        //     _StopOrNot.publish(publisher);
        // }
        // else
        // {
        //     now_Mode = 1;
        //     std_msgs::Bool publisher;
        //     publisher.data = false;
        //     _StopOrNot.publish(publisher);
        // }
    }

    void moving_callback(const std_msgs::Bool::ConstPtr &msg)
    {
        if (msg->data && moving)
        {
            if (goal_num == path_List.size() - 1 && path_List[goal_num].get_missionType() == 'X')
            {
                now_Status++;
            }
            else
            {
                moving = false;
                if (path_List[goal_num].get_missionType() == 'X')
                {
                    mission_num++;
                    goal_num++;
                    while (path_List[goal_num].get_missionType() == '0')
                    {
                        goal_num++;
                    }
                    if (path_List[goal_num].get_missionType() == 'Z')
                    {
                        ROS_INFO("Updating Route ...\n");
                        updateRandomRoute(&path_List[goal_num]);
                    }
                    next_target.pose.position.x = path_List[goal_num].get_x();
                    next_target.pose.position.y = path_List[goal_num].get_y();
                    next_target.pose.orientation.z = path_List[goal_num].get_z();
                    next_target.pose.orientation.w = path_List[goal_num].get_w();
                    _target.publish(next_target);
                    moving = true;
                    ROS_INFO("1 Moving to x:[%f] y:[%f]", path_List[goal_num].get_x(), path_List[goal_num].get_y());
                    cout << endl;
                }
                else
                {
                    doing = true;
                    std_msgs::Char mm;
                    mm.data = path_List[goal_num].get_missionType();
                    _arm.publish(mm);
                    ROS_INFO("Doing Mission Now... [ %c ]", path_List[goal_num].get_missionType());
                    cout << endl;
                    startMissionTime = ros::Time::now().toSec();
                    nh.getParam("/mission_waitTime", mission_waitTime);
                    correctMissionTime(path_List[goal_num].get_missionType());
                }
            }
        }
    }

    void cameraInfo_callback(const std_msgs::Float32MultiArray::ConstPtr &msg)
    {
        if (msg->data.at(4))
        {
            random_blue.update(msg->data.at(0) - 0.24, msg->data.at(1), -1, -1);
        }
        else
        {
            // do_random_blue = false;
        }
        if (msg->data.at(9))
        {
            random_green.update(msg->data.at(5) - 0.24, msg->data.at(6), -1, -1);
        }
        else
        {
            // do_random_green = false;
        }
        if (msg->data.at(14))
        {
            random_red.update(msg->data.at(10) - 0.23, msg->data.at(11), -1, -1);
        }
        else
        {
            // do_random_red = false;
        }
    }

    bool givePath_callback(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res)
    {
        res.plan.poses.clear();
        while (1)
        {
            geometry_msgs::PoseStamped next;
            next.pose.position.x = path_List[mission_num].get_x();
            next.pose.position.y = path_List[mission_num].get_y();
            next.pose.orientation.z = path_List[mission_num].get_z();
            next.pose.orientation.w = path_List[mission_num].get_w();
            res.plan.poses.push_back(next);
            if (path_List[mission_num].get_missionType() == '0')
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

    bool start_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        if (now_Status > 1)
        {
            now_Status = 1;
            mission_num = 0;
            goal_num = 0;
        }
        else
        {
            run_state = 1;
        }
        return true;
    }

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
    ros::Subscriber _cameraInfo = nh.subscribe<std_msgs::Float32MultiArray>("Sample_position", 1000, &mainProgram::cameraInfo_callback, this);       // Get Sample Information from Camera

    // ROS Service Server
    ros::ServiceServer _MissionPath = nh.advertiseService("MissionPath", &mainProgram::givePath_callback, this); // Path giving Service
    ros::ServiceServer _RunState = nh.advertiseService("startRunning", &mainProgram::start_callback, this);      // Start Signal Service

    // ROS Service Client
};

// Main Program

int main(int argc, char **argv)
{
    // ROS initial
    ros::init(argc, argv, "Main_Node_beta");

    // Node Handling Class Initialize

    mainProgram mainClass;
    ros::Time initialTime = ros::Time::now();
    std_msgs::Float32 timePublish;

    // Main Node Update Frequency

    ros::Rate rate(200);

    ifstream inFile;
    string value;
    string line;
    string field;
    string packagePath;
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

                packagePath = ros::package::getPath("main_2022");

                double next_x;
                double next_y;
                double next_z;
                double next_w;
                char next_m;
                int next_p;
                int next_o;

                cout << endl;
                inFile.open(packagePath + "/include/missionPoint.csv");
                cout << "<< missionPoint.csv >> ";
                if (inFile.fail())
                {
                    cout << "Could Not Open !" << endl;
                }
                else
                {
                    cout << "Open Successfully !" << endl;
                }
                cout << endl;
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

                    getline(sin, field, ',');
                    const char *cstr = field.c_str();
                    char b = *cstr;
                    next_m = b;
                    // cout << next_m << " ";

                    getline(sin, field, ',');
                    next_p = atoi(field.c_str());
                    // cout << next_p << " ";

                    getline(sin, field, ',');
                    next_o = atoi(field.c_str());
                    // cout << next_o << " ";

                    missionPoint nextPoint(next_x, next_y, next_z, next_w, next_m, next_p, next_o);
                    mission_List.push_back(nextPoint);
                }
                inFile.close();

                inFile.open(packagePath + "/include/scriptBig.csv");
                cout << "<< scriptBig.csv >> ";
                if (inFile.fail())
                {
                    cout << "Could Not Open !" << endl;
                }
                else
                {
                    cout << "Open Successfully !" << endl;
                }
                cout << endl;
                while (getline(inFile, line))
                {
                    istringstream sin(line);

                    getline(sin, field, ',');
                    next_o = atoi(field.c_str());
                    // cout << next_o << " ";

                    if (next_o)
                    {
                        next_x = mission_List[next_o - 1].get_x();
                        next_y = mission_List[next_o - 1].get_y();
                        next_z = mission_List[next_o - 1].get_z();
                        next_w = mission_List[next_o - 1].get_w();
                        next_m = mission_List[next_o - 1].get_missionType();
                    }
                    else
                    {
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

                        next_m = '0';
                    }

                    if (strcmp(&next_m, "Z1") == 0 && !RANDOM_SAMPLE)
                    {
                        next_m = 'B';
                    }
                    else if (strcmp(&next_m, "Z2") == 0 && !RANDOM_SAMPLE)
                    {
                        next_m = 'R';
                    }
                    else if (strcmp(&next_m, "Z3") == 0 && !RANDOM_SAMPLE)
                    {
                        next_m = 'G';
                    }
                    // cout << next_m << endl;

                    if (side_state == 1)
                    {
                        Path nextMission(next_x, next_y, next_z, next_w, next_m);
                        path_List.push_back(nextMission);
                    }
                    else if (side_state == 2)
                    {
                        Path nextMission(next_x, 3 - next_y, -next_z, next_w, next_m);
                        path_List.push_back(nextMission);
                    }
                    // cout << next_x << " " << next_y << " " << next_z << " " << next_w << " " << next_m << endl;
                }

                mainClass.nh.getParam("/mission_waitTime", mission_waitTime);
                mainClass.nh.param("/missionTime_correct_Type", missionTime_correct_Type, missionTime_correct_Type);
                mainClass.nh.param("/missionTime_correct_Num", missionTime_correct_Num, missionTime_correct_Num);

                cout << endl;
                for (size_t i = 0; i < missionTime_correct_Type.size(); i++)
                {
                    ROS_INFO("Mission [%c] Correct to %f secs", missionTime_correct_Type[i], missionTime_correct_Num[i]);
                }
                cout << endl;

                now_Status++;
                break;

            case READY:
                if (run_state)
                {
                    now_Status++;
                    run_state = 0;
                    if (path_List.size() == 0)
                    {
                        now_Status++;
                    }
                    else
                    {
                        initialTime = ros::Time::now();
                        while (path_List[goal_num].get_missionType() == '0')
                        {
                            goal_num++;
                        }
                        if (path_List[goal_num].get_missionType() == 'Z')
                        {
                            ROS_INFO("Updating Route ...\n");
                            updateRandomRoute(&path_List[goal_num]);
                        }
                        next_target.pose.position.x = path_List[goal_num].get_x();
                        next_target.pose.position.y = path_List[goal_num].get_y();
                        next_target.pose.orientation.z = path_List[goal_num].get_z();
                        next_target.pose.orientation.w = path_List[goal_num].get_w();
                        mainClass._target.publish(next_target);
                        moving = true;
                        ROS_INFO("2 Moving to x:[%f] y:[%f]", path_List[goal_num].get_x(), path_List[goal_num].get_y());
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
                else if (doing && !moving)
                {
                    if (ros::Time::now().toSec() - startMissionTime < mission_waitTime)
                    {
                        // cout << "Doing Mission Now... [ " << path_List[goal_num].get_missionType() << " ]" << endl;
                    }
                    else
                    {
                        doing = false;
                        if (goal_num == path_List.size() - 1)
                        {
                            now_Status++;
                        }
                        else
                        {
                            mission_num++;
                            goal_num++;
                            while (path_List[goal_num].get_missionType() == '0')
                            {
                                goal_num++;
                            }
                            if (path_List[goal_num].get_missionType() == 'Z')
                            {
                                ROS_INFO("Updating Route ...\n");
                                updateRandomRoute(&path_List[goal_num]);
                            }
                            next_target.pose.position.x = path_List[goal_num].get_x();
                            next_target.pose.position.y = path_List[goal_num].get_y();
                            next_target.pose.orientation.z = path_List[goal_num].get_z();
                            next_target.pose.orientation.w = path_List[goal_num].get_w();
                            mainClass._target.publish(next_target);
                            moving = true;
                            ROS_INFO("3 Moving to x:[%f] y:[%f]", path_List[goal_num].get_x(), path_List[goal_num].get_y());
                            cout << endl;
                        }
                    }
                }
                timePublish.data = ros::Time::now().toSec() - initialTime.toSec();
                mainClass._time.publish(timePublish);
                break;

            case FINISH:
                if (!finishMission)
                {
                    timePublish.data = ros::Time::now().toSec() - initialTime.toSec();
                    cout << "Mission Time: " << timePublish.data << endl;
                    mainClass._time.publish(timePublish);
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