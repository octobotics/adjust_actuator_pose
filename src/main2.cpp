#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <thread>
#include <atomic>
#include <set>
#include <cmath>
#include <chrono>
using namespace std::chrono;

ros::Publisher motorDirectionPub;
ros::Subscriber wireValueSub;

std::atomic<double> currentWireValue(0.0);
std::atomic<bool> motorClockwiseStarted(false);
std::atomic<bool> motorAnticlockwiseStarted(false);

ros::Time startTime;


const std::set<double> stopValues = {0.024, 0.048, 0.072, 0.096, 0.12, 0.144, 0.168, 0.192};
const double tolerance = 0.001; 
void clockwise();
void anticlockwise();
void stop();
void wireValueCallback(const std_msgs::Float32::ConstPtr& msg);
bool motorAnticlockwiseServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
bool motorClockwiseServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
bool motorstopServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
void stopIfWireValueReached();

ros::Publisher currentServoPosePub;
int current_value = 100;
const int MIN_VALUE = 65;
const int MAX_VALUE = 254;

void servoPoseCallback(const std_msgs::Int32::ConstPtr& msg) {
    int processedValue = msg->data;
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    
        std_msgs::Int32 processedMsg;
        processedMsg.data = processedValue;
        currentServoPosePub.publish(processedMsg);
    }

    ROS_INFO("Received /servo_pose: %d, Published processed value: %d", msg->data, processedValue);
}

void publishServoPose(ros::Publisher& servoPosePub, int value) {
    std_msgs::Int32 msg;
    msg.data = value;
    servoPosePub.publish(msg);

    ROS_INFO("Published to /servo_pose: %d", value);
}

bool addThreeCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    int newValue = current_value + 13;
    current_value = std::min(newValue, MAX_VALUE);

    ros::NodeHandle nh;
    ros::Publisher servoPosePub = nh.advertise<std_msgs::Int32>("/servo_pose", 1);
    publishServoPose(servoPosePub, current_value);

    res.success = true;
    res.message = "Added 13 to the current value. Resulting value: " + std::to_string(current_value);

    ROS_INFO_STREAM(res.message);

    return true;
}

bool reduceThreeCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    int newValue = current_value - 13;
    current_value = std::max(newValue, MIN_VALUE);

    ros::NodeHandle nh;
    ros::Publisher servoPosePub = nh.advertise<std_msgs::Int32>("/servo_pose", 1);
    publishServoPose(servoPosePub, current_value);

    res.success = true;
    res.message = "Reduced 13 from the current value. Resulting value: " + std::to_string(current_value);

    ROS_INFO_STREAM(res.message);

    return true;
}

void clockwise() {
    std_msgs::Int8 direction;
    direction.data = 2;
    motorDirectionPub.publish(direction);

    motorClockwiseStarted = true;
    motorAnticlockwiseStarted = false;
    startTime = ros::Time::now(); 
    ROS_INFO("Motor started clockwise");
}

void anticlockwise() {
    std_msgs::Int8 direction;
    direction.data = 1;
    motorDirectionPub.publish(direction);
    motorAnticlockwiseStarted = true;
    motorClockwiseStarted = false;
    startTime = ros::Time::now();  
    ROS_INFO("Motor started anticlockwise");
}

void stopIfWireValueReached() {
    ros::Rate rate(10);
    while (ros::ok() && (motorClockwiseStarted || motorAnticlockwiseStarted)) {
        ros::spinOnce();
        double currentWireValueSnapshot = currentWireValue.load();
        for (const double value : stopValues) {
            if (std::fabs(currentWireValueSnapshot - value) <= tolerance) {
                stop();
                return;
            }
        }
        rate.sleep();
    }
}

void stop() {
    std_msgs::Int8 direction;
    direction.data = 0;
    motorDirectionPub.publish(direction);
    motorClockwiseStarted = false;
    motorAnticlockwiseStarted = false;
    ROS_INFO("Motor stopped");
}

bool motorClockwiseServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    clockwise();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    std::thread(stopIfWireValueReached).detach();
    res.success = true;
    res.message = "Clockwise motor control service executed successfully";
    return true;
}

bool motorAnticlockwiseServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    anticlockwise();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    std::thread(stopIfWireValueReached).detach();
    res.success = true;
    res.message = "Anticlockwise motor control service executed successfully";
    return true;
}

bool motorstopServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    stop();
    res.success = true;
    res.message = "Stop motor control service executed successfully";
    return true;
}

void wireValueCallback(const std_msgs::Float32::ConstPtr& msg) {
    currentWireValue.store(msg->data);
    ROS_INFO("Received /wire_value: %f", msg->data);

    
    if (motorClockwiseStarted) {
        double currentWireValueSnapshot = currentWireValue.load();
        if (currentWireValueSnapshot >= 0.192) {
            stop();
        }
    }

    if (motorAnticlockwiseStarted) {
        double currentWireValueSnapshot = currentWireValue.load();
        if (currentWireValueSnapshot <= 0.024) {
            stop();
        }
    }
}

void checkMotorStatus(const ros::TimerEvent& event) {
    if (motorClockwiseStarted || motorAnticlockwiseStarted) {
        ros::spinOnce();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "adjust_and_control_node");
    ros::NodeHandle nh;

    ros::ServiceServer addThreeService = nh.advertiseService("/add_three_service", addThreeCallback);
    ros::ServiceServer reduceThreeService = nh.advertiseService("/reduce_three_service", reduceThreeCallback);
    ros::Publisher servoPosePub = nh.advertise<std_msgs::Int32>("/servo_pose", 1);
    std_msgs::Int32 msg;
    msg.data = 65;
    servoPosePub.publish(msg);
    ros::Subscriber servoPoseSub = nh.subscribe("/servo_pose", 1, servoPoseCallback);
    currentServoPosePub = nh.advertise<std_msgs::Int32>("/current_servo_pose", 1);
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), checkMotorStatus);
    ROS_INFO("Ready to add or reduce values to the current value.");

    wireValueSub = nh.subscribe("/wire_value", 1, wireValueCallback);
    motorDirectionPub = nh.advertise<std_msgs::Int8>("/motor_direction", 1);

    ros::ServiceServer motorClockwiseServiceServer = nh.advertiseService("clockwise", motorClockwiseServiceCallback);
    ros::ServiceServer motorAnticlockwiseServiceServer = nh.advertiseService("anticlockwise", motorAnticlockwiseServiceCallback);
    ros::ServiceServer motorstopServiceServer = nh.advertiseService("stop", motorstopServiceCallback);

    ros::spin();

    return 0;
}
