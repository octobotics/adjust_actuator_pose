#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <future>
#include <atomic>
#include <mutex>
#include <ros/spinner.h>

ros::Publisher motorDirectionPub;
ros::Subscriber wireValueSub;

std::atomic<double> currentWireValue;
std::atomic<double> targetValue;
std::atomic<bool> motorClockwiseStarted(false);
std::atomic<bool> motorAnticlockwiseStarted(false);


void motorAnticlockwiseFunction();
void motorClockwiseFunction();
void clockwise();
void anticlockwise();
void stop();
void wireValueCallback(const std_msgs::Float32::ConstPtr& msg);
bool motorAnticlockwiseServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
bool motorClockwiseServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
bool motorstopServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);



int current_value = 125;

void publishServoPose(ros::Publisher& servoPosePub, int value) {
    std_msgs::Int32 msg;
    msg.data = value;
    servoPosePub.publish(msg);

    ROS_INFO("Published to /servo_pose: %d", value);
}

bool addThreeCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    current_value += 5;

    ros::NodeHandle nh;
    ros::Publisher servoPosePub = nh.advertise<std_msgs::Int32>("/servo_pose", 1);
    publishServoPose(servoPosePub, current_value);

    res.success = true;
    res.message = "Added 13 to the current value. Resulting value: " + std::to_string(current_value);

    ROS_INFO_STREAM(res.message);

    return true;
}

bool reduceThreeCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    current_value -= 5;

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
}

void anticlockwise() {
    std_msgs::Int8 direction;
    direction.data = 1;
    motorDirectionPub.publish(direction);
    motorAnticlockwiseStarted = true;
}

void stop() {
    std_msgs::Int8 direction;
    direction.data = 0;
    motorDirectionPub.publish(direction);
    motorClockwiseStarted = false;
    motorAnticlockwiseStarted = false;
}

bool motorClockwiseServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    

   
    clockwise();
    
    res.success = true;
    res.message = "Clockwise motor control service executed successfully";
    return true;
}

bool motorAnticlockwiseServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
 

    
    anticlockwise();
    
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




int main(int argc, char** argv) {
    ros::init(argc, argv, "adjust_and_control_node");
    ros::NodeHandle nh;
    //motorStarted = false;
    ros::ServiceServer addThreeService = nh.advertiseService("/lac_add_service", addThreeCallback);
    ros::ServiceServer reduceThreeService = nh.advertiseService("/lac_reduce_service", reduceThreeCallback);
    ros::Publisher servoPosePub = nh.advertise<std_msgs::Int32>("/servo_pose", 1);
    ROS_INFO("Ready to add or reduce values to the current value.");

    
    //wireValueSub = nh.subscribe("/wire_value", 1, wireValueCallback);
    motorDirectionPub = nh.advertise<std_msgs::Int8>("/motor_direction", 1);

    ros::ServiceServer motorClockwiseServiceServer = nh.advertiseService("clockwise", motorClockwiseServiceCallback);
    ros::ServiceServer motorAnticlockwiseServiceServer = nh.advertiseService("anticlockwise", motorAnticlockwiseServiceCallback);
    ros::ServiceServer motorstopServiceServer = nh.advertiseService("stop", motorstopServiceCallback);

    ros::spin();

    return 0;
}
