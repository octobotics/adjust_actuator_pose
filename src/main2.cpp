#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <future>
#include <atomic>

ros::Publisher motorDirectionPub;
ros::Subscriber wireValueSub;
std::atomic<double> currentWireValue;
std::atomic<double> targetValue;
std::atomic<bool> motorStarted(false);

void anticlockwise() {  // Rename the function to anticlockwise
    std_msgs::Int8 direction;
    direction.data = 2;  // Change the direction value
    motorDirectionPub.publish(direction);
}

void stop() {
    std_msgs::Int8 direction;
    direction.data = 0;
    motorDirectionPub.publish(direction);
}

void wireValueCallback(const std_msgs::Float32::ConstPtr& msg) {
    currentWireValue = msg->data;

    // Check if the motor needs to start
    if (!motorStarted) {
        anticlockwise();  // Change the function call
        motorStarted = true;
    }

    // Check if the motor needs to stop
    if (motorStarted && currentWireValue <= targetValue) {
        stop();
        motorStarted = false;
    }
}

std::atomic<bool> anticlockwiseMotorStarted(false);

void motorControlFunction() {
    // Store the current wire value
    double initialWireValue = currentWireValue.load();

    // Add 0.024 to a separate variable, making it the target value
    targetValue = initialWireValue + 0.024;

    // Start the motor
    anticlockwise();  // Change the function call
    anticlockwiseMotorStarted = true;

    // Continuously check if the motor needs to stop
    ros::Rate rate(10);  // Adjust the rate as needed
    while (ros::ok() && currentWireValue < targetValue && anticlockwiseMotorStarted) {
        rate.sleep();
        ros::spinOnce();
    }

    // Stop the motor
    stop();
    anticlockwiseMotorStarted = false;
}

bool motorControlServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    anticlockwiseMotorStarted = false;
    std::future<void> motorControlFuture = std::async(std::launch::async, motorControlFunction);
    res.success = true;
    res.message = "Motor control service executed successfully";
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "anticlockwise_node");  // Unique node name
    ros::NodeHandle nh;

    // New code for motor control
    wireValueSub = nh.subscribe("/wire_value", 1, wireValueCallback);
    motorDirectionPub = nh.advertise<std_msgs::Int8>("/motor_direction", 1);

    // Create a service for motor control
    ros::ServiceServer motorControlServiceServer = nh.advertiseService("anticlockwise_service", motorControlServiceCallback);

    ros::spin();

    return 0;
}