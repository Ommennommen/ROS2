#include "rclcpp/rclcpp.hpp"
#include "pid_controller/controller.h"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <chrono>

class PIDNode : public rclcpp::Node {
    public:
        PIDNode() : Node("pid_node") {
            system_.mass = 1;
            system_.position = 1;
            system_.velocity = 1;
            system_.k = 1;
            system_.c = 1;

            controller_.Kd = 20;
            controller_.Ki = 45;
            controller_.Kp = 5;
            controller_.prevErr = 0;
            controller_.totErr = 0;

            publisher_ = this->create_publisher<std_msgs::msg::Float64>("position", 10);
            
            subscription_ = this->create_subscription<std_msgs::msg::Float64>(
                "setpoint", 10,
                std::bind(&PIDNode::setpoint_callback, this, std::placeholders::_1)
            );

            service_ = this->create_service<std_srvs::srv::Trigger>("reset",
                std::bind(&PIDNode::reset_callback, this, std::placeholders::_1, std::placeholders::_2)
            );

            timer_ = this->create_timer(
            std::chrono::milliseconds(10),
            std::bind(&PIDNode::timer_callback, this)
            );
        }
    private:
        Spring system_;
        PID controller_;
        double dt = 0.01;
        double setpoint_ = 0.0;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        void timer_callback();
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
        void setpoint_callback(const std_msgs::msg::Float64::SharedPtr msg);
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
        void reset_callback(const std_srvs::srv::Trigger::Request::SharedPtr request, 
            std_srvs::srv::Trigger::Response::SharedPtr response);
        
};

void PIDNode::reset_callback(const std_srvs::srv::Trigger::Request::SharedPtr request, 
    std_srvs::srv::Trigger::Response::SharedPtr response){
            RCLCPP_INFO(this->get_logger(), "Reset called! Position: %f", system_.position);
            response->success = true;
            system_.position = 1.0;
            system_.velocity = 0;
            controller_.prevErr = 0;
            controller_.totErr = 0;
            }

void PIDNode::setpoint_callback(const std_msgs::msg::Float64::SharedPtr msg) {
     setpoint_ = msg->data; 
}

void PIDNode::timer_callback(){
    double err = setpoint_ - system_.position;
    double force = PID_out(controller_, err, dt);
    double acceleration = (force - system_.k*system_.position - system_.c*system_.velocity) / system_.mass;
    system_.velocity += acceleration * dt;
    system_.position += system_.velocity * dt;
    auto msg = std_msgs::msg::Float64();
    msg.data = system_.position;
    publisher_->publish(msg);
}

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    std::shared_ptr<PIDNode> node = std::make_shared<PIDNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}