#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>  
#include <cstdlib>  // For system() function
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>
#include <iostream>
#include <thread>

#define SHM_KEY 12345

struct shared_data {
    float fAcc[3];
    float fGyro[3];
    float fAngle[3];
    int sReg[3];
};
struct shared_data *shm_data;
class SensorPublisherNode : public rclcpp::Node
{
public:
    SensorPublisherNode() : Node("sensor_publisher")
    {   
        
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);

        mag_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("mag_data", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&SensorPublisherNode::publish_data, this));
    }

private:
    void publish_data()
    {
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.linear_acceleration.x = shm_data->fAcc[0];
        imu_msg.linear_acceleration.y = shm_data->fAcc[1];
        imu_msg.linear_acceleration.z = shm_data->fAcc[2];
        
        imu_msg.angular_velocity.x = shm_data->fGyro[0];
        imu_msg.angular_velocity.y = shm_data->fGyro[1];
        imu_msg.angular_velocity.z = shm_data->fGyro[2];

        imu_msg.orientation.x = shm_data->fAngle[0];  // roll
        imu_msg.orientation.y = shm_data->fAngle[1];  // pitch
        imu_msg.orientation.z = shm_data->fAngle[2];  // yaw
        imu_msg.orientation.w = 1.0;

        imu_publisher_->publish(imu_msg);
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", std::to_string(shm_data->fAngle[1]).c_str());
        auto mag_msg = std_msgs::msg::Float32MultiArray();
        mag_msg.data.push_back(shm_data->sReg[0]);
        mag_msg.data.push_back(shm_data->sReg[1]);
        mag_msg.data.push_back(shm_data->sReg[2]);

        mag_publisher_->publish(mag_msg);

        
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr mag_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
void run_command() {
    // Running the system command in a thread
    system("/home/chippu/ros_ws_cpp/src/cpp_pubsub/src/a.out /dev/ttyUSB0");
}
int main(int argc, char **argv)
{   
    std::thread t(run_command);
    int shmid = shmget(SHM_KEY, sizeof(struct shared_data), 0666);
        if (shmid == -1) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "shmget failed in main");
          return 0;
        }

        shm_data = (struct shared_data *)shmat(shmid, NULL, 0);
        if (shm_data == (void *)-1) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "shmget failed in main");
          return 0;
        }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorPublisherNode>());
    rclcpp::shutdown();
    if (shmdt(shm_data) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "shmdt failed in main");
    }
    return 0;
}
