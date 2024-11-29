#ifndef IMU_FINAL_HPP
#define IMU_FINAL_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <vector>
#include <sstream>

class KalmanFilter {
private:
    float Q, R, P, X, K;
    bool first_run;
public:
    KalmanFilter(float q = 0.1, float r = 0.1) : Q(q), R(r), first_run(true) {}
    float update(float measurement);
    float getState() const { return X; }
};

class ImuFinal : public rclcpp::Node {
public:
    explicit ImuFinal(const std::string &node_name);
    void resetAngles();  // yaw 각도 초기화 함수

private:
    void imuCallback(const std_msgs::msg::String::SharedPtr msg);
    void quaternionToEuler(const double q0, const double q1, 
                          const double q2, const double q3,
                          double& roll, double& pitch, double& yaw);
    void processImuData(const std::vector<double>& data);
    void normalizeAngle(double &angle);
    
    // 서비스 콜백 함수 추가
    void handleResetSignal(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr roll_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pitch_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
    
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    
    // yaw 오프셋 변수
    double yaw_offset = 0.0;
    
    KalmanFilter roll_kf;
    KalmanFilter pitch_kf;
    KalmanFilter yaw_kf;
};

#endif // IMU_FINAL_HPP