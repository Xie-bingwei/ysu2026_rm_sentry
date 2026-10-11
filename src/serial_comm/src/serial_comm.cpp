#include "rclcpp/rclcpp.hpp"
#include "serial_transport/serial_transport.hpp"
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>
#include <wire_protocol/protocol.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <functional>
#include <tf2/LinearMath/Quaternion.h>


using namespace std::chrono_literals;

class Serial_Node: public rclcpp::Node
{
  public:
    Serial_Node()
      : Node("serial_node_cpp")
    {
      RCLCPP_INFO(this->get_logger(),"serial_node启动!");

      //==================================================
      // ROS2 参数
      //==================================================

      //声明并设置默认参数
      this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
      this->declare_parameter<int>("baud_rate", 115200);

      //参数读取
      serial_config_.port_name_ = this->get_parameter("port").as_string();
      serial_config_.baud_rate_ = this->get_parameter("baud_rate").as_int();
      //硬参数
      serial_config_.character_size_ = 8;
      serial_config_.parity_ = asio::serial_port_base::parity::none;
      serial_config_.stop_bits_ = asio::serial_port_base::stop_bits::one;
      serial_config_.flow_control_ = asio::serial_port_base::flow_control::none;

      //串口包协议回调函数设置
      if(!protocol_.set_unpack_callback(0x11, &Serial_Node::handle_joint_state, this))
      {
        RCLCPP_ERROR(this->get_logger(), "gimbal_joint_state 协议回调注册失败");
        return;
      }

      // 必须在串口开启前开启发布者，不然pub_对象可能会是空指针的时候就被串口回调使用
      joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("serial/gimbal_joint_state", 10);
      imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("serial/imu", 10);

      //回调注册完毕后，再开启串口与异步接收
      serial_driver_.start(serial_config_, std::bind(&Serial_Node::serial_receive_callback,this,std::placeholders::_1));

      // 在串口开启前在后都可以
      cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&Serial_Node::cmd_vel_sub_callback,this,std::placeholders::_1));

      // 创建两个定时器模拟两个 topic（必须在串口开启后，尽量）
      //模拟/cmd_vel这种高频消息
      timer1_ = this->create_wall_timer(10ms,std::bind(&Serial_Node::timer1_callback,this));
      //模拟/set_mode这种低频消息
      timer2_ = this->create_wall_timer(500ms,std::bind(&Serial_Node::timer2_callback,this));

    }

  private:
    //
    void cmd_vel_sub_callback(const geometry_msgs::msg::Twist &msg)
    {
      // 相对于云台的线速度、线速度和角速度，单位都是 m/s 和 rad/s 分别
      cmd_vel_field.vx = msg.linear.x;
      cmd_vel_field.vy = msg.linear.y;
      cmd_vel_field.wz = msg.angular.z; //含自旋的角速度wz_spin了，再加上wz_nav2,这里的是wz_final
    }

    void handle_joint_state(fp32 abs_roll,fp32 abs_pitch,fp32 abs_yaw,fp32 rel_pitch,fp32 rel_yaw)
    {
      //先搞时间戳
      const auto stamp = this->now();

      sensor_msgs::msg::JointState msg_jointstate;
      sensor_msgs::msg::Imu msg_imu;

      // 北极熊 IMU 的参考 frame
      msg_imu.header.frame_id = "gimbal_pitch_odom";

      //优先级必须很高
      msg_jointstate.header.stamp = stamp;
      msg_imu.header.stamp = stamp;

      msg_jointstate.name = {
          "gimbal_pitch_joint",      // 小pitch
          "gimbal_yaw_joint",        //小yaw
          "gimbal_pitch_odom_joint",   //fixed
          "gimbal_yaw_odom_joint"    //大yaw
      };

      msg_jointstate.position = {
                static_cast<fp64>(rel_pitch),  // 唯一机械 pitch
                static_cast<fp64>(rel_yaw),    // 唯一机械 yaw
                0.0,                          // 固定层
                0.0                           // 固定层
      };

      tf2::Quaternion q;

      q.setRPY(static_cast<fp64>(abs_roll), static_cast<fp64>(abs_pitch), static_cast<fp64>(abs_yaw));

      msg_imu.orientation.x = q.x();
      msg_imu.orientation.y = q.y();
      msg_imu.orientation.z = q.z();
      msg_imu.orientation.w = q.w();

      joint_state_pub_->publish(msg_jointstate);
      imu_pub_->publish(msg_imu);

      RCLCPP_DEBUG(this->get_logger(),"[RX gimbal_joint_state] pitch=%.3f yaw=%.3f",rel_pitch,rel_yaw);
      RCLCPP_DEBUG(this->get_logger(),"[RX imu] roll=%.3f pitch=%.3f yaw=%.3f",abs_roll,abs_pitch,abs_yaw);
    }


    void serial_receive_callback(std::span<const uint8_t> msg)
    {
      //解包
      protocol_.feed(msg);
    }

    void timer1_callback()
    {
      auto frame = protocol_.pack(0x01, cmd_vel_field.vx,cmd_vel_field.vy,cmd_vel_field.wz);
                
      //异步发送数据
      serial_driver_.async_write(frame);
    }

    void timer2_callback()
    {
      // 模拟不断变化的模式命令
      ++mode_;

      if (mode_ > 3)
      {
        mode_ = 0;
      }

      uint32_t seq = ++event_count_;

      RCLCPP_DEBUG(this->get_logger(),"[set_mode] seq = %u,mode=%d",seq,mode_);

      auto frame = protocol_.pack(0x02, seq,mode_);

      //异步发送数据
      serial_driver_.async_write(frame);
    }

    //ROS
    rclcpp::TimerBase::SharedPtr timer1_;
    rclcpp::TimerBase::SharedPtr timer2_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    //注意，protocol_一定要比serial_driver_早。
    // protocol_ 先构造、后析构；串口析构时先 stop() 并等待接收线程退出。
    wire_protocol::CallbackProtocol protocol_;

    //Serial
    serial_transport::Serial_Config serial_config_;
    serial_transport::SerialTransport serial_driver_;

    //cmd_vel数据
    struct 
    {
      fp32 vx;
      fp32 vy;
      fp32 wz;
    }cmd_vel_field{0.0,0.0,0.0};

    //模拟数据
    int32_t mode_{0};
    uint32_t event_count_{0};

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  auto node_ = std::make_shared<Serial_Node>();
  rclcpp::spin(node_);

  rclcpp::shutdown();
  return 0;
}
