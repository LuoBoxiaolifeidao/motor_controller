#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <thread>               // 补充线程头文件，用于 sleep_for
#include <ros/ros.h>


// ========== 关键修正：外部消息包头文件 + 命名空间 ==========
#include "tactile_sensor_msgs/FingerData.h"
#include "tactile_sensor_msgs/TactileArray.h"

#include "tactile_sensor_driver/ch341_driver.hpp"
#include "tactile_sensor_driver/finger.hpp"
#include "tactile_sensor_driver/sensor_para.hpp"

// 补齐chrono字面量，支持 1s / 500ms
using namespace std::chrono;
using namespace std::chrono_literals;


class TactileSensorNode {
public:
    // 连接状态枚举 (保留原有逻辑)
    enum class ConnectStatus {
        INIT = 0,
        OPEN = 1,
        SET_SPEED = 2,
        SAMPLE_START = 3,
        CHECK = 4,
        SAMPLE_STOP = 5
    };

    TactileSensorNode(ros::NodeHandle& nh) 
        : nh_(nh), 
          connect_status_(ConnectStatus::INIT),
          ch341_check_counter_(0)
    {
        // 读取ROS1节点参数
        nh_.param<double>("polling_rate", polling_rate_, 100.0);
        nh_.param<int>("max_finger_num", max_finger_num_, 5);
        int pca_addr_int;
        nh_.param<int>("pca_base_addr", pca_addr_int, 0x70);
        pca_addr_ = static_cast<uint8_t>(pca_addr_int);

        // 创建发布者，话题+队列大小不变
        pub_tactile_ = nh_.advertise<tactile_sensor_msgs::TactileArray>("tactile_data", 10);

        // 初始化驱动与手指对象
        ch341_ = std::make_shared<Ch341Driver>();
        for (int i = 0; i < max_finger_num_; ++i)
        {
            fingers_.push_back(std::make_unique<Finger>(2 + i, ch341_));
        }

        last_sync_time_ = steady_clock::now();
        ROS_INFO("Tactile Sensor C++ Node Started (ROS1)");
    }

    void run()
    {
        // ROS1 固定频率主循环
        ros::Rate rate(polling_rate_);
        while (ros::ok())
        {
            timer_callback();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void timer_callback()
    {
        switch (connect_status_)
        {
            case ConnectStatus::INIT:
                if (ch341_->init())
                {
                    connect_status_ = ConnectStatus::OPEN;
                }
                else
                {
                    std::this_thread::sleep_for(1s);
                }
                break;

            case ConnectStatus::OPEN:
                if (ch341_->open())
                {
                    connect_status_ = ConnectStatus::SET_SPEED;
                }
                else
                {
                    connect_status_ = ConnectStatus::INIT;
                    std::this_thread::sleep_for(1s);
                }
                break;

            case ConnectStatus::SET_SPEED:
                if (ch341_->set_speed(Ch341Driver::IIC_SPEED_400))
                {
                    ch341_->set_int(false);
                    std::this_thread::sleep_for(500ms);
                    ch341_->set_int(true);
                    connect_status_ = ConnectStatus::SAMPLE_START;
                }
                else
                {
                    ROS_ERROR("Set IIC speed failed");
                    connect_status_ = ConnectStatus::SAMPLE_START;
                }
                break;

            case ConnectStatus::SAMPLE_START:
                connect_status_ = ConnectStatus::CHECK;
                ROS_INFO("Start Sampling...");
                break;

            case ConnectStatus::CHECK:
                if (++ch341_check_counter_ >= static_cast<int>(polling_rate_))
                {
                    ch341_check_counter_ = 0;
                    if (!ch341_->connect_check())
                    {
                        ROS_WARN("CH341 Disconnected!");
                        connect_status_ = ConnectStatus::SAMPLE_STOP;
                    }
                }
                read_and_publish();
                break;

            case ConnectStatus::SAMPLE_STOP:
                for (auto & f : fingers_)
                {
                    if (f->is_connected())
                    {
                        f->sync_sensor();
                        break;
                    }
                }
                ch341_->disconnect();
                connect_status_ = ConnectStatus::INIT;
                break;
        }
    }

    void read_and_publish()
    {
        // 修正：使用外部消息包 tactile_sensor_msgs
        tactile_sensor_msgs::TactileArray tactile_msg;
        tactile_msg.header.stamp = ros::Time::now();
        tactile_msg.header.frame_id = "tactile_sensor_frame";

        uint8_t connected_sensor_chan_mask = 0;
        int connected_sensor_cnt = 0;

        for (size_t i = 0; i < fingers_.size(); ++i)
        {
            auto & finger = fingers_[i];

            // 切换 I2C 多路复用器 (PCA954x 等) 通道
            // 这里假设 pca_idx 对应位掩码
            uint8_t chan_mask = (1 << (i + 2));  // 与 Python 逻辑保持一致 (2+i)
            ch341_->write(pca_addr_, {chan_mask});

            if (!finger->is_connected())
            {
                if (finger->check_sensor())
                {
                    // 格式兼容：%lu 适配 size_t，消除编译警告
                    ROS_INFO("Sensor [%lu] Connected", (unsigned long)i);
                }
            }
            else
            {
                if (finger->cap_read())
                {
                    auto & data = finger->get_read_data();
                    // 修正消息命名空间
                    tactile_sensor_msgs::FingerData f_msg;
                    f_msg.sensor_index = i;

                    // 数组/vector 赋值逻辑不变
                    // 注意：这里需要确保自定义消息中的字段类型与 data 中的一致
                    f_msg.channel_cap_data.assign(data.channel_cap_data.begin(), data.channel_cap_data.end());
                    f_msg.nf.assign(data.nf.begin(), data.nf.end());
                    f_msg.tf.assign(data.tf.begin(), data.tf.end());
                    f_msg.tf_dir.assign(data.tf_dir.begin(), data.tf_dir.end());
                    f_msg.s_prox_cap_data.assign(data.s_prox_cap_data.begin(), data.s_prox_cap_data.end());
                    f_msg.m_prox_cap_data.assign(data.m_prox_cap_data.begin(), data.m_prox_cap_data.end());

                    tactile_msg.fingers.push_back(f_msg);
                    connected_sensor_chan_mask |= chan_mask;
                    connected_sensor_cnt++;
                }
            }
        }

        // 发布消息
        pub_tactile_.publish(tactile_msg);

        // 1秒同步逻辑，代码不变
        auto now_time = steady_clock::now();
        auto elapsed = duration_cast<seconds>(now_time - last_sync_time_).count();

        if (connected_sensor_cnt > 1 && elapsed >= 1)
        {
            last_sync_time_ = now_time;
            ch341_->write(pca_addr_, {connected_sensor_chan_mask});
            ch341_->set_int(true);

            for (auto & f : fingers_)
            {
                if (f->is_connected())
                {
                    // 这里需要 Finger 类暴露 sns_cmd 或者提供 sync 接口
                    // 假设我们在 Finger 中添加一个 sync_sensor 方法
                    // f->sync_sensor(); 
                    break;
                }
            }
        }
    }

    // 成员变量
    ros::NodeHandle nh_;
    double polling_rate_;
    int max_finger_num_;
    uint8_t pca_addr_;

    ConnectStatus connect_status_;
    int ch341_check_counter_;
    steady_clock::time_point last_sync_time_;

    std::shared_ptr<Ch341Driver> ch341_;
    std::vector<std::unique_ptr<Finger>> fingers_;

    ros::Publisher pub_tactile_;
};

int main(int argc, char ** argv) {
    
    // ROS1 节点初始化
    ros::init(argc, argv, "tactile_node");
    ros::NodeHandle nh;
    TactileSensorNode node(nh);
    node.run();

    return 0;
}