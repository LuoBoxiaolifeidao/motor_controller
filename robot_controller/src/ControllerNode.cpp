#include "ControllerNode.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

#define DM2CTRAC 0.0014

// 飞特舵机: 编码器刻度 <-> 角度(度), 4096 tick = 360 deg
static constexpr double FT_TICK_PER_DEG = 4096.0 / 360.0;
static constexpr double FT_DEG_PER_TICK = 360.0 / 4096.0;

// 飞特 dps/dps² → 原始寄存器值
static constexpr double FT_DPS_MAX  = 87.84;
static constexpr double FT_SPD_MAX  = 5000.0;
static constexpr double FT_ACC_REF  = 1000.0;

static double rawSpeedToDps(int speed_raw) {
    // 实测：500 编码值 ≈ 42.4 °/s
    // 比例：1 编码值 ≈ 0.0848 °/s
    // 速度寄存器 bit15 为方向位, 解码后可能为负, 这里只取大小
    return std::abs(speed_raw) * 0.0848;
}

// DPS → 编码值（要发给电机的速度值）
static uint16_t dpsToRawSpeed(double dps) {
    if (dps <= 0) return 0;
    // 1 °/s ≈ 11.8 编码值
    uint16_t value = static_cast<uint16_t>(dps * 11.8);
    if (value > 5000) value = 5000;  // 限制最大编码值
    return value;
}

// 编码值 → DPS²（实际物理加速度）
static double rawAccelToDps2(uint8_t accel_raw) {
    if (accel_raw == 0) return 9999.0;  // 0表示最大加速度，返回一个大值
    return static_cast<double>(accel_raw) * 8.7;
}

// DPS² → 编码值（要发给电机的加速度值）
// 注意: 飞特 acc=0 表示最大加速度, 因此下限必须为 1
static uint8_t dps2ToRawAccel(double dps2) {
    if (dps2 <= 0) return 1;
    uint8_t value = static_cast<uint8_t>(dps2 / 8.7 + 0.5);
    if (value < 1) value = 1;
    if (value > 254) value = 254;
    return value;
}

static void quatToRPY(double x, double y, double z, double w, double& roll, double& pitch, double& yaw) {
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0)
        pitch = std::copysign(M_PI / 2.0, sinp);
    else
        pitch = std::asin(sinp);

    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

// 飞特舵机零点标定在 URDF 0 度对应的电机 180 度位置, 且转动方向与 URDF 相反
//   urdf = 180 - motor_deg      motor_deg = 180 - urdf
static constexpr double FT_ZERO_OFFSET_DEG = 180.0;
static inline double ftMotorToUrdf(double motor_deg) { return FT_ZERO_OFFSET_DEG - motor_deg; }
static inline double ftUrdfToMotor(double urdf_deg)  { return FT_ZERO_OFFSET_DEG - urdf_deg; }

static int32_t encoderDelta(uint32_t current, uint32_t last) {
    uint32_t raw = current - last;
    if (raw < 0x80000000)
        return static_cast<int32_t>(raw);
    return static_cast<int32_t>(raw) - 0x100000000;
}

template<typename T>
static T clampValue(T value, T lo, T hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static void dm2cWrite(double value) {
    std::ofstream("/home/firefly/tashan/mgl/dm2c-controller/dm2c_lift.txt")
        << std::fixed << std::setprecision(2) << value;
}

static void jointsToQ8(const std::vector<JointState>& joints, std::array<double, 8>& q8) {
    constexpr double M_TO_MM    = 1000.0;
    constexpr double DEG_TO_RAD = M_PI / 180.0;
    constexpr double EXT_TO_M   = 1.0 / 1000.0;
    q8[0] = joints[0].position / M_TO_MM;
    q8[1] = joints[1].position * DEG_TO_RAD;
    double ext = joints[2].position * EXT_TO_M / 3.0;
    q8[2] = ext;
    q8[3] = ext;
    q8[4] = ext;
    q8[5] = joints[3].position * DEG_TO_RAD;
    q8[6] = joints[4].position * DEG_TO_RAD;
    q8[7] = joints[5].position * DEG_TO_RAD;
}

static geometry_msgs::Pose frameToPose(const KDL::Frame& frame) {
    geometry_msgs::Pose pose;
    pose.position.x = frame.p.x();
    pose.position.y = frame.p.y();
    pose.position.z = frame.p.z();
    frame.M.GetQuaternion(pose.orientation.x, pose.orientation.y,
                          pose.orientation.z, pose.orientation.w);
    return pose;
}

ControllerNode::ControllerNode()
    : nh_()
{
    left_tool_offset_ = KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(0.0, 0.0, 0.32));
    loadMotorConfig();

    jointDM2C_ = std::make_shared<DM2CController>(LiftColumn::LeftArm, dm2c_cfg_.port, dm2c_cfg_.baudrate);
    jointShoulder_ = std::make_shared<MWD485Controller>(shoulder_cfg_.id, shoulder_cfg_.port, shoulder_cfg_.baudrate);
    jointExtend_   = std::make_shared<MWD485Controller>(extend_cfg_.id, extend_cfg_.port, extend_cfg_.baudrate);
    jointFT_       = nullptr;  // init() 中通过 SerialPort 单例创建

    init();

    arm_joints_sub_  = nh_.subscribe<std_msgs::Float64MultiArray>(
        "left_arm_joints", 1, &ControllerNode::ArmJointsCB, this);
    tcp_sub_         = nh_.subscribe<std_msgs::Float64MultiArray>(
        "left_tcp", 1, &ControllerNode::TcpCB, this);
    zero_srv_        = nh_.advertiseService("set_zero", &ControllerNode::setZeroCB, this);
    joint_state_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("left_arm_joint_states", 1);

    named_targets_["ready"] = {600, 0, 0, 0, 0, 0};
    named_targets_["home"]  = {300, 0, 0, 0, 0, 0};
    named_targets_["safe_carry"]  = {300, 63, 0, 129, 90, -90};
    move_arm_as_.reset(new MoveArmServer(nh_, "/left_arm/move_arm",
        boost::bind(&ControllerNode::executeMoveArm, this, _1), false));
    move_arm_as_->start();

    gripper_as_.reset(new GripperServer(nh_, "/left_gripper/gripper_cmd",
        boost::bind(&ControllerNode::executeGripper, this, _1), false));
    gripper_as_->start();

    loop_thread_ = std::thread(&ControllerNode::loop, this);
    jointShoulder_->brakeLock();
    jointExtend_->brakeLock();
    std::cout<<jointShoulder_->getBrakeState()<<std::endl;
    std::cout<<jointExtend_->getBrakeState()<<std::endl;
}

ControllerNode::~ControllerNode() {
    running_ = false;
    cv_.notify_all();
    if (loop_thread_.joinable()) loop_thread_.join();
    stopAll();
    jointFT_->syncReadEnd();
    jointFT_->end();
}

void ControllerNode::loadMotorConfig()
{
    std::string path = ros::package::getPath("robot_controller") + "/config/motors.yaml";
    try {
        YAML::Node config = YAML::LoadFile(path);
        std::string version = config["version"] ? config["version"].as<std::string>() : "unknown";
        ROS_INFO("[CONFIG] motors.yaml version=%s path=%s", version.c_str(), path.c_str());

        auto motors = config["motors"];
        if (!motors) {
            ROS_WARN("[CONFIG] No 'motors' section, using defaults");
            return;
        }

        auto readStr = [](const YAML::Node& n, const std::string& k, std::string& v)      { if (n[k]) v = n[k].as<std::string>(); };
        auto readInt = [](const YAML::Node& n, const std::string& k, int& v)              { if (n[k]) v = n[k].as<int>(); };
        auto readDbl = [](const YAML::Node& n, const std::string& k, double& v)           { if (n[k]) v = n[k].as<double>(); };
        auto readVec = [](const YAML::Node& n, const std::string& k, std::vector<int>& v) { if (n[k]) v = n[k].as<std::vector<int>>(); };

        if (auto n = motors["left_dm2c"]) {
            readInt(n, "id",         dm2c_cfg_.id);
            readStr(n, "port",       dm2c_cfg_.port);
            readInt(n, "baudrate",   dm2c_cfg_.baudrate);
            readDbl(n, "min_speed",  dm2c_cfg_.min_speed);
            readDbl(n, "max_speed",  dm2c_cfg_.max_speed);
            readInt(n, "max_accel",  dm2c_cfg_.max_accel);
            readDbl(n, "dead_zone",  dm2c_cfg_.dead_zone);
        }

        if (auto n = motors["shoulder_485"]) {
            readInt(n, "id",         shoulder_cfg_.id);
            readStr(n, "port",       shoulder_cfg_.port);
            readInt(n, "baudrate",   shoulder_cfg_.baudrate);
            readDbl(n, "gear_ratio", shoulder_cfg_.gear_ratio);
            readDbl(n, "min_speed",  shoulder_cfg_.min_speed);
            readDbl(n, "max_speed",  shoulder_cfg_.max_speed);
            readInt(n, "max_accel",  shoulder_cfg_.max_accel);
            readDbl(n, "dead_zone",  shoulder_cfg_.dead_zone);
            readDbl(n, "j_max",      shoulder_cfg_.j_max);
        }

        if (auto n = motors["extend_485"]) {
            readInt(n, "id",         extend_cfg_.id);
            readStr(n, "port",       extend_cfg_.port);
            readInt(n, "baudrate",   extend_cfg_.baudrate);
            readDbl(n, "gear_ratio", extend_cfg_.gear_ratio);
            readDbl(n, "min_speed",  extend_cfg_.min_speed);
            readDbl(n, "max_speed",  extend_cfg_.max_speed);
            readInt(n, "max_accel",  extend_cfg_.max_accel);
            readDbl(n, "dead_zone",  extend_cfg_.dead_zone);
            readDbl(n, "j_max",      extend_cfg_.j_max);
        }

        if (auto n = motors["left_ft"]) {
            readStr(n, "port",       ft_cfg_.port);
            readInt(n, "baudrate",   ft_cfg_.baudrate);
            readVec(n, "ids",        ft_cfg_.ids);
            readVec(n, "min_speed",  ft_cfg_.min_speed);
            readVec(n, "max_speed",  ft_cfg_.max_speed);
            readVec(n, "max_accel",  ft_cfg_.max_accel);
            readVec(n, "dead_zone",  ft_cfg_.dead_zone);
            readVec(n, "j_max",      ft_cfg_.j_max);
        }

        if (auto n = motors["left_gripper"]) {
            readInt(n, "ids",        gripper_cfg_.id);
            readStr(n, "port",       gripper_cfg_.port);
            readInt(n, "baudrate",   gripper_cfg_.baudrate);
            readInt(n, "open",       gripper_cfg_.open);
            readInt(n, "close",      gripper_cfg_.close);
            readInt(n, "dead_zone",  gripper_cfg_.dead_zone);
        }
    } 
    catch (const YAML::Exception& e) {
        ROS_ERROR("[CONFIG] Failed to load %s: %s", path.c_str(), e.what());
    }
}

void ControllerNode::init()
{
    curState_.assign(kJointNum, JointState{});
    targetPos_.assign(kJointNum, DBL_MIN);
    targetVel_.assign(kJointNum, 0.0);
    targetAcc_.assign(kJointNum, 0.0);
    finalTarget_.assign(kJointNum, DBL_MIN);

    // FT 腕部通过 SerialPort 单例通信, 与 MWD485 共享同一 fd + mutex
    auto ftSp = SerialPort::create(ft_cfg_.port, ft_cfg_.baudrate);
    jointFT_ = std::make_shared<SMS_STS_Shared>(ftSp);
    if (!jointFT_->begin(ft_cfg_.baudrate, nullptr))
        throw std::runtime_error("Failed to init ft joint");

    jointFT_->syncReadBegin(ft_cfg_.ids.size() + 1, 4, 100);
    jointShoulder_->enable();
    jointShoulder_->clearError();
    jointShoulder_->setAcceleration(static_cast<int32_t>(shoulder_cfg_.max_accel));
    jointExtend_->enable();
    jointExtend_->clearError();
    jointExtend_->setAcceleration(static_cast<int32_t>(extend_cfg_.max_accel));

    // dm2c 没有断电保存, 手动清零避免脏数据
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    double liftPos = 0.0;
    if (std::ifstream file("dm2c_lift.txt"); file)
        file >> liftPos;
    else
        dm2cWrite(0.0);
    dm2c_value_ = {liftPos, jointDM2C_->readActualPosition()};
    running_ = true;
}

void ControllerNode::loop()
{
    ros::Rate rate(100);
    int cnt = 0;
    double lastSaved = -1e9;

    while (ros::ok() && running_) {
        {
            std::lock_guard<std::mutex> lock(curState_mutex_);
            readPdo(Motor::DM2C);
            readPdo(Motor::MWD_Shoulder);
            readPdo(Motor::MWD_Extend);
            readPdo(Motor::FT);
        }
 //       publishState();
        if (++cnt % 10 == 0) {
            publishState();
            double lift;
            {
                std::lock_guard<std::mutex> lock(dm2c_mutex_);
                lift = dm2c_value_.first;
            }
            if (std::abs(lift - lastSaved) > 0.1) {
                lastSaved = lift;
                dm2cWrite(lift);
            }
        }
        rate.sleep();
    }
    stopAll();
}

void ControllerNode::publishState()
{
    std_msgs::Float64MultiArray msg;
    msg.data.reserve(kJointNum * 2);
    for (const auto& s : curState_)
        msg.data.push_back(s.position);
    for (const auto& s : curState_)
        msg.data.push_back(s.velocity);
    joint_state_pub_.publish(msg);

    std::cout << "[STATE] pos:";
    for (const auto& s : curState_)
        std::cout << " " << std::fixed << std::setprecision(2) << s.position;
    std::cout << " | vel:";
    for (const auto& s : curState_)
        std::cout << " " << std::fixed << std::setprecision(2) << s.velocity;
    std::array<double, 8> q8;
    jointsToQ8(curState_, q8);

    left_tcp_ = left_arm_ik::fk(q8);

    left_tool_ = left_tcp_ * left_tool_offset_;
    double tcp_roll, tcp_pitch, tcp_yaw;
    left_tcp_.M.GetRPY(tcp_roll, tcp_pitch, tcp_yaw);

    double tool_roll, tool_pitch, tool_yaw;
    left_tool_.M.GetRPY(tool_roll, tool_pitch, tool_yaw);
    std::cout << " tcp: "
          << "pos=[" << left_tcp_.p.x() << ", " << left_tcp_.p.y() << ", " << left_tcp_.p.z() << "], "
          << "rpy=[" << tcp_roll << ", " << tcp_pitch << ", " << tcp_yaw << "]";

    std::cout << "  tool: "
          << "pos=[" << left_tool_.p.x() << ", " << left_tool_.p.y() << ", " << left_tool_.p.z() << "], "
          << "rpy=[" << tool_roll << ", " << tool_pitch << ", " << tool_yaw << "]"
          << std::endl;
}

void ControllerNode::stopAll()
{
    running_   = false;

    jointDM2C_->stopAllMotion();
    jointShoulder_->stop();
    jointShoulder_->brakeLock();
    jointExtend_->stop();
    jointExtend_->brakeLock();
    for (int id : ft_cfg_.ids)
        jointFT_->EnableTorque(id, 0);

    cv_.notify_all();
}


void ControllerNode::driveJoints(const std::vector<double>& target)
{
    for (int i = 0; i < kJointNum; ++i) {
        finalTarget_[i] = target[i];
        targetPos_[i] = finalTarget_[i];
        std::cout<<"target["<<i<<"]="<<targetPos_[i]<<"  ";
    }
    std::cout<<std::endl;

    jointShoulder_->brakeRelease();
    jointExtend_->brakeRelease();

    writePdo(Motor::DM2C);
    writePdo(Motor::MWD_Shoulder);
    writePdo(Motor::MWD_Extend);
    writePdo(Motor::FT);

    ros::Rate rate(100);
    while (ros::ok() && running_) {
        JointState snapshot[kJointNum];
        {
            std::lock_guard<std::mutex> lock(curState_mutex_);
            for (int i = 0; i < kJointNum; ++i)
                snapshot[i] = curState_[i];
        }
        bool all_in_zone = true;
        double dz[kJointNum] = {dm2c_cfg_.dead_zone, shoulder_cfg_.dead_zone, extend_cfg_.dead_zone,
                                (double)ft_cfg_.dead_zone[0], (double)ft_cfg_.dead_zone[1], (double)ft_cfg_.dead_zone[2]};
        for (int i = 0; i < kJointNum; ++i) {
            if (std::abs(finalTarget_[i] - snapshot[i].position) > dz[i])
                all_in_zone = false;
        }
        if (all_in_zone) { std::cout << "all in zone" << std::endl; jointShoulder_->brakeLock(); jointExtend_->brakeLock(); break; }

        rate.sleep();
    }
    jointShoulder_->brakeLock();
    jointExtend_->brakeLock();
}


void ControllerNode::driveJoints(const std::vector<double>& target, common_comms::MoveArmFeedback& feedback){
    for (int i = 0; i < kJointNum; ++i) {
        finalTarget_[i] = target[i];
        targetPos_[i] = finalTarget_[i];
        std::cout<<"target["<<i<<"]="<<targetPos_[i]<<"  ";
    }
    std::cout<<std::endl;

    jointShoulder_->brakeRelease();
    jointExtend_->brakeRelease();
    writePdo(Motor::DM2C);
    writePdo(Motor::MWD_Shoulder);
    writePdo(Motor::MWD_Extend);
    writePdo(Motor::FT);
    double dz[kJointNum] = {dm2c_cfg_.dead_zone, shoulder_cfg_.dead_zone, extend_cfg_.dead_zone,
                                (double)ft_cfg_.dead_zone[0], (double)ft_cfg_.dead_zone[1], (double)ft_cfg_.dead_zone[2]};
    ros::Rate rate(100);
    while (ros::ok() && running_) {

        feedback.current_pose = frameToPose(left_tool_);
        feedback.state = "moving";
        move_arm_as_->publishFeedback(feedback);

        JointState snapshot[kJointNum];
        {
            std::lock_guard<std::mutex> lock(curState_mutex_);
            for (int i = 0; i < kJointNum; ++i)
                snapshot[i] = curState_[i];
        }
        bool all_in_zone = true;
        double dz[kJointNum] = {dm2c_cfg_.dead_zone, shoulder_cfg_.dead_zone, extend_cfg_.dead_zone,
                                (double)ft_cfg_.dead_zone[0], (double)ft_cfg_.dead_zone[1], (double)ft_cfg_.dead_zone[2]};
        for (int i = 0; i < kJointNum; ++i) {
            if (std::abs(finalTarget_[i] - snapshot[i].position) > dz[i])
                all_in_zone = false;
        }
        if (all_in_zone) { std::cout << "all in zone" << std::endl; jointShoulder_->brakeLock(); jointExtend_->brakeLock(); break; }
        rate.sleep();
    }
    jointShoulder_->brakeLock();
    jointExtend_->brakeLock();
}


void ControllerNode::ArmJointsCB(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() % kJointNum != 0) {
        ROS_ERROR("msg data error, size = %ld", msg->data.size());
        return;
    }

    const int steps = msg->data.size() / kJointNum;
    for (int i = 0; i < steps && ros::ok() && running_; ++i) {
        std::vector<double> target(msg->data.begin() + i * kJointNum,
                                   msg->data.begin() + (i + 1) * kJointNum);
        driveJoints(target);
    }
}

std::vector<double> ControllerNode::mapIkToJoints(const std::array<double, 8>& q8) const
{
    constexpr double M_TO_MM       = 1000.0;
    constexpr double RAD_TO_DEG    = 180.0 / M_PI;
    constexpr double EXT_M_TO_DEG  = 1000.0;   // 伸展直线段 -> AK 关节角度系数

    std::vector<double> j(kJointNum, 0.0);
    j[static_cast<int>(Joint::Lift)]     = q8[0] * M_TO_MM;
    j[static_cast<int>(Joint::Shoulder)] = q8[1] * RAD_TO_DEG;
    j[static_cast<int>(Joint::Extend)]   = (q8[2] + q8[3] + q8[4]) * EXT_M_TO_DEG;
    j[static_cast<int>(Joint::Wrist1)]   = q8[5] * RAD_TO_DEG;
    j[static_cast<int>(Joint::Wrist2)]   = q8[6] * RAD_TO_DEG;
    j[static_cast<int>(Joint::Wrist3)]   = q8[7] * RAD_TO_DEG;
    return j;
}

void ControllerNode::TcpCB(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() < 6) {
        ROS_ERROR("tcp data error, size = %ld", msg->data.size());
        return;
    }
    if (!running_) return;

    ROS_INFO("[TCP] in: pos=[%.3f %.3f %.3f] rpy=[%.3f %.3f %.3f]",
             msg->data[0], msg->data[1], msg->data[2],
             msg->data[5], msg->data[4], msg->data[3]);

    KDL::Vector pos(msg->data[0], msg->data[1], msg->data[2]);
    KDL::Rotation rot = KDL::Rotation::RPY(msg->data[5], msg->data[4], msg->data[3]);
    
    KDL::Frame original_pose(rot, pos);
    KDL::Frame target = original_pose * left_tool_offset_.Inverse();
    const auto& jl = left_arm_ik::JOINT_LOWER;
    const auto& ju = left_arm_ik::JOINT_UPPER;
    std::array<double, 8> q_init = {ik_q_[0], ik_q_[1], ik_q_[2], ik_q_[3], ik_q_[4], msg->data[3], msg->data[4], msg->data[5]};

    auto res = left_arm_ik::solve_ik(target.p, target.M, q_init);
    if (!res.success) {
        ROS_WARN("IK FAIL e_p=%.4f e_r=%.4f", res.pos_err, res.rot_err);
        return;
    }

    ik_q_ = res.q8;
    auto joints = mapIkToJoints(res.q8);
    ROS_INFO("[TCP] out: lift=%.1f sh=%.1f ext=%.1f w1=%.1f w2=%.1f w3=%.1f",
             joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
    ROS_INFO("IK OK e_p=%.4f e_r=%.4f", res.pos_err, res.rot_err);
    driveJoints(joints);
}

void ControllerNode::readPdo(Motor type)
{
    switch (type) {
    case Motor::DM2C: {
        uint32_t enc = jointDM2C_->readActualPosition();
        double speed = std::abs(jointDM2C_->readActualSpeed());
        {
            std::lock_guard<std::mutex> lock(dm2c_mutex_);
            double delta = encoderDelta(enc, dm2c_value_.second) * DM2CTRAC;
            dm2c_value_.first -= delta;  //***
            dm2c_value_.second = enc;
            auto& s = curState_[static_cast<int>(Joint::Lift)];
            s.position = dm2c_value_.first;
            s.velocity = speed;
        }
        break;
    }
    case Motor::MWD_Shoulder: {
        std::lock_guard<std::mutex> lock(ft_mutex_); 
        jointShoulder_->refreshState();
        auto& s = curState_[static_cast<int>(Joint::Shoulder)];
        s.position = jointShoulder_->getMotorAngle() / 100.0 / shoulder_cfg_.gear_ratio;
        s.velocity = std::abs(jointShoulder_->getSpeed()) / shoulder_cfg_.gear_ratio;
        break;
    }
    case Motor::MWD_Extend: {
        std::lock_guard<std::mutex> lock(ft_mutex_); 
        jointExtend_->refreshState();
        auto& s = curState_[static_cast<int>(Joint::Extend)];
        s.position = -(jointExtend_->getMotorAngle() / 100.0 / extend_cfg_.gear_ratio/1.48f);
        s.velocity = std::abs(jointExtend_->getSpeed()) / extend_cfg_.gear_ratio;
        break;
    }
    case Motor::FT: {
        std::lock_guard<std::mutex> lock(ft_mutex_);
        u8 ids[4];
        for (int k = 0; k < 3; ++k)
            ids[k] = static_cast<u8>(ft_cfg_.ids[k]);
        ids[3] = static_cast<u8>(gripper_cfg_.id);

        if (jointFT_->syncReadPacketTx(ids, 4, SMS_STS_PRESENT_POSITION_L, 4) > 0) {
            u8 rxBuf[4];
            for (int k = 0; k < 3; ++k) {
                if (jointFT_->syncReadPacketRx(ids[k], rxBuf) != 4) continue;
                auto& s = curState_[static_cast<int>(Joint::Wrist1) + k];
                s.position = ftMotorToUrdf(jointFT_->syncReadRxPacketToWrod(15) * FT_DEG_PER_TICK);
                s.velocity = rawSpeedToDps(jointFT_->syncReadRxPacketToWrod(15));
            }
            if (jointFT_->syncReadPacketRx(ids[3], rxBuf) == 4) {
                gripper_position_ = jointFT_->syncReadRxPacketToWrod(15);
            }
        }
        break;
    }
    }

}

void ControllerNode::writePdo(Motor type)
{
    switch (type) {
    case Motor::DM2C: {
        double err = targetPos_[static_cast<int>(Joint::Lift)] - curState_[static_cast<int>(Joint::Lift)].position;
        double acc = dm2c_cfg_.max_accel;
        jointDM2C_->moveToRelativePosition(static_cast<int32_t>(-err / DM2CTRAC),
                                           static_cast<int16_t>(dm2c_cfg_.max_speed),
                                           static_cast<uint16_t>(acc),
                                           static_cast<uint16_t>(acc));
        break;
    }
    case Motor::MWD_Shoulder: {
        std::lock_guard<std::mutex> lock(ft_mutex_); 
        int32_t target = static_cast<int32_t>(targetPos_[static_cast<int>(Joint::Shoulder)] * shoulder_cfg_.gear_ratio * 100.0);
        uint16_t speed = static_cast<uint16_t>(shoulder_cfg_.max_speed * shoulder_cfg_.gear_ratio);
        jointShoulder_->multiTurnPosition(target, speed);
        break;
    }
    case Motor::MWD_Extend: {
        std::lock_guard<std::mutex> lock(ft_mutex_); 
        int32_t target = static_cast<int32_t>(-targetPos_[static_cast<int>(Joint::Extend)] * extend_cfg_.gear_ratio * 100.0 * 1.48f);
        uint16_t speed = static_cast<uint16_t>(extend_cfg_.max_speed * extend_cfg_.gear_ratio);
        jointExtend_->multiTurnPosition(target, speed);
        break;
    }
    case Motor::FT: {
        std::lock_guard<std::mutex> lock(ft_mutex_);
        u8  ids[3];
        s16 pos[3];
        u16 spd[3];
        u8  acc[3];
        for (int k = 0; k < 3; ++k) {
            ids[k] = static_cast<u8>(ft_cfg_.ids[k]);
            double motor_deg = ftUrdfToMotor(targetPos_[static_cast<int>(Joint::Wrist1) + k]);
            pos[k] = static_cast<s16>(motor_deg * FT_TICK_PER_DEG);
            spd[k] = dpsToRawSpeed(static_cast<double>(ft_cfg_.max_speed[k]));
            acc[k] = dps2ToRawAccel(static_cast<double>(ft_cfg_.max_accel[k]));
        }
        std::cout<<"pos[0]= "<<pos[0]<<std::endl;
        jointFT_->SyncWritePosEx(ids, 3, pos, spd, acc);
        break;
    }
    }
}

bool ControllerNode::setZeroCB(robot_controller::SetZero::Request &req, robot_controller::SetZero::Response &res)
{
    ROS_INFO("Set zero: %s", req.name.c_str());
    if (req.name == "L_lift") {
        jointDM2C_->setCurrentPositionAsZero();
        dm2c_value_ = {0, 0};
        curState_[static_cast<int>(Joint::Lift)] = JointState{};
        res.status     = jointDM2C_->readStatus();
        res.error_code = jointDM2C_->readAlarm();
    } else if (req.name == "L_shoulder") {
        jointShoulder_->setZeroToROM();
        curState_[static_cast<int>(Joint::Shoulder)] = JointState{};
        res.status     = jointShoulder_->getMotorState();
        res.error_code = jointShoulder_->getErrorState();
    } else if (req.name == "L_extend") {
        jointExtend_->setZeroToROM();
        curState_[static_cast<int>(Joint::Extend)] = JointState{};
        res.status     = jointExtend_->getMotorState();
        res.error_code = jointExtend_->getErrorState();
    } else if (req.name == "L_wrist") {
        std::lock_guard<std::mutex> lock(ft_mutex_);
        for (int k = 0; k < 3; ++k) {
            jointFT_->CalibrationOfs(ft_cfg_.ids[k]);
            curState_[static_cast<int>(Joint::Wrist1) + k] = JointState{};
        }
        res.status     = 0;
        res.error_code = 0;
    } else if (req.name == "L_wrist1" || req.name == "L_wrist2" || req.name == "L_wrist3") {
        int k = req.name.back() - '1';
        std::lock_guard<std::mutex> lock(ft_mutex_);
        jointFT_->CalibrationOfs(ft_cfg_.ids[k]);
        curState_[static_cast<int>(Joint::Wrist1) + k] = JointState{};
        res.status     = 0;
        res.error_code = 0;
    } else {
        return false;
    }
    return true;
}


void ControllerNode::executeMoveArm(const common_comms::MoveArmGoalConstPtr& goal)
{
    common_comms::MoveArmResult result;
    common_comms::MoveArmFeedback feedback;

    if (!running_) {
        result.success = false;
        result.error_msg = "Controller not initialized";
        move_arm_as_->setAborted(result);
        return;
    }

    if (goal->arm_group != "left_arm") {
        result.success = false;
        result.error_msg = "Only left_arm is supported";
        move_arm_as_->setAborted(result);
        ROS_WARN("[MoveArm] unsupported arm_group: %s", goal->arm_group.c_str());
        return;
    }

    std::lock_guard<std::mutex> lock(move_mutex_);

    std::vector<double> target_joints;

    if (!goal->named_target.empty()) {
        auto it = named_targets_.find(goal->named_target);
        if (it == named_targets_.end()) {
            result.success = false;
            result.error_msg = "Unknown named_target: " + goal->named_target;
            move_arm_as_->setAborted(result);
            return;
        }
        ROS_INFO("[MoveArm] executing named_target: %s", goal->named_target.c_str());
        target_joints = it->second;
    } 
    else {
        double roll, pitch, yaw;
        quatToRPY(goal->target_pose.orientation.x, goal->target_pose.orientation.y,
                  goal->target_pose.orientation.z, goal->target_pose.orientation.w,
                  roll, pitch, yaw);

        KDL::Vector pos(goal->target_pose.position.x, goal->target_pose.position.y,
                        goal->target_pose.position.z);
        KDL::Rotation rot = KDL::Rotation::RPY(roll, pitch, yaw);
        KDL::Frame original_pose(rot, pos);
        ROS_INFO("[MoveArm] frame_id=%s original_pose: pos=[%.3f %.3f %.3f] rpy=[%.3f %.3f %.3f]", goal->frame_id.c_str(),
                 original_pose.p.x(), original_pose.p.y(), original_pose.p.z(), roll, pitch, yaw);

        if(goal->frame_id == "base_link"){
            KDL::Frame target = original_pose * left_tool_offset_.Inverse();
            target.M.GetRPY(roll, pitch, yaw);
            std::array<double, 8> q_init = {ik_q_[0], ik_q_[1], ik_q_[2], ik_q_[3], ik_q_[4],yaw, pitch, roll};

            auto res = left_arm_ik::solve_ik(target.p, target.M, q_init);
            if (!res.success) {
                result.success = false;
                result.error_msg = "IK failed";
                move_arm_as_->setAborted(result);
                ROS_WARN("[MoveArm] IK failed pos_err=%.4f rot_err=%.4f", res.pos_err, res.rot_err);
                return;
            }
            ik_q_ = res.q8;
            target_joints = mapIkToJoints(res.q8);
        }
        else if(goal->frame_id == "l_wrist_r_Link"){
            KDL::Frame target = left_tcp_ * left_tool_offset_ * original_pose * left_tool_offset_.Inverse();
            std::array<double, 8> q_init = {ik_q_[0], ik_q_[1], ik_q_[2], ik_q_[3], ik_q_[4],yaw, pitch, roll};
            auto res = left_arm_ik::solve_ik(target.p, target.M, q_init);
            if (!res.success) {
                result.success = false;
                result.error_msg = "IK failed";
                move_arm_as_->setAborted(result);
                ROS_WARN("[MoveArm] IK failed pos_err=%.4f rot_err=%.4f", res.pos_err, res.rot_err);
                return;
            }
            ik_q_ = res.q8;
            target_joints = mapIkToJoints(res.q8);
        }
    }

    ROS_INFO("[MoveArm] IK success, driving joints");
    driveJoints(target_joints, feedback);
    ROS_INFO("[MoveArm] move succesful");

    if (move_arm_as_->isPreemptRequested() || !ros::ok()) {
        move_arm_as_->setPreempted();
        return;
    }

    result.success = true;
    result.error_msg = "";
    move_arm_as_->setSucceeded(result);
}

void ControllerNode::executeGripper(const common_comms::GripperCommandGoalConstPtr& goal)
{
    common_comms::GripperCommandResult result;
    common_comms::GripperCommandFeedback feedback;

    if (!running_) {
        result.success = false;
        result.grasped = false;
        result.error_msg = "";
        gripper_as_->setAborted(result);
        return;
    }

    int target;
    if (goal->command == "open") {
        target = gripper_cfg_.open;
    } 
    else if (goal->command == "close") {
        target = gripper_cfg_.close;
    } 
    else {
        result.success = false;
        result.grasped = false;
        result.error_msg = "";
        gripper_as_->setAborted(result);
        ROS_WARN("[Gripper] unknown command: %s", goal->command.c_str());
        return;
    }

    ROS_INFO("[Gripper] command=%s target=%d", goal->command.c_str(), target);
    {
        std::lock_guard<std::mutex> lock(ft_mutex_);
        jointFT_->EnableTorque(gripper_cfg_.id, 1);
        jointFT_->WritePosEx(gripper_cfg_.id, static_cast<s16>(target), 100, 100);
    }

    ros::Rate rate(20);
    while (ros::ok()) {
        if (gripper_as_->isPreemptRequested()) {
            gripper_as_->setPreempted();
            return;
        }
        int pos = gripper_position_;
        if (std::abs(pos - target) <= gripper_cfg_.dead_zone) {
            ROS_INFO("[Gripper] reached, pos=%d target=%d dz=%d",
                     pos, target, gripper_cfg_.dead_zone);
            break;
        }
        feedback.current_state = goal->command + "_ing";
        gripper_as_->publishFeedback(feedback);

        rate.sleep();
    }
    ROS_INFO("[Gripper] gripper command success!");
    result.success = true;
    result.grasped = true;
    result.error_msg = "";
    gripper_as_->setSucceeded(result);
}
