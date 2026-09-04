#include "DM2CNode.h"

DM2CNode::DM2CNode()
    : nh_(), as_(nh_, "dm2c_move",
                 std::bind(&DM2CNode::executeCB, this, std::placeholders::_1), false),
      leftController_(std::make_shared<DM2CController>(LiftColumn::LeftArm, "/dev/ttysWK0", 38400)),
      rightController_(std::make_shared<DM2CController>(LiftColumn::RightArm, "/dev/ttysWK0", 38400))
{
    as_.start();

    state_srv_ = nh_.advertiseService("dm2c_get_state",
                                       &DM2CNode::getStateCB, this);
    zero_srv_ = nh_.advertiseService("dm2c_set_zero", &DM2CNode::setZeroCB, this);
    stop_sub_ = nh_.subscribe("dm2c_stop", 1,
                               &DM2CNode::stopCB, this);

    ROS_INFO("DM2CNode ready: /dm2c_move, /dm2c_get_state, /dm2c_stop");
}



void DM2CNode::executeCB(const robot_controller::MoveGoalConstPtr &goal)
{
    ROS_INFO("Move goal: name=%s, mode=%u, target=%d, speed=%d, accel=%u, decel=%u",
             goal->name.c_str(), goal->mode, goal->target_position, goal->target_speed,
             goal->accel_time, goal->decel_time);

    if (goal->mode < 0 || goal->mode > 2) {
        ROS_ERROR("Invalid mode: %d", goal->mode);
        as_.setAborted(robot_controller::MoveResult(), "Invalid mode");
        return;
    }

    stop_requested_ = false;
    bool ret = false;
    if (goal->name == "leftarm" || goal->name == "left")
        ret = executeLeft(goal);
    else if(goal->name == "rightarm" || goal->name == "right")
        ret = executeRight(goal);
    else{
        as_.setAborted(robot_controller::MoveResult(), "Invalid name");
        return;
    }

    auto ctrl = getCtrl(goal->name);
    robot_controller::MoveResult result;

    result.final_current_position = ctrl->readActualPosition();
    result.final_speed = ctrl->readActualSpeed();
    result.final_status = ctrl->readStatus();
    result.final_error_code = ctrl->readAlarm();
    if(!ret){
        if(as_.isPreemptRequested() || stop_requested_){
            as_.setPreempted(result, "Goal preempted");
        }
        if(result.final_error_code){
            as_.setAborted(result, "motor error");
        }
        return;
    }
    as_.setSucceeded(result);

}

bool DM2CNode::executeLeft(const robot_controller::MoveGoalConstPtr &goal)
{
    if(goal->mode == 0)//绝对位置
        leftController_->moveToAbsolutePosition(
            goal->target_position, goal->target_speed, goal->accel_time, goal->decel_time);

    else if(goal->mode == 1) //相对位置
        leftController_->moveToRelativePosition(
            goal->target_position, goal->target_speed, goal->accel_time, goal->decel_time);

    else if(goal->mode == 2)  //速度模式
        leftController_->startVelocityMode(
            goal->target_speed, goal->accel_time, goal->decel_time);  

    return waitAndPublishFeedback(leftController_, goal);
}

bool DM2CNode::executeRight(const robot_controller::MoveGoalConstPtr &goal)
{
    if(goal->mode == 0)
        rightController_->moveToAbsolutePosition(
            goal->target_position, goal->target_speed, goal->accel_time, goal->decel_time);

    else if(goal->mode == 1) //相对位置
        rightController_->moveToRelativePosition(
            goal->target_position, goal->target_speed, goal->accel_time, goal->decel_time);  

    else if(goal->mode == 2)  //速度模式
        rightController_->startVelocityMode(
            goal->target_speed, goal->accel_time, goal->decel_time);  

    return waitAndPublishFeedback(rightController_, goal);
}

bool DM2CNode::waitAndPublishFeedback(std::shared_ptr<DM2CController> ctrl, const robot_controller::MoveGoalConstPtr &goal)
{
    ros::Rate rate(10);
    while (ros::ok())
    {
        uint16_t status = ctrl->readStatus();

        if (status & 48)
        {
            ROS_INFO("Move arrived");
            break;
        }

        if (stop_requested_)
        {
            return false;
        }

        if(!publishFeedback(ctrl)){
            return false;
        }
        rate.sleep();
    }
    return true;
}

bool DM2CNode::publishFeedback(std::shared_ptr<DM2CController> controller)
{
    robot_controller::MoveFeedback fb;
    fb.current_position = controller->readActualPosition();
    fb.current_speed = controller->readActualSpeed();
    fb.status = controller->readStatus();
    uint16_t err = controller->readAlarm();
    fb.error_code = err;
    as_.publishFeedback(fb);

    if (err)
    {
        controller->stopAllMotion();
        return false;
    }
    return true;
}

bool DM2CNode::getStateCB(robot_controller::GetState::Request &req, robot_controller::GetState::Response &res)
{
	ROS_INFO("DM2CNode::getStateCB");
    auto ctrl = getCtrl(req.name);
    res.current_position = ctrl->readActualPosition();
    res.current_speed = ctrl->readActualSpeed();
    res.status = ctrl->readStatus();
    res.error_code = ctrl->readAlarm();
    return true;
}

void DM2CNode::stopCB(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("Stop command: %s", msg->data.c_str());
    if (msg->data == "left" || msg->data == "leftArm")
        leftController_->stopAllMotion();
    else if (msg->data == "right" || msg->data == "rightArm")
        rightController_->stopAllMotion();
    else
    {
        leftController_->stopAllMotion();
        rightController_->stopAllMotion();
    }
    std::lock_guard<std::mutex> lock(stop_mutex_);
    stop_requested_ = true;
}

bool DM2CNode::setZeroCB(robot_controller::SetZero::Request &req, robot_controller::SetZero::Response &res)
{
    ROS_INFO("Set zero: %s", req.name.c_str());
    auto ctrl = getCtrl(req.name);
    if (!ctrl) return false;
    ctrl->setCurrentPositionAsZero();
    res.status = ctrl->readStatus();
    res.error_code = ctrl->readAlarm();
    return true;
}

std::shared_ptr<DM2CController> DM2CNode::getCtrl(const std::string& name){
    if(name == "leftarm" || name == "left")  return leftController_;
    if(name == "rightarm" || name == "right") return rightController_;
    return nullptr;
}

