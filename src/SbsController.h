#pragma once

#include<stdio.h>
#include <mc_control/mc_controller.h>
//#include <mc_tasks/PostureTask.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/RelativeEndEffectorTask.h>


#include "api.h"
#include "driver.h"

const double GRAVITY = 9.8;
const double HEIGHTREF=0.9;
const double A_LIM=0.3;


struct SbsController_DLLAPI SbsController : public mc_control::MCController
{
    SbsController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;

    void get_values();
    void set_CtrlPos();
    void state_swiching();
    void set_desiredVel();
    void set_desiredTask();
    void output_data();

    Eigen::Vector3d sat_func(double lim, const Eigen::Vector3d& val);

protected:
  void createGUI();

private:
    Falcon_Driver right_falcon, left_falcon;
    mc_rtc::Configuration config_;
    std::shared_ptr<mc_tasks::CoMTask> comTask;
    std::shared_ptr<mc_tasks::OrientationTask> otTask;
    std::shared_ptr<mc_tasks::RelativeEndEffectorTask> efTask_left, efTask_right;

    FILE * fp;
    bool first;
    double leftFootRatio;
    double omega;
    double ttime, timer_mode;
    int ctrl_mode, ctrl_mode2;
    std::chrono::_V2::system_clock::time_point z_start;
    Eigen::Matrix3d COMShifter_Kp, COMShifter_Kd;

    Eigen::Vector3d posRA_, posRB_;
    Eigen::Vector3d posRA, posRB, posRAp, posRBp, vel_posRA, vel_posRB;
    Eigen::Vector3d Q_ref, Q_ep, Q_epd, W_Q_W;

    Eigen::Vector3d W_p_AW, W_p_BW, W_p_GW, W_p_BW_,W_p_GW_, W_p_AW_ ;
    Eigen::Matrix3d R_0_mIMU;
    Eigen::Matrix3d W_R_A, W_R_B, W_R_H;
    Eigen::Vector3d A_p_BA_ref, B_p_AB_ref, W_p_GW_ref;
    Eigen::Vector3d W_v_GW, W_v_GWd, W_a_GW, W_a_GWd;

    Eigen::Vector3d A_f_A, B_f_B, A_n_A, B_n_B;

    sva::ForceVecd left, right, world_wrench;

    // Set 'true' to lift right foot
    bool rightFootLift_ = false;
};
