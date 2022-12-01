#include "SbsController.h"

SbsController::SbsController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : mc_control::MCController(rm, dt), right_falcon(0), left_falcon(1)
{
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addConstraintSet(selfCollisionConstraint);
  solver().addConstraintSet(*compoundJointConstraint);

  solver().addTask(postureTask);

  std::vector<std::string> activeJoints = {"RCY", "RCR", "RCP", "RKP", "RAP", "RAR", "LCY", "LCR", "LCP", "LKP", "LAP", "LAR"};
  //comTask.reset(new mc_tasks::CoMTask(robots(), robots().robotIndex()));
  //comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0, 100.0, 1000.0);

  //solver().addTask(comTask);

  solver().setContacts({{}});

  Eigen::Vector6d dof = Eigen::Vector6d::Ones();
  dof(0) = 0.0;
  dof(1) = 0.0;
  dof(5) = 0.0;

  // Eigen::Vector6d dof = Eigen::Vector6d::Zero();
  // dof(2) = 1.0;

  addContact({robot().name(), "ground", "LeftFoot", "AllGround", 0.7, dof});
  addContact({robot().name(), "ground", "RightFoot", "AllGround", 0.7, dof});

  otTask = std::make_shared<mc_tasks::OrientationTask>("Body", robots(), 0, 100.0, 1.0);

  otTask->dimWeight(Eigen::MatrixXd::Constant(3, 1, 1000.0));
  solver().addTask(otTask);

  otTask->orientation(Eigen::Matrix3d::Identity());

  postureTask->stiffness(100.0);
  postureTask->weight(1.0);

  std::vector<tasks::qp::JointStiffness> stiffnesses;

  for (int i = 0; i < 12; i++)
  {
    stiffnesses.push_back({activeJoints[i], 1.0});
  }
  postureTask->jointStiffness(solver(), stiffnesses);

  Eigen::VectorXd ww(59);
  ww.head(18) = Eigen::MatrixXd::Constant(18, 1, 1.0);
  ww.tail(41) = Eigen::MatrixXd::Constant(41, 1, 1000.0);

  postureTask->dimWeight(ww);

  efTask_left = std::make_shared<mc_tasks::RelativeEndEffectorTask>("Lleg_Link5", robots(), 0, "Rleg_Link5", 50.0, 1.0);
  efTask_right = std::make_shared<mc_tasks::RelativeEndEffectorTask>("Rleg_Link5", robots(), 0, "Lleg_Link5", 50.0, 1.0);

  // efTask_left->positionTask->stiffness(10.0);
  // efTask_left->orientationTask->stiffness(10.0);
  efTask_left->positionTask->dimWeight(Eigen::MatrixXd::Constant(3, 1, 1000.0));
  efTask_left->orientationTask->dimWeight(Eigen::MatrixXd::Constant(3, 1, 1000.0));

  // efTask_right->positionTask->stiffness(10.0);
  // efTask_right->orientationTask->stiffness(10.0);
  efTask_right->positionTask->dimWeight(Eigen::MatrixXd::Constant(3, 1, 1000.0));
  efTask_right->orientationTask->dimWeight(Eigen::MatrixXd::Constant(3, 1, 1000.0));

  auto stabiConf = robot().module().defaultLIPMStabilizerConfiguration();
  stabiConf.comHeight = 0.9;
  stabiConf.torsoPitch = 0;
  stabiConf.copAdmittance = Eigen::Vector2d{0.005, 0.005};
  stabiConf.zmpcc.comAdmittance = Eigen::Vector2d{0.0, 0.0};
  stabiConf.dcmPropGain = 2.0; //4.0;
  stabiConf.dcmIntegralGain = 10;
  stabiConf.dcmDerivGain = 0.5;
  stabiConf.dcmDerivatorTimeConstant = 5;
  stabiConf.dcmIntegratorTimeConstant = 5;
  


  lipmTask = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(
          solver().robots(),
          solver().realRobots(),
          0,
          stabiConf.leftFootSurface,
          stabiConf.rightFootSurface,
          stabiConf.torsoBodyName,
          solver().dt()); 

  lipmTask->configure(stabiConf);

  solver().addTask(lipmTask);
  
  

  ttime = 0;

  ctrl_mode = 0;
  ctrl_mode2 = 0;
  W_v_GW_ref = Eigen::Vector3d::Zero();
  W_v_GWd = Eigen::Vector3d::Zero();
  Q_epd = Eigen::Vector3d::Zero();
  omega = sqrt(GRAVITY / HEIGHTREF);
  COMShifter_Kp = Eigen::Matrix3d::Zero();
  COMShifter_Kd = Eigen::Matrix3d::Zero();

  for (int i = 0; i < 3; i++)
  {
    COMShifter_Kp(i, i) = 30.0;

    COMShifter_Kd(i, i) = COMShifter_Kp(i, i) / omega + omega;
  }

  first = true;
  // fp = fopen("/home/zyc/data.csv", "w");

  leftFootRatio = 0.5;
  datastore().make_call("KinematicAnchorFrame::" + robot().name(),
                        [this](const mc_rbdyn::Robot &robot)
                        {
                          return sva::interpolate(robot.surfacePose("RightFoot"),
                                                  robot.surfacePose("LeftFoot"),
                                                  leftFootRatio);
                        });

  //createGUI();

  logger().addLogEntries(
      this,
      "leftFootRatio", [this]()
      { return leftFootRatio; },
      "left_foot_center", [this]()
      { return W_p_AW; },
      "right_foot_center", [this]()
      { return W_p_BW; },
      "left_foot_a", [this]()
      { return W_p_AW_; },
      "right_foot_a", [this]()
      { return W_p_BW_; },
      "ref_COM", [this]()
      { return W_p_GW_ref; },
      "real_COM_p", [this]()
      { return W_p_GW; },
      "real_COM_v", [this]()
      { return W_v_GW; },
      "real_COM_a", [this]()
      { return W_a_GW; },
      "ZMP_total", [this]()
      { return W_Q_W; },
      "ZMP_L", [this]()
      { return W_Q_A; },
      "ZMP_R", [this]()
      { return W_Q_B; },
      "EZMP", [this]()
      { return Q_epd; },
      "real_EZMP", [this]()
      { return Q_ep; });

  mc_rtc::log::success("SbsController init done ");
}

bool SbsController::run()
{

  if (first)
  {
    z_start = std::chrono::high_resolution_clock::now();
  }
  // ttime = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - z_start).count();

  get_values();
  set_CtrlPos();
  state_swiching();
  set_desiredVel();
  set_desiredTask();
  // output_data();

  if (first)
    first = false;

  return mc_control::MCController::run();
}

void SbsController::reset(const mc_control::ControllerResetData &reset_data)
{
  mc_control::MCController::reset(reset_data);

  //comTask->reset();
  otTask->reset();
  lipmTask->reset();
}

void SbsController::get_values()
{
  sva::PTransformd ZMP_frame(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  std::vector<std::string> activeJoints = {"LCY", "LCR", "LCP", "LKP", "LAP", "LAR", "RCY", "RCR", "RCP", "RKP", "RAP", "RAR"};
  std::vector<std::string> sensornames = {"LeftFootForceSensor", "RightFootForceSensor"};

  // left = realRobot().surfaceWrench("LeftFootCenter");
  // right = realRobot().surfaceWrench("RightFootCenter");
  left = robot().forceSensor("LeftFootForceSensor").worldWrench(realRobot());
  right = robot().forceSensor("RightFootForceSensor").worldWrench(realRobot());

  W_f_A = left.force();
  W_n_A = left.moment();

  W_f_B = right.force();
  W_n_B = right.moment();

  world_wrench = realRobot().netWrench(sensornames);

  if (world_wrench.force()[2] > 1.0)
    W_Q_W = realRobot().zmp(world_wrench, ZMP_frame);
  // std::vector<std::vector<double> > zyc;
  // zyc=robot().q();

  left = robot().surfaceWrench("LeftFootCenter");
  right = robot().surfaceWrench("RightFootCenter");

  A_f_A = left.force();
  A_n_A = left.moment();

  B_f_B = right.force();
  B_n_B = right.moment();

  W_R_A = realRobot().surfacePose("LeftFootCenter").rotation();
  W_p_AW = realRobot().surfacePose("LeftFootCenter").translation();

  W_p_AW_ = realRobot().surfacePose("LeftFoot").translation();

  W_R_B = realRobot().surfacePose("RightFootCenter").rotation();
  W_p_BW = realRobot().surfacePose("RightFootCenter").translation();

  W_p_BW_= realRobot().surfacePose("RightFoot").translation();

  W_p_GW = realRobot().com();
  // W_v_GW = robot().comVelocity();
  // W_a_GW = robot().comAcceleration();

  if (first)
    W_p_GW_p = W_p_GW;

  W_v_GW = (W_p_GW - W_p_GW_p) / timeStep;

  if (first)
    W_v_GW_p = W_v_GW;

  W_a_GW = (W_v_GW - W_v_GW_p) / timeStep;

  W_p_GW_p = W_p_GW;
  W_v_GW_p = W_v_GW;

  // W_v_GW_ = realRobot().comVelocity();

  // W_p_GW_= realRobot().com();
  // W_p_GW_(2) = 0.0;

  if (W_f_A(2) > 1e-1)
    W_Q_A << -W_n_A(1) / W_f_A(2), W_n_A(0) / W_f_A(2), .0;
  else
    W_Q_A = Eigen::Vector3d::Zero();

  if (W_f_B(2) > 1e-1)
    W_Q_B << -W_n_B(1) / W_f_B(2), W_n_B(0) / W_f_B(2), .0;
  else
    W_Q_B = Eigen::Vector3d::Zero();

  W_Q = Eigen::Vector3d::Zero();

  if (W_f_A(2) > 1e-1)
  {
    if (W_f_B(2) > 1e-1)
    {
      W_Q(0) = (W_f_A(2) * W_Q_A(0) + W_f_B(2) * W_Q_B(0)) / (W_f_A(2) + W_f_B(2));
      W_Q(1) = (W_f_A(2) * W_Q_A(1) + W_f_B(2) * W_Q_B(1)) / (W_f_A(2) + W_f_B(2));
    }
    else
    {
      W_Q(0) = (W_f_A(2) * W_Q_A(0)) / (W_f_A(2));
      W_Q(1) = (W_f_A(2) * W_Q_A(1)) / (W_f_A(2));
    }
  }
  else if (W_f_B(2) > 1e-1)
  {
    W_Q(0) = (W_f_B(2) * W_Q_B(0)) / (W_f_B(2));
    W_Q(1) = (W_f_B(2) * W_Q_B(1)) / (W_f_B(2));
  }

  W_R_H = robot().bodyPosW("Body").rotation();

  R_0_mIMU = robot().bodySensor("Accelerometer").orientation().toRotationMatrix();

  Q_ep = W_p_GW - W_a_GW / (omega * omega);
  Q_ep(2) = Q_ep(2) - HEIGHTREF;
}

void SbsController::set_CtrlPos()
{

  posRB_ = right_falcon.Get_Pos();
  posRA_ = left_falcon.Get_Pos();

  posRB << -(posRB_(2) - 0.12), -posRB_(0), posRB_(1);
  posRA << -(posRA_(2) - 0.12), -posRA_(0), posRA_(1);


  if (rightFootLift_)
  {
    posRB << .0, .0, .01;
  }

  if (first)
  {
    posRAp = posRA;
    posRBp = posRB;
  }

  vel_posRA = (posRA - posRAp) / timeStep;
  vel_posRB = (posRB - posRBp) / timeStep;

  posRAp = posRA;
  posRBp = posRB;
}

void SbsController::state_swiching()
{
  if (ctrl_mode == 1)
  {
    Eigen::Vector3d A_p_QA;
    A_p_QA = W_R_A.transpose() * (Q_epd - W_p_AW);
    ctrl_mode2 = 0;

    if (fabs(A_p_QA(0)) < 0.01 && fabs(A_p_QA(1)) < 0.01)//0.04
    {
      ctrl_mode = 2;
      timer_mode = 0.0;
      
      removeContact({robot().name(), "ground", "RightFoot", "AllGround"});
      solver().addTask(efTask_right);
      lipmTask->setContacts({mc_tasks::lipm_stabilizer::ContactState::Left});
    }
  }
  else if (ctrl_mode == 2)
  {
    ctrl_mode2 = 0;
    timer_mode += timeStep;

    if ((timer_mode > 1.0) && B_f_B(2) > 5.0)
    {
      ctrl_mode = 0;
      solver().removeTask(efTask_right);
      addContact({robot().name(), "ground", "RightFoot", "AllGround"});
      lipmTask->setContacts({mc_tasks::lipm_stabilizer::ContactState::Left, mc_tasks::lipm_stabilizer::ContactState::Right});
      Q_ref = (W_p_AW + W_p_BW) / 2.0;
      Q_ref(2) += HEIGHTREF;
    }
  }
  else if (ctrl_mode == 5)
  {
    Eigen::Vector3d B_p_QB;
    B_p_QB = W_R_B.transpose() * (Q_epd - W_p_BW);
    ctrl_mode2 = 1;
    if (fabs(B_p_QB(0)) < 0.01 && fabs(B_p_QB(1)) < 0.01)
    {
      ctrl_mode = 6;
      timer_mode = 0.0;

      removeContact({robot().name(), "ground", "LeftFoot", "AllGround"});
      solver().addTask(efTask_left);
      lipmTask->setContacts({mc_tasks::lipm_stabilizer::ContactState::Right});
    }
  }
  else if (ctrl_mode == 6)
  {
    ctrl_mode2 = 1;
    timer_mode += timeStep;
    if ((timer_mode > 1.0) && A_f_A(2) > 5.0)
    {
      ctrl_mode = 0;
      solver().removeTask(efTask_left);
      addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
      lipmTask->setContacts({mc_tasks::lipm_stabilizer::ContactState::Left, mc_tasks::lipm_stabilizer::ContactState::Right});
      Q_ref = (W_p_AW + W_p_BW) / 2.0;
      Q_ref(2) += HEIGHTREF;
    }
  }
  else if ((ctrl_mode == 0 && vel_posRB(2) > 0.05 && posRB(2) > 0.0))
  {
    ctrl_mode2 = 0;

    ctrl_mode = 1;
    Q_ref = W_p_AW;
    Q_ref(2) += HEIGHTREF;
  }
  else if ((ctrl_mode == 0 && vel_posRA(2) > 0.05 && posRA(2) > 0.0))
  {
    ctrl_mode2 = 1;

    ctrl_mode = 5;
    Q_ref = W_p_BW;
    Q_ref(2) += HEIGHTREF;
  }
}

void SbsController::set_desiredVel()
{
  Eigen::Vector3d jerk;

  if (first)
  {
    Q_ref = (W_p_AW + W_p_BW) / 2.0;
    Q_ref(2) += HEIGHTREF;
    W_p_GW_ref = W_p_GW;
    W_p_GWd = W_p_GW_ref;
    W_a_GWdp = Eigen::Vector3d::Zero();
  }

  W_a_GW_ref = sat_func(A_LIM, COMShifter_Kp * (Q_ref - W_p_GW_ref) - COMShifter_Kd * W_v_GW_ref);
  jerk = sat_func(10.0, (W_a_GW_ref - W_a_GWdp)/timeStep);
  W_a_GW_ref = W_a_GWdp + jerk * timeStep;
  W_a_GWdp = W_a_GW_ref;

  W_v_GW_ref += W_a_GW_ref * timeStep;
  W_p_GW_ref += W_v_GW_ref * timeStep;

  // double zyc_kp = 10.0;

  // W_a_GWd = zyc_kp * (W_p_GW_ref - W_p_GW) + 2* 0.6 * sqrt(zyc_kp) * (W_v_GW_ref - W_v_GW);
  // W_v_GWd += W_a_GWd * timeStep;
  // W_p_GWd += W_v_GWd * timeStep;

  Q_epd = W_p_GW_ref - W_a_GW_ref/ (omega * omega);
  Q_epd(2) = Q_epd(2) - HEIGHTREF;

  if ((ctrl_mode == 0 || ctrl_mode == 1 || ctrl_mode == 5))
  {
  }
  else if (ctrl_mode2 == 0)
  {
    A_p_BA_ref(0) = posRB(0) * 10.0;
    A_p_BA_ref(1) = -0.21 + posRB(1) * 5.0;
    A_p_BA_ref(2) = posRB(2) * 8.0;


  }
  else if (ctrl_mode2 == 1)
  {
    B_p_AB_ref(0) = posRA(0) * 10.0;
    B_p_AB_ref(1) = 0.21 + posRA(1) * 5.0;
    B_p_AB_ref(2) = posRA(2) * 8.0;
  }
}

void SbsController::set_desiredTask()
{
  std::vector<std::string> activeJoints = {"LCY", "LCR", "LCP", "LKP", "LAP", "LAR", "RCY", "RCR", "RCP", "RKP", "RAP", "RAR"};

  // tra_gen.Cal_Func(ttime);

  // W_a_GWd(1) = tra_gen.s[2];
  // W_v_GWd(1) = tra_gen.s[1];
  // W_p_GW_ref(1) = tra_gen.s[0];

  // comTask->refAccel(W_a_GW_ref);
  // comTask->refVel(W_v_GW_ref);
  // comTask->com(W_p_GW_ref);
  leftFootRatio = lipmTask->leftFootRatio();
  lipmTask->target(W_p_GW_ref, W_v_GW_ref, W_a_GW_ref,Q_epd);

  otTask->orientation(Eigen::Matrix3d::Identity());

  int Joint_Index;
  for (int i = 0; i < 12; i++)
  {
    Joint_Index = robot().jointIndexByName(activeJoints[i]);
    postureTask->target({{activeJoints[i], robot().q()[Joint_Index]}});
  }
  // postureTask->target({{"LSP", {-6.2}}});

  if ((ctrl_mode == 0 || ctrl_mode == 1 || ctrl_mode == 5))
  {
    // otTask->orientation(Eigen::Matrix3d::Identity());
  }
  else if (ctrl_mode2 == 0)
  {
    efTask_right->set_ef_pose(sva::PTransformd(A_p_BA_ref));

    // otTask->orientation(W_R_H);
  }
  else if (ctrl_mode2 == 1)
  {
    efTask_left->set_ef_pose(sva::PTransformd(B_p_AB_ref));
    // otTask->orientation(W_R_H);
  }
}

void SbsController::output_data()
{
  fprintf(fp, "%.3lf,", ttime);

  fprintf(fp, ",%d,%d,", ctrl_mode, ctrl_mode2);

  for (int i = 0; i < 3; i++)
    fprintf(fp, ",%.6lf", Q_ref(i));

  fprintf(fp, ",");
  for (int i = 0; i < 3; i++)
    fprintf(fp, ",%.6lf", Q_epd(i));

  fprintf(fp, ",");
  for (int i = 0; i < 3; i++)
    fprintf(fp, ",%.6lf", Q_ep(i));

  fprintf(fp, ",");
  for (int i = 0; i < 3; i++)
    fprintf(fp, ",%.6lf", W_Q_W(i));

  fprintf(fp, ",");
  for (int i = 0; i < 3; i++)
    fprintf(fp, ",%.6lf", W_p_GW_ref(i));

  fprintf(fp, ",");
  for (int i = 0; i < 3; i++)
    fprintf(fp, ",%.6lf", W_p_GW(i));

  // fprintf(fp, ",");
  // for (int i = 0; i < 3; i++)
  //   fprintf(fp, ",%.6lf", W_v_GWd(i));

  // fprintf(fp, ",");
  // for (int i = 0; i < 3; i++)
  //   fprintf(fp, ",%.6lf", W_v_GW(i));

  // fprintf(fp, ",");
  // for (int i = 0; i < 3; i++)
  //   fprintf(fp, ",%.6lf", W_a_GWd(i));

  // fprintf(fp, ",");
  // for (int i = 0; i < 3; i++)
  //   fprintf(fp, ",%.6lf", W_a_GW(i));

  // for (int i = 0; i < 3; i++)
  // {
  //   for(int j = 0; j < 3; j++)
  //   {
  //     fprintf(fp, ",%.6lf", A_R_B(i,j));
  //   }
  //   fprintf(fp, "\n");
  // }
  fprintf(fp, "\n");
}

Eigen::Vector3d SbsController::sat_func(double _lim, const Eigen::Vector3d &val)
{
  double lim = fabs(_lim);
  Eigen::Vector3d result;

  for (int i = 0; i < 3; i++)
  {
    if (val(i) > lim)
      result(i) = lim;
    else if (val(i) < -lim)
      result(i) = -lim;
    else
      result(i) = val(i);
  }

  return result;
}

void SbsController::createGUI()
{
  // auto & gui = *ctl.gui();
  gui()->addElement({"SbsController", "Task"}, mc_rtc::gui::Label("Lift right foot", [this]()
                                                                  { return rightFootLift_; }),
                    mc_rtc::gui::Checkbox(
                        "Activated", [this]()
                        { return rightFootLift_; },
                        [this]()
                        { rightFootLift_ = !rightFootLift_; }));
}

CONTROLLER_CONSTRUCTOR("SbsController", SbsController)
