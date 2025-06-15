#pragma once

#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

namespace mc_tasks
{
    struct MC_TASKS_DLLAPI EndEffectorTask_NoGUI : public EndEffectorTask
    {
    public:
        EndEffectorTask_NoGUI(const std::string &bodyName,
                              const mc_rbdyn::Robots &robots,
                              unsigned int robotIndex,
                              double stiffness = 10.0,
                              double weight = 1000.0);

    protected:
        void addToGUI(mc_rtc::gui::StateBuilder &gui) override;
    };

    namespace lipm_stabilizer
    {

        struct MC_TASKS_DLLAPI StabilizerTask_Zyc : public StabilizerTask
        {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            /**
             * @brief Creates a stabilizer meta task
             *
             * @param robots Robots on which the task acts
             * @param realRobots Corresponding real robot instances
             * @param robotIndex Index of the robot to stabilize
             * @param leftSurface Left foot surface name. Its origin should be the center of the foot sole
             * @param rightSurface Left foot surface name. Its origin should be the center of the foot sole
             * @param torsoBodyName Body name of the robot's torso (i.e a link above the
             * floating base)
             * @param dt Controller's timestep
             */
            StabilizerTask_Zyc(const mc_rbdyn::Robots &robots,
                               const mc_rbdyn::Robots &realRobots,
                               unsigned int robotIndex,
                               const std::string &leftSurface,
                               const std::string &rightSurface,
                               const std::string &torsoBodyName,
                               double dt);
            /** Update QP task targets.
             *
             * This function is called once the reference has been updated.
             */
            void run();

        private:
            /**
             * @brief Add contact to the task (set tasks target, update support area).
             * This function does not immediately add contact tasks to the solver, this
             * will be done by update() when the task is added to the solver.
             */
            void addContact(ContactState contactState, const internal::Contact &contact);

            /** Check whether the robot is in the air. */
            void checkInTheAir();

            /** Computes the ratio of force distribution between the feet based on
             * the reference ZMP and contact ankle positions.
             */
            void computeLeftFootRatio();

            /** Update real-robot state.
             *
             * \param com Position of the center of mass.
             *
             * \param comd Velocity of the center of mass.
             *
             * \param comdd Acceleration of the center of mass.
             */
            void updateState(const Eigen::Vector3d &com, const Eigen::Vector3d &comd, const Eigen::Vector3d &comdd);

            /**
             * @brief Update contact tasks in the solver
             *
             * @param solver QPSolver holding the tasks
             */
            void updateContacts(mc_solver::QPSolver &solver);

            /** Compute desired wrench based on DCM error. */
            sva::ForceVecd computeDesiredWrench();

            /** Distribute a desired wrench in double support.
             *
             * \param desiredWrench Desired resultant reaction wrench.
             */
            void distributeWrench(const sva::ForceVecd &desiredWrench);

            /**
             * @brief Generate a CoP reference for each contact under the future zmp refence along a horizon.
             * The dynamic of the contact CoP is expected to follow a 1st order dynamic w.r.t the CoP reference using prameter
             * lambda_CoP
             *
             * The desired vertical forces are computed using the ratio (p_left - zmp_ref) / (p_left - p_right).
             * This choice limits the torque at each contact ankle
             *
             * It is advised to provide the future support foot name when using this method using supportFoot method
             *
             * @param zmp_ref  each zmp reference piecewise constant over delta vector lenght in the world frame
             * @param delta horizon timestep
             */
            void distributeCoPonHorizon(const std::vector<Eigen::Vector2d> &zmp_ref, double delta);

            void computeCoPonHorizon(const std::vector<Eigen::Vector2d> &zmp_ref, const double delta, const double t_delay);

            /** Project desired wrench to single support foot.
             *
             * \param desiredWrench Desired resultant reaction wrench.
             *
             * \param footTask Target foot.
             *
             * \param target contact
             */
            void saturateWrench(const sva::ForceVecd &desiredWrench,
                                std::shared_ptr<mc_tasks::force::CoPTask> &footTask,
                                const internal::Contact &contact);

            /** Reset admittance, damping and stiffness for every foot in contact. */
            void setSupportFootGains();

            /** Update CoM task with ZMP Compensation Control.
             *
             * This approach is based on Section 6.2.2 of Dr Nagasaka's PhD thesis
             * "体幹位置コンプライアンス制御によるモデル誤差吸収" (1999) from
             * <https://sites.google.com/site/humanoidchannel/home/publication>.
             * The main differences is that the CoM offset is (1) implemented as CoM
             * damping control with an internal leaky integrator and (2) computed from
             * the distributed rather than reference ZMP.
             *
             */
            void updateCoMTaskZMPCC();

            /** Apply foot force difference control.
             *
             * This method is described in Section III.E of "Biped walking
             * stabilization based on linear inverted pendulum tracking" (Kajita et
             * al., IROS 2010).
             */
            void updateFootForceDifferenceControl();

            /** Update ZMP frame from contact state. */
            void updateZMPFrame();

            /** Get 6D contact admittance vector from 2D CoP admittance. */
            inline sva::ForceVecd contactAdmittance() const noexcept
            {
                return {{c_.copAdmittance.y(), c_.copAdmittance.x(), 0.}, {0., 0., 0.}};
            }

            inline void zmpcc(const ZMPCCConfiguration &zmpccConfig) noexcept
            {
                c_.zmpcc = zmpccConfig;
                zmpcc_.configure(zmpccConfig);
            }

            /** @brief Compute the CoM offset (\alpha) and the ZMP coefficient (\kappa) and the sum wrench from the external
             * wrenches. see Murooka et al. RAL 2021 eq (8)
             *
             *  @tparam TargetOrMeasured Change depending on the used wrenches
             *  @param robot [in] - Robot used to transform surface wrenches (control robot or real robot)
             *  @param offset_gamma [out] - Com offset
             *  @param coef_alpha [out] - ZmP coefficient
             */
            template <sva::ForceVecd ExternalWrench::*TargetOrMeasured>
            void computeWrenchOffsetAndCoefficient(const mc_rbdyn::Robot &robot,
                                                   Eigen::Vector3d &offset_gamma,
                                                   double &coef_kappa) const;

            /** @brief Compute the sum of external wrenches.
             *
             *  @tparam TargetOrMeasured Change depending on the used wrenches
             *  @param robot Robot used to transform surface wrenches (control robot or real robot)
             *  @param com Robot CoM
             */
            template <sva::ForceVecd ExternalWrench::*TargetOrMeasured>
            sva::ForceVecd computeExternalWrenchSum(const mc_rbdyn::Robot &robot, const Eigen::Vector3d &com) const;

            /** @brief Compute the position, force, and moment of the external contacts in the world frame.
             *
             *  @param [in] robot Robot (control robot or real robot)
             *  @param [in] surfaceName Surface name
             *  @param [in] surfaceWrench Surface wrench
             *  @param [out] pos Position of the external contact in the world frame
             *  @param [out] force Force of the external contact in the world frame
             *  @param [out] moment Moment of the external contact in the world frame
             */
            void computeExternalContact(const mc_rbdyn::Robot &robot,
                                        const std::string &surfaceName,
                                        const sva::ForceVecd &surfaceWrench,
                                        Eigen::Vector3d &pos,
                                        Eigen::Vector3d &force,
                                        Eigen::Vector3d &moment) const;

        protected:
            void update(mc_solver::QPSolver &) override;
        };

    }
}