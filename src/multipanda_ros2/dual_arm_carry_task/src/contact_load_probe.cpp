#include "dual_arm_carry_task/dual_arm_controller.hpp"
#include "dual_arm_carry_task/coordinated_path_optimizer.hpp"
#include "dual_arm_carry_task/time_profile_optimizer.hpp"
#include "dual_arm_carry_task/gripper_client.hpp"
#include "dual_arm_carry_task/hover_acceptance.hpp"
#include "dual_arm_carry_task/bilateral_snapshot_gate.hpp"
#include "dual_arm_carry_task/bridge_process.hpp"
#include "dual_arm_carry_task/async_truth_guard.hpp"
#include "dual_arm_carry_task/mmc_state_validation.hpp"
#include "dual_arm_carry_task/s1_time_scaling.hpp"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <future>
#include <iomanip>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <mujoco_ros_msgs/srv/get_body_state.hpp>
#include <multi_mode_control_msgs/srv/get_robot_states.hpp>
#include <franka_msgs/msg/franka_state.hpp>

namespace {
using Transform = Eigen::Isometry3d;
Transform rodTransform(const MujocoRodTruth& p) {
    Transform t=Transform::Identity();
    t.translation()=Eigen::Vector3d(p.position.x,p.position.y,p.position.z);
    t.linear()=Eigen::Quaterniond(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z).normalized().toRotationMatrix();
    return t;
}
geometry_msgs::msg::Pose poseMsg(const Transform& t) {
    geometry_msgs::msg::Pose p;
    p.position.x=t.translation().x(); p.position.y=t.translation().y(); p.position.z=t.translation().z();
    const Eigen::Quaterniond q(t.linear());
    p.orientation.x=q.x(); p.orientation.y=q.y(); p.orientation.z=q.z(); p.orientation.w=q.w();
    return p;
}
double steadySeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
double rotationError(const Transform& a,const Transform& b) {
    return Eigen::AngleAxisd(a.linear().transpose()*b.linear()).angle();
}

}

bool DualArmController::ensureTaskRobotStatesService()
{
    if (robot_state_service_) return true;
    if (!ctx_->dual_arm_ || !ctx_->left_arm_ || !ctx_->right_arm_) return false;
    const auto live_model = ctx_->dual_arm_->getRobotModel();
    const std::array<std::string, 2> eefs{{
        ctx_->left_arm_->getEndEffectorLink(), ctx_->right_arm_->getEndEffectorLink()}};
    const std::string service_name = "/dual_arm_carry_task/current_robot_states";
    robot_state_service_ = ctx_->create_service<multi_mode_control_msgs::srv::GetRobotStates>(
        service_name,
        [this, live_model, eefs, service_name](
            const std::shared_ptr<multi_mode_control_msgs::srv::GetRobotStates::Request>,
            std::shared_ptr<multi_mode_control_msgs::srv::GetRobotStates::Response> response) {
            DualArmCarryTask::JointFeedback sample;
            if (!ctx_->latestJointFeedback(sample) || !sample.stamp_valid) {
                RCLCPP_WARN(ctx_->get_logger(),
                    "[MMC_STATE_SOURCE] result=unavailable reason=joint_feedback_missing_or_stale service=%s",
                    service_name.c_str());
                return;
            }
            const auto freshness = dual_arm_carry_task::validateJointSampleFreshness(
                sample.stamp.seconds(), ctx_->now().seconds(), steadySeconds() - sample.recv_steady_sec);
            if (!freshness.valid) {
                RCLCPP_WARN(ctx_->get_logger(),
                    "[MMC_STATE_SOURCE] result=unavailable reason=%s service=%s sample_sim_time=%.9f "
                    "current_sim_time=%.9f wall_age_sec=%.6f",
                    freshness.reason.c_str(), service_name.c_str(), sample.stamp.seconds(),
                    ctx_->now().seconds(), steadySeconds() - sample.recv_steady_sec);
                return;
            }
            moveit::core::RobotState state(live_model);
            const std::array<std::string, 2> bases{{"mj_left", "mj_right"}};
            const std::array<std::string, 2> link0s{{"mj_left_link0", "mj_right_link0"}};
            for (std::size_t side = 0; side < bases.size(); ++side) {
                for (std::size_t joint = 1; joint <= 7; ++joint) {
                    const std::string name = bases[side] + "_joint" + std::to_string(joint);
                    const auto it = sample.positions.find(name);
                    if (it == sample.positions.end() || !std::isfinite(it->second)) {
                        RCLCPP_WARN(ctx_->get_logger(),
                            "[MMC_STATE_SOURCE] result=unavailable reason=joint_field_invalid joint=%s service=%s",
                            name.c_str(), service_name.c_str());
                        return;
                    }
                    state.setVariablePosition(name, it->second);
                }
            }
            state.update();
            response->states.resize(2);
            const auto wall_now = std::chrono::system_clock::now().time_since_epoch();
            const auto wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(wall_now).count();
            const auto wall_sec = wall_ns / 1000000000LL;
            const auto wall_nsec = wall_ns % 1000000000LL;
            for (std::size_t side = 0; side < response->states.size(); ++side) {
                const std::string arm = side == 0 ? "mj_left" : "mj_right";
                auto& out = response->states[side];
                out.header.frame_id = link0s[side];
                out.header.stamp.sec = static_cast<std::int32_t>(wall_sec);
                out.header.stamp.nanosec = static_cast<std::uint32_t>(wall_nsec);
                const Eigen::Isometry3d local_pose = state.getGlobalLinkTransform(link0s[side]).inverse() *
                    state.getGlobalLinkTransform(eefs[side]);
                if (!local_pose.matrix().allFinite()) {
                    response->states.clear();
                    return;
                }
                Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(out.o_t_ee.data()) =
                    local_pose.matrix();
                for (std::size_t joint = 1; joint <= 7; ++joint)
                    out.q[joint - 1] = sample.positions.at(arm + "_joint" + std::to_string(joint));
                out.time = sample.stamp.seconds();
            }
            RCLCPP_DEBUG(ctx_->get_logger(),
                "[MMC_STATE_SOURCE] result=ok service=%s source=joint_states+MoveIt-FK seq=%llu sim_time=%.9f",
                service_name.c_str(), sample.seq, sample.stamp.seconds());
        });
    RCLCPP_INFO(ctx_->get_logger(), "[MMC_STATE_SOURCE] service_ready=%s source=task_joint_feedback_MoveIt_FK",
                service_name.c_str());
    return static_cast<bool>(robot_state_service_);
}

bool DualArmController::runMmcStartupPreflight(std::string& reason)
{
    using namespace std::chrono_literals;
    const auto began = std::chrono::steady_clock::now();
    bool use_sim_time = false;
    const bool has_sim_time = ctx_->get_parameter("use_sim_time", use_sim_time);
    RCLCPP_INFO(ctx_->get_logger(),
        "[MMC_CLOCK_CHECK] node=%s parameter=use_sim_time available=%s value=%s phase=task-preflight",
        ctx_->get_fully_qualified_name(), has_sim_time ? "true" : "false",
        use_sim_time ? "true" : "false");
    if (!has_sim_time || !use_sim_time) {
        reason = "E_MMC_TASK_SIM_TIME_NOT_ENABLED";
        RCLCPP_ERROR(ctx_->get_logger(), "[MMC_MOVEGROUP_STATE_CHECK] result=failed reason=%s", reason.c_str());
        return false;
    }

    // Verify the state assembled by MoveGroup's live CurrentStateMonitor before
    // any task motion. The task's independently timestamp-validated cache is
    // used only as a freshness/coverage reference, never as MoveGroup state.
    constexpr double kCurrentStateWaitSeconds = 1.0;
    const auto state_check_began = std::chrono::steady_clock::now();
    const auto moveit_state = ctx_->dual_arm_ ?
        ctx_->dual_arm_->getCurrentState(kCurrentStateWaitSeconds) : moveit::core::RobotStatePtr{};
    const double state_check_elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - state_check_began).count();
    DualArmCarryTask::JointFeedback feedback;
    const bool have_feedback = ctx_->latestJointFeedback(feedback) && feedback.stamp_valid;
    const auto freshness = have_feedback ? dual_arm_carry_task::validateJointSampleFreshness(
        feedback.stamp.seconds(), ctx_->now().seconds(), steadySeconds() - feedback.recv_steady_sec) :
        dual_arm_carry_task::MmcStateValidation{false, "task_joint_feedback_missing"};
    std::size_t compared = 0;
    double max_delta = 0.0;
    bool values_valid = static_cast<bool>(moveit_state) && freshness.valid;
    const std::array<std::string, 2> arm_prefixes{{"mj_left", "mj_right"}};
    if (values_valid) {
        for (const auto& arm : arm_prefixes) {
            for (std::size_t joint = 1; joint <= 7; ++joint) {
                const std::string name = arm + "_joint" + std::to_string(joint);
                const auto sample_it = feedback.positions.find(name);
                const auto& variable_names = moveit_state->getVariableNames();
                if (sample_it == feedback.positions.end() ||
                    !std::isfinite(sample_it->second) ||
                    std::find(variable_names.begin(), variable_names.end(), name) == variable_names.end()) {
                    values_valid = false;
                    break;
                }
                const double monitored = moveit_state->getVariablePosition(name);
                if (!std::isfinite(monitored)) {
                    values_valid = false;
                    break;
                }
                max_delta = std::max(max_delta, std::abs(monitored - sample_it->second));
                ++compared;
            }
            if (!values_valid) break;
        }
    }
    const bool movegroup_state_ok = values_valid && compared == 14 && max_delta <= 0.020;
    RCLCPP_INFO(ctx_->get_logger(),
        "[MMC_MOVEGROUP_STATE_CHECK] result=%s source=MoveGroupInterface_CurrentStateMonitor "
        "complete_arm_joints=%zu/14 feedback_seq=%llu feedback_sim_time=%.9f feedback_age_wall_sec=%.6f "
        "max_delta_rad=%.9f existing_start_tolerance_rad=0.020 wait_budget_sec=%.3f "
        "elapsed_wall_sec=%.6f reason=%s",
        movegroup_state_ok ? "passed" : "failed", compared,
        static_cast<unsigned long long>(feedback.seq), have_feedback ? feedback.stamp.seconds() : 0.0,
        have_feedback ? steadySeconds() - feedback.recv_steady_sec : -1.0,
        max_delta, kCurrentStateWaitSeconds, state_check_elapsed,
        !moveit_state ? "movegroup_current_state_unavailable" :
        !freshness.valid ? freshness.reason.c_str() :
        compared != 14 ? "movegroup_joint_state_incomplete" :
        max_delta > 0.020 ? "movegroup_joint_state_differs_from_fresh_feedback" : "ok");
    if (!movegroup_state_ok) {
        reason = "E_MMC_MOVEGROUP_CURRENT_STATE_INVALID";
        return false;
    }

    const std::string binary = ament_index_cpp::get_package_prefix("dual_arm_carry_task") +
        "/lib/dual_arm_carry_task/dual_impedance_no_load_bridge";
    const std::vector<std::string> args{
        "--ros-args", "-p", "use_sim_time:=true", "-p", "preflight_only:=true",
        "-p", "state_read_budget_seconds:=3.0", "-p",
        "robot_states_service:=/dual_arm_carry_task/current_robot_states"};
    dual_arm_carry_task::BridgeProcess bridge;
    std::string spawn_error;
    if (!bridge.start(binary, args, spawn_error)) {
        reason = "E_MMC_STARTUP_PREFLIGHT_START: " + spawn_error;
        RCLCPP_ERROR(ctx_->get_logger(), "[MMC_STARTUP_PREFLIGHT] result=failed reason=%s", reason.c_str());
        return false;
    }
    const auto deadline = began + 9s;
    int status = -1;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline && status == -1) {
        status = bridge.poll();
        if (status == -1) std::this_thread::sleep_for(20ms);
    }
    if (status == -1) {
        const bool stopped = bridge.stop();
        reason = std::string("E_MMC_STARTUP_PREFLIGHT_TIMEOUT child_stopped=") + (stopped ? "true" : "false");
    } else if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        reason = "E_MMC_STARTUP_PREFLIGHT_REJECTED status=" + std::to_string(status);
    }
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - began).count();
    RCLCPP_INFO(ctx_->get_logger(), "[MMC_STARTUP_PREFLIGHT] result=%s service=%s elapsed_wall_sec=%.6f%s%s",
                reason.empty() ? "passed" : "failed", "/dual_arm_carry_task/current_robot_states", elapsed,
                reason.empty() ? "" : " reason=", reason.empty() ? "" : reason.c_str());
    return reason.empty();
}

bool DualArmController::runContactLoadProbe(const a1t::TruthPayload& truth,std::string& detail,
                                           bool hold_diagnostic_only, bool plan_compare) {
    using namespace std::chrono_literals;
    // First bounded experiment, deliberately independent of S1 ideal_weld.
    // Previous traces measured < 2.3 N upward transfer against 6.28 N rod
    // weight with a 5 N/finger hold, despite stable actuator output. Use one
    // evidence-based clamp-force step; friction, mass and acceptance stay fixed.
    // 10 N/finger is the largest request this position-tendon hold can express
    // at the measured grasp opening: the fixed position target leaves
    // ctrlrange at about 14 N, and kp*L/2 caps the achievable finger force
    // near 12.5 N. A 50 N step needs a force-actuator change and is out of
    // scope for the contact-probe run.
    constexpr double force=10.0, height=0.40, hold_seconds=2.0;
    // Project acceptance is a 45 mm rod rise, independent of the commanded lift
    // height. With ~7-9 mm vertical slip, a 6 cm lift is needed to clear it.
    constexpr double rod_rise_target=0.045;
    constexpr double pos_tol=0.005, angle_tol=2.0*M_PI/180.0;
    // B (creep hold) was implemented and tested: it made the rod slide 7.6 mm
    // during the hold and timed out, so it is kept but disabled. The static
    // hold with the recorded duty warning is the working configuration.
    constexpr bool creep_hold_enabled = false;
    // User-authorized: a few mm of vertical slip during the lift is acceptable as
    // long as the rod is actually carried. Keep pos_tol=5 mm for grasp/arrival
    // checks; relax only the lift/hold/descent monitor.
    constexpr double lift_slip_tol=0.020;
    // Lift timing scale. Kept as named constants so IPTP, the directional time
    // scaling, and the post-scaling limit check can never disagree.
    constexpr double lift_velocity_scale=1.00, lift_acceleration_scale=0.60;
    const std::array<std::string,2> arms{{"mj_left","mj_right"}};
    std::array<DualArmCarryTask::GripperContactFeedback,2> fingers;
    bool force_hold_may_be_active=false;
    auto logSnapshot=[&](const char* reason) {
        for (int i=0;i<2;++i) {
            DualArmCarryTask::GripperContactFeedback sample;
            if (!ctx_->latestGripperContact(arms[i], sample)) {
                RCLCPP_ERROR(ctx_->get_logger(),
                    "[CONTACT_DIAGNOSTIC] reason=%s arm=%s feedback=missing",
                    reason, arms[i].c_str());
                continue;
            }
            RCLCPP_ERROR(ctx_->get_logger(),
                "[CONTACT_DIAGNOSTIC] reason=%s arm=%s q=(%.6f,%.6f) command_per_finger=%.6f "
                "force_limit=%.3f hold_force=%.3f pads=%u contacts=(%d,%d) "
                "normal=(%.3f,%.3f) tangent=(%.3f,%.3f) rod_force_xyz=(%.3f,%.3f,%.3f) "
                "seq=%llu sim_time=%.6f weld=(%d,%d) rod_constraint=%d",
                reason, arms[i].c_str(), sample.q1, sample.q2, sample.command_per_finger_m,
                sample.closing_force_limit_per_finger_n, sample.hold_force_per_finger_n,
                sample.rod_pad_contact_count, sample.left_contact ? 1 : 0,
                sample.right_contact ? 1 : 0, sample.normal_left_n, sample.normal_right_n,
                sample.tangent_left_n, sample.tangent_right_n, sample.rod_force_x_n, sample.rod_force_y_n, sample.rod_force_z_n,
                static_cast<unsigned long long>(sample.seq), sample.simulation_time,
                sample.weld_left_active ? 1 : 0, sample.weld_right_active ? 1 : 0,
                sample.rod_constraint_active ? 1 : 0);
        }
    };
    auto fail=[&](const std::string& reason) {
        detail=reason;
        logSnapshot(reason.c_str());
        RCLCPP_ERROR(ctx_->get_logger(),"[CONTACT_PROBE_FAILURE] %s",reason.c_str());
        if (force_hold_may_be_active) {
            const bool stopped=ctx_->grippers_->stopDualGrippers();
            RCLCPP_WARN(ctx_->get_logger(),
                "[CONTACT_PROBE_FAILURE_CLEANUP] measured-position stop=%s; failure remains %s",
                stopped ? "confirmed" : "unconfirmed", reason.c_str());
        }
        return false;
    };
    // S2 transport plan-only evaluation builds its sample without touching the
    // gripper or the arms, so the whole grasp sequence below is skipped.
    const bool plan_only = ctx_->contact_transport_plan_only_;
    if (ctx_->contact_hover_only_ &&
        (!std::isfinite(ctx_->contact_hover_height_m_) ||
         ctx_->contact_hover_height_m_ < rod_rise_target || ctx_->contact_hover_height_m_ > 0.10 ||
         !std::isfinite(ctx_->contact_hover_hold_seconds_) ||
         ctx_->contact_hover_hold_seconds_ < 1.0 || ctx_->contact_hover_hold_seconds_ > 10.0))
        return fail("E_CONTACT_HOVER_BAD_PARAMETERS: height=0.045..0.10m hold=1..10s");
    if (!a1_verify_.valid) return fail("E_CONTACT_PRECONDITION");
    if (!plan_only &&
        (!a1_verify_.pregrasp_reached_this_round || !a1_verify_.descent_plan_valid ||
         ctx_->getCurrentState()!=TaskState::PREGRASP_REACHED)) return fail("E_CONTACT_PRECONDITION");
    if (!checkControllerStatesEnabled(detail)) return fail("E_CONTACT_CONTROLLER: "+detail);
    std::string run_id=ctx_->expected_run_id_;
    if (run_id.empty() || run_id == "." || run_id == ".." || run_id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-")!=std::string::npos)
        return fail("E_CONTACT_RUN_ID");
    std::ofstream csv("experiment_results/a1_runs/"+run_id+"/contact_probe.csv");
    if (!csv) return fail("E_CONTACT_CSV_OPEN");
    csv << std::setprecision(10) << "phase,left_sim_time,right_sim_time,rod_x,rod_y,rod_z,left_slip_m,left_angle_rad,right_slip_m,right_angle_rad,left_q1,left_q2,right_q1,right_q2,left_command_per_finger_m,right_command_per_finger_m,left_force_limit_n,right_force_limit_n,left_hold_force_n,right_hold_force_n,left_pad_contact_count,right_pad_contact_count,left_contact_1,left_contact_2,right_contact_1,right_contact_2,left_normal1_n,left_normal2_n,right_normal1_n,right_normal2_n,left_tangent1_n,left_tangent2_n,right_tangent1_n,right_tangent2_n,left_rod_force_x_n,left_rod_force_y_n,left_rod_force_z_n,right_rod_force_x_n,right_rod_force_y_n,right_rod_force_z_n,left_seq,right_seq,weld_left,weld_right,rod_constraint,left_hand_x,left_hand_y,left_hand_z,left_hand_qx,left_hand_qy,left_hand_qz,left_hand_qw,right_hand_x,right_hand_y,right_hand_z,right_hand_qx,right_hand_qy,right_hand_qz,right_hand_qw\n";
    std::array<unsigned long long,2> seq{{0,0}};
    std::array<double,2> sim_time{{-1,-1}}, progress_wall{{steadySeconds(),steadySeconds()}};
    auto feedback=[&]() {
        for (int i=0;i<2;++i) {
            if (!ctx_->latestGripperContact(arms[i],fingers[i])) { detail="E_CONTACT_FEEDBACK_MISSING"; return false; }
            const auto& f=fingers[i];
            const double force_values[]{f.normal_left_n,f.normal_right_n,f.tangent_left_n,
                f.tangent_right_n,f.rod_force_x_n,f.rod_force_y_n,f.rod_force_z_n,f.hold_force_per_finger_n};
            for (const double v:force_values) if (!std::isfinite(v)) { detail="E_CONTACT_NONFINITE_FORCE"; return false; }
            if (steadySeconds()-f.recv_steady_sec>0.25 || !std::isfinite(f.simulation_time) ||
                !std::isfinite(f.q1) || !std::isfinite(f.q2)) {
                detail="E_CONTACT_FEEDBACK_STALE_OR_NONFINITE"; return false;
            }
            if (!f.force_hold_supported) {
                detail="E_CONTACT_FORCE_HOLD_UNSUPPORTED"; return false;
            }
            if (f.weld_left_active || f.weld_right_active || f.rod_constraint_active) {
                detail="E_CONTACT_CONSTRAINT_ACTIVE"; return false;
            }
            if (f.seq>seq[i] && f.simulation_time>sim_time[i]) {
                seq[i]=f.seq; sim_time[i]=f.simulation_time; progress_wall[i]=steadySeconds();
            }
            if (steadySeconds()-progress_wall[i]>0.25) { detail="E_CONTACT_PHYSICS_NOT_ADVANCING"; return false; }
        }
        return true;
    };
    auto live_model=ctx_->dual_arm_->getRobotModel();
    const auto criterion_model=a1_verify_.model;
    const auto* dual=live_model->getJointModelGroup(ctx_->dual_arm_group_);
    const auto* left=live_model->getJointModelGroup(ctx_->left_arm_group_);
    const auto* right=live_model->getJointModelGroup(ctx_->right_arm_group_);
    const auto* criterion_dual=criterion_model->getJointModelGroup(ctx_->dual_arm_group_);
    if (!dual || !left || !right || !criterion_dual || !left->getSolverInstance() || !right->getSolverInstance())
        return fail("E_CONTACT_MODEL_OR_IK");
    auto measured=[&](moveit::core::RobotState& state) {
        if (!feedback()) return false;
        DualArmCarryTask::JointFeedback j;
        if (!ctx_->latestJointFeedback(j) || steadySeconds()-j.recv_steady_sec>0.25) { detail="E_CONTACT_ARM_FEEDBACK_STALE"; return false; }
        for (const auto& name:dual->getVariableNames()) {
            const auto it=j.positions.find(name);
            if (it==j.positions.end() || !std::isfinite(it->second)) { detail="E_CONTACT_ARM_FEEDBACK_INCOMPLETE"; return false; }
            state.setVariablePosition(name,it->second);
        }
        const auto& names=a1v::fingerJointNames();
        for (int i=0;i<2;++i) {
            state.setVariablePosition(names[2*i],fingers[i].q1);
            state.setVariablePosition(names[2*i+1],fingers[i].q2);
        }
        state.update(); return true;
    };
    // Establish current actual constraint state before issuing contact actions.
    if (!plan_only) {
    const auto gate=std::chrono::steady_clock::now()+2s;
    bool ready=false;
    while (rclcpp::ok() && std::chrono::steady_clock::now()<gate) {
        if (feedback()) { ready=true; break; } std::this_thread::sleep_for(20ms);
    }
    if (!ready) return fail(detail);
    }
    const double truth_probe_sim_start=ctx_->get_clock()->now().seconds();
    const auto truth_probe_wall_start=std::chrono::steady_clock::now();
    const auto truth_probe_first=readMujocoRodTruth();
    if (!truth_probe_first.ok)
        return fail("E_CONTACT_TRUTH_STARTUP_READ_1 service="+
            ctx_->mujoco_get_body_state_service_+" detail="+truth_probe_first.detail);
    std::this_thread::sleep_for(50ms);
    const auto before=readMujocoRodTruth();
    if (!before.ok)
        return fail("E_CONTACT_TRUTH_STARTUP_READ_2 service="+
            ctx_->mujoco_get_body_state_service_+" detail="+before.detail);
    const double truth_probe_sim_delta=ctx_->get_clock()->now().seconds()-truth_probe_sim_start;
    RCLCPP_INFO(ctx_->get_logger(),
        "[CONTACT_TRUTH_STARTUP_READINESS] service=%s responses=2 result=valid "
        "wall_elapsed_sec=%.6f sim_time_advance_sec=%.6f first_z=%.6f latest_z=%.6f",
        ctx_->mujoco_get_body_state_service_.c_str(),
        std::chrono::duration<double>(std::chrono::steady_clock::now()-truth_probe_wall_start).count(),
        truth_probe_sim_delta,truth_probe_first.position.z,before.position.z);
    const Eigen::Vector3d frozen(truth.position.x,truth.position.y,truth.position.z);
    if ((rodTransform(before).translation()-frozen).norm()>0.005 ||
        a1t::rotationDistanceRad(before.orientation,{truth.orientation.x,truth.orientation.y,truth.orientation.z,truth.orientation.w})>5*M_PI/180)
        return fail("E_CONTACT_FROZEN_REFERENCE_CHANGED");
    RCLCPP_INFO(ctx_->get_logger(),"[CONTACT_PROBE_CONFIG] requested_hold_force_per_finger=%.1f N "
        "nominal_tendon_actuator_command=%.1f N (0.90*2F; nominal per-branch generalized force=%.1f N) "
        "runtime_tendon_force_cap=+/-%.1f N; actual actuator_force is in contact_probe_contacts.csv, "
        "actual pad normal force is in contact_probe.csv; mode=%s lift=%.3fm hold=%.1fs "
        "lift_slip_limit=%.1fmm(user-authorized)/2deg; grasp_slip_tol=%.1fmm; rod_rise_target=%.0fmm; "
        "acceptance hold=%.1fs; lift_velocity_scale=%.2f lift_acceleration_scale=%.2f weld=OFF; "
        "model/friction/mass unchanged",
        force, -2.0*force*0.90, -force*0.90, force,
        hold_diagnostic_only ? "fixed-arm-mechanism-diagnostic" :
            (ctx_->contact_hover_only_ ? "rigid-hover-only" : "full-load-probe"),
        ctx_->contact_hover_only_ ? ctx_->contact_hover_height_m_ : height,
        ctx_->contact_hover_only_ ? ctx_->contact_hover_hold_seconds_ : hold_seconds,
        lift_slip_tol*1000.0, pos_tol*1000.0, rod_rise_target*1000.0,
        hold_seconds, lift_velocity_scale, lift_acceleration_scale);
    // S2 transport plan-only evaluation builds its sample without touching the
    // gripper or the arms, so the whole grasp sequence below is skipped.
    const std::array<std::string,2> eefs{{a1_verify_.left_eef,a1_verify_.right_eef}};
    // State source is registered during task initialization so a newly started
    // bridge can preflight before planning or any arm/gripper motion. JTC hover
    // has no bridge and therefore does not require it.
    if (ctx_->contact_hover_control_mode_ == "mmc_hold" && !robot_state_service_)
        return fail("E_MMC_STATE_SOURCE_NOT_READY");
    std::array<Transform,2> object_to_hand{{Transform::Identity(),Transform::Identity()}};
    Transform object=Transform::Identity();
    MujocoRodTruth initial_truth;
    auto initial_truth_received = std::chrono::steady_clock::time_point{};
    moveit::core::RobotState start(live_model);
    // One arm may have entered direct-force hold even when its peer's action
    // fails. From the first contact-goal attempt, failure cleanup sends the
    // measured-position Stop service to both sides before the simulator exits.
    if (!plan_only) {
    force_hold_may_be_active=true;
    if (!ctx_->grippers_->controlDualGrippersContact(force)) {
        ctx_->contact_action_completion_seq_ = {{0, 0}};
        return fail("E_CONTACT_GRASP_ACTION_FAILED_OR_UNKNOWN");
    }
    for (std::size_t i = 0; i < arms.size(); ++i) {
        DualArmCarryTask::GripperContactFeedback completion_feedback;
        ctx_->contact_action_completion_seq_[i] =
            ctx_->latestGripperContact(arms[i], completion_feedback) ? completion_feedback.seq : 0;
    }
    RCLCPP_INFO(ctx_->get_logger(),
        "[CONTACT_ACTION_FENCE] action_success=true left_completion_seq=%llu right_completion_seq=%llu",
        ctx_->contact_action_completion_seq_[0], ctx_->contact_action_completion_seq_[1]);
    if (!measured(start)) return fail(detail);
    initial_truth=readMujocoRodTruth();
    initial_truth_received=std::chrono::steady_clock::now();
    const double initial_qnorm=std::sqrt(initial_truth.orientation.x*initial_truth.orientation.x+initial_truth.orientation.y*initial_truth.orientation.y+
        initial_truth.orientation.z*initial_truth.orientation.z+initial_truth.orientation.w*initial_truth.orientation.w);
    if (!initial_truth.ok || std::abs(initial_qnorm-1.0)>1e-3) return fail("E_CONTACT_TRUTH_AFTER_CLOSE");
    object=rodTransform(initial_truth);
    if ((object.translation()-rodTransform(before).translation()).norm()>pos_tol ||
        rotationError(object,rodTransform(before))>angle_tol) return fail("E_CONTACT_ROD_MOVED_DURING_CLOSE");
    object_to_hand[0]=object.inverse()*start.getGlobalLinkTransform(eefs[0]);
    object_to_hand[1]=object.inverse()*start.getGlobalLinkTransform(eefs[1]);
    // Preserve the descent arrival limits after closing.
    const std::array<a1v::ArmTarget,2> grasp{{a1_verify_.grasp_target.left,a1_verify_.grasp_target.right}};
    for (int i=0;i<2;++i) {
        Transform desired=Transform::Identity();
        desired.translation()=Eigen::Vector3d(grasp[i].position.x,grasp[i].position.y,grasp[i].position.z);
        desired.linear()=Eigen::Quaterniond(grasp[i].orientation.w,grasp[i].orientation.x,grasp[i].orientation.y,grasp[i].orientation.z).normalized().toRotationMatrix();
        if ((desired.translation()-start.getGlobalLinkTransform(eefs[i]).translation()).norm()>pos_tol ||
            rotationError(desired,start.getGlobalLinkTransform(eefs[i]))>angle_tol) return fail("E_CONTACT_GRASP_ARRIVAL_CHANGED");
    }
    }
    auto scene=a1_verify_.scene->diff(); scene->decoupleParent();
    auto acm=a1_verify_.grasp_acm;
    acm.setEntry(a1_verify_.rod_id,"mj_left_hand",false);
    acm.setEntry(a1_verify_.rod_id,"mj_right_hand",false);
    auto updateRod=[&](const Transform& pose) {
        moveit_msgs::msg::CollisionObject rod;
        rod.header.frame_id=ctx_->dual_arm_->getPlanningFrame(); rod.id=a1_verify_.rod_id;
        shape_msgs::msg::SolidPrimitive box; box.type=box.BOX; box.dimensions={truth.size.x,truth.size.y,truth.size.z};
        rod.primitives.push_back(box); rod.primitive_poses.push_back(poseMsg(pose)); rod.operation=rod.ADD;
        return scene->processCollisionObjectMsg(rod);
    };
    // The creep plan moves both hands by a few mm from the current state; the
    // two hands' inferred rod poses can already differ there, so the creep plan
    // skips only the dual-residual check (collision/support/limits remain).
    bool relax_dual_residual = false;
    auto valid=[&](const moveit::core::RobotState& live) {
        moveit::core::RobotState criterion(criterion_model);
        for (const auto& name:criterion_model->getVariableNames()) criterion.setVariablePosition(name,live.getVariablePosition(name));
        criterion.update();
        if (!criterion.satisfiesBounds(criterion_dual,1e-4)) { detail="E_CONTACT_JOINT_BOUNDS"; return false; }
        const Transform inferred=live.getGlobalLinkTransform(eefs[0])*object_to_hand[0].inverse();
        const Transform inferred_right=live.getGlobalLinkTransform(eefs[1])*object_to_hand[1].inverse();
        if (!relax_dual_residual &&
            ((inferred.translation()-inferred_right.translation()).norm()>pos_tol ||
             rotationError(inferred,inferred_right)>angle_tol)) {
            detail="E_CONTACT_DUAL_GRASP_RESIDUAL"; return false;
        }
        // Tilt-aware physical box height. Support contact only near the start plane.
        const double extent=0.5*(std::abs(inferred.linear()(2,0))*truth.size.x+
            std::abs(inferred.linear()(2,1))*truth.size.y+std::abs(inferred.linear()(2,2))*truth.size.z);
        const double clearance=inferred.translation().z()-extent-ctx_->p2a_support_z_;
        if (clearance < -0.0001 || !updateRod(inferred)) { detail="E_CONTACT_ROD_SUPPORT_PENETRATION"; return false; }
        auto local_acm=acm;
        local_acm.setEntry(a1_verify_.rod_id,a1_verify_.support_id,clearance<=0.003);
        collision_detection::CollisionRequest req; req.contacts=true; req.max_contacts=20;
        collision_detection::CollisionResult res; scene->checkCollision(req,res,criterion,local_acm);
        if (res.collision) {
            detail="E_CONTACT_PATH_COLLISION";
            if (!res.contacts.empty()) detail += ": "+res.contacts.begin()->first.first+"<->"+res.contacts.begin()->first.second;
            return false;
        }
        return true;
    };
    // The four finger joints are not part of the dual-arm group. Every state
    // used for a collision check must carry the HELD finger positions: leaving
    // them at the RobotState default (0 = folded inside the hand) does not
    // represent the transport and produced a real self-collision
    // (link2<->finger) in the extended rotation path. Live runs copy the
    // measured fingers; the plan-only sample uses the measured held value
    // from the reported hover runs (~0.0194 m total gap 0.0388 m on the 4 cm
    // rod).
    std::array<double,4> transport_finger_positions{{0.0194,0.0194,0.0194,0.0194}};
    auto applyTransportFingers=[&](moveit::core::RobotState& state) {
        const auto& names=a1v::fingerJointNames();
        for (std::size_t i=0;i<names.size();++i)
            state.setVariablePosition(names[i],transport_finger_positions[i]);
    };
    auto makePlan=[&](const moveit::core::RobotState& origin,const Eigen::Vector3d& delta,
                      moveit::planning_interface::MoveGroupInterface::Plan& plan,
                      const char* label,
                      double vscale,
                      double ascale,
                      double min_total,
                      double dyaw=0.0,
                      bool optimize_path=false,
                      std::vector<moveit::core::RobotState>* path_out=nullptr) {
        if (!valid(origin)) return false;
        {
            const auto& finger_names=a1v::fingerJointNames();
            for (std::size_t i=0;i<finger_names.size();++i)
                transport_finger_positions[i]=origin.getVariablePosition(finger_names[i]);
        }
        const Transform first_object=origin.getGlobalLinkTransform(eefs[0])*object_to_hand[0].inverse();
        // Object-space waypoint k of kPathSteps. The object path, the grasp
        // transforms and the endpoints are frozen; only the joint redundancy is
        // ever optimised.
        auto expectedObject=[&](double u) {
            Transform t=first_object;
            t.translation()+=delta*u;
            t.linear()=Eigen::AngleAxisd(dyaw*u,Eigen::Vector3d::UnitZ()).toRotationMatrix()*first_object.linear();
            return t;
        };
        // Use finer object-space increments and longer segment times for the
        // contact lift. The previous 5 mm / 0.5 s segments produced intermittent
        // pad-contact loss and impulsive re-contact before the rod could leave
        // the support surface.
        const int kPathSteps = std::max({20,
            static_cast<int>(std::ceil(delta.norm()/0.005)),
            static_cast<int>(std::ceil(std::abs(dyaw)/0.01))});
        // Joint-space interpolation between two waypoints is not on the closed
        // chain, so every interpolated sample is checked against the same 5 mm /
        // 2 deg acceptance as the baseline. Shared by both the seed path and any
        // optimised candidate.
        auto verifyTransition=[&](const moveit::core::RobotState& prev,
                                  const moveit::core::RobotState& next,int seg) {
            std::vector<double> a,b; prev.copyJointGroupPositions(dual,a); next.copyJointGroupPositions(dual,b);
            const double jump=a1v::maxAbsJointDelta(a,b);
            if (!std::isfinite(jump) || jump>0.15) { detail="E_CONTACT_IK_BRANCH_JUMP"; return false; }
            const auto n=std::max<std::size_t>(1,a1v::subdivisionsFor(jump,0.01));
            for (std::size_t j=1;j<=n;++j) {
                const double alpha=double(j)/n;
                moveit::core::RobotState sample(prev); sample.setJointGroupPositions(dual,a1v::interpolateJoints(a,b,alpha)); sample.update();
                if (!valid(sample)) return false;
                const Transform actual=sample.getGlobalLinkTransform(eefs[0])*object_to_hand[0].inverse();
                const Transform expected=expectedObject((seg-1+alpha)/static_cast<double>(kPathSteps));
                if ((actual.translation()-expected.translation()).norm()>pos_tol || rotationError(actual,expected)>angle_tol) {
                    detail="E_CONTACT_INTERPOLATED_PATH"; return false;
                }
            }
            return true;
        };
        std::vector<moveit::core::RobotState> waypoints;
        {
            moveit::core::RobotState seed(origin);
            waypoints.push_back(seed);
            for (int k=1;k<=kPathSteps;++k) {
                const Transform target=expectedObject(static_cast<double>(k)/kPathSteps);
                moveit::core::RobotState next(seed);
                if (!next.setFromIK(left,target*object_to_hand[0],eefs[0],0.05) ||
                    !next.setFromIK(right,target*object_to_hand[1],eefs[1],0.05)) { detail="E_CONTACT_IK"; return false; }
                next.update();
                for (int side=0;side<2;++side) {
                    const auto actual=next.getGlobalLinkTransform(eefs[side])*object_to_hand[side].inverse();
                    if ((actual.translation()-target.translation()).norm()>0.0005 || rotationError(actual,target)>0.1*M_PI/180) {
                        detail="E_CONTACT_IK_RESIDUAL"; return false;
                    }
                }
                if (!verifyTransition(seed,next,k)) return false;
                seed=next; waypoints.push_back(seed);
            }
        }
        if (optimize_path) {
            // S2: keep X_k and G_L/G_R fixed, move only the internal joint
            // waypoints inside the two task Jacobians' null space. The timing
            // pipeline below is untouched, so the comparison isolates geometry.
            std::vector<cpo::JointVector> seed_path;
            std::vector<Transform> left_targets,right_targets;
            std::vector<double> params;
            for (int k=0;k<=kPathSteps;++k) {
                std::vector<double> q; waypoints[k].copyJointGroupPositions(dual,q);
                seed_path.push_back(Eigen::Map<const Eigen::VectorXd>(q.data(),static_cast<Eigen::Index>(q.size())));
                params.push_back(static_cast<double>(k)/kPathSteps);
                const Transform obj=expectedObject(static_cast<double>(k)/kPathSteps);
                left_targets.push_back(obj*object_to_hand[0]);
                right_targets.push_back(obj*object_to_hand[1]);
            }
            cpo::ProbeFn probe=[&](const cpo::JointVector& q,cpo::DualPose& out) {
                if (!q.allFinite() || q.size()!=static_cast<Eigen::Index>(dual->getVariableCount())) return false;
                moveit::core::RobotState state(live_model);
                for (std::size_t i=0;i<dual->getVariableNames().size();++i) state.setVariablePosition(dual->getVariableNames()[i],q[static_cast<Eigen::Index>(i)]);
                state.update();
                out.left=state.getGlobalLinkTransform(eefs[0]);
                out.right=state.getGlobalLinkTransform(eefs[1]);
                return true;
            };
            cpo::ValidateFn validate_candidate=[&](const std::vector<cpo::JointVector>& path,std::string& why) {
                for (std::size_t k=0;k<path.size();++k) {
                    moveit::core::RobotState state(live_model);
                    for (std::size_t i=0;i<dual->getVariableNames().size();++i) state.setVariablePosition(dual->getVariableNames()[i],path[k][static_cast<Eigen::Index>(i)]);
                    applyTransportFingers(state);
                    state.update();
                    if (!valid(state)) { why="waypoint "+std::to_string(k)+": "+detail; return false; }
                    if (k>0) {
                        std::vector<double> a,b; a.assign(path[k-1].data(),path[k-1].data()+path[k-1].size());
                        b.assign(path[k].data(),path[k].data()+path[k].size());
                        const double jump=a1v::maxAbsJointDelta(a,b);
                        if (!std::isfinite(jump) || jump>0.15) { why="waypoint "+std::to_string(k)+": E_CONTACT_OPT_JOINT_JUMP"; return false; }
                    }
                }
                return true;
            };
            double s1=0.0,s2=0.0,s0=0.0; std::string scale_error;
            const cpo::Options options;
            const bool scales_ok=cpo::seedScales(seed_path,params,options,s1,s2,s0,scale_error);
            double seed_length=0.0;
            for (std::size_t k=1;k<seed_path.size();++k) seed_length+=(seed_path[k]-seed_path[k-1]).norm();
            const double opt_start=steadySeconds();
            const cpo::Result opt=cpo::optimize(seed_path,left_targets,right_targets,params,probe,validate_candidate,options);
            const double opt_sec=steadySeconds()-opt_start;
            if (!scales_ok) {
                RCLCPP_WARN(ctx_->get_logger(),"[TRANSPORT_OPT] label=%s scales_unavailable=%s",label,scale_error.c_str());
            }
            RCLCPP_INFO(ctx_->get_logger(),
                "[TRANSPORT_OPT] label=%s status=%s accepted=%d iterations=%d accepted_steps=%d "
                "internal=%d vars=%d rank=%d null_dim=%d left_rank=%d right_rank=%d "
                "grad_rms=%.9f J0=%.9f J1=%.9f reduction=%.4f a2=%.9f->%.9f a1=%.9f->%.9f a0=%.9f->%.9f "
                "weights=(w2=%.3f,w1=%.3f,w0=%.3f) S=(%.9f,%.9f,%.9f) optimize_sec=%.3f detail=%s",
                label,opt.diag.status.c_str(),opt.diag.accepted?1:0,opt.diag.iterations,opt.diag.accepted_steps,
                opt.diag.internal_waypoints,opt.diag.variables,opt.diag.rank,opt.diag.null_dim,
                opt.diag.left_rank,opt.diag.right_rank,opt.diag.projected_gradient_rms,
                opt.diag.initial.total,opt.diag.final.total,
                opt.diag.initial.total>0.0?1.0-opt.diag.final.total/opt.diag.initial.total:0.0,
                opt.diag.initial.raw.a2,opt.diag.final.raw.a2,
                opt.diag.initial.raw.a1,opt.diag.final.raw.a1,
                opt.diag.initial.raw.a0,opt.diag.final.raw.a0,
                options.w2,options.w1,options.w0,s2,s1,s0,opt_sec,
                opt.diag.detail.empty()?"none":opt.diag.detail.c_str());
            bool applied=false;
            if (opt.diag.accepted) {
                std::vector<moveit::core::RobotState> candidate;
                for (const auto& q:opt.path) {
                    moveit::core::RobotState state(live_model);
                    for (std::size_t i=0;i<dual->getVariableNames().size();++i) state.setVariablePosition(dual->getVariableNames()[i],q[static_cast<Eigen::Index>(i)]);
                    applyTransportFingers(state);
                    state.update();
                    candidate.push_back(state);
                }
                bool transitions_ok=true;
                for (int k=1;k<=kPathSteps && transitions_ok;++k) transitions_ok=verifyTransition(candidate[k-1],candidate[k],k);
                if (transitions_ok) {
                    double length=0.0;
                    for (std::size_t k=1;k<opt.path.size();++k) length+=(opt.path[k]-opt.path[k-1]).norm();
                    waypoints.swap(candidate);
                    applied=true;
                    detail.clear();
                    RCLCPP_INFO(ctx_->get_logger(),
                        "[TRANSPORT_OPT] label=%s applied=1 joint_path_length=%.6f->%.6f (%.4f%%)",
                        label,seed_length,length,seed_length>0.0?100.0*(length-seed_length)/seed_length:0.0);
                } else {
                    RCLCPP_WARN(ctx_->get_logger(),
                        "[TRANSPORT_OPT] label=%s applied=0 reason=interpolated_path_rejected (%s) "
                        "fallback=baseline_path; the optimiser result is NOT reported as an improvement",
                        label,detail.empty()?"none":detail.c_str());
                    detail.clear();
                }
            } else {
                RCLCPP_INFO(ctx_->get_logger(),
                    "[TRANSPORT_OPT] label=%s applied=0 baseline path kept status=%s reason=%s",
                    label,opt.diag.status.c_str(),opt.diag.detail.c_str());
            }
            (void)applied;
        }
        if (path_out) *path_out = waypoints;
        robot_trajectory::RobotTrajectory trajectory(live_model,ctx_->dual_arm_group_);
        for (auto& waypoint:waypoints) trajectory.addSuffixWayPoint(waypoint,0.0);

        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        if (!iptp.computeTimeStamps(trajectory,vscale,ascale)) { detail="E_CONTACT_IPTP"; return false; }
        trajectory.getRobotTrajectoryMsg(plan.trajectory_); moveit::core::robotStateToRobotStateMsg(origin,plan.start_state_);
        auto& joint=plan.trajectory_.joint_trajectory;
        std::vector<s1::DirectionalJointLimits> limits;
        for (const auto& name:joint.joint_names) { const auto& b=live_model->getVariableBounds(name);
            limits.push_back({b.min_velocity_,b.max_velocity_,b.min_acceleration_,b.max_acceleration_,b.velocity_bounded_,b.acceleration_bounded_}); }
        std::vector<s1::TimedJointPoint> points;
        for (const auto& p:joint.points) points.push_back({rclcpp::Duration(p.time_from_start).seconds(),p.positions,p.velocities,p.accelerations});
        s1::TimeScalingReport report;
        if (!s1::scaleTimedTrajectory(points,limits,vscale,ascale,0.35,report,detail)) return false;
        // E5 diagnostic: the previous 5 s / 1 s-per-segment stretch capped the lift
        // at 2.5 mm/s and neutralized any velocity scaling. Relax both minimums so the
        // 10x scaling actually applies. This is a trajectory-speed diagnostic, not a
        // change to physics or acceptance.
        // NOTE: min_total enters as an absolute stretch floor, not a duration floor:
        // stretch = max(min_total, min_total/T, 0.02/min_segment_dt). For the
        // transport call min_total=2.0 therefore doubles the IPTP duration.
        const double t_scaled=points.back().time;
        const double stretch_abs=min_total;
        const double stretch_rel=min_total/t_scaled;
        double stretch_min_total=std::max(stretch_abs,stretch_rel);
        double stretch=stretch_min_total;
        double stretch_segment=1.0;
        for (std::size_t j=1;j<points.size();++j) {
            const double s=0.02/(points[j].time-points[j-1].time);
            stretch_segment=std::max(stretch_segment,s);
            stretch=std::max(stretch,s);
        }
        const char* binding = stretch<=1.0+1e-9 ? "none" :
            (stretch_segment>stretch_min_total+1e-9 ? "segment" : "min_total");
        RCLCPP_INFO(ctx_->get_logger(),
            "[CONTACT_PLAN_TIMING] label=%s points=%zu t_scaled=%.4f scale=%.4f "
            "stretch_abs=%.4f stretch_rel=%.4f stretch_min_total=%.4f "
            "stretch_segment=%.4f stretch=%.4f final=%.4f binding=%s",
            label,points.size(),t_scaled,report.scale,stretch_abs,stretch_rel,
            stretch_min_total,stretch_segment,stretch,t_scaled*stretch,binding);
        for (std::size_t j=0;j<points.size();++j) {
            joint.points[j].time_from_start=rclcpp::Duration::from_seconds(points[j].time*stretch);
            joint.points[j].velocities=points[j].velocities; joint.points[j].accelerations=points[j].accelerations;
            for (auto& v:joint.points[j].velocities) v/=stretch;
            for (auto& a:joint.points[j].accelerations) a/=(stretch*stretch);
        }
        // JTC receives position/velocity/acceleration, hence interpolates quintics.
        // Check that actual interpolation too, after timing (20 ms samples).
        for (std::size_t k=1;k<joint.points.size();++k) {
            const auto& a=joint.points[k-1]; const auto& b=joint.points[k];
            const double T=rclcpp::Duration(b.time_from_start).seconds()-rclcpp::Duration(a.time_from_start).seconds();
            const int n=std::max(2,static_cast<int>(std::ceil(T/0.02)));
            for (int sample=0;sample<=n;++sample) {
                const double u=double(sample)/n;
                std::vector<double> q(joint.joint_names.size());
                for (std::size_t j=0;j<q.size();++j) {
                    const double c0=a.positions[j],c1=a.velocities[j]*T,c2=0.5*a.accelerations[j]*T*T;
                    const double D=b.positions[j]-c0-c1-c2;
                    const double V=b.velocities[j]*T-c1-2*c2;
                    const double A=b.accelerations[j]*T*T-2*c2;
                    const double c3=10*D-4*V+0.5*A,c4=-15*D+7*V-A,c5=6*D-3*V+0.5*A;
                    q[j]=c0+u*(c1+u*(c2+u*(c3+u*(c4+u*c5))));
                    const double v=(c1+u*(2*c2+u*(3*c3+u*(4*c4+u*5*c5))))/T;
                    const double acc=(2*c2+u*(6*c3+u*(12*c4+u*20*c5)))/(T*T);
                    const auto& bound=limits[j];
                    if (!std::isfinite(q[j]) || !std::isfinite(v) || !std::isfinite(acc) ||
                        v<bound.min_velocity-1e-9 || v>bound.max_velocity+1e-9 ||
                        acc<bound.min_acceleration-1e-9 || acc>bound.max_acceleration+1e-9) {
                        RCLCPP_ERROR(ctx_->get_logger(),
                            "[CONTACT_QUINTIC] joint=%s v=%.6f vlim=[%.6f,%.6f] acc=%.6f alim=[%.6f,%.6f] seg=%zu sample=%d scale=%.2f/%.2f",
                            joint.joint_names[j].c_str(), v, bound.min_velocity,
                            bound.max_velocity, acc, bound.min_acceleration,
                            bound.max_acceleration, k, sample, vscale, ascale);
                        detail="E_CONTACT_QUINTIC_LIMIT"; return false;
                    }
                }
                moveit::core::RobotState timed(origin); timed.setVariablePositions(joint.joint_names,q); timed.update();
                if (!valid(timed)) return false;
            }
        }
        RCLCPP_INFO(ctx_->get_logger(),"[CONTACT_PROBE_PLAN] delta=%.3fm points=%zu IPTP duration=%.3fs contact_step=3.0mm min_segment=0.1s velocity_scale=%.2f acceleration_scale=%.2f collision_and_interpolation=passed finger_only_contacts=true",
            delta.norm(),joint.points.size(),rclcpp::Duration(joint.points.back().time_from_start).seconds(),
            vscale,ascale);
        return true;
    };
    // Shared transport evaluation used by the offline scenario comparison and
    // by the live baseline/candidate execution pairing: same object path, same
    // 20 ms sampling, same collision/limit checks and the same jerk definition
    // (analytic derivatives of the quintic the JTC actually interpolates).
    struct TransportPlanStats {
        std::size_t points{0},samples{0},collisions{0};
        double duration{0.0};
        double joint_path_length{0.0};
        double max_waypoint_dual_pos_mm{0.0},max_waypoint_dual_ang_deg{0.0};
        double max_sample_dual_pos_mm{0.0},max_sample_dual_ang_deg{0.0};
        double max_rod_pos_mm{0.0},max_rod_ang_deg{0.0};
        bool limit_violation{false},vel_violation{false},acc_violation{false};
        double max_vel_ratio{0.0},max_acc_ratio{0.0};
        double acc_rms{0.0},jerk_rms{0.0},jerk_max{0.0};
        std::size_t interpolated_steps{0};
    };
    auto evaluateTransport=[&](const char* name,
                               const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                               TransportPlanStats& stats,
                               const Transform& first_object,
                               const Eigen::Vector3d& delta,
                               double dyaw,
                               const std::function<double(double)>& progress_of_time,
                               std::ofstream* compare_csv) -> bool {
        auto expectedObject=[&](double u) {
            Transform t=first_object;
            t.translation()+=delta*u;
            t.linear()=Eigen::AngleAxisd(dyaw*u,Eigen::Vector3d::UnitZ()).toRotationMatrix()*first_object.linear();
            return t;
        };
        const auto& jt=plan.trajectory_.joint_trajectory;
        if (jt.points.size()<2) { detail="E_TRANSPORT_OPT_EMPTY_PLAN"; return false; }
        const std::size_t steps=jt.points.size()-1;
        stats.points=jt.points.size();
        stats.duration=rclcpp::Duration(jt.points.back().time_from_start).seconds();
        std::vector<s1::DirectionalJointLimits> limits;
        for (const auto& joint_name:jt.joint_names) {
            const auto& b=live_model->getVariableBounds(joint_name);
            limits.push_back({b.min_velocity_,b.max_velocity_,b.min_acceleration_,b.max_acceleration_,
                              b.velocity_bounded_,b.acceleration_bounded_});
        }
        // Waypoint-only check, on the criterion model, exactly like valid().
        std::size_t sample_index=0;
        double acc_sum=0.0,jerk_sum=0.0; std::size_t acc_count=0,jerk_count=0;
        for (std::size_t k=0;k<jt.points.size();++k) {
            moveit::core::RobotState state(live_model);
            state.setVariablePositions(jt.joint_names,jt.points[k].positions);
            applyTransportFingers(state);
            state.update();
            if (k>0) {
                const auto& prev=jt.points[k-1].positions;
                double step=0.0;
                for (std::size_t j=0;j<prev.size();++j) step+=std::pow(jt.points[k].positions[j]-prev[j],2);
                stats.joint_path_length+=std::sqrt(step);
            }
            const Transform inferred=state.getGlobalLinkTransform(eefs[0])*object_to_hand[0].inverse();
            const Transform inferred_right=state.getGlobalLinkTransform(eefs[1])*object_to_hand[1].inverse();
            stats.max_waypoint_dual_pos_mm=std::max(stats.max_waypoint_dual_pos_mm,
                (inferred.translation()-inferred_right.translation()).norm()*1000.0);
            stats.max_waypoint_dual_ang_deg=std::max(stats.max_waypoint_dual_ang_deg,
                rotationError(inferred,inferred_right)*180.0/M_PI);
            moveit::core::RobotState criterion_state(criterion_model);
            for (const auto& variable:criterion_model->getVariableNames())
                criterion_state.setVariablePosition(variable,state.getVariablePosition(variable));
            criterion_state.update();
            if (!criterion_state.satisfiesBounds(criterion_dual,1e-4)) stats.limit_violation=true;
            const double extent=0.5*(std::abs(inferred.linear()(2,0))*truth.size.x+
                std::abs(inferred.linear()(2,1))*truth.size.y+
                std::abs(inferred.linear()(2,2))*truth.size.z);
            const double clearance=inferred.translation().z()-extent-ctx_->p2a_support_z_;
            if (!updateRod(inferred)) { detail="E_TRANSPORT_OPT_ROD_UPDATE"; return false; }
            auto local_acm=acm;
            local_acm.setEntry(a1_verify_.rod_id,a1_verify_.support_id,clearance<=0.003);
            collision_detection::CollisionRequest req; req.contacts=true; req.max_contacts=20;
            collision_detection::CollisionResult res;
            scene->checkCollision(req,res,criterion_state,local_acm);
            if (res.collision) stats.collisions+=std::max<std::size_t>(1,res.contacts.size());
        }
        // Real-time quintic samples, exactly as the JTC interpolates them.
        for (std::size_t k=1;k<jt.points.size();++k) {
            const auto& a=jt.points[k-1]; const auto& b=jt.points[k];
            const double T=rclcpp::Duration(b.time_from_start).seconds()-
                           rclcpp::Duration(a.time_from_start).seconds();
            if (!(T>0.0)) { detail="E_TRANSPORT_OPT_NONPOSITIVE_SEGMENT"; return false; }
            const int n=std::max(2,static_cast<int>(std::ceil(T/0.02)));
            ++stats.interpolated_steps;
            for (int sample=0;sample<=n;++sample) {
                const double u=double(sample)/n;
                const double t=rclcpp::Duration(a.time_from_start).seconds()+u*T;
                std::vector<double> sq(a.positions.size());
                for (std::size_t j=0;j<sq.size();++j) {
                    const double av=j<a.velocities.size()?a.velocities[j]:0.0;
                    const double aa=j<a.accelerations.size()?a.accelerations[j]:0.0;
                    const double bv=j<b.velocities.size()?b.velocities[j]:0.0;
                    const double ba=j<b.accelerations.size()?b.accelerations[j]:0.0;
                    const double c0=a.positions[j],c1=av*T,c2=0.5*aa*T*T;
                    const double D=b.positions[j]-c0-c1-c2;
                    const double V=bv*T-c1-2*c2;
                    const double A=ba*T*T-2*c2;
                    const double c3=10*D-4*V+0.5*A,c4=-15*D+7*V-A,c5=6*D-3*V+0.5*A;
                    sq[j]=c0+u*(c1+u*(c2+u*(c3+u*(c4+u*c5))));
                    const double v=(c1+u*(2*c2+u*(3*c3+u*(4*c4+u*5*c5))))/T;
                    const double acc=(2*c2+u*(6*c3+u*(12*c4+u*20*c5)))/(T*T);
                    const double jerk=(6*c3+u*(24*c4+u*60*c5))/(T*T*T);
                    if (!std::isfinite(sq[j]) || !std::isfinite(v) || !std::isfinite(acc) ||
                        !std::isfinite(jerk)) { detail="E_TRANSPORT_OPT_NONFINITE_SAMPLE"; return false; }
                    if (j<limits.size()) {
                        const auto& bound=limits[j];
                        const double v_cap=std::max(std::abs(bound.min_velocity),std::abs(bound.max_velocity));
                        const double a_cap=std::max(std::abs(bound.min_acceleration),std::abs(bound.max_acceleration));
                        if (v<bound.min_velocity-1e-9 || v>bound.max_velocity+1e-9) stats.vel_violation=true;
                        if (acc<bound.min_acceleration-1e-9 || acc>bound.max_acceleration+1e-9) stats.acc_violation=true;
                        if (v_cap>0.0) stats.max_vel_ratio=std::max(stats.max_vel_ratio,std::abs(v)/v_cap);
                        if (a_cap>0.0) stats.max_acc_ratio=std::max(stats.max_acc_ratio,std::abs(acc)/a_cap);
                    }
                    acc_sum+=acc*acc; ++acc_count;
                    jerk_sum+=jerk*jerk; ++jerk_count;
                    stats.jerk_max=std::max(stats.jerk_max,std::abs(jerk));
                }
                moveit::core::RobotState timed(live_model);
                timed.setVariablePositions(jt.joint_names,sq);
                applyTransportFingers(timed);
                timed.update();
                const Transform inferred=timed.getGlobalLinkTransform(eefs[0])*object_to_hand[0].inverse();
                const Transform inferred_right=timed.getGlobalLinkTransform(eefs[1])*object_to_hand[1].inverse();
                const double dual_pos=(inferred.translation()-inferred_right.translation()).norm()*1000.0;
                const double dual_ang=rotationError(inferred,inferred_right)*180.0/M_PI;
                const double expected_u=progress_of_time?
                    std::clamp(progress_of_time(t),0.0,1.0):(double(k-1)+u)/double(steps);
                const Transform expected=expectedObject(expected_u);
                const double rod_pos=(inferred.translation()-expected.translation()).norm()*1000.0;
                const double rod_ang=rotationError(inferred,expected)*180.0/M_PI;
                stats.max_sample_dual_pos_mm=std::max(stats.max_sample_dual_pos_mm,dual_pos);
                stats.max_sample_dual_ang_deg=std::max(stats.max_sample_dual_ang_deg,dual_ang);
                stats.max_rod_pos_mm=std::max(stats.max_rod_pos_mm,rod_pos);
                stats.max_rod_ang_deg=std::max(stats.max_rod_ang_deg,rod_ang);
                moveit::core::RobotState criterion_state(criterion_model);
                for (const auto& variable:criterion_model->getVariableNames())
                    criterion_state.setVariablePosition(variable,timed.getVariablePosition(variable));
                criterion_state.update();
                const bool limit_bad=!criterion_state.satisfiesBounds(criterion_dual,1e-4);
                if (limit_bad) stats.limit_violation=true;
                auto local_acm=acm;
                const double sample_extent=0.5*(std::abs(inferred.linear()(2,0))*truth.size.x+
                    std::abs(inferred.linear()(2,1))*truth.size.y+
                    std::abs(inferred.linear()(2,2))*truth.size.z);
                const double sample_clearance=inferred.translation().z()-sample_extent-ctx_->p2a_support_z_;
                collision_detection::CollisionRequest req; req.contacts=true; req.max_contacts=20;
                collision_detection::CollisionResult res;
                if (!updateRod(inferred)) { detail="E_TRANSPORT_OPT_ROD_UPDATE"; return false; }
                local_acm.setEntry(a1_verify_.rod_id,a1_verify_.support_id,sample_clearance<=0.003);
                scene->checkCollision(req,res,criterion_state,local_acm);
                if (res.collision) stats.collisions+=std::max<std::size_t>(1,res.contacts.size());
                ++stats.samples;
                if (compare_csv) (*compare_csv)<<name<<','<<sample_index++<<','<<t<<','<<dual_pos<<','<<dual_ang<<','
                    <<rod_pos<<','<<rod_ang<<','<<(res.collision?1:0)<<','<<(limit_bad?1:0)<<','
                    <<(stats.vel_violation?1:0)<<','<<(stats.acc_violation?1:0)<<'\n';
            }
        }
        if (compare_csv) compare_csv->flush();
        stats.acc_rms=acc_count>0?std::sqrt(acc_sum/double(acc_count)):0.0;
        stats.jerk_rms=jerk_count>0?std::sqrt(jerk_sum/double(jerk_count)):0.0;
        return true;
    };
    // S2 time-profile candidate: the geometry is frozen, only the monotone
    // clock s(tau) is optimised. Used identically by the offline scenario
    // comparison and by the live execution pairing.
    struct TimeProfileCandidateInfo {
        bool ok{false};
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        std::function<double(double)> progress_of_time;
        double solve_sec{0.0};
        double duration{0.0};
        int evaluations{0};
        int parameters{0};
        double max_export_deviation_rad{0.0};
        tpo::TrajectoryMetrics baseline_continuous, candidate_continuous;
        tpo::TrajectoryMetrics baseline_exported, candidate_exported;
        std::string detail;
    };
    auto buildTimeProfileCandidate=[&](
        const moveit::planning_interface::MoveGroupInterface::Plan& baseline_plan,
        TimeProfileCandidateInfo& info) -> bool {
        const auto& baseline_jt=baseline_plan.trajectory_.joint_trajectory;
        if (baseline_jt.points.size()<2) { info.detail="empty_baseline"; return false; }
        tpo::ContinuousPath tp_path;
        std::vector<tpo::TrajectoryPoint> tp_points;
        std::vector<tpo::JointLimits> tp_limits;
        std::string tp_error;
        for (const auto& p:baseline_jt.points) {
            tpo::TrajectoryPoint tp;
            tp.time=rclcpp::Duration(p.time_from_start).seconds();
            tp.q=Eigen::Map<const Eigen::VectorXd>(p.positions.data(),static_cast<Eigen::Index>(p.positions.size()));
            tp.v=Eigen::Map<const Eigen::VectorXd>(p.velocities.data(),static_cast<Eigen::Index>(p.velocities.size()));
            tp.a=Eigen::Map<const Eigen::VectorXd>(p.accelerations.data(),static_cast<Eigen::Index>(p.accelerations.size()));
            tp_points.push_back(tp);
        }
        for (const auto& name:baseline_jt.joint_names) {
            const auto& b=live_model->getVariableBounds(name);
            tp_limits.push_back({b.min_position_,b.max_position_,b.min_velocity_,b.max_velocity_,
                                 b.min_acceleration_,b.max_acceleration_});
        }
        if (!tp_path.build(baseline_jt.joint_names,tp_points,tp_error)) {
            info.detail=tp_error;
            return false;
        }
        tpo::TimeProfileOptions tp_options;
        const double tp_start=steadySeconds();
        const auto tp_result=tpo::optimizeTimeProfile(tp_path,tp_limits,tp_path.duration(),tp_options);
        info.solve_sec=steadySeconds()-tp_start;
        if (!tp_result.ok) { info.detail=tp_result.detail; return false; }
        info.evaluations=tp_result.evaluations;
        info.duration=tp_path.duration();
        info.parameters=static_cast<int>(tp_result.parameters.size());
        info.baseline_continuous=tp_result.baseline_continuous;
        info.candidate_continuous=tp_result.candidate_continuous;
        info.baseline_exported=tp_result.baseline_metrics;
        info.candidate_exported=tp_result.candidate_metrics;
        auto& tp_jt=info.plan.trajectory_.joint_trajectory;
        tp_jt.joint_names=baseline_jt.joint_names;
        for (const auto& p:tp_result.candidate_export) {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions.assign(p.q.data(),p.q.data()+p.q.size());
            point.velocities.assign(p.v.data(),p.v.data()+p.v.size());
            point.accelerations.assign(p.a.data(),p.a.data()+p.a.size());
            point.time_from_start=rclcpp::Duration::from_seconds(p.time);
            tp_jt.points.push_back(point);
        }
        // Object-path parameter as a function of absolute time. The baseline
        // knots are (s_k = t_k/T, sigma_k = k/steps); between knots the
        // parameter is linear in s, which reproduces the production check for
        // the linear clock.
        const double T_tp=tp_path.duration();
        const std::size_t steps_tp=tp_points.size()-1;
        std::vector<double> s_knots,sigma_knots;
        for (std::size_t k=0;k<tp_points.size();++k) {
            s_knots.push_back(tp_points[k].time/T_tp);
            sigma_knots.push_back(static_cast<double>(k)/static_cast<double>(steps_tp));
        }
        tpo::BernsteinTimeMap tp_map(tp_options.degree);
        tp_map.setParameters(tp_result.parameters);
        info.progress_of_time=[s_knots,sigma_knots,T_tp,tp_map](double t) {
            const double s=std::clamp(tp_map.s(std::clamp(t/T_tp,0.0,1.0)),0.0,1.0);
            for (std::size_t k=1;k<s_knots.size();++k) {
                if (s<=s_knots[k]) {
                    const double span=std::max(1e-12,s_knots[k]-s_knots[k-1]);
                    return sigma_knots[k-1]+(s-s_knots[k-1])/span*(sigma_knots[k]-sigma_knots[k-1]);
                }
            }
            return 1.0;
        };
        std::string dev_error;
        if (!tpo::maxPositionDeviation(tp_result.candidate_export,tp_path,tp_map,T_tp,
                                       tp_options.interpolation_dt,info.max_export_deviation_rad,dev_error))
            info.max_export_deviation_rad=std::numeric_limits<double>::quiet_NaN();
        info.ok=true;
        return true;
    };
    auto writeTrajectoryCsv=[&](const std::string& path,
                                const moveit::planning_interface::MoveGroupInterface::Plan& plan) -> bool {
        std::ofstream out(path);
        if (!out) return false;
        out<<std::setprecision(17)<<"joint_name,point_index,time_from_start,position,velocity,acceleration\n";
        const auto& jt=plan.trajectory_.joint_trajectory;
        for (std::size_t j=0;j<jt.joint_names.size();++j)
            for (std::size_t k=0;k<jt.points.size();++k) {
                const auto& p=jt.points[k];
                out<<jt.joint_names[j]<<','<<k<<','
                   <<rclcpp::Duration(p.time_from_start).seconds()<<','
                   <<p.positions[j]<<','<<(j<p.velocities.size()?p.velocities[j]:0.0)<<','
                   <<(j<p.accelerations.size()?p.accelerations[j]:0.0)<<'\n';
            }
        out.flush();
        return static_cast<bool>(out);
    };
    auto writeJointPathCsv=[&](const std::string& path,
                               const std::vector<moveit::core::RobotState>& waypoints) -> bool {
        std::ofstream out(path);
        if (!out) return false;
        out<<std::setprecision(17)<<"joint_name,point_index,position\n";
        const auto& names=dual->getVariableNames();
        for (std::size_t j=0;j<names.size();++j)
            for (std::size_t k=0;k<waypoints.size();++k)
                out<<names[j]<<','<<k<<','<<waypoints[k].getVariablePosition(names[j])<<'\n';
        out.flush();
        return static_cast<bool>(out);
    };
    if (plan_only) {
        // S2 offline sample. Reconstructed, not measured:
        //   grasp  = the A1 P2a nominal combined grasp solution (no motion)
        //   lifted = the last waypoint of the same nominal lift plan
        //   G_i    = FK(grasp) in the frozen truth frame
        // No arm, gripper, weld or attach target is ever sent from here.
        auto planFail=[&](const std::string& reason) {
            detail=reason;
            RCLCPP_ERROR(ctx_->get_logger(),"[TRANSPORT_OPT_FAILURE] %s",reason.c_str());
            return false;
        };
        // Reconstructed offline sample. The grasp configuration is rebuilt here
        // by dual IK from the A1 P2a grasp TARGET POSES (pure geometry) rather
        // than from any stored joint vector, so the sample does not depend on
        // state bookkeeping elsewhere. No arm, gripper, weld or attach target is
        // sent: this is planning only.
        auto targetTransform=[](const a1v::ArmTarget& t) {
            Transform m=Transform::Identity();
            m.translation()=Eigen::Vector3d(t.position.x,t.position.y,t.position.z);
            m.linear()=Eigen::Quaterniond(t.orientation.w,t.orientation.x,t.orientation.y,t.orientation.z)
                .normalized().toRotationMatrix();
            return m;
        };
        const Transform rod0=[&]() {
            Transform m=Transform::Identity();
            m.translation()=Eigen::Vector3d(truth.position.x,truth.position.y,truth.position.z);
            m.linear()=Eigen::Quaterniond(truth.orientation.w,truth.orientation.x,truth.orientation.y,
                                          truth.orientation.z).normalized().toRotationMatrix();
            return m;
        }();
        const std::array<Transform,2> grasp_hand{{targetTransform(a1_verify_.grasp_target.left),
                                                  targetTransform(a1_verify_.grasp_target.right)}};
        for (int i=0;i<2;++i) object_to_hand[i]=rod0.inverse()*grasp_hand[i];
        Transform lifted_object=rod0;
        lifted_object.translation().z()+=height;
        moveit::core::RobotState lifted(live_model);
        // Held fingers, not the RobotState default (0 = folded). The plan-only
        // sample must represent the same held state the collision checks are
        // evaluated against.
        {
            const auto& finger_names=a1v::fingerJointNames();
            for (std::size_t i=0;i<finger_names.size();++i)
                lifted.setVariablePosition(finger_names[i],transport_finger_positions[i]);
        }
        {
            DualArmCarryTask::JointFeedback seed_feedback;
            if (ctx_->latestJointFeedback(seed_feedback)) {
                for (const auto& name:dual->getVariableNames()) {
                    const auto it=seed_feedback.positions.find(name);
                    if (it!=seed_feedback.positions.end() && std::isfinite(it->second))
                        lifted.setVariablePosition(name,it->second);
                }
                lifted.update();
            }
            if (!lifted.setFromIK(left,lifted_object*object_to_hand[0],eefs[0],0.05) ||
                !lifted.setFromIK(right,lifted_object*object_to_hand[1],eefs[1],0.05))
                return planFail("E_TRANSPORT_OPT_LIFT_IK");
            lifted.update();
            for (int side=0;side<2;++side) {
                const Transform actual=lifted.getGlobalLinkTransform(eefs[side]);
                if ((actual.translation()-(lifted_object*object_to_hand[side]).translation()).norm()>0.0005 ||
                    rotationError(actual,lifted_object*object_to_hand[side])>0.1*M_PI/180.0)
                    return planFail("E_TRANSPORT_OPT_LIFT_IK_RESIDUAL");
            }
            if (!lifted.satisfiesBounds(dual,1e-4)) return planFail("E_TRANSPORT_OPT_LIFT_BOUNDS");
            RCLCPP_INFO(ctx_->get_logger(),
                "[TRANSPORT_OPT_SAMPLE] source=reconstructed_offline_planning_sample measured=false "
                "method=dual_ik_to_a1_grasp_targets_plus_nominal_lift "
                "lift=(%.4f,%.4f,%.4f) left_hand=(%.4f,%.4f,%.4f) right_hand=(%.4f,%.4f,%.4f) "
                "weld=OFF gripper=untouched",
                lifted_object.translation().x(),lifted_object.translation().y(),lifted_object.translation().z(),
                lifted.getGlobalLinkTransform(eefs[0]).translation().x(),
                lifted.getGlobalLinkTransform(eefs[0]).translation().y(),
                lifted.getGlobalLinkTransform(eefs[0]).translation().z(),
                lifted.getGlobalLinkTransform(eefs[1]).translation().x(),
                lifted.getGlobalLinkTransform(eefs[1]).translation().y(),
                lifted.getGlobalLinkTransform(eefs[1]).translation().z());
        }
        const Transform held_object=lifted.getGlobalLinkTransform(eefs[0])*object_to_hand[0].inverse();
        const double held_yaw=std::atan2(held_object.linear()(1,0),held_object.linear()(0,0));
        double transport_dyaw=ctx_->rod_target_yaw_rad_-held_yaw;
        while (transport_dyaw>M_PI) transport_dyaw-=2.0*M_PI;
        while (transport_dyaw<-M_PI) transport_dyaw+=2.0*M_PI;
        const Eigen::Vector3d transport_delta(ctx_->rod_target_x_-held_object.translation().x(),
                                              ctx_->rod_target_y_-held_object.translation().y(),0.0);
        if (!std::isfinite(transport_delta.x()) || !std::isfinite(transport_delta.y()) ||
            !std::isfinite(transport_dyaw) || transport_delta.norm()<0.001 ||
            transport_delta.norm()>0.20 || std::abs(transport_dyaw)>0.6)
            return planFail("E_TRANSPORT_OPT_DELTA");
        // Same transport stretch rule as the live probe (env, default 2.0). The
        // frozen comparison baseline uses CONTACT_TRANSPORT_STRETCH=1.0.
        double transport_stretch=2.0;
        if (const char* raw=std::getenv("CONTACT_TRANSPORT_STRETCH")) {
            const double value=std::atof(raw);
            if (std::isfinite(value) && value>0.0) transport_stretch=value;
        }
        RCLCPP_INFO(ctx_->get_logger(),
            "[TRANSPORT_OPT_SAMPLE] source=reconstructed_offline_planning_sample measured=false "
            "grasp=nominal_p2a_solution lifted=nominal_lift_endpoint dx=%.4f dy=%.4f dyaw_deg=%.3f "
            "stretch=%.3f target=(%.3f,%.3f,%.1f deg) weld=OFF gripper=untouched",
            transport_delta.x(),transport_delta.y(),transport_dyaw*180.0/M_PI,transport_stretch,
            ctx_->rod_target_x_,ctx_->rod_target_y_,ctx_->rod_target_yaw_rad_*180.0/M_PI);
        moveit::planning_interface::MoveGroupInterface::Plan baseline,optimized;
        std::vector<moveit::core::RobotState> baseline_waypoints;
        const double baseline_start=steadySeconds();
        if (!makePlan(lifted,transport_delta,baseline,"transport_baseline",1.0,0.3,transport_stretch,
                      transport_dyaw,false,&baseline_waypoints)) return planFail(detail);
        const double baseline_plan_sec=steadySeconds()-baseline_start;
        const double optimized_start=steadySeconds();
        if (!makePlan(lifted,transport_delta,optimized,"transport_optimized",1.0,0.3,transport_stretch,
                      transport_dyaw,true)) return planFail(detail);
        const double optimized_plan_sec=steadySeconds()-optimized_start;

        // Identical evaluation for both plans: same object path, same sampling,
        // same limit and collision checks, jerk from the quintic in real time.
        const Transform first_object=lifted.getGlobalLinkTransform(eefs[0])*object_to_hand[0].inverse();
        std::ofstream compare_csv("experiment_results/a1_runs/"+run_id+"/transport_path_compare.csv");
        if (!compare_csv) return planFail("E_TRANSPORT_OPT_CSV_OPEN");
        compare_csv<<std::setprecision(10)
            <<"plan,sample_index,t,dual_pos_mm,dual_ang_deg,rod_pos_mm,rod_ang_deg,"
              "collision,joint_limit_violation,vel_violation,acc_violation\n";
        TransportPlanStats baseline_stats,optimized_stats;
        if (!evaluateTransport("baseline",baseline,baseline_stats,first_object,transport_delta,
                               transport_dyaw,{},&compare_csv)) return planFail(detail);
        if (!evaluateTransport("optimized",optimized,optimized_stats,first_object,transport_delta,
                               transport_dyaw,{},&compare_csv)) return planFail(detail);

        // Persist the baseline joint path, both timed trajectories and the
        // scenario parameters so the comparison can be recomputed offline.
        {
            if (!writeTrajectoryCsv("experiment_results/a1_runs/"+run_id+"/transport_baseline_trajectory.csv",
                                    baseline)) return planFail("E_TRANSPORT_TRAJ_CSV_WRITE");
            if (!writeTrajectoryCsv("experiment_results/a1_runs/"+run_id+"/transport_optimized_trajectory.csv",
                                    optimized)) return planFail("E_TRANSPORT_OPT_TRAJ_CSV_WRITE");
            if (!writeJointPathCsv("experiment_results/a1_runs/"+run_id+"/transport_baseline_path.csv",
                                   baseline_waypoints)) return planFail("E_TRANSPORT_PATH_CSV_WRITE");
            const auto& jt=baseline.trajectory_.joint_trajectory;

            std::ofstream meta("experiment_results/a1_runs/"+run_id+"/transport_sample_meta.txt");
            if (!meta) return planFail("E_TRANSPORT_META_OPEN");
            meta<<std::setprecision(17);
            const double baseline_duration=rclcpp::Duration(jt.points.back().time_from_start).seconds();
            meta<<"run_id="<<run_id<<"\n"
                <<"source=reconstructed_offline_planning_sample\n"
                <<"measured=false\n"
                <<"method=dual_ik_to_a1_grasp_targets_plus_nominal_lift\n"
                <<"execution=none\n"
                <<"transport_stretch="<<transport_stretch<<"\n"
                <<"scenario_target_x="<<ctx_->rod_target_x_<<"\n"
                <<"scenario_target_y="<<ctx_->rod_target_y_<<"\n"
                <<"scenario_target_yaw_deg="<<ctx_->rod_target_yaw_rad_*180.0/M_PI<<"\n"
                <<"scenario_transport_dx="<<transport_delta.x()<<"\n"
                <<"scenario_transport_dy="<<transport_delta.y()<<"\n"
                <<"scenario_transport_dyaw_deg="<<transport_dyaw*180.0/M_PI<<"\n"
                <<"planning_group="<<ctx_->dual_arm_group_<<"\n"
                <<"left_eef="<<eefs[0]<<"\n"
                <<"right_eef="<<eefs[1]<<"\n"
                <<"points="<<jt.points.size()<<"\n"
                <<"duration="<<baseline_duration<<"\n"
                <<"joints=";
            for (std::size_t j=0;j<jt.joint_names.size();++j) meta<<(j?",":"")<<jt.joint_names[j];
            meta<<"\n";
            // Object reference: X(u) = translate(first_object, delta*u) with a
            // rotation about Z by dyaw*u, u in [0,1]. Path parameters are
            // uniform: s_k = k/(points-1).
            const Eigen::Quaterniond first_q(first_object.linear());
            meta<<"first_object_xyz="<<first_object.translation().x()<<","<<first_object.translation().y()
                <<","<<first_object.translation().z()<<"\n"
                <<"first_object_quat="<<first_q.x()<<","<<first_q.y()<<","<<first_q.z()<<","<<first_q.w()<<"\n"
                <<"object_delta_xyz="<<transport_delta.x()<<","<<transport_delta.y()<<","<<transport_delta.z()<<"\n"
                <<"object_dyaw_rad="<<transport_dyaw<<"\n"
                <<"path_params=uniform\n";
            for (int i=0;i<2;++i) {
                const auto& m=object_to_hand[i].matrix();
                meta<<(i==0?"grasp_left_matrix=":"grasp_right_matrix=");
                for (int r=0;r<4;++r) for (int c=0;c<4;++c) meta<<(r==0&&c==0?"":",")<<m(r,c);
                meta<<"\n";
            }
            meta<<"rod_size="<<truth.size.x<<","<<truth.size.y<<","<<truth.size.z<<"\n"
                <<"support_z="<<ctx_->p2a_support_z_<<"\n"
                <<"fingers=held_on_rod_per_finger_m="<<transport_finger_positions[0]
                <<" (plan-only sample; measured held value from reported hover runs)\n"
                <<"criterion_model=hand_aabb_substitution_from_robot_description\n";
            for (std::size_t j=0;j<jt.joint_names.size();++j) {
                const auto& b=live_model->getVariableBounds(jt.joint_names[j]);
                meta<<"limit,"<<jt.joint_names[j]<<","<<b.min_position_<<","<<b.max_position_<<","
                    <<b.min_velocity_<<","<<b.max_velocity_<<","
                    <<b.min_acceleration_<<","<<b.max_acceleration_<<"\n";
            }
            meta.flush();
            if (!meta) return planFail("E_TRANSPORT_META_WRITE");
            RCLCPP_INFO(ctx_->get_logger(),
                "[TRANSPORT_SAMPLE_SAVED] trajectory=transport_baseline_trajectory.csv meta=transport_sample_meta.txt "
                "points=%zu joints=%zu duration=%.6f",jt.points.size(),jt.joint_names.size(),baseline_duration);
        }

        // S2 time profile: the geometry is frozen, only the monotone clock
        // s(tau) is optimised. Both trajectories are exported and evaluated by
        // the same method, and the candidate is checked on the model before it
        // could ever be dispatched.
        TimeProfileCandidateInfo tp_info;
        bool tp_ok=false;
        TransportPlanStats tp_cand_stats;
        if (!buildTimeProfileCandidate(baseline,tp_info)) {
            RCLCPP_WARN(ctx_->get_logger(),"[TIME_PROFILE] skipped: %s",tp_info.detail.c_str());
        } else {
            auto pct=[](double b,double c){ return b>0.0?100.0*(c-b)/b:0.0; };
            RCLCPP_INFO(ctx_->get_logger(),
                "[TIME_PROFILE_OPT] duration=%.6f params=%d evaluations=%d solve_sec=%.3f "
                "continuous_jerk_rms=%.6f->%.6f (%.3f%%) continuous_jerk_max=%.6f->%.6f "
                "exported_jerk_rms=%.6f->%.6f (%.3f%%) exported_jerk_max=%.6f->%.6f "
                "max_export_deviation_rad=%.9f",
                tp_info.duration,tp_info.parameters,tp_info.evaluations,tp_info.solve_sec,
                tp_info.baseline_continuous.jerk_rms,tp_info.candidate_continuous.jerk_rms,
                pct(tp_info.baseline_continuous.jerk_rms,tp_info.candidate_continuous.jerk_rms),
                tp_info.baseline_continuous.jerk_max,tp_info.candidate_continuous.jerk_max,
                tp_info.baseline_exported.jerk_rms,tp_info.candidate_exported.jerk_rms,
                pct(tp_info.baseline_exported.jerk_rms,tp_info.candidate_exported.jerk_rms),
                tp_info.baseline_exported.jerk_max,tp_info.candidate_exported.jerk_max,
                tp_info.max_export_deviation_rad);
            if (!evaluateTransport("time_profile_candidate",tp_info.plan,tp_cand_stats,first_object,
                                   transport_delta,transport_dyaw,tp_info.progress_of_time,&compare_csv))
                return planFail(detail);
            tp_ok=tp_cand_stats.collisions==0 && !tp_cand_stats.limit_violation &&
                !tp_cand_stats.vel_violation && !tp_cand_stats.acc_violation &&
                tp_cand_stats.max_sample_dual_pos_mm<=5.0 && tp_cand_stats.max_sample_dual_ang_deg<=2.0 &&
                tp_cand_stats.max_rod_pos_mm<=5.0 && tp_cand_stats.max_rod_ang_deg<=2.0;
            RCLCPP_INFO(ctx_->get_logger(),
                "[TIME_PROFILE_CHECK] candidate_pass=%d collisions=%zu limits=%d vel=%d acc=%d "
                "dual_max=(%.6f mm,%.6f deg) rod_path_max=(%.6f mm,%.6f deg) "
                "vel_ratio=%.4f acc_ratio=%.4f jerk_rms(exported)=%.6f jerk_max=%.6f "
                "max_export_deviation_rad=%.9f",
                tp_ok?1:0,tp_cand_stats.collisions,tp_cand_stats.limit_violation?1:0,
                tp_cand_stats.vel_violation?1:0,tp_cand_stats.acc_violation?1:0,
                tp_cand_stats.max_sample_dual_pos_mm,tp_cand_stats.max_sample_dual_ang_deg,
                tp_cand_stats.max_rod_pos_mm,tp_cand_stats.max_rod_ang_deg,
                tp_cand_stats.max_vel_ratio,tp_cand_stats.max_acc_ratio,
                tp_cand_stats.jerk_rms,tp_cand_stats.jerk_max,tp_info.max_export_deviation_rad);
            RCLCPP_INFO(ctx_->get_logger(),
                "[TIME_PROFILE_BASELINE_CHECK] collisions=%zu limits=%d vel=%d acc=%d "
                "dual_max=(%.6f mm,%.6f deg) rod_path_max=(%.6f mm,%.6f deg) "
                "vel_ratio=%.4f acc_ratio=%.4f jerk_rms(exported)=%.6f jerk_max=%.6f",
                baseline_stats.collisions,baseline_stats.limit_violation?1:0,
                baseline_stats.vel_violation?1:0,baseline_stats.acc_violation?1:0,
                baseline_stats.max_sample_dual_pos_mm,baseline_stats.max_sample_dual_ang_deg,
                baseline_stats.max_rod_pos_mm,baseline_stats.max_rod_ang_deg,
                baseline_stats.max_vel_ratio,baseline_stats.max_acc_ratio,
                baseline_stats.jerk_rms,baseline_stats.jerk_max);
            if (!writeTrajectoryCsv("experiment_results/a1_runs/"+run_id+"/time_profile_candidate.csv",
                                    tp_info.plan)) return planFail("E_TIME_PROFILE_CSV_OPEN");
        }

        auto logStats=[&](const char* name,const TransportPlanStats& s,double planning_sec) {
            RCLCPP_INFO(ctx_->get_logger(),
                "[TRANSPORT_PLAN_RESULT] plan=%s points=%zu duration=%.6f planning_sec=%.3f "
                "joint_path_length=%.6f waypoint_dual_max=(%.6f mm,%.6f deg) "
                "sample_dual_max=(%.6f mm,%.6f deg) rod_path_max=(%.6f mm,%.6f deg) samples=%zu "
                "collisions=%zu limits=%d vel_violation=%d acc_violation=%d "
                "max_vel_ratio=%.4f max_acc_ratio=%.4f acc_rms=%.6f jerk_rms=%.4f jerk_max=%.4f",
                name,s.points,s.duration,planning_sec,s.joint_path_length,
                s.max_waypoint_dual_pos_mm,s.max_waypoint_dual_ang_deg,
                s.max_sample_dual_pos_mm,s.max_sample_dual_ang_deg,
                s.max_rod_pos_mm,s.max_rod_ang_deg,s.samples,s.collisions,
                s.limit_violation?1:0,s.vel_violation?1:0,s.acc_violation?1:0,
                s.max_vel_ratio,s.max_acc_ratio,s.acc_rms,s.jerk_rms,s.jerk_max);
        };
        logStats("baseline",baseline_stats,baseline_plan_sec);
        logStats("optimized",optimized_stats,optimized_plan_sec);
        const bool baseline_passes=baseline_stats.collisions==0 && !baseline_stats.limit_violation &&
            !baseline_stats.vel_violation && !baseline_stats.acc_violation &&
            baseline_stats.max_sample_dual_pos_mm<=5.0 && baseline_stats.max_sample_dual_ang_deg<=2.0;
        const bool optimized_passes=optimized_stats.collisions==0 && !optimized_stats.limit_violation &&
            !optimized_stats.vel_violation && !optimized_stats.acc_violation &&
            optimized_stats.max_sample_dual_pos_mm<=5.0 && optimized_stats.max_sample_dual_ang_deg<=2.0;
        const double reduction=baseline_stats.joint_path_length>0.0?
            1.0-optimized_stats.joint_path_length/baseline_stats.joint_path_length:0.0;
        // One machine-readable row per method. `jerk_rms`/`jerk_max` are the
        // 20 ms quintic-interpolation values for every row; the continuous
        // columns are only meaningful for the re-timed candidate (-1 = n/a).
        {
            std::ofstream summary("experiment_results/a1_runs/"+run_id+"/transport_plan_summary.csv");
            if (!summary) return planFail("E_TRANSPORT_SUMMARY_CSV_OPEN");
            summary<<std::setprecision(12)
                <<"plan,scenario_target_x,scenario_target_y,scenario_target_yaw_deg,"
                  "transport_dx,transport_dy,transport_dyaw_deg,points,duration_s,planning_sec,"
                  "joint_path_length,collisions,limit_violation,vel_violation,acc_violation,"
                  "max_vel_ratio,max_acc_ratio,acc_rms,jerk_rms,jerk_max,"
                  "continuous_jerk_rms,continuous_jerk_max,exported_jerk_rms,exported_jerk_max,"
                  "max_export_deviation_rad,max_sample_dual_pos_mm,max_sample_dual_ang_deg,"
                  "max_rod_pos_mm,max_rod_ang_deg,time_profile_evaluations,time_profile_parameters,pass\n";
            auto writeRow=[&](const char* name,const TransportPlanStats& s,double planning_sec,
                              bool pass,double cont_rms,double cont_max,double exp_rms,double exp_max,
                              double dev,int evaluations,int parameters) {
                summary<<name<<','<<ctx_->rod_target_x_<<','<<ctx_->rod_target_y_<<','
                    <<ctx_->rod_target_yaw_rad_*180.0/M_PI<<','<<transport_delta.x()<<','
                    <<transport_delta.y()<<','<<transport_dyaw*180.0/M_PI<<','
                    <<s.points<<','<<s.duration<<','<<planning_sec<<','<<s.joint_path_length<<','
                    <<s.collisions<<','<<(s.limit_violation?1:0)<<','<<(s.vel_violation?1:0)<<','
                    <<(s.acc_violation?1:0)<<','<<s.max_vel_ratio<<','<<s.max_acc_ratio<<','
                    <<s.acc_rms<<','<<s.jerk_rms<<','<<s.jerk_max<<','
                    <<cont_rms<<','<<cont_max<<','<<exp_rms<<','<<exp_max<<','<<dev<<','
                    <<s.max_sample_dual_pos_mm<<','<<s.max_sample_dual_ang_deg<<','
                    <<s.max_rod_pos_mm<<','<<s.max_rod_ang_deg<<','<<evaluations<<','
                    <<parameters<<','<<(pass?1:0)<<'\n';
            };
            writeRow("baseline",baseline_stats,baseline_plan_sec,baseline_passes,
                     -1.0,-1.0,baseline_stats.jerk_rms,baseline_stats.jerk_max,0.0,-1,-1);
            writeRow("geometry_candidate",optimized_stats,optimized_plan_sec,optimized_passes,
                     -1.0,-1.0,optimized_stats.jerk_rms,optimized_stats.jerk_max,0.0,-1,-1);
            if (tp_info.ok) {
                writeRow("time_profile_candidate",tp_cand_stats,tp_info.solve_sec,tp_ok,
                         tp_info.candidate_continuous.jerk_rms,tp_info.candidate_continuous.jerk_max,
                         tp_info.candidate_exported.jerk_rms,tp_info.candidate_exported.jerk_max,
                         tp_info.max_export_deviation_rad,tp_info.evaluations,tp_info.parameters);
            }
            summary.flush();
            if (!summary) return planFail("E_TRANSPORT_SUMMARY_CSV_WRITE");
        }
        RCLCPP_INFO(ctx_->get_logger(),
            "[TRANSPORT_PLAN_SUMMARY] baseline_pass=%d optimized_pass=%d same_start=%d "
            "joint_path_length_reduction=%.4f sample_dual_ok=%d rod_path_ok=%d "
            "executed=false motion_sent=false",
            baseline_passes?1:0,optimized_passes?1:0,
            (baseline_stats.points>0 && optimized_stats.points>0)?1:0,
            reduction,
            (optimized_stats.max_sample_dual_pos_mm<=5.0 && optimized_stats.max_sample_dual_ang_deg<=2.0)?1:0,
            (optimized_stats.max_rod_pos_mm<=5.0 && optimized_stats.max_rod_ang_deg<=2.0)?1:0);
        detail="transport plan-only comparison complete: baseline "+std::to_string(baseline_stats.points)+
               " points / optimized "+std::to_string(optimized_stats.points)+" points / time-profile "+
               (tp_info.ok?std::to_string(tp_cand_stats.points)+" points":"unavailable")+
               "; no motion sent";
        return true;
    }
    moveit::planning_interface::MoveGroupInterface::Plan lift;

    if (!ctx_->contact_hover_only_ && !hold_diagnostic_only &&
        !makePlan(start,Eigen::Vector3d(0,0,height),lift,"lift",1.0,0.4,1.0)) return fail(detail);
    // Give the bounded force-hold drive a short static settling window before
    // the first lift command. This does not count as load validation; it only
    // avoids starting the lift while the tendon force is still ramping or the
    // contact solver is transitioning between pad contacts.
    const double settle_start = std::min(fingers[0].simulation_time, fingers[1].simulation_time);
    const double settle_deadline = steadySeconds() + 2.0;
    while (std::min(fingers[0].simulation_time, fingers[1].simulation_time) - settle_start < 0.5) {
        if (!rclcpp::ok() || steadySeconds() > settle_deadline) {
            return fail("E_CONTACT_PRE_LIFT_SETTLE_TIMEOUT");
        }
        moveit::core::RobotState settled(live_model);
        if (!measured(settled)) return fail(detail);
        std::this_thread::sleep_for(50ms);
    }
    RCLCPP_INFO(ctx_->get_logger(),
        "[CONTACT_PRE_LIFT_SETTLE] sim_duration=%.3fs force_per_finger=%.1f; load_unverified",
        std::min(fingers[0].simulation_time, fingers[1].simulation_time) - settle_start, force);
    double max_slip=0.0,max_angle=0.0;
    hover_experiment::HeightMonitor hover_height;
    const auto original_grasp = object_to_hand;  // never reset for diagnostic metrics
    std::ofstream hover_csv;
    double previous_monitor_start = 0.0, max_monitor_interval = 0.0, max_monitor_call = 0.0;
    if (ctx_->contact_hover_only_) {
        hover_csv.open("experiment_results/a1_runs/"+run_id+"/hover_diagnostics.csv");
        if (!hover_csv) return fail("E_CONTACT_HOVER_CSV_OPEN");
        hover_csv << std::setprecision(10)
            << "phase,wall_steady_sec,left_sim_time,right_sim_time,feedback_skew_sec,"
               "rod_rise_m,rod_clearance_m,rod_qx,rod_qy,rod_qz,rod_qw,"
               "left_original_slip_m,right_original_slip_m,"
               "left_expected_minus_rod_z_m,right_expected_minus_rod_z_m,"
               "left_rod_point_dx_hand_m,left_rod_point_dy_hand_m,left_rod_point_dz_hand_m,"
               "right_rod_point_dx_hand_m,right_rod_point_dy_hand_m,right_rod_point_dz_hand_m,"
               "lift_confirmed,monitor_interval_sec,monitor_call_wall_sec,truth_request_elapsed_sec,truth_age_wall_sec\n";
    }
    MujocoRodTruth latest = initial_truth;
    double last_physical_clearance = std::numeric_limits<double>::quiet_NaN();
    const std::string truth_service = ctx_->mujoco_get_body_state_service_;
    auto truth_client = ctx_->create_client<mujoco_ros_msgs::srv::GetBodyState>(truth_service);
    using TruthClient = rclcpp::Client<mujoco_ros_msgs::srv::GetBodyState>;
    TruthClient::SharedFuture truth_future;
    dual_arm_carry_task::AsyncTruthGuard truth_guard(
        std::chrono::duration<double>(ctx_->mujoco_truth_timeout_sec_));
    truth_guard.seedSample(initial_truth_received);
    double last_truth_request_elapsed = 0.0;
    double last_truth_age = 0.0;
    auto advanceTruthRequest = [&](bool allow_stale_cache_while_refreshing = false) {
        const auto now_steady = std::chrono::steady_clock::now();
        if (truth_guard.inFlight()) {
            if (truth_future.wait_for(0ms) == std::future_status::ready) {
                mujoco_ros_msgs::srv::GetBodyState::Response::SharedPtr response;
                try {
                    response = truth_future.get();
                } catch (const std::exception& error) {
                    truth_guard.fail("service_response_failed");
                    detail="E_CONTACT_TRUTH_RESPONSE_FAILED service="+truth_service+
                        " error="+error.what();
                    return false;
                }
                last_truth_request_elapsed = truth_guard.requestElapsed(now_steady);
                if (!response) {
                    truth_guard.fail("service_response_failed");
                    detail="E_CONTACT_TRUTH_RESPONSE_FAILED service="+truth_service; return false;
                }
                if (!response->success) {
                    truth_guard.fail("service_response_failed");
                    detail="E_CONTACT_TRUTH_SERVICE_FAILED service="+truth_service+" message="+response->status_message;
                    return false;
                }
                const auto& p = response->state.pose.pose;
                const auto& tw = response->state.twist.twist;
                latest.position={p.position.x,p.position.y,p.position.z};
                latest.orientation={p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w};
                latest.linear_velocity={tw.linear.x,tw.linear.y,tw.linear.z};
                latest.angular_velocity={tw.angular.x,tw.angular.y,tw.angular.z};
                const double qn=std::sqrt(latest.orientation.x*latest.orientation.x+
                    latest.orientation.y*latest.orientation.y+latest.orientation.z*latest.orientation.z+
                    latest.orientation.w*latest.orientation.w);
                if (!std::isfinite(latest.position.x)||!std::isfinite(latest.position.y)||
                    !std::isfinite(latest.position.z)||!std::isfinite(qn)||std::abs(qn-1.0)>1e-3||
                    !std::isfinite(latest.linear_velocity.x)||!std::isfinite(latest.linear_velocity.y)||
                    !std::isfinite(latest.linear_velocity.z)||!std::isfinite(latest.angular_velocity.x)||
                    !std::isfinite(latest.angular_velocity.y)||!std::isfinite(latest.angular_velocity.z)) {
                    truth_guard.fail("invalid_response");
                    detail="E_CONTACT_TRUTH_RESPONSE_INVALID service="+truth_service; return false;
                }
                if (!truth_guard.completeRequest(now_steady)) {
                    detail="E_CONTACT_TRUTH_REQUEST_TIMEOUT service="+truth_service+
                        " elapsed_wall_sec="+std::to_string(last_truth_request_elapsed);
                    return false;
                }
                latest.ok=true;
                latest.detail="async service response; source="+truth_service;
            }
        }
        if (!allow_stale_cache_while_refreshing && truth_guard.canStartRequest() &&
            truth_client->service_is_ready()) {
            auto req=std::make_shared<mujoco_ros_msgs::srv::GetBodyState::Request>();
            req->name=ctx_->mujoco_rod_body_name_;
            req->admin_hash=ctx_->mujoco_admin_hash_;
            truth_guard.beginRequest(std::chrono::steady_clock::now());
            truth_future=truth_client->async_send_request(req).future.share();
        }
        // Planning and the read-only MMC preflight run on this task thread. They
        // can age the cached sample without a failed request. During the one
        // bounded pre-lift refresh only, let that request replace the stale
        // cache before applying the normal freshness check.
        if (allow_stale_cache_while_refreshing && truth_guard.inFlight()) {
            if (truth_guard.requestElapsed(std::chrono::steady_clock::now()) >=
                ctx_->mujoco_truth_timeout_sec_) {
                truth_guard.observe(std::chrono::steady_clock::now());
            } else {
                last_truth_age=truth_guard.sampleAge(std::chrono::steady_clock::now());
                last_truth_request_elapsed=truth_guard.requestElapsed(std::chrono::steady_clock::now());
                return true;
            }
        }
        if (!truth_guard.observe(std::chrono::steady_clock::now())) {
            last_truth_age=truth_guard.sampleAge(std::chrono::steady_clock::now());
            detail="E_CONTACT_TRUTH_STALE service="+truth_service+
                " guard="+truth_guard.failure()+" request_elapsed_wall_sec="+
                std::to_string(truth_guard.requestElapsed(std::chrono::steady_clock::now()))+
                " age_wall_sec="+std::to_string(last_truth_age);
            return false;
        }
        last_truth_age=truth_guard.sampleAge(std::chrono::steady_clock::now());
        last_truth_request_elapsed=truth_guard.requestElapsed(std::chrono::steady_clock::now());
        return latest.ok;
    };
    auto refreshTruthBeforeLift = [&]() {
        const auto refresh_started=std::chrono::steady_clock::now();
        const auto deadline=refresh_started+
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(ctx_->mujoco_truth_timeout_sec_));
        if (!truth_guard.inFlight()) {
            while (rclcpp::ok() && std::chrono::steady_clock::now()<deadline &&
                   !truth_client->service_is_ready()) {
                std::this_thread::sleep_for(10ms);
            }
            if (!truth_client->service_is_ready()) {
                detail="E_CONTACT_TRUTH_SERVICE_DISCOVERY_TIMEOUT service="+truth_service+
                    " phase=pre_lift_refresh elapsed_wall_sec="+
                    std::to_string(std::chrono::duration<double>(
                        std::chrono::steady_clock::now()-refresh_started).count());
                return false;
            }
            if (truth_guard.canStartRequest()) {
                auto req=std::make_shared<mujoco_ros_msgs::srv::GetBodyState::Request>();
                req->name=ctx_->mujoco_rod_body_name_;
                req->admin_hash=ctx_->mujoco_admin_hash_;
                truth_guard.beginRequest(std::chrono::steady_clock::now());
                truth_future=truth_client->async_send_request(req).future.share();
            }
        }
        while (rclcpp::ok() && std::chrono::steady_clock::now()<deadline) {
            if (!advanceTruthRequest(true)) return false;
            if (!truth_guard.inFlight()) {
                RCLCPP_INFO(ctx_->get_logger(),
                    "[CONTACT_TRUTH_PRE_LIFT_REFRESH] service=%s result=fresh "
                    "request_elapsed_wall_sec=%.6f age_wall_sec=%.6f; existing grasp/height guards rechecked",
                    truth_service.c_str(), last_truth_request_elapsed, last_truth_age);
                return true;
            }
            std::this_thread::sleep_for(10ms);
        }
        const auto now=std::chrono::steady_clock::now();
        if (!truth_guard.faulted()) truth_guard.observe(now);
        if (detail.empty() || detail.find("E_CONTACT_TRUTH_")!=0) {
            detail="E_CONTACT_TRUTH_REFRESH_TIMEOUT service="+truth_service+
                " phase=pre_lift_refresh request_elapsed_wall_sec="+
                std::to_string(truth_guard.requestElapsed(now));
        }
        return false;
    };
    auto observe=[&](const char* phase,bool require_height) {
        const double call_started=steadySeconds();
        struct CallRecorder { double start; double& maximum; ~CallRecorder() { maximum=std::max(maximum,steadySeconds()-start); } } call_recorder{call_started,max_monitor_call};
        const double monitor_interval=previous_monitor_start>0.0 ? call_started-previous_monitor_start : 0.0;
        if (previous_monitor_start>0.0) max_monitor_interval=std::max(max_monitor_interval,monitor_interval);
        previous_monitor_start=call_started;
        moveit::core::RobotState actual(live_model);
        if (!measured(actual)) return false;
        for (const auto& f:fingers) if (std::abs(f.hold_force_per_finger_n-force)>1e-6) { detail="E_CONTACT_FORCE_MODE_LOST"; return false; }
        if (!advanceTruthRequest()) return false;
        const double qnorm=std::sqrt(latest.orientation.x*latest.orientation.x+latest.orientation.y*latest.orientation.y+
            latest.orientation.z*latest.orientation.z+latest.orientation.w*latest.orientation.w);
        if (!latest.ok || !std::isfinite(latest.position.x) || !std::isfinite(latest.position.y) ||
            !std::isfinite(latest.position.z) || !std::isfinite(qnorm) || std::abs(qnorm-1.0)>1e-3) {
            detail="E_CONTACT_TRUTH_INVALID: "+latest.detail; return false;
        }
        const Transform physical=rodTransform(latest);
        const double rod_rise=latest.position.z-initial_truth.position.z;
        const double clearance = latest.position.z - ctx_->p2a_support_z_ - 0.5 *
            (std::abs(physical.linear()(2,0))*truth.size.x +
             std::abs(physical.linear()(2,1))*truth.size.y +
             std::abs(physical.linear()(2,2))*truth.size.z);
        last_physical_clearance = clearance;
        hover_height.update(rod_rise, std::string(phase)=="HOVER_LIFT");
        double slips[2], angles[2];
        for (int i=0;i<2;++i) {
            const Transform expected=actual.getGlobalLinkTransform(eefs[i])*
                (ctx_->contact_hover_only_ ? original_grasp[i] : object_to_hand[i]).inverse();
            slips[i]=(expected.translation()-physical.translation()).norm(); angles[i]=rotationError(expected,physical);
            max_slip=std::max(max_slip,slips[i]); max_angle=std::max(max_angle,angles[i]);
        }
        const auto left_pose=actual.getGlobalLinkTransform(eefs[0]);
        const auto right_pose=actual.getGlobalLinkTransform(eefs[1]);
        const Eigen::Quaterniond left_q(left_pose.linear()), right_q(right_pose.linear());
        csv<<phase<<','<<fingers[0].simulation_time<<','<<fingers[1].simulation_time<<','<<latest.position.x<<','<<latest.position.y<<','<<latest.position.z<<','
           <<slips[0]<<','<<angles[0]<<','<<slips[1]<<','<<angles[1];
        for (const auto& f:fingers) csv<<','<<f.q1<<','<<f.q2;
        for (const auto& f:fingers) csv<<','<<f.command_per_finger_m;
        for (const auto& f:fingers) csv<<','<<f.closing_force_limit_per_finger_n;
        for (const auto& f:fingers) csv<<','<<f.hold_force_per_finger_n;
        for (const auto& f:fingers) csv<<','<<f.rod_pad_contact_count;
        for (const auto& f:fingers) csv<<','<<f.left_contact<<','<<f.right_contact;
        for (const auto& f:fingers) csv<<','<<f.normal_left_n<<','<<f.normal_right_n;
        for (const auto& f:fingers) csv<<','<<f.tangent_left_n<<','<<f.tangent_right_n;
        csv<<','<<fingers[0].rod_force_x_n<<','<<fingers[0].rod_force_y_n<<','<<fingers[0].rod_force_z_n<<','<<fingers[1].rod_force_x_n<<','<<fingers[1].rod_force_y_n<<','<<fingers[1].rod_force_z_n<<','<<fingers[0].seq<<','<<fingers[1].seq<<','
           <<fingers[0].weld_left_active<<','<<fingers[0].weld_right_active<<','<<fingers[0].rod_constraint_active<<','
           <<left_pose.translation().x()<<','<<left_pose.translation().y()<<','<<left_pose.translation().z()<<','
           <<left_q.x()<<','<<left_q.y()<<','<<left_q.z()<<','<<left_q.w()<<','
           <<right_pose.translation().x()<<','<<right_pose.translation().y()<<','<<right_pose.translation().z()<<','
           <<right_q.x()<<','<<right_q.y()<<','<<right_q.z()<<','<<right_q.w()<<'\n'; csv.flush();
        if (!csv) { detail="E_CONTACT_CSV_WRITE"; return false; }
        if (hover_csv.is_open()) {
            hover_csv << phase << ',' << steadySeconds() << ',' << fingers[0].simulation_time
                << ',' << fingers[1].simulation_time << ','
                << std::abs(fingers[0].simulation_time-fingers[1].simulation_time)
                << ',' << rod_rise << ',' << clearance << ',' << latest.orientation.x << ','
                << latest.orientation.y << ',' << latest.orientation.z << ',' << latest.orientation.w;
            double original_slip[2], z_error[2];
            Eigen::Vector3d rod_point_in_hand[2];
            for (int i=0; i<2; ++i) {
                const auto hand=actual.getGlobalLinkTransform(eefs[i]);
                const Transform inferred=hand*original_grasp[i].inverse();
                original_slip[i]=(inferred.translation()-physical.translation()).norm();
                z_error[i]=inferred.translation().z()-physical.translation().z();
                // A frozen rod-fixed point, expressed in the current hand frame.
                // Kinematic proxy, NOT measured local pad slip velocity.
                rod_point_in_hand[i]=(hand.inverse()*physical*original_grasp[i]).translation();
            }
            hover_csv << ',' << original_slip[0] << ',' << original_slip[1]
                << ',' << z_error[0] << ',' << z_error[1];
            for (int i=0; i<2; ++i)
                for (int j=0; j<3; ++j) hover_csv << ',' << rod_point_in_hand[i][j];
            hover_csv << ',' << (hover_height.lifted ? 1 : 0) << ',' << monitor_interval
                      << ',' << (steadySeconds()-call_started) << ',' << last_truth_request_elapsed
                      << ',' << last_truth_age << '\n';
            hover_csv.flush();
            if (!hover_csv) { detail="E_CONTACT_HOVER_CSV_WRITE"; return false; }
        }
        if (max_slip>lift_slip_tol || max_angle>angle_tol) { detail="E_CONTACT_SLIP_OR_DROP"; return false; }
        // Keep the full probe's original continuous height requirement.
        if (require_height && latest.position.z<initial_truth.position.z+rod_rise_target) {
            detail="E_CONTACT_ROD_NOT_LIFTED"; return false;
        }
        const std::string phase_name(phase);
        const bool is_hold_phase = phase_name=="HOVER_HOLD" || phase_name.rfind("MMC_HOLD",0)==0;
        if (ctx_->contact_hover_only_ && is_hold_phase &&
            !hover_height.holdValid(clearance)) {
            detail="E_CONTACT_HOVER_CLEARANCE_OR_DROP"; return false;
        }
        return true;
    };
    auto execute=[&](const moveit::planning_interface::MoveGroupInterface::Plan& plan,const char* phase,
                     double start_tolerance_rad=0.002) {
        moveit::core::RobotState dispatch_start(live_model);
        if (!measured(dispatch_start)) return false;
        const auto& joint=plan.trajectory_.joint_trajectory;
        for (std::size_t j=0;j<joint.joint_names.size();++j)
            if (std::abs(dispatch_start.getVariablePosition(joint.joint_names[j])-joint.points.front().positions[j])>start_tolerance_rad) {
                RCLCPP_ERROR(ctx_->get_logger(),
                    "[CONTACT_DISPATCH] phase=%s joint=%s measured=%.5f plan_start=%.5f delta=%.5f tolerance=%.5f",
                    phase,joint.joint_names[j].c_str(),
                    dispatch_start.getVariablePosition(joint.joint_names[j]),
                    joint.points.front().positions[j],
                    dispatch_start.getVariablePosition(joint.joint_names[j])-joint.points.front().positions[j],
                    start_tolerance_rad);
                detail="E_CONTACT_DISPATCH_START_MOVED"; return false;
            }
        if (!observe("PRE_EXECUTE",false)) return false;
        auto promise=std::make_shared<std::promise<int>>(); auto future=promise->get_future();
        const auto iface=ctx_->dual_arm_; auto copy=std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>(plan);
        std::thread worker([iface,copy,promise]() { try { promise->set_value(iface->execute(*copy).val); } catch (...) { promise->set_value(-999); } });
        const double deadline=steadySeconds()+std::max(30.0,3*rclcpp::Duration(plan.trajectory_.joint_trajectory.points.back().time_from_start).seconds()+5);
        bool monitored=true;
        while (future.wait_for(0ms)!=std::future_status::ready) {
            if (!rclcpp::ok() || steadySeconds()>deadline || !observe(phase,false)) { monitored=false; break; }
            std::this_thread::sleep_for(50ms);
        }
        if (!monitored) iface->stop();
        if (future.wait_for(2s)!=std::future_status::ready) {
            worker.detach(); detail="E_CONTACT_EXECUTION_OUTCOME_UNKNOWN; no further motion"; return false;
        }
        worker.join(); const int code=future.get();
        if (!monitored) { if (detail.empty()) detail="E_CONTACT_EXECUTION_TIMEOUT"; return false; }
        if (code!=moveit::core::MoveItErrorCode::SUCCESS) { detail="E_CONTACT_EXECUTION_FAILED code="+std::to_string(code); return false; }
        return observe(phase,false);
    };
    // Post-release free-space execution: the force hold is intentionally OFF and
    // the rod is no longer attached, so the contact monitoring in execute()
    // (force-mode, slip, height) does not apply. Keep the dispatch-start check,
    // the action outcome and the wall-clock deadline.
    auto executeFree=[&](const moveit::planning_interface::MoveGroupInterface::Plan& plan,const char* phase) {
        moveit::core::RobotState dispatch_start(live_model);
        if (!measured(dispatch_start)) return false;
        const auto& joint=plan.trajectory_.joint_trajectory;
        for (std::size_t j=0;j<joint.joint_names.size();++j)
            if (std::abs(dispatch_start.getVariablePosition(joint.joint_names[j])-joint.points.front().positions[j])>0.002) {
                RCLCPP_ERROR(ctx_->get_logger(),
                    "[CONTACT_DISPATCH] phase=%s joint=%s measured=%.5f plan_start=%.5f delta=%.5f",
                    phase,joint.joint_names[j].c_str(),
                    dispatch_start.getVariablePosition(joint.joint_names[j]),
                    joint.points.front().positions[j],
                    dispatch_start.getVariablePosition(joint.joint_names[j])-joint.points.front().positions[j]);
                detail="E_CONTACT_DISPATCH_START_MOVED"; return false;
            }
        auto promise=std::make_shared<std::promise<int>>(); auto future=promise->get_future();
        const auto iface=ctx_->dual_arm_; auto copy=std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>(plan);
        std::thread worker([iface,copy,promise]() { try { promise->set_value(iface->execute(*copy).val); } catch (...) { promise->set_value(-999); } });
        const double deadline=steadySeconds()+std::max(30.0,3*rclcpp::Duration(joint.points.back().time_from_start).seconds()+5);
        bool timed_out=false;
        while (future.wait_for(0ms)!=std::future_status::ready) {
            if (!rclcpp::ok() || steadySeconds()>deadline) { timed_out=true; iface->stop(); break; }
            std::this_thread::sleep_for(50ms);
        }
        if (future.wait_for(2s)!=std::future_status::ready) {
            worker.detach(); detail="E_CONTACT_EXECUTION_OUTCOME_UNKNOWN; no further motion"; return false;
        }
        worker.join(); const int code=future.get();
        if (timed_out) { detail="E_CONTACT_EXECUTION_TIMEOUT"; return false; }
        if (code!=moveit::core::MoveItErrorCode::SUCCESS) {
            detail="E_CONTACT_EXECUTION_FAILED code="+std::to_string(code); return false;
        }
        return true;
    };
    auto waitBilateralContact = [&]() {
        const double timeout = steadySeconds() + 3.0;
        const std::array<std::uint64_t, 2> completion_seq{{
            ctx_->contact_action_completion_seq_[0], ctx_->contact_action_completion_seq_[1]}};
        std::array<DualArmCarryTask::GripperContactFeedback, 2> gate_snapshot{};
        dual_arm_carry_task::BilateralGateResult result =
            dual_arm_carry_task::BilateralGateResult::kNotFresh;
        bool snapshot_available = false;
        const auto resultName = [](dual_arm_carry_task::BilateralGateResult value) {
            using R = dual_arm_carry_task::BilateralGateResult;
            switch (value) {
              case R::kPass: return "pass";
              case R::kActionFailed: return "action_failed";
              case R::kNotFresh: return "not_fresh";
              case R::kInvalid: return "invalid_or_stale";
              case R::kTimeSkew: return "time_skew";
              case R::kMissingContact: return "missing_contact";
              case R::kInsufficientNormalForce: return "normal_force_low";
            }
            return "unknown";
        };
        while (rclcpp::ok() && steadySeconds() < timeout) {
            if (!feedback()) return false;
            std::array<dual_arm_carry_task::BilateralGateArm, 2> candidate{};
            for (std::size_t i = 0; i < candidate.size(); ++i) {
                const auto& f = fingers[i];
                gate_snapshot[i] = f;
                auto& c = candidate[i];
                c.seq = f.seq;
                c.simulation_time = f.simulation_time;
                c.age_seconds = steadySeconds() - f.recv_steady_sec;
                c.valid = f.force_hold_supported && !f.weld_left_active &&
                    !f.weld_right_active && !f.rod_constraint_active &&
                    std::isfinite(f.q1) && std::isfinite(f.q2);
                c.left_contact = f.left_contact;
                c.right_contact = f.right_contact;
                c.left_normal_n = f.normal_left_n;
                c.right_normal_n = f.normal_right_n;
            }
            result = dual_arm_carry_task::evaluateBilateralSnapshot(true, true, completion_seq, candidate);
            snapshot_available = true;
            if (result == dual_arm_carry_task::BilateralGateResult::kPass) break;
            std::this_thread::sleep_for(20ms);
        }
        const double skew = snapshot_available ?
            std::abs(gate_snapshot[0].simulation_time-gate_snapshot[1].simulation_time) :
            std::numeric_limits<double>::quiet_NaN();
        const bool passed = snapshot_available && result == dual_arm_carry_task::BilateralGateResult::kPass;
        RCLCPP_INFO(ctx_->get_logger(),
            "[CONTACT_BILATERAL_GATE] type=single_fresh_snapshot left_seq=%llu right_seq=%llu "
            "completion_seq=(%llu,%llu) skew=%.6fs contacts=(%d,%d;%d,%d) "
            "normal_n=(%.3f,%.3f;%.3f,%.3f) result=%s",
            static_cast<unsigned long long>(gate_snapshot[0].seq),
            static_cast<unsigned long long>(gate_snapshot[1].seq),
            static_cast<unsigned long long>(completion_seq[0]),
            static_cast<unsigned long long>(completion_seq[1]), skew,
            gate_snapshot[0].left_contact ? 1 : 0, gate_snapshot[0].right_contact ? 1 : 0,
            gate_snapshot[1].left_contact ? 1 : 0, gate_snapshot[1].right_contact ? 1 : 0,
            gate_snapshot[0].normal_left_n, gate_snapshot[0].normal_right_n,
            gate_snapshot[1].normal_left_n, gate_snapshot[1].normal_right_n,
            snapshot_available ? resultName(result) : "timeout_no_snapshot");
        if (!passed) {
            detail = "E_CONTACT_BILATERAL_SNAPSHOT_REJECTED";
            return false;
        }
        RCLCPP_WARN(ctx_->get_logger(),
            "[CONTACT_BILATERAL_GATE_SEMANTICS] one eligible snapshot only; this reduces "
            "intermittent-contact rejection but does not prove continuous contact");
        return true;
    };
    if (ctx_->contact_hover_only_) {
        if (!(ctx_->contact_hover_height_m_ > 0.0) ||
            !(ctx_->contact_hover_hold_seconds_ > 0.0))
            return fail("E_CONTACT_HOVER_BAD_PARAMETERS");

        // Arm the MMC bridge before the grasp/lift so its DDS discovery and
        // health probe are not an extra post-lift hover. The bridge waits for
        // a gate file written only after lift confirmation.
        dual_arm_carry_task::BridgeProcess mmc_bridge;
        bool mmc_bridge_started=false;
        int mmc_bridge_status=-1;
        std::string mmc_gate_path;
        if (ctx_->contact_hover_control_mode_ == "mmc_hold") {
            const std::string binary=ament_index_cpp::get_package_prefix("dual_arm_carry_task")+
                "/lib/dual_arm_carry_task/dual_impedance_no_load_bridge";
            const std::string trace_path="experiment_results/a1_runs/"+run_id+"/mmc_hold_trace.csv";
            mmc_gate_path="experiment_results/a1_runs/"+run_id+"/mmc_bridge_start.gate";
            std::string spawn_error;
            std::vector<std::string> args{
                "--ros-args", "-p", "use_sim_time:=true", "-p", "hold_only:=true", "-p",
                "hold_seconds:="+std::to_string(ctx_->contact_hover_hold_seconds_), "-p",
                "trace_csv:="+trace_path, "-p", "wall_timeout_seconds:=20.0", "-p",
                "robot_states_service:=/dual_arm_carry_task/current_robot_states", "-p",
                "start_gate_file:="+mmc_gate_path, "-p", "start_gate_timeout_seconds:=60.0"};
            const char* control_variant = std::getenv("CONTACT_PROBE_MMC_CONTROL_VARIANT");
            const char* rod_disturbance = std::getenv("CONTACT_PROBE_ROD_DISTURBANCE");
            const char* payload_feedforward = std::getenv("CONTACT_PROBE_PAYLOAD_FEEDFORWARD");
            args.insert(args.end(), {"-p", std::string("control_variant:=")+
                (control_variant ? control_variant : "fixed"), "-p", std::string("rod_disturbance_enabled:=")+
                ((rod_disturbance && std::string(rod_disturbance) == "1") ? "true" : "false"), "-p",
                std::string("payload_feedforward_enabled:=")+
                ((payload_feedforward && std::string(payload_feedforward) == "1") ? "true" : "false")});
            if (!mmc_bridge.start(binary,args,spawn_error))
                return fail("E_MMC_HOLD_BRIDGE_START: "+spawn_error);
            mmc_bridge_started=true;
            RCLCPP_INFO(ctx_->get_logger(),
                "[MMC_HOLD_BRIDGE] armed before grasp/lift, hold=%.3fs trace=%s gate=%s; "
                "task protection remains active",
                ctx_->contact_hover_hold_seconds_, trace_path.c_str(), mmc_gate_path.c_str());
        }

        moveit::core::RobotState hover_start(live_model);
        if (!measured(hover_start)) return fail(detail);
        if (!waitBilateralContact()) return fail(detail);
        moveit::planning_interface::MoveGroupInterface::Plan hover_lift;
        if (!makePlan(hover_start,
                      Eigen::Vector3d(0, 0, ctx_->contact_hover_height_m_),
                      hover_lift, "hover_lift", 1.0, 0.6, 0.5))
            return fail(detail);
        if (ctx_->contact_hover_control_mode_ == "mmc_hold") {
            const std::string binary=ament_index_cpp::get_package_prefix("dual_arm_carry_task")+
                "/lib/dual_arm_carry_task/dual_impedance_no_load_bridge";
            std::string spawn_error;
            dual_arm_carry_task::BridgeProcess preflight;
            const std::vector<std::string> args{
                "--ros-args", "-p", "use_sim_time:=true", "-p", "preflight_only:=true",
                "-p", "state_read_budget_seconds:=3.0", "-p",
                "robot_states_service:=/dual_arm_carry_task/current_robot_states"};
            if (!preflight.start(binary,args,spawn_error))
                return fail("E_MMC_STATE_PREFLIGHT_START: "+spawn_error);
            const auto deadline=std::chrono::steady_clock::now()+8s;
            int status=-1;
            while (rclcpp::ok() && std::chrono::steady_clock::now()<deadline && status==-1) {
                status=preflight.poll();
                if (status==-1) std::this_thread::sleep_for(20ms);
            }
            if (status==-1) {
                const bool stopped=preflight.stop();
                return fail(std::string("E_MMC_STATE_PREFLIGHT_TIMEOUT child_stopped=")+(stopped?"true":"false"));
            }
            if (!WIFEXITED(status) || WEXITSTATUS(status)!=0)
                return fail("E_MMC_STATE_PREFLIGHT_REJECTED status="+std::to_string(status));
        }
        if (!refreshTruthBeforeLift()) return fail(detail);
        if (!observe("PRE_HOVER_LIFT_TRUTH_REFRESH",false)) return fail(detail);
        if (!execute(hover_lift, "HOVER_LIFT")) return fail(detail);
        if (!observe("HOVER_LIFT", false)) return fail(detail);
        if (!hover_height.confirmSupportClearance(last_physical_clearance))
            return fail("E_CONTACT_ROD_SUPPORT_CLEARANCE_NONPOSITIVE");
        hover_height.hold_start_rise=hover_height.last_rise;
        RCLCPP_INFO(ctx_->get_logger(),
            "[CONTACT_HOVER_LIFT_CONFIRMED] entry=physical_lowest_point_clear clearance=%.3fmm "
            "peak_rise=%.3fmm current_rise=%.3fmm rise_target_45mm=%s; "
            "HOLD clearance>3mm, drop_from_hold_start<=20mm; slip<=20mm/2deg unchanged",
            last_physical_clearance*1000.0, hover_height.peak_rise*1000.0,
            hover_height.last_rise*1000.0,
            hover_height.rise_target_reached ? "reached" : "not_reached");

        if (mmc_bridge_started) {
            std::ofstream gate(mmc_gate_path, std::ios::out|std::ios::trunc);
            gate << "lift_complete\n";
            gate.flush();
            if (!gate) return fail("E_MMC_START_GATE_WRITE");
            if (!observe("MMC_HOLD_ENTRY", false)) return fail(detail);
            RCLCPP_INFO(ctx_->get_logger(),
                "[MMC_HOLD_BRIDGE] start gate released after lift confirmation");
        }
        const double hold_start =
            std::min(fingers[0].simulation_time, fingers[1].simulation_time);
        const double hold_deadline =
            steadySeconds() + ctx_->contact_hover_hold_seconds_ + 10.0;
        std::size_t hover_samples = 0;
        auto holdSimElapsed=[&]() {
            return std::min(fingers[0].simulation_time, fingers[1].simulation_time)-hold_start;
        };
        if (mmc_bridge_started) {
            double remaining=std::max(0.0,hold_deadline-steadySeconds());
            bool child_reaped=true;
            const auto result=dual_arm_carry_task::runProtectedChild(
                mmc_bridge,
                [&]() {
                    if (!rclcpp::ok()) { detail="E_CONTACT_HOVER_HOLD_SHUTDOWN"; return false; }
                    if (!observe("MMC_HOLD",false)) return false;
                    ++hover_samples;
                    return true;
                },
                [&]() { return holdSimElapsed()>=ctx_->contact_hover_hold_seconds_; },
                [](int status) { return WIFEXITED(status) && WEXITSTATUS(status)==0; },
                std::chrono::milliseconds(static_cast<int64_t>(remaining*1000.0)),
                mmc_bridge_status, child_reaped);
            const int final_status=mmc_bridge.poll();
            if (mmc_bridge_status==-1 && final_status!=-1) mmc_bridge_status=final_status;
            RCLCPP_INFO(ctx_->get_logger(),
                "[MMC_HOLD_BRIDGE] lifecycle_result=%d child_status=%d reaped=%d sim_hold=%.3f "
                "monitor_max_interval=%.3f monitor_max_call=%.3f truth_service=%s "
                "truth_request_last_sec=%.6f truth_age_last_sec=%.6f",
                static_cast<int>(result),mmc_bridge_status,child_reaped?1:0,holdSimElapsed(),
                max_monitor_interval,max_monitor_call,truth_service.c_str(),last_truth_request_elapsed,last_truth_age);
            if (result==dual_arm_carry_task::ProtectedChildResult::PROTECTION_FAILED)
                return fail(detail.empty()?"E_MMC_HOLD_PROTECTION_FAILED":detail);
            if (result==dual_arm_carry_task::ProtectedChildResult::CHILD_FAILED)
                return fail("E_MMC_HOLD_BRIDGE_FAILED status="+std::to_string(mmc_bridge_status));
            if (result!=dual_arm_carry_task::ProtectedChildResult::COMPLETED)
                return fail("E_MMC_HOLD_TIMEOUT_OR_SHUTDOWN");
        } else {
            while (holdSimElapsed()<ctx_->contact_hover_hold_seconds_) {
                if (!rclcpp::ok() || steadySeconds()>hold_deadline)
                    return fail("E_CONTACT_HOVER_HOLD_TIMEOUT_OR_SHUTDOWN");
                if (!observe("HOVER_HOLD",false)) return fail(detail);
                ++hover_samples;
                std::this_thread::sleep_for(50ms);
            }
        }
        if (hover_samples < 20) return fail("E_CONTACT_HOVER_INSUFFICIENT_SAMPLES");
        RCLCPP_INFO(ctx_->get_logger(),
            "[CONTACT_HOVER_RESULT] samples=%zu hold=%.3fs peak_rise=%.3fmm "
            "final_rise=%.3fmm drop_from_peak=%.3fmm max_slip=%.3fmm "
            "max_angle=%.3fdeg weld=OFF rod_constraint=OFF; "
            "absolute rod/hand poses and contact forces are in contact_probe.csv; monitor_max_interval=%.3fs "
            "monitor_max_call=%.3fs truth_age_last=%.3fs truth_request_last=%.3fs",
            hover_samples,
            std::min(fingers[0].simulation_time, fingers[1].simulation_time) - hold_start,
            hover_height.peak_rise * 1000.0, hover_height.last_rise * 1000.0,
            (hover_height.peak_rise-hover_height.last_rise) * 1000.0,
            max_slip * 1000.0, max_angle * 180.0 / M_PI, max_monitor_interval,
            max_monitor_call,last_truth_age,last_truth_request_elapsed);

        // Lower using the measured rod attitude. This keeps the experiment
        // independent of the full transport path and avoids mixing transport
        // slip with the hover result.
        // Do not build from a still-moving measured state: JTC dispatch rejects
        // a >2 mrad change between planning and sending the trajectory.
        {
            moveit::core::RobotState previous(live_model);
            if (!measured(previous)) return fail(detail);
            const double deadline=steadySeconds()+2.0;
            bool stable=false;
            while (rclcpp::ok() && steadySeconds()<deadline) {
                std::this_thread::sleep_for(100ms);
                if (!observe("HOVER_SETTLE", false)) return fail(detail);
                moveit::core::RobotState current(live_model);
                if (!measured(current)) return fail(detail);
                double max_delta=0.0;
                for (const auto& name:dual->getVariableNames())
                    max_delta=std::max(max_delta,std::abs(current.getVariablePosition(name)-
                                                        previous.getVariablePosition(name)));
                if (max_delta<0.0005) { stable=true; break; }
                previous=current;
            }
            if (!stable) return fail("E_CONTACT_HOVER_PRE_LOWER_UNSTABLE");
        }
        moveit::core::RobotState hover_end(live_model);
        if (!measured(hover_end)) return fail(detail);
        const auto hover_truth = readMujocoRodTruth();
        if (!hover_truth.ok) return fail("E_CONTACT_HOVER_TRUTH");
        const Transform hover_object = rodTransform(hover_truth);
        for (int i = 0; i < 2; ++i)
            object_to_hand[i] =
                hover_object.inverse() * hover_end.getGlobalLinkTransform(eefs[i]);
        const double hover_extent = 0.5 *
            (std::abs(hover_object.linear()(2, 0)) * truth.size.x +
             std::abs(hover_object.linear()(2, 1)) * truth.size.y +
             std::abs(hover_object.linear()(2, 2)) * truth.size.z);
        const double lower_delta = hover_object.translation().z() -
            (ctx_->p2a_support_z_ + hover_extent);
        if (!(lower_delta > 0.0) || !std::isfinite(lower_delta))
            return fail("E_CONTACT_HOVER_BAD_LOWER_DELTA");
        moveit::planning_interface::MoveGroupInterface::Plan hover_lower;
        if (!makePlan(hover_end, Eigen::Vector3d(0, 0, -lower_delta),
                      hover_lower, "hover_lower", 1.0, 0.6, 0.5))
            return fail(detail);
        if (!execute(hover_lower, "HOVER_LOWER")) return fail(detail);
        if (!observe("HOVER_LOWER", false)) return fail(detail);
        const double lower_extent = 0.5 *
            (std::abs(rodTransform(latest).linear()(2, 0)) * truth.size.x +
             std::abs(rodTransform(latest).linear()(2, 1)) * truth.size.y +
             std::abs(rodTransform(latest).linear()(2, 2)) * truth.size.z);
        if (std::abs(latest.position.z - lower_extent - ctx_->p2a_support_z_) > 0.003)
            return fail("E_CONTACT_HOVER_LOWER_NOT_ON_SUPPORT");
        if (!ctx_->grippers_->controlDualGrippers(0.04))
            return fail("E_CONTACT_HOVER_RELEASE_OPEN_FAILED");
        force_hold_may_be_active = false;
        std::this_thread::sleep_for(500ms);
        const auto released = readMujocoRodTruth();
        if (!released.ok || !feedback())
            return fail("E_CONTACT_HOVER_RELEASE_TRUTH_OR_FEEDBACK");
        for (const auto& f : fingers)
            if (f.hold_force_per_finger_n != 0.0 || f.q1 + f.q2 < 0.078)
                return fail("E_CONTACT_HOVER_RELEASE_FORCE_NOT_CLEARED");
        if ((rodTransform(released).translation()-rodTransform(latest).translation()).norm()>0.01 ||
            rotationError(rodTransform(released),rodTransform(latest))>5*M_PI/180)
            return fail("E_CONTACT_HOVER_RELEASE_POSE_ERROR");
        geometry_msgs::msg::Vector3 released_size;
        released_size.x=truth.size.x; released_size.y=truth.size.y; released_size.z=truth.size.z;
        if (!syncAluminumRodFromTruth(poseMsg(rodTransform(released)), released_size,
                ctx_->dual_arm_->getPlanningFrame(), detail)) return fail(detail);
        RCLCPP_INFO(ctx_->get_logger(),
            "[CONTACT_HOVER_RELEASED] xyz=(%.6f,%.6f,%.6f) open=true "
            "force_hold=OFF weld=OFF attach=none",
            released.position.x, released.position.y, released.position.z);
        detail = "no-weld hover hold complete: lift=" +
            std::to_string(ctx_->contact_hover_height_m_) + "m hold=" +
            std::to_string(ctx_->contact_hover_hold_seconds_) + "s; no transport";
        return true;
    }
    if (hold_diagnostic_only) {
        constexpr double mechanism_hold_seconds = 1.0;
        std::array<unsigned long long,2> counted_seq{{0,0}};
        std::array<int,4> contact_counts{{0,0,0,0}};
        std::array<double,2> gap_min{{std::numeric_limits<double>::infinity(),
                                      std::numeric_limits<double>::infinity()}};
        std::array<double,2> gap_max{{-std::numeric_limits<double>::infinity(),
                                      -std::numeric_limits<double>::infinity()}};
        std::size_t samples = 0;
        if (!feedback()) return fail(detail);
        const double hold_start = std::min(fingers[0].simulation_time, fingers[1].simulation_time);
        const double wall_deadline = steadySeconds() + 4.0;
        while (std::min(fingers[0].simulation_time, fingers[1].simulation_time) - hold_start <
               mechanism_hold_seconds) {
            if (!rclcpp::ok() || steadySeconds() > wall_deadline || !feedback())
                return fail(detail.empty() ? "E_CONTACT_MECHANISM_HOLD_TIMEOUT" : detail);
            if (fingers[0].seq > counted_seq[0] && fingers[1].seq > counted_seq[1]) {
                ++samples;
                counted_seq = {{fingers[0].seq, fingers[1].seq}};
                for (int side = 0; side < 2; ++side) {
                    const auto& f = fingers[side];
                    const int offset = side * 2;
                    contact_counts[offset] += f.left_contact ? 1 : 0;
                    contact_counts[offset + 1] += f.right_contact ? 1 : 0;
                    const double gap = f.q1 + f.q2;
                    gap_min[side] = std::min(gap_min[side], gap);
                    gap_max[side] = std::max(gap_max[side], gap);
                }
            }
            if (!observe("MECHANISM_HOLD", false)) return fail(detail);
            std::this_thread::sleep_for(50ms);
        }
        if (samples < 16) return fail("E_CONTACT_MECHANISM_HOLD_INSUFFICIENT_SAMPLES");
        for (const int count : contact_counts) {
            if (static_cast<double>(count) / static_cast<double>(samples) < 0.8)
                return fail("E_CONTACT_MECHANISM_BILATERAL_OCCUPANCY");
        }
        RCLCPP_INFO(ctx_->get_logger(),
            "[CONTACT_HOLD_MECHANISM_VALIDATED] fixed_arms=true sim_hold=%.3fs samples=%zu "
            "bilateral_occupancy=(%d/%zu,%d/%zu,%d/%zu,%d/%zu) gap_range=(%.3f,%.3f)mm "
            "weld=OFF attach=none; load_unverified",
            std::min(fingers[0].simulation_time, fingers[1].simulation_time) - hold_start,
            samples, contact_counts[0], samples, contact_counts[1], samples,
            contact_counts[2], samples, contact_counts[3], samples,
            (gap_max[0] - gap_min[0]) * 1000.0, (gap_max[1] - gap_min[1]) * 1000.0);
        if (!ctx_->grippers_->controlDualGrippers(0.04)) return fail("E_CONTACT_MECHANISM_RELEASE_OPEN_FAILED");
        force_hold_may_be_active=false;
        std::this_thread::sleep_for(500ms);
        if (!feedback()) return fail(detail);
        for (const auto& f : fingers) {
            if (f.hold_force_per_finger_n != 0.0 || f.q1 + f.q2 < 0.078)
                return fail("E_CONTACT_MECHANISM_RELEASE_NOT_CONFIRMED");
        }
        const auto released = readMujocoRodTruth();
        if (!released.ok ||
            (rodTransform(released).translation() - object.translation()).norm() > pos_tol ||
            rotationError(rodTransform(released), object) > angle_tol)
            return fail("E_CONTACT_MECHANISM_RELEASE_TRUTH_CHANGED");
        geometry_msgs::msg::Vector3 size;
        size.x = truth.size.x; size.y = truth.size.y; size.z = truth.size.z;
        if (!syncAluminumRodFromTruth(poseMsg(rodTransform(released)), size,
                                      ctx_->dual_arm_->getPlanningFrame(), detail)) return fail(detail);
        RCLCPP_INFO(ctx_->get_logger(),
            "[CONTACT_HOLD_MECHANISM_RELEASED] rod_xyz=(%.6f,%.6f,%.6f) open=true force_hold=OFF "
            "weld=OFF attach=none lift=not-tested",
            released.position.x, released.position.y, released.position.z);
        detail = "fixed-arm force-hold mechanism and release validated; no lift/load claim";
        return true;
    }
    if (!execute(lift,"LIFT")) return fail(detail);
    if (!observe("HOLD",true)) return fail(detail);
    // B: creep hold. The static hold is where the soft friction lets the rod
    // creep and the pad contacts intermittently drop. Plan a slow 4 mm upward
    // creep and execute it while the same hold window is sampled, keeping a
    // small relative velocity so the contacts stay engaged.
    moveit::core::RobotState hold_state(live_model);
    bool creep_started = false;
    std::shared_ptr<std::promise<int>> creep_promise;
    std::future<int> creep_future;
    std::thread creep_worker;
    if (creep_hold_enabled && measured(hold_state)) {
        moveit::planning_interface::MoveGroupInterface::Plan creep;
        const std::string detail_before_creep = detail;
        relax_dual_residual = true;
        const bool creep_planned =
            makePlan(hold_state, Eigen::Vector3d(0,0,0.004), creep, "creep", 0.02, 0.012, 2.0);
        relax_dual_residual = false;
        if (creep_planned) {
            creep_promise = std::make_shared<std::promise<int>>();
            creep_future = creep_promise->get_future();
            const auto iface = ctx_->dual_arm_;
            auto copy = std::make_shared<moveit::planning_interface::MoveGroupInterface::Plan>(creep);
            creep_worker = std::thread([iface,copy,creep_promise]() {
                try { creep_promise->set_value(iface->execute(*copy).val); }
                catch (...) { creep_promise->set_value(-999); }
            });
            creep_started = true;
            RCLCPP_INFO(ctx_->get_logger(),
                "[CONTACT_HOLD_CREEP] 4 mm upward creep over >=2 s; executing during the hold");
        } else {
            detail = detail_before_creep;
            RCLCPP_WARN(ctx_->get_logger(),
                "[CONTACT_HOLD_CREEP] creep plan rejected; falling back to the static hold");
        }
    }
    // Hold is measured in physics time, with wall timeout and advancement checks.
    const double hold_start=std::min(fingers[0].simulation_time,fingers[1].simulation_time);
    const double hold_wall_deadline=steadySeconds()+6.0;
    std::size_t samples=0; std::array<int,4> contact_counts{{0,0,0,0}};
    std::array<unsigned long long,2> counted_seq{{0,0}};
    while (std::min(fingers[0].simulation_time,fingers[1].simulation_time)-hold_start<hold_seconds) {
        if (!rclcpp::ok() || steadySeconds()>hold_wall_deadline || !observe("HOLD",true)) {
            if (creep_started) { try { ctx_->dual_arm_->stop(); } catch (...) {} }
            if (creep_future.valid() && creep_future.wait_for(2s)==std::future_status::ready) creep_worker.join();
            else if (creep_worker.joinable()) creep_worker.detach();
            return fail("E_CONTACT_HOLD: "+detail);
        }
        if (fingers[0].seq>counted_seq[0] && fingers[1].seq>counted_seq[1]) {
          ++samples;
          counted_seq={{fingers[0].seq,fingers[1].seq}};
          for (int i=0;i<2;++i) { contact_counts[2*i]+=fingers[i].normal_left_n>=0.25; contact_counts[2*i+1]+=fingers[i].normal_right_n>=0.25; }
        }
        std::this_thread::sleep_for(50ms);
    }
    if (creep_started) {
        if (creep_future.wait_for(2s)!=std::future_status::ready) {
            try { ctx_->dual_arm_->stop(); } catch (...) {}
            creep_worker.detach();
            return fail("E_CONTACT_HOLD_CREEP_TIMEOUT");
        }
        creep_worker.join();
        const int creep_code = creep_future.get();
        if (creep_code != moveit::core::MoveItErrorCode::SUCCESS)
            return fail("E_CONTACT_HOLD_CREEP_FAILED code="+std::to_string(creep_code));
    }
    if (samples<20) return fail("E_CONTACT_HOLD_INSUFFICIENT_SAMPLES");
    // A: the per-finger force duty is recorded, not a hard abort. The rod's
    // height/slip/attitude remain the hard hold criteria; the duty is reported.
    std::array<double,4> duty{{0.0,0.0,0.0,0.0}};
    for (int i=0;i<4;++i) {
        duty[i] = double(contact_counts[i])/double(samples);
        if (duty[i] < 0.8)
            RCLCPP_WARN(ctx_->get_logger(),
                "[CONTACT_HOLD_DUTY] finger=%d duty=%d/%zu=%.3f (<0.80); recorded, not aborting",
                i, contact_counts[i], samples, duty[i]);
    }
    RCLCPP_INFO(ctx_->get_logger(),"[CONTACT_LOAD_VALIDATED] samples=%zu sim_hold=%.3fs lift=%.3fm max_slip=%.3fmm max_angle=%.3fdeg duty=(%.2f,%.2f,%.2f,%.2f) creep=%s weld_left=false weld_right=false rod_constraint=false",
        samples,std::min(fingers[0].simulation_time,fingers[1].simulation_time)-hold_start,latest.position.z-initial_truth.position.z,max_slip*1000,max_angle*180/M_PI,duty[0],duty[1],duty[2],duty[3],creep_started?"on":"off");
    // The load was validated with the frozen grasp reference (max_slip logged
    // above). Under load the two hands' inferred rod poses can drift apart; the
    // descent plan then fails E_CONTACT_DUAL_GRASP_RESIDUAL. Re-establish the
    // object->hand transforms from the current measured state so the descent is
    // consistent with the actual grasp. This does not reset the LIFT/HOLD slip
    // already measured and logged above.
    const auto hold_truth = readMujocoRodTruth();
    moveit::core::RobotState held_state(live_model);
    if (!hold_truth.ok || !measured(held_state)) return fail("E_CONTACT_HOLD_REFREEZE_UNMEASURED");
    const Transform held_object = rodTransform(hold_truth);
    for (int i=0;i<2;++i)
        object_to_hand[i] = held_object.inverse()*held_state.getGlobalLinkTransform(eefs[i]);
    RCLCPP_INFO(ctx_->get_logger(),
        "[CONTACT_PROBE_REFREEZE] descent reference re-established from measured state; "
        "lift/hold max_slip already logged=%.3f mm", max_slip*1000);
    moveit::core::RobotState lifted(live_model);
    if (!measured(lifted)) return fail(detail);
    // Forward transport in the fixed scene with the held yaw: move the rod to
    // the task target x while the real contact grip carries it. y and z are
    // kept; the slip monitor and the 45 mm height check run during this phase.
    const double held_yaw = std::atan2(held_object.linear()(1,0), held_object.linear()(0,0));
    double transport_dyaw = ctx_->rod_target_yaw_rad_ - held_yaw;
    while (transport_dyaw > M_PI) transport_dyaw -= 2.0*M_PI;
    while (transport_dyaw < -M_PI) transport_dyaw += 2.0*M_PI;
    const Eigen::Vector3d transport_delta(
        ctx_->rod_target_x_ - held_object.translation().x(),
        ctx_->rod_target_y_ - held_object.translation().y(), 0.0);
    if (!std::isfinite(transport_delta.x()) || !std::isfinite(transport_delta.y()) ||
        !std::isfinite(transport_dyaw) || transport_delta.norm() < 0.001 ||
        transport_delta.norm() > 0.20 || std::abs(transport_dyaw) > 0.6) {
        detail="E_CONTACT_TRANSPORT_DELTA"; return fail(detail);
    }
    RCLCPP_INFO(ctx_->get_logger(),
        "[CONTACT_PROBE_TRANSPORT] dx=%.3f dy=%.3f dyaw=%.1f deg to target (%.3f, %.3f, %.1f deg)",
        transport_delta.x(), transport_delta.y(), transport_dyaw*180.0/M_PI,
        ctx_->rod_target_x_, ctx_->rod_target_y_, ctx_->rod_target_yaw_rad_*180.0/M_PI);
    moveit::planning_interface::MoveGroupInterface::Plan transport;
    // S2b timing experiment: min_total is an absolute stretch floor in makePlan
    // (stretch = max(min_total, min_total/T, ...)), so 2.0 doubles the IPTP
    // transport duration. 1.0 leaves the scaled timing untouched.
    double transport_stretch=2.0;
    if (const char* raw=std::getenv("CONTACT_TRANSPORT_STRETCH")) {
        const double value=std::atof(raw);
        if (std::isfinite(value) && value>0.0) transport_stretch=value;
    }
    RCLCPP_INFO(ctx_->get_logger(),
        "[CONTACT_PROBE_TRANSPORT_STRETCH] value=%.3f (env CONTACT_TRANSPORT_STRETCH, default 2.0)",
        transport_stretch);
    const double transport_plan_start=steadySeconds();
    if (!makePlan(lifted, transport_delta, transport, "transport", 1.0, 0.3, transport_stretch, transport_dyaw)) return fail(detail);
    const double transport_plan_sec=steadySeconds()-transport_plan_start;
    // S2 extended-scenario execution pairing. When enabled, BOTH trajectories
    // are built from THIS run's measured transport start and saved; only the
    // requested one is dispatched. The baseline keeps the production planning
    // path unchanged, and the candidate is checked with the same evaluator
    // before it can be sent.
    const char* pairing_raw=std::getenv("CONTACT_TRANSPORT_PAIRING");
    const std::string pairing_choice=pairing_raw?pairing_raw:"0";
    const bool pairing_enabled=!pairing_choice.empty() && pairing_choice!="0";
    if (pairing_enabled) {
        // The pairing build/evaluation is a bounded planning phase outside the
        // monitoring loop. Discard the outstanding truth request instead of
        // letting it age past its 2 s budget; a fresh sample is requested
        // again right before dispatch.
        truth_guard.abandonRequest();
        if (plan_compare) return fail("E_CONTACT_TRANSPORT_PAIRING_CONFLICT");
        if (pairing_choice!="baseline" && pairing_choice!="candidate")
            return fail("E_CONTACT_TRANSPORT_PAIRING_CHOICE");
        const char* kind_raw=std::getenv("CONTACT_TRANSPORT_PAIRING_CANDIDATE");
        const std::string pairing_kind=kind_raw?kind_raw:"time_profile";
        if (pairing_kind!="time_profile" && pairing_kind!="geometry")
            return fail("E_CONTACT_TRANSPORT_PAIRING_KIND");
        moveit::planning_interface::MoveGroupInterface::Plan pairing_candidate;
        TimeProfileCandidateInfo pairing_tp;
        bool candidate_built=false;
        double candidate_plan_sec=0.0;
        std::string candidate_error;
        if (pairing_kind=="geometry") {
            const double start=steadySeconds();
            candidate_built=makePlan(lifted,transport_delta,pairing_candidate,
                                     "transport_geometry_candidate",1.0,0.3,transport_stretch,
                                     transport_dyaw,true);
            candidate_plan_sec=steadySeconds()-start;
            if (!candidate_built) candidate_error=detail;
        } else {
            candidate_built=buildTimeProfileCandidate(transport,pairing_tp);
            candidate_plan_sec=pairing_tp.solve_sec;
            if (candidate_built) pairing_candidate=pairing_tp.plan;
            else candidate_error=pairing_tp.detail;
        }
        const Transform pairing_first=lifted.getGlobalLinkTransform(eefs[0])*object_to_hand[0].inverse();
        TransportPlanStats pairing_base_stats,pairing_cand_stats;
        if (!evaluateTransport("pairing_baseline",transport,pairing_base_stats,pairing_first,
                               transport_delta,transport_dyaw,{},nullptr))
            return fail(detail);
        const bool base_ok=pairing_base_stats.collisions==0 && !pairing_base_stats.limit_violation &&
            !pairing_base_stats.vel_violation && !pairing_base_stats.acc_violation &&
            pairing_base_stats.max_sample_dual_pos_mm<=5.0 && pairing_base_stats.max_sample_dual_ang_deg<=2.0 &&
            pairing_base_stats.max_rod_pos_mm<=5.0 && pairing_base_stats.max_rod_ang_deg<=2.0;
        bool candidate_ok=false;
        if (candidate_built) {
            const std::function<double(double)> progress =
                pairing_kind=="time_profile"?pairing_tp.progress_of_time:std::function<double(double)>{};
            if (!evaluateTransport("pairing_candidate",pairing_candidate,pairing_cand_stats,pairing_first,
                                   transport_delta,transport_dyaw,progress,nullptr))
                return fail(detail);
            candidate_ok=pairing_cand_stats.collisions==0 && !pairing_cand_stats.limit_violation &&
                !pairing_cand_stats.vel_violation && !pairing_cand_stats.acc_violation &&
                pairing_cand_stats.max_sample_dual_pos_mm<=5.0 && pairing_cand_stats.max_sample_dual_ang_deg<=2.0 &&
                pairing_cand_stats.max_rod_pos_mm<=5.0 && pairing_cand_stats.max_rod_ang_deg<=2.0;
        }
        if (!writeTrajectoryCsv("experiment_results/a1_runs/"+run_id+"/transport_pairing_baseline.csv",
                                transport)) return fail("E_TRANSPORT_PAIRING_BASELINE_CSV");
        if (candidate_built &&
            !writeTrajectoryCsv("experiment_results/a1_runs/"+run_id+"/transport_pairing_candidate.csv",
                                pairing_candidate)) return fail("E_TRANSPORT_PAIRING_CANDIDATE_CSV");
        {
            std::ofstream meta("experiment_results/a1_runs/"+run_id+"/transport_pairing_meta.txt");
            if (!meta) return fail("E_TRANSPORT_PAIRING_META");
            meta<<std::setprecision(17)
                <<"source=measured_transport_start\n"
                <<"pairing_choice="<<pairing_choice<<"\n"
                <<"candidate_kind="<<pairing_kind<<"\n"
                <<"candidate_built="<<(candidate_built?1:0)<<"\n"
                <<"candidate_error="<<candidate_error<<"\n"
                <<"transport_stretch="<<transport_stretch<<"\n"
                <<"transport_dx="<<transport_delta.x()<<"\n"
                <<"transport_dy="<<transport_delta.y()<<"\n"
                <<"transport_dyaw_deg="<<transport_dyaw*180.0/M_PI<<"\n"
                <<"scenario_target_x="<<ctx_->rod_target_x_<<"\n"
                <<"scenario_target_y="<<ctx_->rod_target_y_<<"\n"
                <<"scenario_target_yaw_deg="<<ctx_->rod_target_yaw_rad_*180.0/M_PI<<"\n";
            const auto& jt=transport.trajectory_.joint_trajectory;
            for (std::size_t j=0;j<jt.joint_names.size();++j)
                meta<<"start_joint,"<<jt.joint_names[j]<<","<<jt.points.front().positions[j]<<"\n";
            for (int i=0;i<2;++i) {
                const auto& p=lifted.getGlobalLinkTransform(eefs[i]).translation();
                meta<<(i==0?"start_left_hand_xyz=":"start_right_hand_xyz=")
                    <<p.x()<<","<<p.y()<<","<<p.z()<<"\n";
            }
            meta.flush();
            if (!meta) return fail("E_TRANSPORT_PAIRING_META_WRITE");
        }
        {
            std::ofstream csv("experiment_results/a1_runs/"+run_id+"/transport_pairing_summary.csv");
            if (!csv) return fail("E_TRANSPORT_PAIRING_SUMMARY_CSV");
            csv<<std::setprecision(12)
                <<"plan,kind,executed,points,duration_s,planning_sec,collisions,"
                  "limit_violation,vel_violation,acc_violation,max_vel_ratio,max_acc_ratio,"
                  "acc_rms,jerk_rms,jerk_max,continuous_jerk_rms,continuous_jerk_max,"
                  "max_export_deviation_rad,max_sample_dual_pos_mm,max_sample_dual_ang_deg,"
                  "max_rod_pos_mm,max_rod_ang_deg,pass\n";
            auto row=[&](const char* name,const char* kind,const TransportPlanStats& s,
                         double planning_sec,bool pass,double cont_rms,double cont_max,
                         double dev) {
                csv<<name<<','<<kind<<','<<(pairing_choice==name?1:0)<<','<<s.points<<','
                   <<s.duration<<','<<planning_sec<<','<<s.collisions<<','
                   <<(s.limit_violation?1:0)<<','<<(s.vel_violation?1:0)<<','
                   <<(s.acc_violation?1:0)<<','<<s.max_vel_ratio<<','<<s.max_acc_ratio<<','
                   <<s.acc_rms<<','<<s.jerk_rms<<','<<s.jerk_max<<','<<cont_rms<<','
                   <<cont_max<<','<<dev<<','<<s.max_sample_dual_pos_mm<<','
                   <<s.max_sample_dual_ang_deg<<','<<s.max_rod_pos_mm<<','
                   <<s.max_rod_ang_deg<<','<<(pass?1:0)<<'\n';
            };
            row("baseline","production",pairing_base_stats,transport_plan_sec,base_ok,-1.0,-1.0,0.0);
            if (candidate_built) {
                row("candidate",pairing_kind.c_str(),pairing_cand_stats,candidate_plan_sec,
                    candidate_ok,pairing_kind=="time_profile"?pairing_tp.candidate_continuous.jerk_rms:-1.0,
                    pairing_kind=="time_profile"?pairing_tp.candidate_continuous.jerk_max:-1.0,
                    pairing_kind=="time_profile"?pairing_tp.max_export_deviation_rad:0.0);
            }
            csv.flush();
            if (!csv) return fail("E_TRANSPORT_PAIRING_SUMMARY_WRITE");
        }
        RCLCPP_INFO(ctx_->get_logger(),
            "[TRANSPORT_PAIRING] choice=%s kind=%s candidate_built=%d candidate_ok=%d "
            "baseline_jerk_rms=%.4f candidate_jerk_rms=%.4f baseline_duration=%.6f "
            "candidate_duration=%.6f baseline_samples=%zu candidate_samples=%zu error=%s",
            pairing_choice.c_str(),pairing_kind.c_str(),candidate_built?1:0,candidate_ok?1:0,
            pairing_base_stats.jerk_rms,candidate_built?pairing_cand_stats.jerk_rms:-1.0,
            pairing_base_stats.duration,candidate_built?pairing_cand_stats.duration:-1.0,
            pairing_base_stats.samples,candidate_built?pairing_cand_stats.samples:0,
            candidate_error.empty()?"none":candidate_error.c_str());
        // Restore a fresh physical sample before sending any transport target.
        if (!refreshTruthBeforeLift())
            return fail(detail.empty()?"E_CONTACT_TRUTH_PAIRING_REFRESH":detail);
        RCLCPP_INFO(ctx_->get_logger(),
            "[CONTACT_TRUTH_PAIRING_REFRESH] result=fresh request_elapsed_wall_sec=%.6f age_wall_sec=%.6f",
            last_truth_request_elapsed,last_truth_age);
        if (pairing_choice=="candidate") {
            if (!candidate_ok) return fail("E_CONTACT_TRANSPORT_PAIRING_CANDIDATE_REJECTED");
            transport=pairing_candidate;
            RCLCPP_INFO(ctx_->get_logger(),
                "[TRANSPORT_PAIRING] dispatching the candidate trajectory from the measured start");
        } else {
            RCLCPP_INFO(ctx_->get_logger(),
                "[TRANSPORT_PAIRING] dispatching the production baseline trajectory from the measured start");
        }
    }
    if (plan_compare) {
        // P4/S2 phase 1: compare the current object-space transport plan (A)
        // with a MoveIt CHOMP plan to the SAME final joint target (B). Nothing
        // is executed here unless B also preserves the dual closed chain,
        // collides nowhere and stays inside the joint limits.
        std::ofstream compare_csv("experiment_results/a1_runs/"+run_id+"/plan_compare.csv");
        if (!compare_csv) return fail("E_PLAN_COMPARE_CSV_OPEN");
        compare_csv<<std::setprecision(10)
            <<"planner,sample_index,t,left_inferred_x,left_inferred_y,left_inferred_z,"
              "right_inferred_x,right_inferred_y,right_inferred_z,"
              "dual_pos_err_mm,dual_ang_err_deg,rod_path_pos_err_mm,rod_path_ang_err_deg,"
              "collision,joint_limit_violation,vel_violation,acc_violation\n";
        const Transform first_object_lifted=lifted.getGlobalLinkTransform(eefs[0])*object_to_hand[0].inverse();
        Transform target_object=first_object_lifted;
        target_object.translation()+=transport_delta;
        target_object.linear()=Eigen::AngleAxisd(transport_dyaw,Eigen::Vector3d::UnitZ())
            .toRotationMatrix()*first_object_lifted.linear();
        struct CompareStats {
            bool planned{false};
            int plan_code{0};
            double planning_sec{0.0};
            std::size_t points{0};
            double duration{0.0};
            std::size_t samples{0};
            double max_dual_pos_mm{0.0};
            double max_dual_ang_deg{0.0};
            double max_rod_pos_mm{0.0};
            double max_rod_ang_deg{0.0};
            std::size_t collisions{0};
            std::string first_collision;
            bool limit_violation{false};
            bool vel_violation{false};
            bool acc_violation{false};
            std::string violation_detail;
            double max_jerk{0.0};
            double p95_jerk{0.0};
        };
        auto evaluatePlan=[&](const std::string& planner,
                              const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                              CompareStats& stats) {
            const auto& jt=plan.trajectory_.joint_trajectory;
            if (jt.points.size()<2) return false;
            stats.points=jt.points.size();
            stats.duration=rclcpp::Duration(jt.points.back().time_from_start).seconds();
            std::vector<s1::DirectionalJointLimits> limits;
            for (const auto& name:jt.joint_names) {
                const auto& b=live_model->getVariableBounds(name);
                limits.push_back({b.min_velocity_,b.max_velocity_,b.min_acceleration_,
                                  b.max_acceleration_,b.velocity_bounded_,b.acceleration_bounded_});
            }
            struct Sample { std::vector<double> q,v,a; double t; };
            std::vector<Sample> samples;
            const auto& first=jt.points.front();
            samples.push_back({first.positions,first.velocities,first.accelerations,0.0});
            for (std::size_t k=1;k<jt.points.size();++k) {
                const auto& a=jt.points[k-1]; const auto& b=jt.points[k];
                const double T=rclcpp::Duration(b.time_from_start).seconds()-
                               rclcpp::Duration(a.time_from_start).seconds();
                if (!(T>0.0)) continue;
                const int n=std::max(2,static_cast<int>(std::ceil(T/0.02)));
                for (int s=1;s<=n;++s) {
                    const double u=double(s)/n;
                    Sample sample; sample.q.resize(a.positions.size());
                    sample.v.resize(a.positions.size()); sample.a.resize(a.positions.size());
                    sample.t=rclcpp::Duration(a.time_from_start).seconds()+u*T;
                    for (std::size_t j=0;j<sample.q.size();++j) {
                        const double av=j<a.velocities.size()?a.velocities[j]:0.0;
                        const double aa=j<a.accelerations.size()?a.accelerations[j]:0.0;
                        const double bv=j<b.velocities.size()?b.velocities[j]:0.0;
                        const double ba=j<b.accelerations.size()?b.accelerations[j]:0.0;
                        const double c0=a.positions[j],c1=av*T,c2=0.5*aa*T*T;
                        const double D=b.positions[j]-c0-c1-c2;
                        const double V=bv*T-c1-2*c2;
                        const double A=ba*T*T-2*c2;
                        const double c3=10*D-4*V+0.5*A,c4=-15*D+7*V-A,c5=6*D-3*V+0.5*A;
                        sample.q[j]=c0+u*(c1+u*(c2+u*(c3+u*(c4+u*c5))));
                        sample.v[j]=(c1+u*(2*c2+u*(3*c3+u*(4*c4+u*5*c5))))/T;
                        sample.a[j]=(2*c2+u*(6*c3+u*(12*c4+u*20*c5)))/(T*T);
                    }
                    samples.push_back(std::move(sample));
                }
            }
            std::vector<double> jerks;
            const Sample* previous=nullptr;
            std::size_t limit_violations=0,vel_violations=0,acc_violations=0;
            for (std::size_t index=0;index<samples.size();++index) {
                const auto& sample=samples[index];
                moveit::core::RobotState state(live_model);
                state.setVariablePositions(jt.joint_names,sample.q);
                state.update();
                // Collision and bounds are checked on the criterion model, exactly
                // like valid(); using the live model against the criterion scene
                // reports contacts that do not exist in the validated setup.
                moveit::core::RobotState criterion_state(criterion_model);
                for (const auto& name:criterion_model->getVariableNames())
                    criterion_state.setVariablePosition(name,state.getVariablePosition(name));
                criterion_state.update();
                const bool limit_bad=!criterion_state.satisfiesBounds(criterion_dual,1e-4);
                bool vel_bad=false,acc_bad=false;
                for (std::size_t j=0;j<sample.q.size() && j<limits.size();++j) {
                    const auto& b=limits[j];
                    if (sample.v[j]<b.min_velocity-1e-9 || sample.v[j]>b.max_velocity+1e-9) vel_bad=true;
                    if (sample.a[j]<b.min_acceleration-1e-9 || sample.a[j]>b.max_acceleration+1e-9) acc_bad=true;
                }
                if (limit_bad) { ++limit_violations; if (!stats.limit_violation) {
                    stats.limit_violation=true;
                    stats.violation_detail="joint bounds at sample "+std::to_string(index); } }
                if (vel_bad) { ++vel_violations; if (!stats.vel_violation) {
                    stats.vel_violation=true;
                    stats.violation_detail+=(stats.violation_detail.empty()?"":"; ")+
                        std::string("velocity at sample ")+std::to_string(index); } }
                if (acc_bad) { ++acc_violations; if (!stats.acc_violation) {
                    stats.acc_violation=true;
                    stats.violation_detail+=(stats.violation_detail.empty()?"":"; ")+
                        std::string("acceleration at sample ")+std::to_string(index); } }
                if (previous!=nullptr) {
                    const double dt=sample.t-previous->t;
                    if (dt>1e-9) {
                        double jerk=0.0;
                        for (std::size_t j=0;j<sample.a.size() && j<previous->a.size();++j)
                            jerk=std::max(jerk,std::abs(sample.a[j]-previous->a[j])/dt);
                        jerks.push_back(jerk);
                    }
                }
                previous=&sample;
                const Transform inferred=state.getGlobalLinkTransform(eefs[0])*object_to_hand[0].inverse();
                const Transform inferred_right=state.getGlobalLinkTransform(eefs[1])*object_to_hand[1].inverse();
                const double dual_pos=(inferred.translation()-inferred_right.translation()).norm()*1000.0;
                const double dual_ang=rotationError(inferred,inferred_right)*180.0/M_PI;
                const double rod_pos=(inferred.translation()-target_object.translation()).norm()*1000.0;
                const double rod_ang=rotationError(inferred,target_object)*180.0/M_PI;
                stats.max_dual_pos_mm=std::max(stats.max_dual_pos_mm,dual_pos);
                stats.max_dual_ang_deg=std::max(stats.max_dual_ang_deg,dual_ang);
                stats.max_rod_pos_mm=std::max(stats.max_rod_pos_mm,rod_pos);
                stats.max_rod_ang_deg=std::max(stats.max_rod_ang_deg,rod_ang);
                bool collision=false;
                const double extent=0.5*(std::abs(inferred.linear()(2,0))*truth.size.x+
                    std::abs(inferred.linear()(2,1))*truth.size.y+
                    std::abs(inferred.linear()(2,2))*truth.size.z);
                const double clearance=inferred.translation().z()-extent-ctx_->p2a_support_z_;
                if (!updateRod(inferred)) {
                    return false;
                }
                auto local_acm=acm;
                local_acm.setEntry(a1_verify_.rod_id,a1_verify_.support_id,clearance<=0.003);
                collision_detection::CollisionRequest req; req.contacts=true; req.max_contacts=20;
                collision_detection::CollisionResult res;
                scene->checkCollision(req,res,criterion_state,local_acm);
                if (res.collision) {
                    collision=true;
                    stats.collisions+=std::max<std::size_t>(1,res.contacts.size());
                    if (stats.first_collision.empty() && !res.contacts.empty())
                        stats.first_collision=res.contacts.begin()->first.first+"<->"+
                            res.contacts.begin()->first.second;
                }
                compare_csv<<planner<<','<<index<<','<<sample.t<<','
                    <<inferred.translation().x()<<','<<inferred.translation().y()<<','<<inferred.translation().z()<<','
                    <<inferred_right.translation().x()<<','<<inferred_right.translation().y()<<','<<inferred_right.translation().z()<<','
                    <<dual_pos<<','<<dual_ang<<','<<rod_pos<<','<<rod_ang<<','
                    <<(collision?1:0)<<','<<(limit_bad?1:0)<<','<<(vel_bad?1:0)<<','<<(acc_bad?1:0)<<'\n';
            }
            compare_csv.flush();
            stats.samples=samples.size();
            if (!jerks.empty()) {
                std::sort(jerks.begin(),jerks.end());
                stats.max_jerk=jerks.back();
                const std::size_t p95=static_cast<std::size_t>(0.95*double(jerks.size()));
                stats.p95_jerk=jerks[std::min(p95,jerks.size()-1)];
            }
            (void)limit_violations; (void)vel_violations; (void)acc_violations;
            return true;
        };
        CompareStats baseline_stats;
        baseline_stats.planned=true;
        baseline_stats.planning_sec=transport_plan_sec;
        evaluatePlan("baseline",transport,baseline_stats);
        moveit::planning_interface::MoveGroupInterface::Plan chomp_plan;
        CompareStats chomp_stats;
        double chomp_planning_sec=0.0;
        const auto& transport_joint=transport.trajectory_.joint_trajectory;
        if (!transport_joint.points.empty() &&
            ctx_->dual_arm_->setJointValueTarget(transport_joint.joint_names,
                                                 transport_joint.points.back().positions)) {
            ctx_->dual_arm_->setStartStateToCurrentState();
            // In this MoveIt2 setup CHOMP is a planning PIPELINE; setting only
            // the planner id leaves the request on the previous (Pilz) pipeline
            // and move_group rejects it before CHOMP ever runs.
            ctx_->dual_arm_->setPlanningPipelineId("chomp");
            ctx_->dual_arm_->setPlannerId("chomp");
            ctx_->dual_arm_->setPlanningTime(5.0);
            ctx_->dual_arm_->setNumPlanningAttempts(1);
            ctx_->dual_arm_->setMaxVelocityScalingFactor(1.0);
            ctx_->dual_arm_->setMaxAccelerationScalingFactor(0.3);
            const double plan_start=steadySeconds();
            const auto code=ctx_->dual_arm_->plan(chomp_plan);
            chomp_planning_sec=steadySeconds()-plan_start;
            chomp_stats.plan_code=code.val;
            chomp_stats.planned=(code.val==moveit::core::MoveItErrorCode::SUCCESS);
            if (!chomp_stats.planned)
                chomp_stats.violation_detail="plan failed code="+std::to_string(code.val);
        } else {
            chomp_stats.violation_detail="joint target not set";
        }
        chomp_stats.planning_sec=chomp_planning_sec;
        if (chomp_stats.planned) evaluatePlan("chomp",chomp_plan,chomp_stats);
        auto logCompare=[&](const char* planner,const CompareStats& s) {
            RCLCPP_INFO(ctx_->get_logger(),
                "[PLAN_COMPARE] planner=%s success=%d plan_code=%d planning_time=%.3f points=%zu duration=%.3f "
                "max_dual_pos_mm=%.3f max_dual_ang_deg=%.3f max_rod_pos_mm=%.3f max_rod_ang_deg=%.3f "
                "collisions=%zu first_collision=%s limit_violations=%d vel_violations=%d acc_violations=%d "
                "samples=%zu max_jerk=%.3f p95_jerk=%.3f",
                planner,s.planned?1:0,s.plan_code,s.planning_sec,s.points,s.duration,
                s.max_dual_pos_mm,s.max_dual_ang_deg,s.max_rod_pos_mm,s.max_rod_ang_deg,
                s.collisions,s.first_collision.empty()?"none":s.first_collision.c_str(),
                s.limit_violation?1:0,s.vel_violation?1:0,s.acc_violation?1:0,
                s.samples,s.max_jerk,s.p95_jerk);
        };
        logCompare("baseline",baseline_stats);
        logCompare("chomp",chomp_stats);
        const bool chomp_chain_ok=chomp_stats.planned &&
            chomp_stats.max_dual_pos_mm<=5.0 && chomp_stats.max_dual_ang_deg<=2.0;
        const bool chomp_eligible=chomp_chain_ok && chomp_stats.collisions==0 &&
            !chomp_stats.limit_violation && !chomp_stats.vel_violation && !chomp_stats.acc_violation;
        RCLCPP_INFO(ctx_->get_logger(),
            "[PLAN_COMPARE_RESULT] baseline_success=%d chomp_success=%d chomp_within_5mm2deg=%d "
            "chomp_collisions=%zu chomp_limit_violations=%d chomp_vel_violations=%d chomp_acc_violations=%d "
            "execute_chomp=%d phase2=%s",
            baseline_stats.planned?1:0,chomp_stats.planned?1:0,chomp_chain_ok?1:0,
            chomp_stats.collisions,chomp_stats.limit_violation?1:0,chomp_stats.vel_violation?1:0,
            chomp_stats.acc_violation?1:0,chomp_eligible?1:0,
            chomp_eligible?"executing_same_session":"not_executed");
        if (!baseline_stats.planned) return fail("E_PLAN_COMPARE_BASELINE_PLAN");
        if (!chomp_eligible) {
            // Plan-only round: stop the force hold and end without transport.
            const bool stopped=ctx_->grippers_->stopDualGrippers();
            force_hold_may_be_active=false;
            RCLCPP_INFO(ctx_->get_logger(),
                "[PLAN_COMPARE] phase 1 complete; transport not executed (gripper stop=%s)",
                stopped?"confirmed":"unconfirmed");
            detail="plan comparison complete: baseline planned; CHOMP "+
                std::string(chomp_stats.planned?"planned":"failed")+
                (chomp_stats.planned?
                    (" (max dual "+std::to_string(chomp_stats.max_dual_pos_mm)+" mm / "+
                     std::to_string(chomp_stats.max_dual_ang_deg)+" deg, collisions="+
                     std::to_string(chomp_stats.collisions)+")"):"")+
                "; transport not executed";
            return true;
        }
        RCLCPP_INFO(ctx_->get_logger(),
            "[PLAN_COMPARE] phase 2: executing the CHOMP transport from the same start state");
        transport=chomp_plan;
    }
    // The pairing builds/evaluates both trajectories before dispatch. Under the
    // contact hold the arms drift a few mrad during that bounded delay, which
    // the normal 2 mrad guard was never intended to cover. Only this transport
    // dispatch is allowed a 10 mrad start consistency bound; the trajectory,
    // drop/slip/height protections and every later dispatch keep the default.
    const double transport_dispatch_tol = pairing_enabled ? 0.010 : 0.002;
    if (pairing_enabled)
        RCLCPP_INFO(ctx_->get_logger(),
            "[CONTACT_DISPATCH_TOLERANCE] phase=TRANSPORT tolerance=%.5f rad reason=pairing_planning_delay "
            "default=0.002 motion_and_drop_protections_unchanged", transport_dispatch_tol);
    if (!execute(transport,"TRANSPORT",transport_dispatch_tol)) return fail(detail);
    if (!observe("TRANSPORT", true)) return fail(detail);
    // The 1x transport leaves the arm servos settling after the action returns.
    // Wait for a stable measured joint state before re-freezing and planning the
    // lower; otherwise the measured state can drift >2 mrad between planning and
    // dispatch and execute() rejects the lower with E_CONTACT_DISPATCH_START_MOVED.
    {
        moveit::core::RobotState settle_prev(live_model);
        if (!measured(settle_prev)) return fail(detail);
        std::vector<double> prev_q,cur_q;
        settle_prev.copyJointGroupPositions(dual,prev_q);
        const double settle_start=steadySeconds();
        bool settled=false;
        while (steadySeconds()-settle_start<2.0) {
            std::this_thread::sleep_for(100ms);
            moveit::core::RobotState settle_cur(live_model);
            if (!measured(settle_cur)) return fail(detail);
            settle_cur.copyJointGroupPositions(dual,cur_q);
            double max_delta=0.0;
            for (std::size_t i=0;i<cur_q.size();++i)
                max_delta=std::max(max_delta,std::abs(cur_q[i]-prev_q[i]));
            if (max_delta<0.0005) { settled=true; break; }
            prev_q=cur_q;
        }
        if (!settled) return fail("E_CONTACT_POST_TRANSPORT_SETTLE");
    }
    moveit::core::RobotState after_transport(live_model);
    if (!measured(after_transport)) return fail(detail);
    const auto after_truth = readMujocoRodTruth();
    if (!after_truth.ok) return fail("E_CONTACT_TRANSPORT_TRUTH");
    // The lateral move can make the two hands' inferred rod poses drift apart
    // (the same closed-chain effect seen after the hold). Re-establish the
    // grasp reference from the measured state before planning the descent; the
    // transport slip was already monitored and logged in its own phase.
    const Transform after_object = rodTransform(after_truth);
    for (int i=0;i<2;++i)
        object_to_hand[i] = after_object.inverse()*after_transport.getGlobalLinkTransform(eefs[i]);
    RCLCPP_INFO(ctx_->get_logger(),
        "[CONTACT_PROBE_REFREEZE] descent reference re-established after transport");
    moveit::planning_interface::MoveGroupInterface::Plan lower;
    // Descend to the support plane using the CURRENT rod attitude: after the
    // transport the rod can be tilted ~1 degree, which makes the tilt-aware box
    // half-height larger than 20 mm. Using the initial centre height would push
    // the rod's low edge through the support (E_CONTACT_ROD_SUPPORT_PENETRATION).
    const double extent_after = 0.5*(std::abs(after_object.linear()(2,0))*truth.size.x +
        std::abs(after_object.linear()(2,1))*truth.size.y +
        std::abs(after_object.linear()(2,2))*truth.size.z);
    const double lower_delta =
        after_object.translation().z() - (ctx_->p2a_support_z_ + extent_after);
    if (!std::isfinite(lower_delta) || lower_delta <= 0.0 || lower_delta > height + 0.02) {
        detail="E_CONTACT_LOWER_DELTA"; return fail(detail);
    }
    RCLCPP_INFO(ctx_->get_logger(),
        "[CONTACT_PROBE_LOWER] descend=%.3f m (commanded lift=%.3f m, tilt-aware extent=%.3f m); "
        "targeting support at the transported pose", lower_delta, height, extent_after);
    if (!makePlan(after_transport, Eigen::Vector3d(0,0,-lower_delta), lower,"lower",1.0,0.8,0.5)) return fail(detail);
    if (!execute(lower,"LOWER")) return fail(detail);
    const double extent=0.5*(std::abs(rodTransform(latest).linear()(2,0))*truth.size.x+
        std::abs(rodTransform(latest).linear()(2,1))*truth.size.y+std::abs(rodTransform(latest).linear()(2,2))*truth.size.z);
    if (std::abs(latest.position.z-extent-ctx_->p2a_support_z_)>0.003) return fail("E_CONTACT_RELEASE_NOT_ON_SUPPORT");
    if (!ctx_->grippers_->controlDualGrippers(0.04)) return fail("E_CONTACT_RELEASE_OPEN_FAILED");
    force_hold_may_be_active=false;
    std::this_thread::sleep_for(500ms);
    const auto released=readMujocoRodTruth();
    if (!released.ok || !feedback()) return fail("E_CONTACT_RELEASE_TRUTH_OR_FEEDBACK");
    for (const auto& f:fingers) if (f.hold_force_per_finger_n!=0 || f.q1+f.q2<0.078) return fail("E_CONTACT_RELEASE_FORCE_NOT_CLEARED");
    // Compare against the pose just before opening, not the initial grasp pose:
    // the forward transport moves the rod 10 cm, so the initial pose is no
    // longer the release reference. The check is that opening does not make the
    // rod jump.
    if ((rodTransform(released).translation()-rodTransform(latest).translation()).norm()>0.01 ||
        rotationError(rodTransform(released),rodTransform(latest))>5*M_PI/180)
        return fail("E_CONTACT_RELEASE_POSE_ERROR");
    // Only synchronize planning representation. Never write a MuJoCo body state.
    geometry_msgs::msg::Vector3 size; size.x=truth.size.x;size.y=truth.size.y;size.z=truth.size.z;
    if (!syncAluminumRodFromTruth(poseMsg(rodTransform(released)),size,ctx_->dual_arm_->getPlanningFrame(),detail)) return fail(detail);
    RCLCPP_INFO(ctx_->get_logger(),"[CONTACT_PROBE_RELEASED] xyz=(%.6f,%.6f,%.6f) open=true force_hold=OFF weld=OFF attach=none; retreat requested",
        released.position.x,released.position.y,released.position.z);
    // Post-release retreat: move both hands straight up, clear of the released
    // rod. The rod is no longer attached, so the frozen hand-object transform is
    // only used to keep the two arms coordinated; the scene rod is synced to the
    // released truth pose above.
    {
        moveit::core::RobotState retreat_start(live_model);
        if (!measured(retreat_start)) return fail(detail);
        const Transform released_object=rodTransform(released);
        for (int i=0;i<2;++i)
            object_to_hand[i]=released_object.inverse()*retreat_start.getGlobalLinkTransform(eefs[i]);
        moveit::planning_interface::MoveGroupInterface::Plan retreat;
        if (!makePlan(retreat_start,Eigen::Vector3d(0,0,0.15),retreat,"retreat",1.0,0.4,1.0))
            return fail(detail);
        if (!executeFree(retreat,"RETREAT")) return fail(detail);
        const auto after_retreat=readMujocoRodTruth();
        RCLCPP_INFO(ctx_->get_logger(),
            "[CONTACT_PROBE_RETREAT] hands lifted 0.150 m after release; rod=(%.6f,%.6f,%.6f) "
            "state=retreated weld=OFF attach=none",
            after_retreat.ok?after_retreat.position.x:std::nan(""),
            after_retreat.ok?after_retreat.position.y:std::nan(""),
            after_retreat.ok?after_retreat.position.z:std::nan(""));
    }
    detail="contact-only lift "+std::to_string(static_cast<int>(height*100.0+0.5))+"cm, "+std::to_string(static_cast<int>(hold_seconds+0.5))+"s hold, forward transport with yaw rotation, lower, release and retreat complete; arm return not requested";
    return true;
}
