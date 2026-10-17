#include "dual_arm_carry_task/dual_arm_controller.hpp"
#include "dual_arm_carry_task/coordinated_path_optimizer.hpp"
#include "dual_arm_carry_task/time_profile_optimizer.hpp"
#include "dual_arm_carry_task/gripper_client.hpp"
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
            const double force_values[]{f.normal_left_n,f.normal_right_n,f.tangent_left_n,f.tangent_right_n,f.rod_force_z_n,f.hold_force_per_finger_n};
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
    const auto before=readMujocoRodTruth();
    if (!before.ok) return fail("E_CONTACT_TRUTH_BEFORE: "+before.detail);
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
        hold_diagnostic_only ? "fixed-arm-mechanism-diagnostic" : "full-load-probe",
        height, hold_seconds, lift_slip_tol*1000.0, pos_tol*1000.0, rod_rise_target*1000.0,
        hold_seconds, lift_velocity_scale, lift_acceleration_scale);
    // S2 transport plan-only evaluation builds its sample without touching the
    // gripper or the arms, so the whole grasp sequence below is skipped.
    const std::array<std::string,2> eefs{{a1_verify_.left_eef,a1_verify_.right_eef}};
    std::array<Transform,2> object_to_hand{{Transform::Identity(),Transform::Identity()}};
    Transform object=Transform::Identity();
    MujocoRodTruth initial_truth;
    moveit::core::RobotState start(live_model);
    // One arm may have entered direct-force hold even when its peer's action
    // fails. From the first contact-goal attempt, failure cleanup sends the
    // measured-position Stop service to both sides before the simulator exits.
    if (!plan_only) {
    force_hold_may_be_active=true;
    if (!ctx_->grippers_->controlDualGrippersContact(force)) return fail("E_CONTACT_GRASP_ACTION_FAILED_OR_UNKNOWN");
    if (!measured(start)) return fail(detail);
    initial_truth=readMujocoRodTruth();
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
    auto makePlan=[&](const moveit::core::RobotState& origin,const Eigen::Vector3d& delta,
                      moveit::planning_interface::MoveGroupInterface::Plan& plan,
                      const char* label,
                      double vscale,
                      double ascale,
                      double min_total,
                      double dyaw=0.0,
                      bool optimize_path=false) {
        if (!valid(origin)) return false;
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
        const double baseline_start=steadySeconds();
        if (!makePlan(lifted,transport_delta,baseline,"transport_baseline",1.0,0.3,transport_stretch,
                      transport_dyaw,false)) return planFail(detail);
        const double baseline_plan_sec=steadySeconds()-baseline_start;
        const double optimized_start=steadySeconds();
        if (!makePlan(lifted,transport_delta,optimized,"transport_optimized",1.0,0.3,transport_stretch,
                      transport_dyaw,true)) return planFail(detail);
        const double optimized_plan_sec=steadySeconds()-optimized_start;

        // Identical evaluation for both plans: same object path, same sampling,
        // same limit and collision checks, jerk from the quintic in real time.
        const Transform first_object=lifted.getGlobalLinkTransform(eefs[0])*object_to_hand[0].inverse();
        auto expectedObject=[&](double u) {
            Transform t=first_object;
            t.translation()+=transport_delta*u;
            t.linear()=Eigen::AngleAxisd(transport_dyaw*u,Eigen::Vector3d::UnitZ()).toRotationMatrix()*first_object.linear();
            return t;
        };
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
        std::ofstream compare_csv("experiment_results/a1_runs/"+run_id+"/transport_path_compare.csv");
        if (!compare_csv) return planFail("E_TRANSPORT_OPT_CSV_OPEN");
        compare_csv<<std::setprecision(10)
            <<"plan,sample_index,t,dual_pos_mm,dual_ang_deg,rod_pos_mm,rod_ang_deg,"
              "collision,joint_limit_violation,vel_violation,acc_violation\n";
        // `progress_of_time`, when given, returns the object-path parameter for
        // an absolute time. That is needed whenever the trajectory is re-timed:
        // the object parameter no longer follows the segment fraction.
        auto evaluateTransport=[&](const char* name,
                                   const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                   TransportPlanStats& stats,
                                   const std::function<double(double)>& progress_of_time={}) -> bool {
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
            std::vector<double> q;
            for (std::size_t j=0;j<jt.joint_names.size();++j) q.push_back(jt.points.front().positions[j]);
            // Waypoint-only check, on the criterion model, exactly like valid().
            std::size_t sample_index=0;
            double acc_sum=0.0,jerk_sum=0.0; std::size_t acc_count=0,jerk_count=0;
            for (std::size_t k=0;k<jt.points.size();++k) {
                moveit::core::RobotState state(live_model);
                state.setVariablePositions(jt.joint_names,jt.points[k].positions);
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
                    timed.setVariablePositions(jt.joint_names,sq); timed.update();
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
                    compare_csv<<name<<','<<sample_index++<<','<<t<<','<<dual_pos<<','<<dual_ang<<','
                        <<rod_pos<<','<<rod_ang<<','<<(res.collision?1:0)<<','<<(limit_bad?1:0)<<','
                        <<(stats.vel_violation?1:0)<<','<<(stats.acc_violation?1:0)<<'\n';
                }
            }
            compare_csv.flush();
            stats.acc_rms=acc_count>0?std::sqrt(acc_sum/double(acc_count)):0.0;
            stats.jerk_rms=jerk_count>0?std::sqrt(jerk_sum/double(jerk_count)):0.0;
            return true;
        };
        TransportPlanStats baseline_stats,optimized_stats;
        if (!evaluateTransport("baseline",baseline,baseline_stats)) return planFail(detail);
        if (!evaluateTransport("optimized",optimized,optimized_stats)) return planFail(detail);

        // Persist the baseline transport trajectory and its provenance so the
        // time profile can be optimised offline without restarting the
        // simulation. Long format keeps the joint order explicit.
        {
            std::ofstream traj("experiment_results/a1_runs/"+run_id+"/transport_baseline_trajectory.csv");
            if (!traj) return planFail("E_TRANSPORT_TRAJ_CSV_OPEN");
            traj<<std::setprecision(17)
                <<"joint_name,point_index,time_from_start,position,velocity,acceleration\n";
            const auto& jt=baseline.trajectory_.joint_trajectory;
            for (std::size_t j=0;j<jt.joint_names.size();++j) {
                for (std::size_t k=0;k<jt.points.size();++k) {
                    const auto& p=jt.points[k];
                    traj<<jt.joint_names[j]<<','<<k<<','
                        <<rclcpp::Duration(p.time_from_start).seconds()<<','
                        <<p.positions[j]<<','<<(j<p.velocities.size()?p.velocities[j]:0.0)<<','
                        <<(j<p.accelerations.size()?p.accelerations[j]:0.0)<<'\n';
                }
            }
            traj.flush();
            if (!traj) return planFail("E_TRANSPORT_TRAJ_CSV_WRITE");

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
                <<"fingers=open_per_a1_grasp_solution\n"
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
        {
            tpo::ContinuousPath tp_path;
            std::vector<tpo::TrajectoryPoint> tp_points;
            std::vector<tpo::JointLimits> tp_limits;
            std::string tp_error;
            const auto& baseline_jt=baseline.trajectory_.joint_trajectory;
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
            if (!tp_path.build(baseline_jt.joint_names,tp_points,tp_error))
                { RCLCPP_WARN(ctx_->get_logger(),"[TIME_PROFILE] skipped: %s",tp_error.c_str()); }
            else {
                tpo::TimeProfileOptions tp_options;
                const double tp_start=steadySeconds();
                const auto tp_result=tpo::optimizeTimeProfile(tp_path,tp_limits,tp_path.duration(),tp_options);
                const double tp_sec=steadySeconds()-tp_start;
                if (!tp_result.ok) {
                    RCLCPP_WARN(ctx_->get_logger(),"[TIME_PROFILE] failed: %s",tp_result.detail.c_str());
                } else {
                    auto pct=[](double b,double c){ return b>0.0?100.0*(c-b)/b:0.0; };
                    RCLCPP_INFO(ctx_->get_logger(),
                        "[TIME_PROFILE_OPT] duration=%.6f degree=%d params=%d evaluations=%d solve_sec=%.3f "
                        "continuous_jerk_rms=%.6f->%.6f (%.3f%%) continuous_jerk_max=%.6f->%.6f "
                        "exported_jerk_rms=%.6f->%.6f (%.3f%%) exported_jerk_max=%.6f->%.6f",
                        tp_path.duration(),tp_options.degree,static_cast<int>(tp_result.parameters.size()),
                        tp_result.evaluations,tp_sec,
                        tp_result.baseline_continuous.jerk_rms,tp_result.candidate_continuous.jerk_rms,
                        pct(tp_result.baseline_continuous.jerk_rms,tp_result.candidate_continuous.jerk_rms),
                        tp_result.baseline_continuous.jerk_max,tp_result.candidate_continuous.jerk_max,
                        tp_result.baseline_metrics.jerk_rms,tp_result.candidate_metrics.jerk_rms,
                        pct(tp_result.baseline_metrics.jerk_rms,tp_result.candidate_metrics.jerk_rms),
                        tp_result.baseline_metrics.jerk_max,tp_result.candidate_metrics.jerk_max);
                    // Build the candidate JointTrajectory exactly as it would be
                    // dispatched, then check it on the model.
                    moveit::planning_interface::MoveGroupInterface::Plan tp_plan;
                    auto& tp_jt=tp_plan.trajectory_.joint_trajectory;
                    tp_jt.joint_names=baseline_jt.joint_names;
                    for (const auto& p:tp_result.candidate_export) {
                        trajectory_msgs::msg::JointTrajectoryPoint point;
                        point.positions.assign(p.q.data(),p.q.data()+p.q.size());
                        point.velocities.assign(p.v.data(),p.v.data()+p.v.size());
                        point.accelerations.assign(p.a.data(),p.a.data()+p.a.size());
                        point.time_from_start=rclcpp::Duration::from_seconds(p.time);
                        tp_jt.points.push_back(point);
                    }
                    // Object-path parameter as a function of s. The baseline
                    // knots are (s_k = t_k/T, sigma_k = k/steps); between knots
                    // the parameter is linear in s, which reproduces the
                    // production check for the linear clock.
                    const double T_tp=tp_path.duration();
                    const std::size_t steps_tp=tp_points.size()-1;
                    std::vector<double> s_knots,sigma_knots;
                    for (std::size_t k=0;k<tp_points.size();++k) {
                        s_knots.push_back(tp_points[k].time/T_tp);
                        sigma_knots.push_back(static_cast<double>(k)/static_cast<double>(steps_tp));
                    }
                    auto sigmaOf=[&](double s) {
                        s=std::clamp(s,0.0,1.0);
                        for (std::size_t k=1;k<s_knots.size();++k) {
                            if (s<=s_knots[k]) {
                                const double span=std::max(1e-12,s_knots[k]-s_knots[k-1]);
                                return sigma_knots[k-1]+(s-s_knots[k-1])/span*(sigma_knots[k]-sigma_knots[k-1]);
                            }
                        }
                        return 1.0;
                    };
                    tpo::BernsteinTimeMap tp_map(tp_options.degree);
                    tp_map.setParameters(tp_result.parameters);
                    const auto progress=[&](double t){ return sigmaOf(tp_map.s(t/T_tp)); };
                    TransportPlanStats tp_base_stats,tp_cand_stats;
                    if (!evaluateTransport("time_profile_baseline",baseline,tp_base_stats)) return planFail(detail);
                    if (!evaluateTransport("time_profile_candidate",tp_plan,tp_cand_stats,progress)) return planFail(detail);
                    const bool tp_ok=tp_cand_stats.collisions==0 && !tp_cand_stats.limit_violation &&
                        !tp_cand_stats.vel_violation && !tp_cand_stats.acc_violation &&
                        tp_cand_stats.max_sample_dual_pos_mm<=5.0 && tp_cand_stats.max_sample_dual_ang_deg<=2.0 &&
                        tp_cand_stats.max_rod_pos_mm<=5.0 && tp_cand_stats.max_rod_ang_deg<=2.0;
                    RCLCPP_INFO(ctx_->get_logger(),
                        "[TIME_PROFILE_CHECK] candidate_pass=%d collisions=%zu limits=%d vel=%d acc=%d "
                        "dual_max=(%.6f mm,%.6f deg) rod_path_max=(%.6f mm,%.6f deg) "
                        "vel_ratio=%.4f acc_ratio=%.4f jerk_rms(exported)=%.6f jerk_max=%.6f",
                        tp_ok?1:0,tp_cand_stats.collisions,tp_cand_stats.limit_violation?1:0,
                        tp_cand_stats.vel_violation?1:0,tp_cand_stats.acc_violation?1:0,
                        tp_cand_stats.max_sample_dual_pos_mm,tp_cand_stats.max_sample_dual_ang_deg,
                        tp_cand_stats.max_rod_pos_mm,tp_cand_stats.max_rod_ang_deg,
                        tp_cand_stats.max_vel_ratio,tp_cand_stats.max_acc_ratio,
                        tp_cand_stats.jerk_rms,tp_cand_stats.jerk_max);
                    RCLCPP_INFO(ctx_->get_logger(),
                        "[TIME_PROFILE_BASELINE_CHECK] collisions=%zu limits=%d vel=%d acc=%d "
                        "dual_max=(%.6f mm,%.6f deg) rod_path_max=(%.6f mm,%.6f deg) "
                        "vel_ratio=%.4f acc_ratio=%.4f jerk_rms(exported)=%.6f jerk_max=%.6f",
                        tp_base_stats.collisions,tp_base_stats.limit_violation?1:0,
                        tp_base_stats.vel_violation?1:0,tp_base_stats.acc_violation?1:0,
                        tp_base_stats.max_sample_dual_pos_mm,tp_base_stats.max_sample_dual_ang_deg,
                        tp_base_stats.max_rod_pos_mm,tp_base_stats.max_rod_ang_deg,
                        tp_base_stats.max_vel_ratio,tp_base_stats.max_acc_ratio,
                        tp_base_stats.jerk_rms,tp_base_stats.jerk_max);
                    std::ofstream tpc("experiment_results/a1_runs/"+run_id+"/time_profile_candidate.csv");
                    if (!tpc) return planFail("E_TIME_PROFILE_CSV_OPEN");
                    tpc<<std::setprecision(17)<<"joint_name,point_index,time_from_start,position,velocity,acceleration\n";
                    for (std::size_t j=0;j<tp_jt.joint_names.size();++j)
                        for (std::size_t k=0;k<tp_jt.points.size();++k) {
                            const auto& p=tp_jt.points[k];
                            tpc<<tp_jt.joint_names[j]<<','<<k<<','
                               <<rclcpp::Duration(p.time_from_start).seconds()<<','
                               <<p.positions[j]<<','<<p.velocities[j]<<','<<p.accelerations[j]<<'\n';
                        }
                    tpc.flush();
                }
            }
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
               " points / optimized "+std::to_string(optimized_stats.points)+" points; no motion sent";
        return true;
    }
    moveit::planning_interface::MoveGroupInterface::Plan lift;

    if (!makePlan(start,Eigen::Vector3d(0,0,height),lift,"lift",1.0,0.4,1.0)) return fail(detail);
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
    MujocoRodTruth latest;
    auto observe=[&](const char* phase,bool require_height) {
        moveit::core::RobotState actual(live_model);
        if (!measured(actual)) return false;
        for (const auto& f:fingers) if (std::abs(f.hold_force_per_finger_n-force)>1e-6) { detail="E_CONTACT_FORCE_MODE_LOST"; return false; }
        latest=readMujocoRodTruth();
        const double qnorm=std::sqrt(latest.orientation.x*latest.orientation.x+latest.orientation.y*latest.orientation.y+
            latest.orientation.z*latest.orientation.z+latest.orientation.w*latest.orientation.w);
        if (!latest.ok || std::abs(qnorm-1.0)>1e-3) { detail="E_CONTACT_TRUTH_INVALID: "+latest.detail; return false; }
        const Transform physical=rodTransform(latest);
        double slips[2], angles[2];
        for (int i=0;i<2;++i) {
            const Transform expected=actual.getGlobalLinkTransform(eefs[i])*object_to_hand[i].inverse();
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
        if (max_slip>lift_slip_tol || max_angle>angle_tol) { detail="E_CONTACT_SLIP_OR_DROP"; return false; }
        if (require_height && latest.position.z<initial_truth.position.z+rod_rise_target) { detail="E_CONTACT_ROD_NOT_LIFTED"; return false; }
        return true;
    };
    auto execute=[&](const moveit::planning_interface::MoveGroupInterface::Plan& plan,const char* phase) {
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
    if (!execute(transport,"TRANSPORT")) return fail(detail);
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
