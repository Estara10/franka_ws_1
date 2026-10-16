#include "dual_arm_carry_task/dual_arm_controller.hpp"
#include "dual_arm_carry_task/gripper_client.hpp"
#include "dual_arm_carry_task/s1_time_scaling.hpp"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/conversions.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
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

bool DualArmController::runContactLoadProbe(const a1t::TruthPayload& truth,std::string& detail) {
    using namespace std::chrono_literals;
    // First bounded experiment, deliberately independent of S1 ideal_weld.
    constexpr double force=5.0, height=0.05, hold_seconds=2.0;
    constexpr double pos_tol=0.005, angle_tol=2.0*M_PI/180.0;
    const std::array<std::string,2> arms{{"mj_left","mj_right"}};
    std::array<DualArmCarryTask::GripperContactFeedback,2> fingers;
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
                "normal=(%.3f,%.3f) tangent=(%.3f,%.3f) rod_force_z=%.3f "
                "seq=%llu sim_time=%.6f weld=(%d,%d) rod_constraint=%d",
                reason, arms[i].c_str(), sample.q1, sample.q2, sample.command_per_finger_m,
                sample.closing_force_limit_per_finger_n, sample.hold_force_per_finger_n,
                sample.rod_pad_contact_count, sample.left_contact ? 1 : 0,
                sample.right_contact ? 1 : 0, sample.normal_left_n, sample.normal_right_n,
                sample.tangent_left_n, sample.tangent_right_n, sample.rod_force_z_n,
                static_cast<unsigned long long>(sample.seq), sample.simulation_time,
                sample.weld_left_active ? 1 : 0, sample.weld_right_active ? 1 : 0,
                sample.rod_constraint_active ? 1 : 0);
        }
    };
    auto fail=[&](const std::string& reason) {
        detail=reason;
        logSnapshot(reason.c_str());
        RCLCPP_ERROR(ctx_->get_logger(),"[CONTACT_PROBE_FAILURE] %s",reason.c_str());
        return false;
    };
    if (!a1_verify_.valid || !a1_verify_.pregrasp_reached_this_round || !a1_verify_.descent_plan_valid ||
        ctx_->getCurrentState()!=TaskState::PREGRASP_REACHED) return fail("E_CONTACT_PRECONDITION");
    if (!checkControllerStatesEnabled(detail)) return fail("E_CONTACT_CONTROLLER: "+detail);
    std::string run_id=ctx_->expected_run_id_;
    if (run_id.empty() || run_id == "." || run_id == ".." || run_id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-")!=std::string::npos)
        return fail("E_CONTACT_RUN_ID");
    std::ofstream csv("experiment_results/a1_runs/"+run_id+"/contact_probe.csv");
    if (!csv) return fail("E_CONTACT_CSV_OPEN");
    csv << std::setprecision(10) << "phase,sim_time,rod_x,rod_y,rod_z,left_slip_m,left_angle_rad,right_slip_m,right_angle_rad,left_q1,left_q2,right_q1,right_q2,left_command_per_finger_m,right_command_per_finger_m,left_force_limit_n,right_force_limit_n,left_hold_force_n,right_hold_force_n,left_pad_contact_count,right_pad_contact_count,left_contact_1,left_contact_2,right_contact_1,right_contact_2,left_normal1_n,left_normal2_n,right_normal1_n,right_normal2_n,left_tangent1_n,left_tangent2_n,right_tangent1_n,right_tangent2_n,left_rod_force_z_n,right_rod_force_z_n,left_seq,right_seq,weld_left,weld_right,rod_constraint\n";
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
    const auto gate=std::chrono::steady_clock::now()+2s;
    bool ready=false;
    while (rclcpp::ok() && std::chrono::steady_clock::now()<gate) {
        if (feedback()) { ready=true; break; } std::this_thread::sleep_for(20ms);
    }
    if (!ready) return fail(detail);
    const auto before=readMujocoRodTruth();
    if (!before.ok) return fail("E_CONTACT_TRUTH_BEFORE: "+before.detail);
    const Eigen::Vector3d frozen(truth.position.x,truth.position.y,truth.position.z);
    if ((rodTransform(before).translation()-frozen).norm()>0.005 ||
        a1t::rotationDistanceRad(before.orientation,{truth.orientation.x,truth.orientation.y,truth.orientation.z,truth.orientation.w})>5*M_PI/180)
        return fail("E_CONTACT_FROZEN_REFERENCE_CHANGED");
    RCLCPP_INFO(ctx_->get_logger(),"[CONTACT_PROBE_CONFIG] force_per_finger=5N lift=0.05m hold=2s slip_limit=5mm/2deg weld=OFF; model/friction/mass unchanged");
    if (!ctx_->grippers_->controlDualGrippersContact(force)) return fail("E_CONTACT_GRASP_ACTION_FAILED_OR_UNKNOWN");
    moveit::core::RobotState start(live_model);
    if (!measured(start)) return fail(detail);
    const auto initial_truth=readMujocoRodTruth();
    const double initial_qnorm=std::sqrt(initial_truth.orientation.x*initial_truth.orientation.x+initial_truth.orientation.y*initial_truth.orientation.y+
        initial_truth.orientation.z*initial_truth.orientation.z+initial_truth.orientation.w*initial_truth.orientation.w);
    if (!initial_truth.ok || std::abs(initial_qnorm-1.0)>1e-3) return fail("E_CONTACT_TRUTH_AFTER_CLOSE");
    const Transform object=rodTransform(initial_truth);
    if ((object.translation()-rodTransform(before).translation()).norm()>pos_tol ||
        rotationError(object,rodTransform(before))>angle_tol) return fail("E_CONTACT_ROD_MOVED_DURING_CLOSE");
    const std::array<std::string,2> eefs{{a1_verify_.left_eef,a1_verify_.right_eef}};
    const std::array<Transform,2> object_to_hand{{object.inverse()*start.getGlobalLinkTransform(eefs[0]),
                                               object.inverse()*start.getGlobalLinkTransform(eefs[1])}};
    // Preserve the descent arrival limits after closing.
    const std::array<a1v::ArmTarget,2> grasp{{a1_verify_.grasp_target.left,a1_verify_.grasp_target.right}};
    for (int i=0;i<2;++i) {
        Transform desired=Transform::Identity();
        desired.translation()=Eigen::Vector3d(grasp[i].position.x,grasp[i].position.y,grasp[i].position.z);
        desired.linear()=Eigen::Quaterniond(grasp[i].orientation.w,grasp[i].orientation.x,grasp[i].orientation.y,grasp[i].orientation.z).normalized().toRotationMatrix();
        if ((desired.translation()-start.getGlobalLinkTransform(eefs[i]).translation()).norm()>pos_tol ||
            rotationError(desired,start.getGlobalLinkTransform(eefs[i]))>angle_tol) return fail("E_CONTACT_GRASP_ARRIVAL_CHANGED");
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
    auto valid=[&](const moveit::core::RobotState& live) {
        moveit::core::RobotState criterion(criterion_model);
        for (const auto& name:criterion_model->getVariableNames()) criterion.setVariablePosition(name,live.getVariablePosition(name));
        criterion.update();
        if (!criterion.satisfiesBounds(criterion_dual,1e-4)) { detail="E_CONTACT_JOINT_BOUNDS"; return false; }
        const Transform inferred=live.getGlobalLinkTransform(eefs[0])*object_to_hand[0].inverse();
        const Transform inferred_right=live.getGlobalLinkTransform(eefs[1])*object_to_hand[1].inverse();
        if ((inferred.translation()-inferred_right.translation()).norm()>pos_tol || rotationError(inferred,inferred_right)>angle_tol) {
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
    auto makePlan=[&](const moveit::core::RobotState& origin,double delta,
                      moveit::planning_interface::MoveGroupInterface::Plan& plan) {
        robot_trajectory::RobotTrajectory trajectory(live_model,ctx_->dual_arm_group_);
        moveit::core::RobotState seed(origin); trajectory.addSuffixWayPoint(seed,0.0);
        if (!valid(seed)) return false;
        const Transform first_object=seed.getGlobalLinkTransform(eefs[0])*object_to_hand[0].inverse();
        // Use finer object-space increments and longer segment times for the
        // contact lift. The previous 5 mm / 0.5 s segments produced intermittent
        // pad-contact loss and impulsive re-contact before the rod could leave
        // the support surface.
        constexpr int kPathSteps = 20;
        for (int k=1;k<=kPathSteps;++k) {
            Transform target=first_object;
            target.translation().z()+=delta*k/static_cast<double>(kPathSteps);
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
            std::vector<double> a,b; seed.copyJointGroupPositions(dual,a); next.copyJointGroupPositions(dual,b);
            const double jump=a1v::maxAbsJointDelta(a,b);
            if (!std::isfinite(jump) || jump>0.15) { detail="E_CONTACT_IK_BRANCH_JUMP"; return false; }
            const auto n=std::max<std::size_t>(1,a1v::subdivisionsFor(jump,0.01));
            for (std::size_t j=1;j<=n;++j) {
                const double alpha=double(j)/n;
                moveit::core::RobotState sample(seed); sample.setJointGroupPositions(dual,a1v::interpolateJoints(a,b,alpha)); sample.update();
                if (!valid(sample)) return false;
                const Transform actual=sample.getGlobalLinkTransform(eefs[0])*object_to_hand[0].inverse();
                Transform expected=first_object; expected.translation().z()+=delta*(k-1+alpha)/static_cast<double>(kPathSteps);
                if ((actual.translation()-expected.translation()).norm()>pos_tol || rotationError(actual,expected)>angle_tol) {
                    detail="E_CONTACT_INTERPOLATED_PATH"; return false;
                }
            }
            seed=next; trajectory.addSuffixWayPoint(seed,0.0);
        }
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        if (!iptp.computeTimeStamps(trajectory,0.05,0.03)) { detail="E_CONTACT_IPTP"; return false; }
        trajectory.getRobotTrajectoryMsg(plan.trajectory_); moveit::core::robotStateToRobotStateMsg(origin,plan.start_state_);
        auto& joint=plan.trajectory_.joint_trajectory;
        std::vector<s1::DirectionalJointLimits> limits;
        for (const auto& name:joint.joint_names) { const auto& b=live_model->getVariableBounds(name);
            limits.push_back({b.min_velocity_,b.max_velocity_,b.min_acceleration_,b.max_acceleration_,b.velocity_bounded_,b.acceleration_bounded_}); }
        std::vector<s1::TimedJointPoint> points;
        for (const auto& p:joint.points) points.push_back({rclcpp::Duration(p.time_from_start).seconds(),p.positions,p.velocities,p.accelerations});
        s1::TimeScalingReport report;
        if (!s1::scaleTimedTrajectory(points,limits,0.05,0.03,0.01,report,detail)) return false;
        // At least 5 s for 5 cm, and at least 1 s per fine contact-lift
        // segment. The latter bounds the vertical step to 2.5 mm/s and reduces
        // solver-impact-driven contact loss; it is not a success criterion.
        double stretch=std::max(1.0,5.0/points.back().time);
        for (std::size_t j=1;j<points.size();++j) stretch=std::max(stretch,1.0/(points[j].time-points[j-1].time));
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
                        v<0.05*bound.min_velocity-1e-9 || v>0.05*bound.max_velocity+1e-9 ||
                        acc<0.03*bound.min_acceleration-1e-9 || acc>0.03*bound.max_acceleration+1e-9) {
                        detail="E_CONTACT_QUINTIC_LIMIT"; return false;
                    }
                }
                moveit::core::RobotState timed(origin); timed.setVariablePositions(joint.joint_names,q); timed.update();
                if (!valid(timed)) return false;
            }
        }
        RCLCPP_INFO(ctx_->get_logger(),"[CONTACT_PROBE_PLAN] delta=%.3fm points=%zu IPTP duration=%.3fs contact_step=2.5mm min_segment=1.0s collision_and_interpolation=passed finger_only_contacts=true",
            delta,joint.points.size(),rclcpp::Duration(joint.points.back().time_from_start).seconds());
        return true;
    };
    moveit::planning_interface::MoveGroupInterface::Plan lift;
    if (!makePlan(start,height,lift)) return fail(detail);
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
        csv<<phase<<','<<fingers[0].simulation_time<<','<<latest.position.x<<','<<latest.position.y<<','<<latest.position.z<<','
           <<slips[0]<<','<<angles[0]<<','<<slips[1]<<','<<angles[1];
        for (const auto& f:fingers) csv<<','<<f.q1<<','<<f.q2;
        for (const auto& f:fingers) csv<<','<<f.command_per_finger_m;
        for (const auto& f:fingers) csv<<','<<f.closing_force_limit_per_finger_n;
        for (const auto& f:fingers) csv<<','<<f.hold_force_per_finger_n;
        for (const auto& f:fingers) csv<<','<<f.rod_pad_contact_count;
        for (const auto& f:fingers) csv<<','<<f.left_contact<<','<<f.right_contact;
        for (const auto& f:fingers) csv<<','<<f.normal_left_n<<','<<f.normal_right_n;
        for (const auto& f:fingers) csv<<','<<f.tangent_left_n<<','<<f.tangent_right_n;
        csv<<','<<fingers[0].rod_force_z_n<<','<<fingers[1].rod_force_z_n<<','<<fingers[0].seq<<','<<fingers[1].seq<<','
           <<fingers[0].weld_left_active<<','<<fingers[0].weld_right_active<<','<<fingers[0].rod_constraint_active<<'\n'; csv.flush();
        if (max_slip>pos_tol || max_angle>angle_tol) { detail="E_CONTACT_SLIP_OR_DROP"; return false; }
        if (require_height && latest.position.z<initial_truth.position.z+height-pos_tol) { detail="E_CONTACT_ROD_NOT_LIFTED"; return false; }
        return true;
    };
    auto execute=[&](const moveit::planning_interface::MoveGroupInterface::Plan& plan,const char* phase) {
        moveit::core::RobotState dispatch_start(live_model);
        if (!measured(dispatch_start)) return false;
        const auto& joint=plan.trajectory_.joint_trajectory;
        for (std::size_t j=0;j<joint.joint_names.size();++j)
            if (std::abs(dispatch_start.getVariablePosition(joint.joint_names[j])-joint.points.front().positions[j])>0.002) {
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
    if (!execute(lift,"LIFT")) return fail(detail);
    if (!observe("HOLD",true)) return fail(detail);
    // Hold is measured in physics time, with wall timeout and advancement checks.
    const double hold_start=std::min(fingers[0].simulation_time,fingers[1].simulation_time);
    const double hold_wall_deadline=steadySeconds()+6.0;
    std::size_t samples=0; std::array<int,4> contact_counts{{0,0,0,0}};
    std::array<unsigned long long,2> counted_seq{{0,0}};
    while (std::min(fingers[0].simulation_time,fingers[1].simulation_time)-hold_start<hold_seconds) {
        if (!rclcpp::ok() || steadySeconds()>hold_wall_deadline || !observe("HOLD",true)) return fail("E_CONTACT_HOLD: "+detail);
        if (fingers[0].seq>counted_seq[0] && fingers[1].seq>counted_seq[1]) {
          ++samples;
          counted_seq={{fingers[0].seq,fingers[1].seq}};
          for (int i=0;i<2;++i) { contact_counts[2*i]+=fingers[i].normal_left_n>=0.25; contact_counts[2*i+1]+=fingers[i].normal_right_n>=0.25; }
        }
        std::this_thread::sleep_for(50ms);
    }
    if (samples<20) return fail("E_CONTACT_HOLD_INSUFFICIENT_SAMPLES");
    for (const int count:contact_counts) if (double(count)/samples<0.8) return fail("E_CONTACT_HOLD_PAD_FORCE_NOT_SUSTAINED");
    RCLCPP_INFO(ctx_->get_logger(),"[CONTACT_LOAD_VALIDATED] samples=%zu sim_hold=%.3fs lift=%.3fm max_slip=%.3fmm max_angle=%.3fdeg weld_left=false weld_right=false rod_constraint=false",
        samples,std::min(fingers[0].simulation_time,fingers[1].simulation_time)-hold_start,latest.position.z-initial_truth.position.z,max_slip*1000,max_angle*180/M_PI);
    moveit::core::RobotState lifted(live_model);
    if (!measured(lifted)) return fail(detail);
    moveit::planning_interface::MoveGroupInterface::Plan lower;
    if (!makePlan(lifted,-height,lower)) return fail(detail);
    if (!execute(lower,"LOWER")) return fail(detail);
    const double extent=0.5*(std::abs(rodTransform(latest).linear()(2,0))*truth.size.x+
        std::abs(rodTransform(latest).linear()(2,1))*truth.size.y+std::abs(rodTransform(latest).linear()(2,2))*truth.size.z);
    if (std::abs(latest.position.z-extent-ctx_->p2a_support_z_)>0.003) return fail("E_CONTACT_RELEASE_NOT_ON_SUPPORT");
    if (!ctx_->grippers_->controlDualGrippers(0.04)) return fail("E_CONTACT_RELEASE_OPEN_FAILED");
    std::this_thread::sleep_for(500ms);
    const auto released=readMujocoRodTruth();
    if (!released.ok || !feedback()) return fail("E_CONTACT_RELEASE_TRUTH_OR_FEEDBACK");
    for (const auto& f:fingers) if (f.hold_force_per_finger_n!=0 || f.q1+f.q2<0.078) return fail("E_CONTACT_RELEASE_FORCE_NOT_CLEARED");
    if ((rodTransform(released).translation()-object.translation()).norm()>0.01 || rotationError(rodTransform(released),object)>5*M_PI/180)
        return fail("E_CONTACT_RELEASE_POSE_ERROR");
    // Only synchronize planning representation. Never write a MuJoCo body state.
    geometry_msgs::msg::Vector3 size; size.x=truth.size.x;size.y=truth.size.y;size.z=truth.size.z;
    if (!syncAluminumRodFromTruth(poseMsg(rodTransform(released)),size,ctx_->dual_arm_->getPlanningFrame(),detail)) return fail(detail);
    RCLCPP_INFO(ctx_->get_logger(),"[CONTACT_PROBE_RELEASED] xyz=(%.6f,%.6f,%.6f) open=true force_hold=OFF weld=OFF attach=none; retreat/carry not requested",
        released.position.x,released.position.y,released.position.z);
    detail="5cm contact-only lift, 2s hold, lower and release complete; full transport unverified";
    return true;
}
