#pragma once
#include <moveit/collision_detection/collision_matrix.h>
#include <multi_mode_control_msgs/srv/get_robot_states.hpp>
#include "carry_task_context.hpp"

// A1 P2a/P2b: the verification model, world and ACM that produced the static
// result, kept so the pre-grasp path is checked against EXACTLY the same
// criterion. Built independently of the shared RobotModel and the live scene.
struct A1VerifyContext {
    bool valid {false};
    std::shared_ptr<moveit::core::RobotModel> model;
    planning_scene::PlanningScenePtr scene;
    collision_detection::AllowedCollisionMatrix base_acm;   // used for the pre-grasp state
    collision_detection::AllowedCollisionMatrix grasp_acm;  // finger-rod allowed at the grasp height
    // RobotState has no default constructor, so these are held by pointer.
    std::shared_ptr<moveit::core::RobotState> start;     // the ACTUAL start, in `model`
    std::shared_ptr<moveit::core::RobotState> pregrasp;  // chosen combined pre-grasp, in `model`
    std::shared_ptr<moveit::core::RobotState> grasp;     // chosen combined grasp, in `model`
    a1v::DualTarget pregrasp_target;
    std::string rod_id;
    std::string support_id;
    std::string mesh_path;
    a1v::Aabb hand_aabb;
    std::string left_eef;
    std::string right_eef;
    // The chosen pre-grasp solution by joint name, so the planning goal can be
    // rebuilt in whichever model is being used.
    std::map<std::string, double> pregrasp_joints;
    // The exact plan P2b verified. Executed verbatim -- never re-planned.
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_valid {false};

    // A1 P3a-2b: the descent to the grasp pose, generated from the ACTUAL
    // reached pre-grasp state and verified this round. Executed verbatim.
    a1v::DualTarget grasp_target;
    std::map<std::string, double> grasp_joints;
    std::shared_ptr<moveit::core::RobotState> descent_start;
    std::vector<std::shared_ptr<moveit::core::RobotState>> descent_waypoints;
    moveit::planning_interface::MoveGroupInterface::Plan descent_plan;
    bool descent_plan_valid {false};
    // Measured departure of the verified path from a straight vertical line.
    a1v::StraightDescentDeviation descent_deviation;
    double descent_span_m {0.0};
    bool pregrasp_reached_this_round {false};
};

// A1 P3a-2b: physical rod truth read from MuJoCo, kept separate from anything
// the planning scene reports so the two are never confused.
struct MujocoRodTruth {
    bool ok {false};
    a1v::Vec3 position;
    a1v::Quat orientation;
    a1v::Vec3 linear_velocity;
    a1v::Vec3 angular_velocity;
    std::string detail;
};

class DualArmController {
    DualArmCarryTask* ctx_;
    A1VerifyContext a1_verify_;
    rclcpp::Service<multi_mode_control_msgs::srv::GetRobotStates>::SharedPtr robot_state_service_;
public:
    DualArmController(DualArmCarryTask* ctx) : ctx_(ctx) {}
    void initialize();
    bool ensureTaskRobotStatesService();
    bool runMmcStartupPreflight(std::string& reason);
    void diagnoseCollision();
    void initializeCollisionMatrix();
    void allow_gripper_collision(bool allow);
    void publishAluminumRod();
    void executeInit();
    // A1 P3a-2a: minimal explicit empty-load gripper cycle. The arms are never
    // commanded; this only exercises the gripper action interface.
    void runGripperSelftest();
    // A1 P1b (no motion): freeze the truth reference, sync the planning scene.
    bool prepareFromA1Truth();
    bool syncAluminumRodFromTruth(const geometry_msgs::msg::Pose& pose,
                                  const geometry_msgs::msg::Vector3& size,
                                  const std::string& planning_frame,
                                  std::string& reason);
    // A1 P2a (no motion): static validity of the frozen truth -- geometry,
    // combined dual-arm IK, and environment/self/dual-arm collision.
    bool runP2aChecks(const a1t::TruthPayload& truth, a1v::SampleValidation& out);
    // A1 P2a: independent clone of the robot model in which ONLY the two hand
    // links have their collision geometry replaced by the conservative AABB
    // box of the simulation's hand_c mesh. The shared RobotModel is never
    // modified in place. Returns nullptr and a reason on any load failure.
    std::shared_ptr<moveit::core::RobotModel> buildMujocoHandModel(
        const a1v::Aabb& hand_aabb,
        const std::vector<std::string>& hand_links,
        std::string& reason);
    // A1 P2b (no execution): plan the joint pre-grasp path and re-verify every
    // waypoint and interpolated state on the P2a criterion model.
    bool runP2bPathCheck(a1v::SampleValidation& out);
    // A1 P3a-1: execute the SAME plan P2b verified, in this simulation instance,
    // then stop. Never re-plans, never descends, never touches gripper/weld.
    bool executePregrasp(const a1t::TruthPayload& truth, std::string& detail);
    // A1 P3a-2b: open both grippers, then descend from the reached pre-grasp
    // pose to the grasp pose along world Z and stop. Never closes, welds,
    // attaches or lifts.
    bool runGraspDescent(const a1t::TruthPayload& truth, std::string& detail);
    bool runGraspContact(const a1t::TruthPayload& truth, std::string& detail);
    bool runContactLoadProbe(const a1t::TruthPayload& truth, std::string& detail,
                             bool hold_diagnostic_only = false,
                             bool plan_compare = false);
    bool runIdealWeldCarry(const a1t::TruthPayload& truth, std::string& detail);
    // A1 P3a-2b: build the Cartesian descent from the ACTUAL reached pre-grasp
    // state, solve it with combined-state IK on the criterion model, and verify
    // every waypoint and interpolated state. Nothing is dispatched here.
    bool buildAndVerifyGraspDescent(const a1t::TruthPayload& truth,
                                    const a1v::FingerJointSnapshot& fingers,
                                    std::string& detail);
    bool awaitFreshOpenFingerSnapshot(
        double opened_after_steady_sec,
        const std::array<std::uint64_t, 2>& baseline_seq,
        a1v::FingerJointSnapshot& out,
        std::string& detail);
    // Reads the rod's pose straight from MuJoCo. Explicitly NOT the planning
    // scene: the two are reported side by side, never interchanged.
    // `service_override` exists only so a negative example can point at a
    // name that does not exist; normal callers leave it empty.
    MujocoRodTruth readMujocoRodTruth(const std::string& service_override = std::string());

    // A1 P3a-2b: one short window of PROVABLY FRESH joint feedback.
    //
    // Each sample must carry a joint_states sequence number that has not been
    // counted yet, so re-reading the same state can never fill the window, and
    // the settle test is the WHOLE window's per-joint spread rather than the
    // difference between two adjacent samples.
    struct FreshArrival {
        bool ok {false};
        std::string reason_code;
        std::string detail;
        std::size_t samples {0};
        double window_sec {0.0};
        double worst_joint_change_rad {0.0};
        std::string worst_joint;
        unsigned long long last_seq {0};
        std::map<std::string, double> last_positions;   // the newest fresh sample
    };
    FreshArrival awaitFreshArrival(const std::vector<std::string>& joint_names,
                                   double window_sec,
                                   double settle_tol_rad);
    // Builds a RobotState from one fresh feedback sample. Fails, rather than
    // filling in defaults, if a joint in `positions` is missing.
    bool freshStateFor(const std::map<std::string, double>& positions,
                       moveit::core::RobotState& out);
    bool checkControllerStatesEnabled(std::string& detail);
    void executeApproach();
    void executeGrasp();
    bool attachObject();
    void executeLift();
    void executeTransport();
    void executeRotate();
    void executeDescend();
    void executePlace();
    bool detachAndReplaceObject();
    void executeRetreat();
};
