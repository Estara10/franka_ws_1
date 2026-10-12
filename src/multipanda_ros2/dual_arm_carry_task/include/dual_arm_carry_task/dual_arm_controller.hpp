#pragma once
#include <moveit/collision_detection/collision_matrix.h>
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
};

class DualArmController {
    DualArmCarryTask* ctx_;
    A1VerifyContext a1_verify_;
public:
    DualArmController(DualArmCarryTask* ctx) : ctx_(ctx) {}
    void initialize();
    void diagnoseCollision();
    void initializeCollisionMatrix();
    void allow_gripper_collision(bool allow);
    void publishAluminumRod();
    void executeInit();
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
