#pragma once
#include "carry_task_context.hpp"

class DualArmController {
    DualArmCarryTask* ctx_;
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
