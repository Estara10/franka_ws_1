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
    void executeApproach();
    void executeGrasp();
    void attachObject();
    void executeLift();
    void executeTransport();
    void executeRotate();
    void executeDescend();
    void executePlace();
    void detachAndReplaceObject();
    void executeRetreat();
};
