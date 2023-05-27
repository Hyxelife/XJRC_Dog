#pragma once
#include "eigen-3.4.0/Eigen/Eigen"
#include "PacePlanner.h"
#include "Sensor.h"
#include "LegController.h"

class Controller
{
public:
    Controller(std::vector<std::string> serialName,std::vector<AxisMovement> motorAngle, std::vector<AxisMovement> realAngle,std::vector<std::vector<float>> motorSign,LegController::VMCParam param);
    ~Controller();

    bool Update(float velX,float velY,float velYaw);
    void EnableVMC(bool enable);
    void Start(float startUpTime);
    void Exit();
    void RawControl(std::vector<FeetMovement> moves);
    void ClearControlHistory();
    void ClearTime();
    PacePlanner& GetPacePlanner();

protected:
    bool m_bVMCCtrl;
    LegController* m_pControllers[4];
    PacePlanner m_planner;
    clock_t m_time;
    std::vector<FeetMovement> m_pos;
    std::vector<bool> m_touchStatus;
};
