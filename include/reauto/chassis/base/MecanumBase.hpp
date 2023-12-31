#pragma once

#include "pros/abstract_motor.hpp"
#include "reauto/chassis/template/HolonomicRobotTemplate.hpp"
#include <initializer_list>
#include <memory>
#include <vector>

namespace reauto
{
class MecanumBase: public HolonomicRobotTemplate
{
public:
    explicit MecanumBase(std::initializer_list<int8_t> left, std::initializer_list<int8_t> right, pros::MotorGears gearset);

    void setLeftFwdVoltage(double voltage) override;
    void setRightFwdVoltage(double voltage) override;

    void setLeftStrafeVoltage(double voltage) override;
    void setRightStrafeVoltage(double voltage) override;

    void setLeftFwdVelocity(double velocity) override;
    void setRightFwdVelocity(double velocity) override;

    void setLeftStrafeVelocity(double velocity) override;
    void setRightStrafeVelocity(double velocity) override;

    void setFwdVoltage(double voltage) override;
    void setFwdVelocity(double velocity) override;

    void setTurnVoltage(double voltage) override;
    void setTurnVelocity(double velocity) override;

    void setStrafeVoltage(double fwdPower, double sidePower) override;
    void setStrafeVelocity(double fwdVel, double sideVel) override;

    void setBrakeMode(pros::MotorBrake mode) override;
    void brake() override;
    
    // get motors
    MotorSet* getLeftMotors() const override;
    MotorSet* getRightMotors() const override;

    // get the left velocity
    double getLeftVelocity() const override;

private:
    // we need individual motors, PROS 4 removes the ability to
    // index for setters.

    std::shared_ptr<MotorSet> m_left;
    std::shared_ptr<MotorSet> m_right;
};
}
