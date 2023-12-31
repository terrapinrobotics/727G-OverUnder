#pragma once

#include <initializer_list>
#include <iostream>
#include <variant>
#include "pros/abstract_motor.hpp"
#include "pros/misc.hpp"
#include "reauto/chassis/impl/MotionChassis.hpp"
#include "reauto/chassis/impl/HolonomicMode.hpp"
#include "reauto/odom/Odometry.hpp"

namespace reauto {

template <HolonomicMode HoloMode = HolonomicMode::NONE>
class ChassisBuilder {
public:
    // pass motor ports
    ChassisBuilder& motors(std::initializer_list<int8_t> left, std::initializer_list<int8_t> right, pros::MotorGears gears) {
        m_left = left;
        m_right = right;
        m_gearset = gears;
        return *this;
    }

    // pass reference to controller
    ChassisBuilder& controller(pros::Controller& c) {
        m_controller = &c;
        return *this;
    }

    // pass IMU port(s)
    ChassisBuilder& imu(uint8_t port) {
        m_imuA = port;
        return *this;
    }

    ChassisBuilder& imu(uint8_t portA, uint8_t portB) {
        m_imuA = portA;
        m_imuB = portB;
        return *this;
    }

    // pass tracking wheel config (port, center dist)
    ChassisBuilder& trackingWheels(std::pair<int8_t, double> left, std::pair<int8_t, double> right, std::pair<int8_t, double> back, double diam) {
        m_firstTWheelPort = left.first;
        m_firstTWheelDist = left.second;

        m_secondTWheelPort = right.first;
        m_secondTWheelDist = right.second;

        m_thirdTWheelPort = back.first;
        m_thirdTWheelDist = back.second;

        m_tConfig = TrackingConfiguration::LRB;
        m_tWheelDiam = diam;
        return *this;
    }

    ChassisBuilder& trackingWheels(std::pair<int8_t, double> first, std::pair<int8_t, double> second, double diam, bool centerConfig = false) {
        if (centerConfig) {
            // center + back wheels
            m_firstTWheelPort = first.first;
            m_firstTWheelDist = first.second;
            m_secondTWheelPort = second.first;
            m_secondTWheelDist = second.second;

            m_tConfig = TrackingConfiguration::CB;
        }

        else {
            // left + right wheels
            m_firstTWheelPort = first.first;
            m_firstTWheelDist = first.second;
            m_secondTWheelPort = second.first;
            m_secondTWheelDist = second.second;

            m_tConfig = TrackingConfiguration::LR;
        }

        m_tWheelDiam = diam;
        return *this;
    }

    // set the chassis track width
    ChassisBuilder& setChassisConstants(double track_width, double wheel_diam, double rpm) {
        m_trackWidth = track_width;
        m_wheelDiam = wheel_diam;
        m_chassisRPM = rpm;
        return *this;
    }

    // set odom preferences (advanced)
    ChassisBuilder& odomPrefs(OdomPrefs prefs) {
        m_odomPrefs = prefs;
        return *this;
    }

    // build the chassis with feedback
    std::shared_ptr<MotionChassis> build() {
        std::cout << "Build called..." << std::endl;

        // cout tconfig
        std::cout << "TConfig: " << static_cast<int>(m_tConfig) << std::endl;

        return std::make_shared<MotionChassis>(m_left, m_right, m_gearset, *m_controller, HoloMode, m_imuA, m_imuB, m_firstTWheelPort, m_firstTWheelDist, m_secondTWheelPort, m_secondTWheelDist, m_thirdTWheelPort, m_thirdTWheelDist, m_tWheelDiam, m_tConfig, m_trackWidth, m_wheelDiam, m_chassisRPM, m_odomPrefs);
    }

private:
    // wheel ports
    std::initializer_list<int8_t> m_left;
    std::initializer_list<int8_t> m_right;

    // gearset
    pros::MotorGears m_gearset;

    // ptr to controller
    pros::Controller* m_controller;

    // IMU ports
    uint8_t m_imuA = 0;
    uint8_t m_imuB = 0;

    // tracking wheel ports
    int8_t m_firstTWheelPort = 0;
    int8_t m_secondTWheelPort = 0;
    int8_t m_thirdTWheelPort = 0;

    // tracking wheel center dist
    double m_firstTWheelDist = 0;
    double m_secondTWheelDist = 0;
    double m_thirdTWheelDist = 0;

    // tracking wheel config
    TrackingConfiguration m_tConfig = TrackingConfiguration::NA;

    // tracking wheel diameter
    double m_tWheelDiam = 0;

    // robot properties
    double m_trackWidth = 0;
    double m_wheelDiam = 0;
    double m_chassisRPM = 0;

    // odom preferences
    OdomPrefs m_odomPrefs;
};
}