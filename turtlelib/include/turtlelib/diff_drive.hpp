#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Model kinematics of a differential drive robot given wheel track and radius

#include "rigid2d.hpp"
#include <iosfwd> // contains forward definitions for iostream objects
#include <cstdlib> // standard library, use for absolute value
#include <math.h> // standard math library, used for sin and cos in transformations
#include <iostream> // standard input/output stream library
#include <vector> // standard vector library

namespace turtlelib
{
    /// \brief Model kinematics of a diff drive robot
    class DiffDrive
    {
        private:
            std::vector<double> phi = {0.0, 0.0}; // right, left wheel position
            std::vector<double> q {0.0, 0.0, 0.0}; // robot configuration, x, y, theta respectively
            double wheel_radius = 0.033; // wheel radius, m
            double wheel_track = 0.16; // wheel track, m
        
    public:
        /// \brief Create a diff drive robot with default values
        DiffDrive();

        /// \brief Create a diff drive robot given a configuration. Default wheel phi values.
        /// \param qx - robot config, x
        /// \param qy - robot config, y
        /// \param qtheta - robot config, theta
        DiffDrive(const double qx, const double qy, const double qtheta);

        /// \brief Create a diff drive robot given a configuration. Default wheel phi values.
        /// \param qnew - robot config, x, y, theta
        explicit DiffDrive(const std::vector<double> qnew);

        /// \brief Create a diff drive robot given a configuration and wheel positions
        /// \param phi_r - right wheel position
        /// \param phi_l - left wheel position
        /// \param qx - robot config, x
        /// \param qy - robot config, y
        /// \param qtheta - robot config, theta
        DiffDrive(const double phi_r, const double phi_l, const double qx, const double qy, const double qtheta);

        /// \brief Create a diff drive robot given a configuration and wheel positions
        /// \param phinew - right, left wheel position
        /// \param qnew - robot config, x, y, theta
        DiffDrive(const std::vector<double> phinew, const std::vector<double> qnew);

        /// \brief set wheel radius of robot 
        /// \param wheel_radius - wheel_radius to set 
        void setWheelRadius(double wheel_radius);

        /// \brief set wheel track of robot 
        /// \param wheel_track - wheel_track to set 
        void setWheelTrack(double wheel_track);

        /// \brief Compute forward kinematics
        /// \param phi_new - new wheel positions
        void fkinematics(const std::vector<double> phi_new);

        /// \brief Compute inverse kinematics
        /// \param twist0 - desired body twist
        std::vector<double> ikinematics(Twist2D twist0);

    };
}

#endif