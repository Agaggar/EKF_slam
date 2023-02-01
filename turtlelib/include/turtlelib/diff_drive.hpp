#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Model kinematics of a differential drive robot given wheel track and radius

#include "turtlelib/rigid2d.hpp"
#include <iosfwd> // contains forward definitions for iostream objects
#include <cstdlib> // standard library, use for absolute value
#include <math.h> // standard math library, used for sin and cos in transformations
#include <iostream> // standard input/output stream library

namespace turtlelib
{
    /// \brief Model kinematics of a diff drive robot
    class DiffDrive
    {
        private:
            double phi_r = 0.0; // right wheel position
            double phi_l = 0.0; // left wheel position
            double qx = 0.0; // robot configuration, x
            double qy = 0.0; // robot configuration, y
            double qtheta = 0.0; // robot configuration, theta
        
        public:
            /// \brief Create a diff drive robot with default values
            DiffDrive();

            /// \brief Create a diff drive robot given a configuration
            /// \param qx - robot config, x
            /// \param qy - robot config, y
            /// \param qtheta - robot config, theta
            DiffDrive(const double qx, const double qy, const double qtheta);

            /// \brief Create a diff drive robot given a configuration and wheel positions
            /// \param phi_r - right wheel position
            /// \param phi_l - left wheel position
            /// \param qx - robot config, x
            /// \param qy - robot config, y
            /// \param qtheta - robot config, theta
            DiffDrive(const double phi_r, const double phi_l, const double qx, const double qy, const double qtheta);

            /// \brief Compute forward kinematics
            /// \param phi_rprime - new right wheel position
            /// \param phi_lprime - new left wheel position
            void fkinematics(double phi_rprime, double phi_lprime);

            /// \brief Compute inverse kinematics
            /// \param twist0 - desired body twist
            Vector2D ikinematics(turtlelib::Twist2D twist0);

    };
}

#endif