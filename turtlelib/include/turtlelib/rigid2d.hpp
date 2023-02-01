#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include <iosfwd> // contains forward definitions for iostream objects
#include <cstdlib> // standard library, use for absolute value
#include <math.h> // standard math library, used for sin and cos in transformations
#include <iostream> // standard input/output stream library

namespace turtlelib
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI=3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        if (abs(d1-d2) < epsilon) {
            return true;
        } else {
            return false;
        };
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    constexpr double deg2rad(double deg)
    {
        return deg/180.0 * PI;
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return rad/PI * 180.0;
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");

    static_assert(almost_equal(deg2rad(180.0), PI), "deg2rad failed");

    static_assert(almost_equal(rad2deg(PI), 180.0), "rad2deg failed");

    /* purposefully failing tests
    static_assert(almost_equal(0.0, 2.0), "almost_equal *purposely* failed");

    static_assert(almost_equal(deg2rad(180.0), 2*PI), "deg2rad *purposely* failed");

    static_assert(almost_equal(rad2deg(2*PI), 180.0), "deg2rad *purposely* failed");
    */

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// \brief normalize vector
        /// \return this vector
        Vector2D normalize();

        /// \brief addition operator between vectors (assigns to lhs vector)
        /// \param rhs - the vector to add
        /// \return a *reference* to the newly transformed operator
        Vector2D & operator+=(Vector2D rhs);

        /// \brief addition operator between vectors (returns new vector)
        /// \param rhs - the vector to add
        /// \return a new vector
        Vector2D & operator+(Vector2D rhs);

        /// \brief subtraction operator between vectors (assigns to lhs vector)
        /// \param rhs - the vector to subtract
        /// \return a *reference* to the newly transformed operator
        Vector2D & operator-=(Vector2D rhs);

        /// \brief subtraction operator between vectors (returns new vector)
        /// \param rhs - the vector to subtract
        /// \return a new vector
        Vector2D & operator-(Vector2D rhs);

        /// \brief multiplication operator between vector and scalar (assigns to lhs vector)
        /// \param rhs - the scalar to scale by
        /// \return a *reference* to the newly transformed operator
        Vector2D & operator*=(double rhs);

        /// \brief multiplication operator between vector and scalar (returns new vector)
        /// \param rhs - the scalar to scale by
        /// \return a new vector
        Vector2D & operator*(double rhs);
    };

    /// \brief compute dot product of two vectors
    /// \param vec1 - first vector
    /// \param vec2 - second vector
    /// \return scalar result of dot product
    double dot(Vector2D vec1, Vector2D vec2);

    /// \brief compute magnitude of vector
    /// \param vec - vector who's magnitude is being computed
    /// \return double with magnitude
    double magnitude(Vector2D vec);

    /// \brief compute angle btwn two vectors
    /// \param vec1 - first vector
    /// \param vec2 - second vector
    /// \return double with magnitude of angle in radians
    double angle(Vector2D vec1, Vector2D vec2);

    /// \brief multiplication operator between vector and scalar (assigns to rhs vector)
    /// \param lhs - the scalar value to scale by
    /// \param rhs - the vector to be scaled
    /// \return a *reference* to the newly transformed operator
    Vector2D operator*=(double lhs, Vector2D rhs);

    /// \brief multiplication operator between vector and scalar
    /// \param lhs - the scalar value to scale by
    /// \param rhs - the vector to be scaled
    /// \return a new vector to the newly transformed operator
    Vector2D operator*(double lhs, Vector2D rhs);

    /// \brief A 2-D Twist
    struct Twist2D
    {
        /// \brief the angular component of the twist
        double angular = 0.0;

        /// \brief the x component of the linear component
        double linearx = 0.0;

        /// \brief the y component of the linear component 
        double lineary = 0.0; // y component
    };

    /// \brief normalize an angle to within a certain range
    /// \param rad - angle to normalize in radians
    /// \returns normalized angle in radians
    double normalize_angle(double rad);

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// \param os - stream to output to
    /// \param t - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & t);

    /// \brief input a 2 dimensional twist
    ///   entered as follows:
    ///   [w x y] or w x y
    /// \param is - stream from which to read
    /// \param t [out] - output twist
    ///
    std::istream & operator>>(std::istream & is, Twist2D & t);


    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// \param os - stream to output to
    /// \param v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param v [out] - output vector
    ///
    /// The way input works is (more or less): what the user types is stored in a buffer until the user types
    /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin) and processing stops.
    ///
    /// We have lower level control however.
    /// std::peek() looks at the next unprocessed character in the buffer without removing it
    ///     https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// std::get() removes the next unprocessed character from the buffer.
    ///     https://en.cppreference.com/w/cpp/io/basic_istream/get
    /// When you call std::peek() it will wait for there to be at least one character in the buffer (e.g., the user types a character)
    /// HINT: this function can be written in under 20 lines and uses only std::peek(), std::get(), istream::operator>>() and a little logic
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
        private:    
            double r11 = 1.0;
            double r12 = 0.0;
            double r13 = 0.0;
            double r21 = 0.0;
            double r22 = 1.0;
            double r23 = 0.0;
            double r31 = 0.0;
            double r32 = 0.0;
            double r33 = 1.0;
            double rot = 0.0;

    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(const Vector2D& trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param radians - the rotation, in radians
        Transform2D(const Vector2D& trans, double radians);

        /// \brief Create a transformation with a Twist2D
        /// \param twist - twist
        Transform2D(const Twist2D& twist);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(const Vector2D& v) const;

        /// \brief invert the transformation
        /// \return the inverse transformation. 
        Transform2D inv() const;

        /// \brief compose this transform with another and store the result 
        /// in this object
        /// \param rhs - the first transform to apply
        /// \return a *reference* to the newly transformed operator
        ///     this is because the function is "Transform2D &" and not just "Transform2D"
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief the translational component of the transform
        /// \return the x,y translation
        Vector2D translation() const;

        /// \brief get the angular displacement of the transform
        /// \return the angular displacement, in radians
        double rotation() const;

        /// \brief get the adjoint for this transformation
        /// \return 2D adjoint matrix
        Transform2D adj() const;

        /// \brief convert twist to a different reference frame
        /// \param new_frame - twist to convert
        /// \return the same twist, represented in the new frame
        Twist2D conv_diff_frame(const Twist2D& new_frame) const;

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

        /// \brief integrate a twist for one unit time 
        /// \param twist0 - twist to integrate 
        /// \return the transformation result from integrating by one time-unit 
        Transform2D integrate_twist(Twist2D twist0);

    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// deg: 90 x: 3 y: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    /// For example:
    /// 90 2 3
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

}

#endif
