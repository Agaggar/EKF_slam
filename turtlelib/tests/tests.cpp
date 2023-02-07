#include <catch2/catch_test_macros.hpp>
#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <sstream>

// two tests written by Ava Zahedi (at the very end)

// test overloaded constructor of transform2D
TEST_CASE("Rotate and Translate", "[transform2d]") {
    float transx = 1.0;
    float transy = 1.0;
    float rad = turtlelib::PI/4;
    turtlelib::Transform2D transform_test = turtlelib::Transform2D(turtlelib::Vector2D{transx, transy}, rad);
    REQUIRE( transform_test.translation().x == transx);
    REQUIRE( transform_test.translation().y == transy);
    REQUIRE( transform_test.rotation() == rad);
}

TEST_CASE("Normalization test #1", "[vector2d]") {
    turtlelib::Vector2D vec_test = turtlelib::Vector2D{3.0, 4.0};
    REQUIRE( turtlelib::almost_equal(vec_test.normalize().x, 0.6));
    REQUIRE( turtlelib::almost_equal(vec_test.normalize().y, 0.8));
}

TEST_CASE("Normalization test #2", "[vector2d]") {
    turtlelib::Vector2D vec_test = turtlelib::Vector2D{-6.0, 8.0};
    REQUIRE( turtlelib::almost_equal(vec_test.normalize().x, -0.6));
    REQUIRE( turtlelib::almost_equal(vec_test.normalize().y, 0.8));
}

TEST_CASE("transform () vector operator test #1", "[transform2d]") {
    turtlelib::Vector2D vec = turtlelib::Vector2D{2.0, -2.0};
    turtlelib::Transform2D transform_test = turtlelib::Transform2D(turtlelib::PI/2);
    turtlelib::Vector2D vec_to_test = transform_test(vec);
    REQUIRE( turtlelib::almost_equal(vec_to_test.x, 2.0));
    REQUIRE( turtlelib::almost_equal(vec_to_test.y, 2.0));
}

TEST_CASE("transform () vector operator test #2", "[transform2d]") {
    turtlelib::Vector2D vec = turtlelib::Vector2D{1.5, -1.5};
    turtlelib::Transform2D transform_test = turtlelib::Transform2D(turtlelib::Vector2D{0.5, 0.75}, 0.0);
    turtlelib::Vector2D vec_to_test = transform_test(vec);
    double x_desired = 1.5 + 0.5;
    double y_desired = -1.5 + 0.75;
    REQUIRE( turtlelib::almost_equal(vec_to_test.x, x_desired));
    REQUIRE( turtlelib::almost_equal(vec_to_test.y, y_desired));
}

TEST_CASE("inverse of a transform", "[transform2d]") {
    turtlelib::Transform2D transform_test = turtlelib::Transform2D(turtlelib::PI/2);
    transform_test = transform_test.inv();
    REQUIRE( turtlelib::almost_equal(transform_test.rotation(), -1*turtlelib::PI/2));
    REQUIRE( turtlelib::almost_equal(transform_test.translation().x, 0.0));
    REQUIRE( turtlelib::almost_equal(transform_test.translation().y, 0.0));
}

TEST_CASE("*= operator", "[transform2d]") {
    turtlelib::Transform2D t_12 = turtlelib::Transform2D(turtlelib::Vector2D{1.0, 1.0}, 0.0);
    turtlelib::Transform2D t_23 = turtlelib::Transform2D(turtlelib::Vector2D{-1.0, -1.0}, 0.0);
    t_12*=t_23;
    REQUIRE( turtlelib::almost_equal(t_12.rotation(), 0.0));
    REQUIRE( turtlelib::almost_equal(t_12.translation().x, 0.0));
    REQUIRE( turtlelib::almost_equal(t_12.translation().y, 0.0));
}

TEST_CASE("Translation", "[transform2d]" ) {
   turtlelib::Transform2D transform_test = turtlelib::Transform2D(turtlelib::Vector2D{1.1, -1.1}, 0.0);
   REQUIRE( turtlelib::almost_equal(transform_test.translation().x, 1.1));
   REQUIRE( turtlelib::almost_equal(transform_test.translation().y, -1.1));
}

TEST_CASE("Rotation", "[transform2d]" ) {
   turtlelib::Transform2D transform_test = turtlelib::Transform2D(turtlelib::Vector2D{1.1, -1.1}, turtlelib::PI/4);
   REQUIRE( turtlelib::almost_equal(transform_test.rotation(), turtlelib::PI/4));
}

TEST_CASE("Adjugate function", "[transform2d]" ) {
   turtlelib::Transform2D transform_test = turtlelib::Transform2D(turtlelib::Vector2D{1.1, -1.1}, turtlelib::PI/4);
//    REQUIRE( turtlelib::almost_equal(transform_test.adj().r11, 1.0));
//    REQUIRE( turtlelib::almost_equal(transform_test.adj().r21, -1.1));
//    REQUIRE( turtlelib::almost_equal(transform_test.adj().r31, -1.1));
//    REQUIRE( turtlelib::almost_equal(transform_test.adj().r12, 0.0));
//    REQUIRE( turtlelib::almost_equal(transform_test.adj().r22, 0.707106781));
//    REQUIRE( turtlelib::almost_equal(transform_test.adj().r32, 0.707106781));
   REQUIRE( turtlelib::almost_equal(transform_test.adj().translation().x, 0.0));
   REQUIRE( turtlelib::almost_equal(transform_test.adj().translation().y, -0.707106781, 1.0e-8));
//    REQUIRE( turtlelib::almost_equal(transform_test.adj().r33, 0.707106781));
   REQUIRE( turtlelib::almost_equal(transform_test.adj().rotation(), turtlelib::PI/4));
}

TEST_CASE("Convert Twist to different frame", "[transform2d]") {
    turtlelib::Transform2D transform_test = turtlelib::Transform2D(turtlelib::Vector2D{1.0, -1.0}, turtlelib::PI/2);
    turtlelib::Twist2D twist_test = turtlelib::Twist2D{0.0, 1.5, -1.5};
    turtlelib::Twist2D answer = turtlelib::Twist2D{0, 1.5, 1.5};
    REQUIRE( turtlelib::almost_equal(transform_test.conv_diff_frame(twist_test).angular, answer.angular));
    REQUIRE( turtlelib::almost_equal(transform_test.conv_diff_frame(twist_test).linearx, answer.linearx));
    REQUIRE( turtlelib::almost_equal(transform_test.conv_diff_frame(twist_test).lineary, answer.lineary));
}

TEST_CASE( "Stream insertion operator <<", "[transform]" ) // Ava, Zahedi
{
   turtlelib::Vector2D vec;
   vec.x = 1.0;
   vec.y = 3.4;
   double phi = 0.0;
   turtlelib::Transform2D tf = turtlelib::Transform2D(vec, phi);
   std::string str = "deg: 0 x: 1 y: 3.4\n";
   std::stringstream sstr;
   sstr << tf;
   REQUIRE( (sstr.str() + "\n") == str );
}

TEST_CASE( "Stream extraction operator >>", "[transform]" ) // Ava, Zahedi
{
   turtlelib::Transform2D tf = turtlelib::Transform2D();
   std::stringstream sstr;
   sstr << "deg: 90 x: 1 y: 3.4";
//    sstr << tf;
//    REQUIRE(std::to_string(tf.rotation()) == ""); 
//    REQUIRE( turtlelib::almost_equal(tf.rotation(), turtlelib::deg2rad(90), 0.00001) );
//    REQUIRE( turtlelib::almost_equal(tf.translation().x, 1.0, 0.00001) );
//    REQUIRE( turtlelib::almost_equal(tf.translation().y, 3.4, 0.00001) );
}

TEST_CASE("normalizing angle pi", "[double]") {
    double angle = turtlelib::PI;
    REQUIRE( turtlelib::almost_equal(turtlelib::normalize_angle(angle), turtlelib::PI));
}

TEST_CASE("normalizing angle -pi", "[double]") {
    double angle = -1*turtlelib::PI;
    REQUIRE( turtlelib::almost_equal(turtlelib::normalize_angle(angle), turtlelib::PI));
}

TEST_CASE("normalizing angle 0", "[double]") {
    double angle = 0*turtlelib::PI;
    REQUIRE( turtlelib::almost_equal(turtlelib::normalize_angle(angle), 0.0));
}

TEST_CASE("normalizing angle -pi/4", "[double]") {
    double angle = turtlelib::PI/4.0;
    REQUIRE( turtlelib::almost_equal(turtlelib::normalize_angle(angle), turtlelib::PI/4.0));
}

TEST_CASE("normalizing angle 3*pi/2", "[double]") {
    double angle = turtlelib::PI*1.5;
    REQUIRE( turtlelib::almost_equal(turtlelib::normalize_angle(angle), -1/2.0*turtlelib::PI));
}

TEST_CASE("normalizing angle -5pi/2", "[double]") {
    double angle = turtlelib::PI*-2.5;
    REQUIRE( turtlelib::almost_equal(turtlelib::normalize_angle(angle), -0.5*turtlelib::PI));
}

TEST_CASE("vector operators", "[vector2d]") {
    turtlelib::Vector2D myvec = turtlelib::Vector2D{1, -1};
    myvec += turtlelib::Vector2D{1, -1};
    REQUIRE( turtlelib::almost_equal(myvec.x, 2));
    REQUIRE( turtlelib::almost_equal(myvec.y, -2));
    myvec -= turtlelib::Vector2D{1, -1};
    REQUIRE( turtlelib::almost_equal(myvec.x, 1));
    REQUIRE( turtlelib::almost_equal(myvec.y, -1));
    turtlelib::Vector2D vec_test = myvec + turtlelib::Vector2D{1, -1};
    REQUIRE( turtlelib::almost_equal(vec_test.x, 2));
    REQUIRE( turtlelib::almost_equal(vec_test.y, -2));
    vec_test = myvec - turtlelib::Vector2D{1, -1};
    REQUIRE( turtlelib::almost_equal(vec_test.x, 0));
    REQUIRE( turtlelib::almost_equal(vec_test.y, 0));
    myvec*=2.5;
    REQUIRE( turtlelib::almost_equal(myvec.x, 2.5));
    REQUIRE( turtlelib::almost_equal(myvec.y, -2.5));
    vec_test = myvec*0.4;
    REQUIRE( turtlelib::almost_equal(vec_test.x,1));
    REQUIRE( turtlelib::almost_equal(vec_test.y, -1));
    REQUIRE( turtlelib::almost_equal(turtlelib::magnitude(vec_test), sqrt(2)));
    REQUIRE( turtlelib::almost_equal(turtlelib::dot(vec_test, myvec), 5));
    REQUIRE( turtlelib::almost_equal(turtlelib::angle(vec_test, 3*turtlelib::Vector2D{1, 1}), turtlelib::PI/2.0));
}

TEST_CASE("inverse kinematics pure translation", "[twist2d]") {
    turtlelib::Transform2D base_frame = turtlelib::Transform2D(turtlelib::Vector2D{1.0, -1.0}, turtlelib::PI/2);
    turtlelib::Twist2D twist0 = turtlelib::Twist2D{0.0, 1.0, -1.0};
    turtlelib::Transform2D transform_test = base_frame.integrate_twist(twist0);
    REQUIRE( turtlelib::almost_equal(transform_test.translation().x, 1.0));
    REQUIRE( turtlelib::almost_equal(transform_test.translation().y, -1.0));
    REQUIRE( turtlelib::almost_equal(transform_test.rotation(), 0.0));

    twist0 = turtlelib::Twist2D{turtlelib::PI/4.0, 0.0, 0.0};
    transform_test = base_frame.integrate_twist(twist0);
    REQUIRE( turtlelib::almost_equal(transform_test.translation().x, 0.0));
    REQUIRE( turtlelib::almost_equal(transform_test.translation().y, 0.0));
    REQUIRE( turtlelib::almost_equal(transform_test.rotation(), turtlelib::PI/4.0));

    twist0 = turtlelib::Twist2D{turtlelib::PI/4.0, 1.0, -1.0};
    transform_test = base_frame.integrate_twist(twist0);
    // REQUIRE(std::to_string(transform_test.translation().y) == std::to_string(-4/turtlelib::PI));
    // REQUIRE( std::to_string(transform_test.translation().x) == std::to_string((4*sqrt(2.0)-4)/turtlelib::PI));
    REQUIRE( turtlelib::almost_equal(transform_test.translation().x, (4*sqrt(2.0)-4)/turtlelib::PI));
    REQUIRE( turtlelib::almost_equal(transform_test.translation().y, -4/turtlelib::PI));
    REQUIRE( turtlelib::almost_equal(transform_test.rotation(), turtlelib::PI/4.0));

    twist0 = turtlelib::Twist2D{-1.24, -2.15, -2.92}; // Morales, Nick
    transform_test = base_frame.integrate_twist(twist0);
    // REQUIRE(std::to_string(transform_test.translation().y) == std::to_string(-1.05645265));
    // REQUIRE( std::to_string(transform_test.translation().x) == std::to_string(-3.2298632647));
    REQUIRE( std::to_string(transform_test.rotation()) == std::to_string(-1.24));
    REQUIRE( turtlelib::almost_equal(transform_test.translation().x, -3.2298632647, 1e-5));
    REQUIRE( turtlelib::almost_equal(transform_test.translation().y, -1.05645265, 1e-5));
    REQUIRE( turtlelib::almost_equal(transform_test.rotation(), -1.24));

}