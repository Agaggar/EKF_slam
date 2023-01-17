#include <catch2/catch_test_macros.hpp>
#include "turtlelib/rigid2d.hpp"

TEST_CASE("Rotate and Translate", "[transform2d]") {
    float transx = 1.0;
    float transy = 1.0;
    float rad = turtlelib::PI/4;
    turtlelib::Transform2D transform_test = turtlelib::Transform2D(turtlelib::Vector2D{transx, transy}, rad);
    REQUIRE( transform_test.translation().x == transx);
    REQUIRE( transform_test.translation().y == transy);
    REQUIRE( transform_test.rotation() == rad);
}