#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <iostream>

int main(void) {

    turtlelib::Transform2D T_ab, T_bc;
    std::cout << "Enter transform T_{a,b}): ";
    std::cin >> T_ab;
    std::cout << T_ab << std::endl;

    std::cout << "Enter transform T_{b,c}): ";
    std::cin >> T_bc;
    std::cout << T_bc << std::endl;

    std::cout << "T_{a,b}): " << T_ab << std::endl;
    std::cout << "T_{b,a}): " << T_ab.inv() << std::endl;
    std::cout << "T_{b,c}): " << T_bc << std::endl;
    std::cout << "T_{c,b}): " << T_bc.inv() << std::endl;
    turtlelib::Transform2D T_ac = T_ab;
    T_ac*=T_bc;
    std::cout << "T_{a,c}): " << T_ac << std::endl;
    std::cout << "T_{c,a}): " << T_ac.inv() << std::endl;

    turtlelib::Vector2D v_b;
    std::cout << "Enter vector v_b: ";
    std::cin >> v_b;
    std::cout << "v_bhat: " << v_b.normalize() << std::endl;
    std::cout << "v_a: " << T_ab(v_b) << std::endl;
    std::cout << "v_b: " << v_b << std::endl;
    std::cout << "v_c: " << (T_bc.inv())(v_b) << std::endl;

    turtlelib::Twist2D V_b;
    std::cout << "Enter twist V_b: ";
    std::cin >> V_b;
    std::cout << "V_a: " << T_ab.conv_diff_frame(V_b) << std::endl;
    std::cout << "V_b: " << V_b << std::endl;
    std::cout << "V_c: " << (T_bc.inv()).conv_diff_frame(V_b) << std::endl;
    
    // turtlelib::Transform2D::Transform2D();
    return 0;
}
