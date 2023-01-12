#include "rigid2d.hpp"
#include <iostream>

namespace turtlelib {

    std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
        return os << "[" << v.x << " " << v.y << "]";
    }

    std::istream & operator>>(std::istream & is, Vector2D & v) {
        char c1 = is.peek();
        if (c1 == '[') {
            is.get();
        }
        is >> v.x >> v.y;
        std::cin.ignore(50, '\n');
        return is;
    }

    std::ostream & operator<<(std::ostream & os, const Twist2D & t) {
        return os << "[" << t.angular << " " << t.linearx << " " << t.lineary << "]";
    };

    std::istream & operator>>(std::istream & is, Twist2D & t) {
        char c1 = is.peek();
        if (c1 == '[') {
            is.get();
        }
        is >> t.angular >> t.linearx >> t.lineary;
        std::cin.ignore(50, '\n');
        return is;
    };
}

int main(void) {
    turtlelib::Vector2D myvec;
    std::cout << "enter the vector you'd like to save: ";
    std::cin >> myvec;
    std::cout << myvec << std::endl;
    std::cin.clear();

    turtlelib::Twist2D mytwist;
    std::cout << "enter the twist you'd like to save: ";
    std::cin >> mytwist;
    std::cout << mytwist << std::endl;
    
    // turtlelib::Transform2D::Transform2D();
    return 0;
}