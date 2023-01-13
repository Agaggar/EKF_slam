#include "rigid2d.hpp"
#include <iostream>

namespace turtlelib {
    Vector2D Vector2D::normalize() {
            Vector2D normalize = *this;
            double mag = sqrt(pow(x,2) + pow(y, 2));
            normalize.x = normalize.x/mag;
            normalize.y = normalize.y/mag;
            return normalize;
    };

    Transform2D::Transform2D() {};

    Transform2D::Transform2D(const Vector2D& trans) {
            r31 = trans.x;
            r32 = trans.y;
    };

    Transform2D::Transform2D(double radians):
        rot(radians) 
        {
        r11 = cos(radians);
        r12 = -1*sin(radians);
        r21 = sin(radians);
        r22 = cos(radians);
    };

    Transform2D::Transform2D(const Vector2D& trans, double radians):
        // r11(cos(radians));
        rot(radians)
        {
        r11 = cos(radians);
        r12 = -1*sin(radians);
        r21 = sin(radians);
        r22 = cos(radians);
        r13 = trans.x;
        r23 = trans.y;
        // rot = radians;
    };

    Vector2D Transform2D::operator()(const Vector2D& v) const {
        Vector2D tran_vec;
        tran_vec.x = r11 * v.x + r12 * v.y + r13;
        tran_vec.y = r21 * v.x + r22 * v.y + r23;
        return tran_vec;
    };

    Transform2D Transform2D::inv() const {
        Transform2D inv_trans = *this;
        inv_trans.r12 = r21;
        inv_trans.r13 = -1*(r13 * r11 + r23 * r21);
        inv_trans.r21 = r12;
        inv_trans.r23 = -1 * r23 * r11 + r13 * r21;
        inv_trans.rot = -1*inv_trans.rotation();
        return inv_trans;
    };

    Transform2D & Transform2D::operator*=(const Transform2D & rhs) {
        Transform2D temp = *this;
        temp.r11 = r11*rhs.r11 + r12*rhs.r21 + r13*rhs.r31;
        temp.r12 = r11*rhs.r12 + r12*rhs.r22 + r13*rhs.r32;
        temp.r13 = r11*rhs.r13 + r12*rhs.r23 + r13*rhs.r33;
        temp.r21 = r21*rhs.r11 + r22*rhs.r21 + r23*rhs.r31;
        temp.r22 = r21*rhs.r12 + r22*rhs.r22 + r23*rhs.r32;
        temp.r23 = r21*rhs.r13 + r22*rhs.r23 + r23*rhs.r33;
        temp.r31 = r31*rhs.r11 + r32*rhs.r21 + r33*rhs.r31;
        temp.r32 = r31*rhs.r12 + r32*rhs.r22 + r33*rhs.r32;
        temp.r33 = r31*rhs.r13 + r32*rhs.r23 + r33*rhs.r33;
        temp.rot = acos(temp.r11);
        *this = temp;
        return *this;
    };

    Vector2D Transform2D::translation() const {
        Vector2D tran_trans;
        tran_trans.x = r13;
        tran_trans.y = r23;
        return tran_trans;
    };

    double Transform2D::rotation() const {
        return rot;
    };

    Transform2D Transform2D::adj() const {
        Transform2D temp = *this;
        temp.r11 = 1.0;
        temp.r12 = 0.0;
        temp.r13 = 0.0;
        temp.r21 = r23;
        temp.r22 = r11;
        temp.r23 = r12;
        temp.r31 = -1*r13;
        temp.r32 = r21;
        temp.r33 = r22;
        return temp;
    };

    Twist2D Transform2D::conv_diff_frame(const Twist2D& new_frame) {
        Twist2D new_twist = new_frame;
        Transform2D tran_adj = adj();
        new_twist.linearx = tran_adj.r21*new_frame.angular + tran_adj.r22*new_frame.linearx + tran_adj.r23*new_frame.lineary;
        new_twist.lineary = tran_adj.r31*new_frame.angular + tran_adj.r32*new_frame.linearx + tran_adj.r33*new_frame.lineary;
        return new_twist;
    };

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf) {
        return os << "deg: " << rad2deg(tf.rotation()) << " x: " << tf.r13 << " y: " << tf.r23;
    };

    std::istream & operator>>(std::istream & is, Transform2D & tf) {
        double deg, transx, transy;
        is >> deg >> transx >> transy;
        Vector2D temp;
        temp.x = transx;
        temp.y = transy;
        tf = Transform2D(temp, deg2rad(deg));
        std::cin.ignore(50, '\n');
        return is;
    };

    std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
        return os << "[" << v.x << " " << v.y << "]";
    };

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs) {
        return lhs*=rhs;
    };

    std::istream & operator>>(std::istream & is, Vector2D & v) {
        char c1 = is.peek();
        if (c1 == '[') {
            is.get();
        }
        is >> v.x >> v.y;
        std::cin.ignore(50, '\n');
        return is;
    };

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
