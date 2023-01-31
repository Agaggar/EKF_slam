#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <cmath>

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

    Transform2D::Transform2D(double radians) {
        r11 = cos(radians);
        r12 = -1*sin(radians);
        r21 = sin(radians);
        r22 = cos(radians);
        rot = radians;
    };

    Transform2D::Transform2D(const Vector2D& trans, double radians) {
        r11 = cos(radians);
        r12 = -1*sin(radians);
        r21 = sin(radians);
        r22 = cos(radians);
        r13 = trans.x;
        r23 = trans.y;
        rot = radians;
    };

    Transform2D::Transform2D(const Twist2D& twist) {
        r11 = cos(twist.angular);
        r12 = -1*sin(twist.angular);
        r21 = sin(twist.angular);
        r22 = cos(twist.angular);
        r13 = twist.linearx;
        r23 = twist.lineary;
        rot = twist.angular;
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
        temp.rot = rot;
        return temp;
    };

    Twist2D Transform2D::conv_diff_frame(const Twist2D& new_frame) const {
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

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs) {
        Transform2D new_trans = lhs;
        return new_trans*=rhs;
    };
 

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

    double normalize_angle(double rad) {
        if (rad > -PI && rad <= PI) {
            return rad;
        }
        else {
            int rotations = (int) ((rad) / PI / 2.0);
            double normalized = rad - (rotations * 2*PI);
            if (normalized > -PI && normalized <= PI) {
                return normalized;
            }
            if (normalized < 0.0) {
                return normalized + 2*PI;
            }
            else {
                return normalized - 2*PI;
            }
        }
    };

    Vector2D & Vector2D::operator+=(Vector2D rhs) {
        this->x = this->x + rhs.x;
        this->y = this->y + rhs.y;
        return *this;
    };

    Vector2D & Vector2D::operator+(Vector2D rhs) {
        Vector2D myvec = *this;
        return myvec += rhs;
    };

    Vector2D & Vector2D::operator-=(Vector2D rhs) {
        this->x = this->x - rhs.x;
        this->y = this->y - rhs.y;
        return *this;
    };

    Vector2D & Vector2D::operator-(Vector2D rhs) {
        Vector2D myvec = *this;
        return myvec -= rhs;
    };

    Vector2D & Vector2D::operator*=(double rhs) {
        this->x = this->x * rhs;
        this->y = this->y * rhs;
        return *this;
    };

    Vector2D & Vector2D::operator*(double rhs) {
        Vector2D myvec = *this;
        return myvec *= rhs;
    };

    Vector2D operator*(double lhs, Vector2D rhs) {
        Vector2D myvec = rhs;
        return myvec*=lhs;
    };

    Vector2D operator*=(double lhs, Vector2D rhs) {
        return rhs*=lhs;
    };

    double dot(Vector2D vec1, Vector2D vec2) {
        return vec1.x*vec2.x + vec1.y*vec2.y;
    }

    double magnitude(Vector2D vec) {
        return sqrt(pow(vec.x,2) + pow(vec.y, 2));
    };

    double angle(Vector2D vec1, Vector2D vec2) {
        return acos(dot(vec1, vec2)/magnitude(vec1)/magnitude(vec2));
    };

    Transform2D Transform2D::integrate_twist(Twist2D twist0) {
        Transform2D Tbbprime;
        if (turtlelib::almost_equal(twist0.angular, 0.0, 1e-6)) {
            Tbbprime = Transform2D{Vector2D{twist0.linearx, twist0.lineary}, 0.0};
        }
        else {
            double dtheta = twist0.angular;
            double ys = -1.0*twist0.linearx/twist0.angular;
            double xs = twist0.lineary/twist0.angular;
            Transform2D Tsb = Transform2D{Vector2D{xs, ys}, dtheta};
            Transform2D Tssprime = Transform2D{dtheta};
            Tbbprime = (Tsb.inv() * Tssprime) * Tsb;
        }
        Twist2D dqb = Twist2D{Tbbprime.rotation(), Tbbprime.translation().x, Tbbprime.translation().y};
        Twist2D dq = Transform2D{twist0.angular}.adj().conv_diff_frame(dqb);
        // return Transform2D{dq};
        return Tbbprime;
    };
}

// int main(void) {
//     turtlelib::Vector2D myvec;
//     std::cout << "enter the vector you'd like to save: ";
//     std::cin >> myvec;
//     std::cout << myvec << std::endl;
//     std::cin.clear();

//     // turtlelib::Twist2D mytwist;
//     // std::cout << "enter the twist you'd like to save: ";
//     // std::cin >> mytwist;
//     // std::cout << mytwist << std::endl;
    
//     // turtlelib::Transform2D::Transform2D();
//     return 0;
// }