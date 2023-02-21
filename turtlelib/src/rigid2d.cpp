#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <iostream>
// #include <cmath>
#include <vector>

namespace turtlelib
{
Vector2D Vector2D::normalize()
{
  double mag = sqrt(pow(x, 2) + pow(y, 2));
  this->x /= mag;
  this->y /= mag;
  return *this;
}

Transform2D::Transform2D() {}

Transform2D::Transform2D(const Vector2D & trans)
: r31(trans.x),
  r32(trans.y)
{}

Transform2D::Transform2D(double radians)
: r11(cos(radians)),
  r12(-sin(radians)),
  r21(sin(radians)),
  r22(cos(radians)),
  rot(radians)
{}

Transform2D::Transform2D(const Vector2D & trans, double radians)
: r11(cos(radians)),
  r12(-sin(radians)),
  r13(trans.x),
  r21(sin(radians)),
  r22(cos(radians)),
  r23(trans.y),
  rot(radians)
{}

Transform2D::Transform2D(const Twist2D & twist)
: r11(cos(twist.angular)),
  r12(-sin(twist.angular)),
  r13(twist.linearx),
  r21(sin(twist.angular)),
  r22(cos(twist.angular)),
  r23(twist.lineary),
  rot(twist.angular)
{}

Vector2D Transform2D::operator()(const Vector2D & v) const
{
  return Vector2D{r11 * v.x + r12 * v.y + r13, r21 * v.x + r22 * v.y + r23};
}

Transform2D Transform2D::inv() const
{
  Transform2D inv_trans = *this;
  inv_trans.r12 = r21;
  inv_trans.r13 = -(r13 * r11 + r23 * r21);
  inv_trans.r21 = r12;
  inv_trans.r23 = -r23 * r11 + r13 * r21;
  inv_trans.rot = -inv_trans.rotation();
  return inv_trans;
}

Transform2D & Transform2D::operator*=(const Transform2D & rhs)
{
  Transform2D temp = *this;
  temp.r11 = r11 * rhs.r11 + r12 * rhs.r21 + r13 * rhs.r31;
  temp.r12 = r11 * rhs.r12 + r12 * rhs.r22 + r13 * rhs.r32;
  temp.r13 = r11 * rhs.r13 + r12 * rhs.r23 + r13 * rhs.r33;
  temp.r21 = r21 * rhs.r11 + r22 * rhs.r21 + r23 * rhs.r31;
  temp.r22 = r21 * rhs.r12 + r22 * rhs.r22 + r23 * rhs.r32;
  temp.r23 = r21 * rhs.r13 + r22 * rhs.r23 + r23 * rhs.r33;
  temp.r31 = r31 * rhs.r11 + r32 * rhs.r21 + r33 * rhs.r31;
  temp.r32 = r31 * rhs.r12 + r32 * rhs.r22 + r33 * rhs.r32;
  temp.r33 = r31 * rhs.r13 + r32 * rhs.r23 + r33 * rhs.r33;
  temp.rot = acos(temp.r11);
  *this = temp;
  return *this;
}

Vector2D Transform2D::translation() const
{
  return Vector2D{r13, r23};
}

double Transform2D::rotation() const
{
  return rot;
}

void Transform2D::setRotation(double rot_new)
{
  rot = rot_new;
}

Transform2D Transform2D::adj() const
{
  Transform2D temp = *this;
  temp.r11 = 1.0;
  temp.r12 = 0.0;
  temp.r13 = 0.0;
  temp.r21 = r23;
  temp.r22 = r11;
  temp.r23 = r12;
  temp.r31 = -r13;
  temp.r32 = r21;
  temp.r33 = r22;
  temp.rot = rot;
  return temp;
}

Twist2D Transform2D::conv_diff_frame(const Twist2D & new_frame) const
{
  Twist2D new_twist = new_frame;
  Transform2D tran_adj = adj();
  new_twist.linearx = tran_adj.r21 * new_frame.angular + tran_adj.r22 * new_frame.linearx +
    tran_adj.r23 * new_frame.lineary;
  new_twist.lineary = tran_adj.r31 * new_frame.angular + tran_adj.r32 * new_frame.linearx +
    tran_adj.r33 * new_frame.lineary;
  return new_twist;
}

std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
{
  return os << "deg: " << rad2deg(tf.rotation()) << " x: " << tf.r13 << " y: " << tf.r23;
}

std::istream & operator>>(std::istream & is, Transform2D & tf)
{
  double deg, transx, transy;
  is >> deg >> transx >> transy;
  tf = Transform2D(Vector2D{transx, transy}, deg2rad(deg));
//   std::cin.ignore(50, '\n');
  std::cin.clear();
  return is;
}

Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
{
  Transform2D new_trans = lhs;
  return new_trans *= rhs;
}


std::ostream & operator<<(std::ostream & os, const Vector2D & v)
{
  return os << "[" << v.x << " " << v.y << "]";
}

std::istream & operator>>(std::istream & is, Vector2D & v)
{
  if (is.peek() == '[') {
    is.get();
  }
  is >> v.x >> v.y;
//   std::cin.ignore(50, '\n');
  std::cin.clear();
  return is;
}

std::ostream & operator<<(std::ostream & os, const Twist2D & t)
{
  return os << "[" << t.angular << " " << t.linearx << " " << t.lineary << "]";
}

std::istream & operator>>(std::istream & is, Twist2D & t)
{
  if (is.peek() == '[') {
    is.get();
  }
  is >> t.angular >> t.linearx >> t.lineary;
//   std::cin.ignore(50, '\n');
  std::cin.clear();
  return is;
}

double normalize_angle(double rad)
{
  if (rad > -PI && rad <= PI) {
    return rad;
  } else {
    int rotations = (int) ((rad) / PI / 2.0);
    double normalized = rad - (rotations * 2 * PI);
    if (normalized > -PI && normalized <= PI) {
      return normalized;
    }
    if (normalized < 0.0) {
      return normalized + 2 * PI;
    } else {
      return normalized - 2 * PI;
    }
  }
}

Vector2D & Vector2D::operator+=(Vector2D rhs)
{
  this->x = this->x + rhs.x;
  this->y = this->y + rhs.y;
  return *this;
}

Vector2D & Vector2D::operator+(Vector2D rhs)
{
  Vector2D myvec = *this;
  return myvec += rhs;
}

Vector2D & Vector2D::operator-=(Vector2D rhs)
{
  this->x = this->x - rhs.x;
  this->y = this->y - rhs.y;
  return *this;
}

Vector2D & Vector2D::operator-(Vector2D rhs)
{
  Vector2D myvec = *this;
  return myvec -= rhs;
}

Vector2D & Vector2D::operator*=(double rhs)
{
  this->x = this->x * rhs;
  this->y = this->y * rhs;
  return *this;
}

Vector2D & Vector2D::operator*(double rhs)
{
  Vector2D myvec = *this;
  return myvec *= rhs;
}

Vector2D operator*(double lhs, Vector2D rhs)
{
  Vector2D myvec = rhs;
  return myvec *= lhs;
}

Vector2D operator*=(double lhs, Vector2D rhs)
{
  return rhs *= lhs;
}

double dot(Vector2D vec1, Vector2D vec2)
{
  return vec1.x * vec2.x + vec1.y * vec2.y;
}

double magnitude(Vector2D vec)
{
  return sqrt(pow(vec.x, 2) + pow(vec.y, 2));
}

double angle(Vector2D vec1, Vector2D vec2)
{
  return acos(dot(vec1, vec2) / magnitude(vec1) / magnitude(vec2));
}

DiffDrive::DiffDrive() {}

DiffDrive::DiffDrive(const double qx, const double qy, const double qtheta)
{
  q = std::vector<double> {qx, qy, qtheta};
}

DiffDrive::DiffDrive(
  const double phi_r, const double phi_l, const double qx, const double qy,
  const double qtheta)
{
  phi.at(0) = phi_r;
  phi.at(1) = phi_l;
  q = std::vector<double> {qx, qy, qtheta};
}

DiffDrive::DiffDrive(const std::vector<double> qnew)
{
  q = qnew;
}

DiffDrive::DiffDrive(const std::vector<double> phinew, const std::vector<double> qnew)
{
  phi = phinew;
  q = qnew;
}

void DiffDrive::setWheelRadius(double wheel_radius)
{
  this->wheel_radius = wheel_radius;
}

void DiffDrive::setWheelTrack(double wheel_track)
{
  this->wheel_track = wheel_track;
}

Transform2D DiffDrive::integrate_twist(Twist2D twist0)
{
  Transform2D Tbbprime;
  if (turtlelib::almost_equal(twist0.angular, 0.0, 1e-6)) {
    Tbbprime = Transform2D{Vector2D{twist0.linearx, twist0.lineary}, 0.0};
    Tbbprime.setRotation(0.0);
  } else {
    double dtheta = twist0.angular;
    double ys = -twist0.linearx / twist0.angular;
    double xs = twist0.lineary / twist0.angular;
    Transform2D Tsb = Transform2D{Vector2D{xs, ys}, 0.0};
    Transform2D Tssprime = Transform2D{dtheta};
    Tbbprime = (Tsb.inv() * Tssprime) * Tsb;
  }
  return Tbbprime;
}

void DiffDrive::fkinematics(const double phi_l_new, const double phi_r_new)
{
  // Twist2D Vb = velToTwist(std::vector<double>(phi_l_new - phi.at(0), phi_r_new - phi.at(1)));
  // Transform2D Tbbprime = integrate_twist(Vb);
  // Transform2D Twb = Transform2D(Vector2D{q.at(0), q.at(1)}, q.at(2));
  // Transform2D Twbprime = Twb * Tbbprime;
  // Twist2D dq{Twbprime.rotation(), Twbprime.translation().x, Twbprime.translation().y};
  std::vector<std::vector<double>> hstar {{-wheel_radius / 2.0 / wheel_track,
    wheel_radius / 2.0 / wheel_track},
    {wheel_radius / 2.0 * cos(q.at(2)), wheel_radius / 2.0 * cos(q.at(2))},
    {wheel_radius / 2.0 * sin(q.at(2)), wheel_radius / 2.0 * sin(q.at(2))}};
  Twist2D dq = {hstar.at(0).at(0) * (phi_l_new - phi.at(0)) + hstar.at(0).at(1) * (phi_r_new - phi.at(1)),
    hstar.at(1).at(0) * (phi_l_new - phi.at(0)) + hstar.at(1).at(1) * (phi_r_new - phi.at(1)),
    hstar.at(2).at(0) * (phi_l_new - phi.at(0)) + hstar.at(2).at(1) * (phi_r_new - phi.at(1))};
  // Twist2D dqb = Twist2D{Tbbprime.rotation(), Tbbprime.translation().x, Tbbprime.translation().y};
  // Transform2D adj_theta = Transform2D{q.at(2)}.adj();
  // adj_theta.setRotation(q.at(2));
  // Twist2D dq = adj_theta.conv_diff_frame(Vb);
  q.at(0) += dq.linearx;
  q.at(1) += dq.lineary;
  q.at(2) += dq.angular;
}

void DiffDrive::fkinematics(const std::vector<double> phiprime)
{
  // Twist2D Vb = velToTwist(phiprime);
  // Transform2D Tbbprime = integrate_twist(Vb);
  // Transform2D Twb = Transform2D(Vector2D{q.at(0), q.at(1)}, q.at(2));
  // Transform2D Twbprime = Twb * Tbbprime;
  // Twist2D dq{Twbprime.rotation(), Twbprime.translation().x, Twbprime.translation().y};
  std::vector<std::vector<double>> hstar {{-wheel_radius / 2.0 / wheel_track,
    wheel_radius / 2.0 / wheel_track},
    {wheel_radius / 2.0 * cos(q.at(2)), wheel_radius / 2.0 * cos(q.at(2))},
    {wheel_radius / 2.0 * sin(q.at(2)), wheel_radius / 2.0 * sin(q.at(2))}};
  Twist2D dq = {hstar.at(0).at(0) * phiprime.at(0) + hstar.at(0).at(1) * phiprime.at(1),
    hstar.at(1).at(0) * phiprime.at(0) + hstar.at(1).at(1) * phiprime.at(1),
    hstar.at(2).at(0) * phiprime.at(0) + hstar.at(2).at(1) * phiprime.at(1)};
  // Twist2D dqb = Twist2D{Tbbprime.rotation(), Tbbprime.translation().x, Tbbprime.translation().y};
  // Transform2D adj_theta = Transform2D{q.at(2)}.adj();
  // adj_theta.setRotation(q.at(2));
  // Twist2D dq = adj_theta.conv_diff_frame(Vb);
  q.at(0) += dq.linearx;
  q.at(1) += dq.lineary;
  q.at(2) += dq.angular;
  phi.at(0) += phiprime.at(0);
  phi.at(1) += phiprime.at(1);
  // q.at(2) = turtlelib::normalize_angle(q.at(2));
}

Twist2D DiffDrive::velToTwist(const std::vector<double> phiprime)
{
  // std::vector<std::vector<double>> hstar {{-wheel_radius/2.0/wheel_track, wheel_radius/2.0/wheel_track},
  //                                         {wheel_radius/2.0*cos(q.at(2)), wheel_radius/2.0*cos(q.at(2))},
  //                                         {wheel_radius/2.0*sin(q.at(2)), wheel_radius/2.0*sin(q.at(2))}};
  std::vector<std::vector<double>> hstar {{-wheel_radius / 2.0 / wheel_track,
    wheel_radius / 2.0 / wheel_track},
    {wheel_radius / 2.0, wheel_radius / 2.0},
    {0.0, 0.0}};
  return {hstar.at(0).at(0) * phiprime.at(0) + hstar.at(0).at(1) * phiprime.at(1),
    hstar.at(1).at(0) * phiprime.at(0) + hstar.at(1).at(1) * phiprime.at(1),
    hstar.at(2).at(0) * phiprime.at(0) + hstar.at(2).at(1) * phiprime.at(1)};
}

std::vector<double> DiffDrive::ikinematics(Twist2D twist0)
{
  // std::vector<std::vector<double>> H {{-wheel_track/wheel_radius, 1.0, 0.0},
  //                                     {wheel_track/wheel_radius, 1.0, 0.0}};
  // std::vector<double> wheel_vel {H.at(0).at(0) * twist0.angular + H.at(0).at(1) * twist0.linearx + H.at(0).at(2) * twist0.lineary,
  //                                H.at(1).at(0) * twist0.angular + H.at(1).at(1) * twist0.linearx + H.at(1).at(2) * twist0.lineary};
  if (!almost_equal(twist0.lineary, 0.0)) {
    throw std::logic_error("wheels slipped! since y velocity is not 0");
  } else {
    return {(-0.5*wheel_track * twist0.angular + twist0.linearx) / wheel_radius,
      (0.5*wheel_track * twist0.angular + twist0.linearx) / wheel_radius};
  }
}

std::vector<double> DiffDrive::getWheelPos()
{
  return this->phi;
}

void DiffDrive::setWheelPos(std::vector<double> phinew)
{
  this->phi = phinew;
}

std::vector<double> DiffDrive::getCurrentConfig()
{
  return this->q;
}

void DiffDrive::setCurrentConfig(std::vector<double> qnew)
{
  this->q = qnew;
}
}
