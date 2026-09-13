// Per-track constant-velocity Kalman filter for 2D obstacle tracking.
//
// State:       x = [px, py, vx, vy]^T   (map frame, meters / meters-per-second)
// Measurement: z = [px, py]^T           (cluster centroid, map frame)
//
// F(dt) = [[1, 0, dt, 0],
//          [0, 1, 0, dt],
//          [0, 0, 1,  0],
//          [0, 0, 0,  1]]
// H     = [[1, 0, 0, 0],
//          [0, 1, 0, 0]]
//
// Process noise Q(dt) is the standard continuous white-noise-acceleration
// (CWNA) model (Bar-Shalom, "Estimation with Applications to Tracking and
// Navigation", sec. 6.2.2): the true acceleration is modeled as zero-mean
// white noise with spectral density q = accel_noise_std^2, independent
// between the x and y axes. Integrating that noise through the constant-
// velocity kinematics over one step gives, for each axis's (position,
// velocity) pair:
//   [[dt^4/4, dt^3/2],
//    [dt^3/2, dt^2  ]] * q
// placed at the appropriate (p, v) indices of the 4x4 state; the x/y axes
// are uncoupled (no cross terms). This is a physically meaningful noise
// model (bounded, physically-plausible accelerations) rather than an
// arbitrary tuning knob.
//
// Measurement noise R = diag(measurement_noise_std^2, measurement_noise_std^2).
//
// Only Eigen (already a ROS 2 / system dependency, header-only) is used --
// no external filtering framework.
#ifndef PREDICTIVE_NAV_TRACKING__KALMAN_FILTER_HPP_
#define PREDICTIVE_NAV_TRACKING__KALMAN_FILTER_HPP_

#include <Eigen/Dense>

namespace predictive_nav_tracking
{

// A state/covariance pair produced by propagate(); never mutates the filter
// that produced it, so it is safe to use for future trajectory prediction.
struct PropagatedState
{
  Eigen::Vector4d x;    // [px, py, vx, vy]
  Eigen::Matrix4d P;
};

class ConstantVelocityKalmanFilter2D
{
public:
  static Eigen::Matrix4d stateTransition(double dt)
  {
    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;
    return F;
  }

  static Eigen::Matrix4d processNoise(double dt, double accel_noise_std)
  {
    const double q = accel_noise_std * accel_noise_std;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;

    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    // x axis: indices (0=px, 2=vx)
    Q(0, 0) = q * dt4 / 4.0;
    Q(0, 2) = q * dt3 / 2.0;
    Q(2, 0) = q * dt3 / 2.0;
    Q(2, 2) = q * dt2;
    // y axis: indices (1=py, 3=vy)
    Q(1, 1) = q * dt4 / 4.0;
    Q(1, 3) = q * dt3 / 2.0;
    Q(3, 1) = q * dt3 / 2.0;
    Q(3, 3) = q * dt2;
    return Q;
  }

  // measurement_noise_std: 1-sigma position measurement noise (meters), used
  // to build R = diag(sigma^2, sigma^2) at update() time.
  // accel_noise_std: 1-sigma unmodeled-acceleration noise (m/s^2), used to
  // build Q(dt) at predict() time.
  ConstantVelocityKalmanFilter2D(double accel_noise_std, double measurement_noise_std)
  : accel_noise_std_(accel_noise_std), measurement_noise_std_(measurement_noise_std)
  {
    x_.setZero();
    P_.setIdentity();
  }

  void initialize(double px, double py, double initial_position_variance,
    double initial_velocity_variance)
  {
    x_ << px, py, 0.0, 0.0;
    P_.setZero();
    P_(0, 0) = initial_position_variance;
    P_(1, 1) = initial_position_variance;
    P_(2, 2) = initial_velocity_variance;
    P_(3, 3) = initial_velocity_variance;
  }

  // Advances the live state in place: x = F x, P = F P F^T + Q(dt).
  void predict(double dt)
  {
    const Eigen::Matrix4d F = stateTransition(dt);
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + processNoise(dt, accel_noise_std_);
  }

  // Corrects the live state in place with measurement z = [zx, zy].
  void update(double zx, double zy)
  {
    static const Eigen::Matrix<double, 2, 4> H = [] {
        Eigen::Matrix<double, 2, 4> m = Eigen::Matrix<double, 2, 4>::Zero();
        m(0, 0) = 1.0;
        m(1, 1) = 1.0;
        return m;
      } ();

    const Eigen::Matrix2d R =
      Eigen::Matrix2d::Identity() * (measurement_noise_std_ * measurement_noise_std_);

    const Eigen::Vector2d z(zx, zy);
    const Eigen::Vector2d y = z - H * x_;
    const Eigen::Matrix2d S = H * P_ * H.transpose() + R;
    const Eigen::Matrix<double, 4, 2> K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    P_ = (Eigen::Matrix4d::Identity() - K * H) * P_;
  }

  // Returns the state/covariance that would result from predicting the live
  // filter forward by `dt`, WITHOUT modifying the live state. Used both for
  // association gating (peek at the current scan time) and for future
  // trajectory prediction (peek at t+0.5s, t+1.0s, ... independently -- the
  // CWNA Q(dt) formula already integrates the noise from 0 to dt, so a
  // single-shot propagate(dt) from the live state is exact, not an
  // approximation of chained smaller steps).
  PropagatedState propagate(double dt) const
  {
    const Eigen::Matrix4d F = stateTransition(dt);
    PropagatedState result;
    result.x = F * x_;
    result.P = F * P_ * F.transpose() + processNoise(dt, accel_noise_std_);
    return result;
  }

  const Eigen::Vector4d & state() const {return x_;}
  const Eigen::Matrix4d & covariance() const {return P_;}

  double px() const {return x_(0);}
  double py() const {return x_(1);}
  double vx() const {return x_(2);}
  double vy() const {return x_(3);}

private:
  double accel_noise_std_;
  double measurement_noise_std_;
  Eigen::Vector4d x_;
  Eigen::Matrix4d P_;
};

}  // namespace predictive_nav_tracking

#endif  // PREDICTIVE_NAV_TRACKING__KALMAN_FILTER_HPP_
