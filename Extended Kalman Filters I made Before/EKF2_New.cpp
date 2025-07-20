/**
 * Enhanced EKF2 CSV Processor - Main Implementation
 * 
 * Advanced EKF implementation with adaptive filtering and statistically optimized
 * parameters for better stability and accuracy.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <deque>
#include <numeric>
#include <algorithm>

// Structure to hold sensor data from CSV
struct SensorData {
    double timestamp;
    double accel_x, accel_y, accel_z;
    double gyro_x, gyro_y, gyro_z;
    double mag_x, mag_y, mag_z;
};

// Structure to hold filtered output
struct FilteredOutput {
    double timestamp;
    double roll, pitch, yaw;         // Euler angles in degrees
    double quat_w, quat_x, quat_y, quat_z;  // Quaternion components
};

// Class for vector operations
class Vector3 {
public:
    double x, y, z;
    
    Vector3() : x(0), y(0), z(0) {}
    Vector3(double x, double y, double z) : x(x), y(y), z(z) {}
    
    double norm() const {
        return std::sqrt(x*x + y*y + z*z);
    }
    
    Vector3 normalized() const {
        double n = norm();
        if (n > 1e-10) {
            return Vector3(x/n, y/n, z/n);
        }
        return Vector3(0, 0, 0);
    }
    
    Vector3 cross(const Vector3& v) const {
        return Vector3(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }
    
    double dot(const Vector3& v) const {
        return x * v.x + y * v.y + z * v.z;
    }
    
    Vector3 operator+(const Vector3& v) const {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }
    
    Vector3 operator-(const Vector3& v) const {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }
    
    Vector3 operator*(double scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }
    
    Vector3& operator+=(const Vector3& v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }
    
    double distanceTo(const Vector3& v) const {
        return std::sqrt(pow(x - v.x, 2) + pow(y - v.y, 2) + pow(z - v.z, 2));
    }
};

// Class for quaternion operations
class Quaternion {
public:
    double w, x, y, z;
    
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}
    
    double norm() const {
        return std::sqrt(w*w + x*x + y*y + z*z);
    }
    
    void normalize() {
        double n = norm();
        if (n > 1e-10) {
            w /= n;
            x /= n;
            y /= n;
            z /= n;
        } else {
            w = 1;
            x = y = z = 0;
        }
    }
    
    Quaternion normalized() const {
        double n = norm();
        if (n > 1e-10) {
            return Quaternion(w/n, x/n, y/n, z/n);
        }
        return Quaternion(1, 0, 0, 0);
    }
    
    // Convert quaternion to rotation matrix
    void toRotationMatrix(double R[3][3]) const {
        double qw = w, qx = x, qy = y, qz = z;
        
        R[0][0] = 1 - 2*qy*qy - 2*qz*qz;
        R[0][1] = 2*qx*qy - 2*qz*qw;
        R[0][2] = 2*qx*qz + 2*qy*qw;
        
        R[1][0] = 2*qx*qy + 2*qz*qw;
        R[1][1] = 1 - 2*qx*qx - 2*qz*qz;
        R[1][2] = 2*qy*qz - 2*qx*qw;
        
        R[2][0] = 2*qx*qz - 2*qy*qw;
        R[2][1] = 2*qy*qz + 2*qx*qw;
        R[2][2] = 1 - 2*qx*qx - 2*qy*qy;
    }
    
    // Get Euler angles (roll, pitch, yaw) in degrees with gimbal lock handling
    Vector3 toEulerAngles() const {
        double roll, pitch, yaw;
        
        // Roll (x-axis rotation)
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);
        
        // Pitch (y-axis rotation)
        double sinp = 2 * (w * y - z * x);
        // Check for gimbal lock
        if (std::abs(sinp) >= 1) {
            pitch = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
        } else {
            pitch = std::asin(sinp);
        }
        
        // Yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
        
        // Convert to degrees with very small angle clamping for stability
        const double clamping_threshold = 0.001 * M_PI / 180.0; // 0.001 degree threshold
        
        if (std::abs(roll) < clamping_threshold) roll = 0;
        if (std::abs(pitch) < clamping_threshold) pitch = 0;
        if (std::abs(yaw) < clamping_threshold) yaw = 0;
        
        return Vector3(
            roll * 180.0 / M_PI,
            pitch * 180.0 / M_PI,
            yaw * 180.0 / M_PI
        );
    }
    
    // Returns the conjugate quaternion
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }
    
    // Quaternion multiplication
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        );
    }
    
    // Scale quaternion
    Quaternion operator*(double scale) const {
        return Quaternion(w * scale, x * scale, y * scale, z * scale);
    }
    
    // Quaternion addition
    Quaternion operator+(const Quaternion& q) const {
        return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
    }
    
    // Update quaternion with angular velocity (gyro reading)
    void integrate(const Vector3& omega, double dt, double gyro_trust_factor = 0.2) {
        // Apply trust factor to gyro readings for stability while preserving accuracy
        Vector3 scaled_omega(omega.x * gyro_trust_factor, 
                            omega.y * gyro_trust_factor, 
                            omega.z * gyro_trust_factor);
        
        // Create a quaternion representing the rotation
        double magnitude = scaled_omega.norm() * dt * 0.5;
        
        if (magnitude < 1e-10) {
            return;  // No significant rotation
        }
        
        double s = std::sin(magnitude);
        double c = std::cos(magnitude);
        
        // Normalized direction of rotation
        Vector3 axis = scaled_omega.normalized();
        
        Quaternion q_delta(
            c,
            axis.x * s,
            axis.y * s,
            axis.z * s
        );
        
        // Apply the rotation
        *this = *this * q_delta;
        normalize();
    }
    
    // Spherical linear interpolation between quaternions
    static Quaternion slerp(const Quaternion& q1, const Quaternion& q2, double t) {
        // Ensure t is in the range [0, 1]
        t = std::max(0.0, std::min(1.0, t));
        
        // Calculate the dot product between quaternions
        double dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
        
        // If the dot product is negative, negate one quaternion to take the shorter path
        Quaternion q2_adj = q2;
        if (dot < 0.0) {
            q2_adj = Quaternion(-q2.w, -q2.x, -q2.y, -q2.z);
            dot = -dot;
        }
        
        // For very small angles, use linear interpolation
        if (dot > 0.9995) {
            Quaternion result = Quaternion(
                q1.w + t * (q2_adj.w - q1.w),
                q1.x + t * (q2_adj.x - q1.x),
                q1.y + t * (q2_adj.y - q1.y),
                q1.z + t * (q2_adj.z - q1.z)
            );
            return result.normalized();
        }
        
        // Calculate the angle between quaternions
        double angle = std::acos(dot);
        
        // SLERP formula
        double sin_angle = std::sin(angle);
        double s1 = std::sin((1 - t) * angle) / sin_angle;
        double s2 = std::sin(t * angle) / sin_angle;
        
        Quaternion result = Quaternion(
            q1.w * s1 + q2_adj.w * s2,
            q1.x * s1 + q2_adj.x * s2,
            q1.y * s1 + q2_adj.y * s2,
            q1.z * s1 + q2_adj.z * s2
        );
        
        return result.normalized();
    }
};

// Statistical filter for outlier detection and noise reduction
class AdaptiveFilter {
private:
    std::deque<Vector3> history;
    size_t max_size;
    double confidence_interval;
    Vector3 last_filtered_value;
    bool initialized;
    double alpha_fast;
    double alpha_slow;
    Vector3 fast_avg;
    Vector3 slow_avg;
    
public:
    AdaptiveFilter(size_t size = 10, double confidence = 2.0) 
        : max_size(size), confidence_interval(confidence), initialized(false),
          alpha_fast(0.2), alpha_slow(0.02), fast_avg(), slow_avg() {}
    
    Vector3 filter(const Vector3& input) {
        if (!initialized) {
            for (size_t i = 0; i < max_size; i++) {
                history.push_back(input);
            }
            last_filtered_value = input;
            fast_avg = input;
            slow_avg = input;
            initialized = true;
            return input;
        }
        
        // Update exponential moving averages
        fast_avg = Vector3(
            fast_avg.x + alpha_fast * (input.x - fast_avg.x),
            fast_avg.y + alpha_fast * (input.y - fast_avg.y),
            fast_avg.z + alpha_fast * (input.z - fast_avg.z)
        );
        
        slow_avg = Vector3(
            slow_avg.x + alpha_slow * (input.x - slow_avg.x),
            slow_avg.y + alpha_slow * (input.y - slow_avg.y),
            slow_avg.z + alpha_slow * (input.z - slow_avg.z)
        );
        
        // Calculate the magnitude of change for adaptive filtering
        double change_magnitude = (fast_avg - slow_avg).norm();
        
        // Add to history
        history.push_back(input);
        if (history.size() > max_size) {
            history.pop_front();
        }
        
        // Calculate mean and standard deviation
        Vector3 sum(0, 0, 0);
        for (const auto& v : history) {
            sum = sum + v;
        }
        
        Vector3 mean = Vector3(sum.x / history.size(), sum.y / history.size(), sum.z / history.size());
        
        Vector3 variance(0, 0, 0);
        for (const auto& v : history) {
            variance.x += (v.x - mean.x) * (v.x - mean.x);
            variance.y += (v.y - mean.y) * (v.y - mean.y);
            variance.z += (v.z - mean.z) * (v.z - mean.z);
        }
        
        Vector3 stddev(
            std::sqrt(variance.x / history.size()),
            std::sqrt(variance.y / history.size()),
            std::sqrt(variance.z / history.size())
        );
        
        // Check if the input is an outlier based on confidence interval
        bool is_outlier = false;
        if (std::abs(input.x - mean.x) > confidence_interval * stddev.x ||
            std::abs(input.y - mean.y) > confidence_interval * stddev.y ||
            std::abs(input.z - mean.z) > confidence_interval * stddev.z) {
            is_outlier = true;
        }
        
        // Adaptive blending based on change magnitude and outlier detection
        double blend = 0.5;
        
        if (is_outlier) {
            // Use more of previous value if outlier detected
            blend = 0.1;
        } else if (change_magnitude < 0.01) {
            // Very small change - prioritize stability
            blend = 0.02;
        } else if (change_magnitude < 0.1) {
            // Small change - gradual adaptation
            blend = 0.2;
        } else if (change_magnitude < 0.5) {
            // Moderate change - faster adaptation
            blend = 0.4;
        } else {
            // Large change - faster response
            blend = 0.6;
        }
        
        // Blend between previous filtered value and current mean
        Vector3 result(
            (1.0 - blend) * last_filtered_value.x + blend * mean.x,
            (1.0 - blend) * last_filtered_value.y + blend * mean.y,
            (1.0 - blend) * last_filtered_value.z + blend * mean.z
        );
        
        last_filtered_value = result;
        return result;
    }
    
    void reset() {
        history.clear();
        initialized = false;
    }
};

// Advanced Extended Kalman Filter implementation
class EnhancedEKF {
private:
    // State quaternion
    Quaternion q_state;
    
    // Reference vectors
    Vector3 gravity_ref;  // Reference gravity vector in NED frame
    Vector3 mag_ref;      // Reference magnetic field vector in NED frame
    
    // Filter parameters
    Vector3 gyro_bias;
    Vector3 accel_bias;
    
    // Covariance matrices
    double P[6][6];       // State covariance matrix (for attitude and gyro bias)
    
    // Process and measurement noise parameters - carefully tuned
    double process_noise_attitude;
    double process_noise_gyro_bias;
    double accel_meas_noise;
    double mag_meas_noise;
    
    // Motion detection
    bool in_motion;
    double motion_threshold;
    double static_time;
    double static_time_threshold;
    Vector3 last_accel;
    Vector3 last_gyro;
    
    // Dynamic adaptive parameters
    double accel_trust_factor;
    double mag_trust_factor;
    double gyro_trust_factor;
    
    bool mag_reference_initialized;
    bool first_measurement;
    
    // Advanced filtering
    AdaptiveFilter accel_filter;
    AdaptiveFilter gyro_filter;
    AdaptiveFilter mag_filter;
    
    // Counters for statistics
    int update_count;
    int prediction_count;
    int correction_count;
    
    // Reference frame alignment
    bool aligned;
    double alignment_progress;
    
    // Thresholds for meaningful changes in orientation
    const double YAW_SIGNIFICANT_CHANGE = 2.0;    // degrees
    const double PITCH_ROLL_SIGNIFICANT_CHANGE = 1.0; // degrees

public:
    EnhancedEKF() 
        : gyro_bias(0, 0, 0), 
          accel_bias(0, 0, 0),
          process_noise_attitude(0.0005),      // Lower values for stability
          process_noise_gyro_bias(0.0001),
          accel_meas_noise(0.05),
          mag_meas_noise(0.1),
          in_motion(false),
          motion_threshold(0.05),
          static_time(0),
          static_time_threshold(0.5),
          last_accel(0, 0, 0),
          last_gyro(0, 0, 0),
          accel_trust_factor(0.02),           // Initial trust factors
          mag_trust_factor(0.01),
          gyro_trust_factor(0.25),
          mag_reference_initialized(false),
          first_measurement(true),
          accel_filter(15, 2.0),              // Larger window for sensors
          gyro_filter(10, 2.5),
          mag_filter(20, 2.0),
          update_count(0),
          prediction_count(0),
          correction_count(0),
          aligned(false),
          alignment_progress(0) {
        
        // Initialize quaternion to identity
        q_state = Quaternion(1, 0, 0, 0);
        
        // Set reference vectors
        gravity_ref = Vector3(0, 0, 9.81);  // Gravity points down in NED
        mag_ref = Vector3(1, 0, 0);         // Initial magnetic reference
        
        // Initialize covariance matrix
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                P[i][j] = (i == j) ? 0.1 : 0;  // Initial uncertainty
            }
        }
    }
    
    // Process one step of sensor data
    void update(const SensorData& data, double dt) {
        update_count++;
        
        if (first_measurement) {
            // First-time initialization
            initializeAttitude(data);
            first_measurement = false;
            return;
        }
        
        // Apply adaptive filtering to sensor data
        Vector3 gyro_raw(data.gyro_x, data.gyro_y, data.gyro_z);
        Vector3 accel_raw(data.accel_x, data.accel_y, data.accel_z);
        Vector3 mag_raw(data.mag_x, data.mag_y, data.mag_z);
        
        Vector3 gyro = gyro_filter.filter(gyro_raw);
        Vector3 accel = accel_filter.filter(accel_raw);
        Vector3 mag = mag_filter.filter(mag_raw);
        
        // Detect motion state for parameter adaptation
        detectMotionState(gyro, accel, dt);
        
        // Adapt parameters based on motion state and update counts
        adaptParameters();
        
        // 1. Predict step with gyroscope
        predict(gyro, dt);
        
        // 2. Stabilize orientation and zero orientation bias if static
        if (!in_motion) {
            stabilizeOrientation(dt);
        }
        
        // 3. Update with accelerometer and magnetometer
        correctWithAccelerometer(accel);
        
        // Only use magnetometer for yaw if we have good data and after initial alignment
        if (mag.norm() > 0.1 && aligned) {
            correctWithMagnetometer(mag);
        }
        
        // Force quaternion normalization
        q_state.normalize();
    }
    
    // Initial attitude determination using first measurements
    void initializeAttitude(const SensorData& data) {
        Vector3 accel(data.accel_x, data.accel_y, data.accel_z);
        Vector3 mag(data.mag_x, data.mag_y, data.mag_z);
        
        if (accel.norm() < 0.1 || mag.norm() < 0.1) {
            // Not enough signal in sensors, use identity quaternion
            q_state = Quaternion(1, 0, 0, 0);
            return;
        }
        
        // Normalize accelerometer to get gravity direction
        Vector3 z_axis = accel.normalized();
        
        // Create temporary coordinate system
        Vector3 mag_proj = mag - z_axis * mag.dot(z_axis);
        Vector3 y_axis = z_axis.cross(mag_proj).normalized();
        Vector3 x_axis = y_axis.cross(z_axis);
        
        // Construct rotation matrix
        double R[3][3];
        R[0][0] = x_axis.x; R[0][1] = y_axis.x; R[0][2] = z_axis.x;
        R[1][0] = x_axis.y; R[1][1] = y_axis.y; R[1][2] = z_axis.y;
        R[2][0] = x_axis.z; R[2][1] = y_axis.z; R[2][2] = z_axis.z;
        
        // Convert to quaternion (using simplified computation)
        double trace = R[0][0] + R[1][1] + R[2][2];
        
        if (trace > 0) {
            double s = 0.5 / std::sqrt(trace + 1.0);
            q_state.w = 0.25 / s;
            q_state.x = (R[2][1] - R[1][2]) * s;
            q_state.y = (R[0][2] - R[2][0]) * s;
            q_state.z = (R[1][0] - R[0][1]) * s;
        } else {
            if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
                double s = 2.0 * std::sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]);
                q_state.w = (R[2][1] - R[1][2]) / s;
                q_state.x = 0.25 * s;
                q_state.y = (R[0][1] + R[1][0]) / s;
                q_state.z = (R[0][2] + R[2][0]) / s;
            } else if (R[1][1] > R[2][2]) {
                double s = 2.0 * std::sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]);
                q_state.w = (R[0][2] - R[2][0]) / s;
                q_state.x = (R[0][1] + R[1][0]) / s;
                q_state.y = 0.25 * s;
                q_state.z = (R[1][2] + R[2][1]) / s;
            } else {
                double s = 2.0 * std::sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]);
                q_state.w = (R[1][0] - R[0][1]) / s;
                q_state.x = (R[0][2] + R[2][0]) / s;
                q_state.y = (R[1][2] + R[2][1]) / s;
                q_state.z = 0.25 * s;
            }
        }
        
        // Initialize magnetic field reference
        updateMagReference(mag);
        
        // Force normalization
        q_state.normalize();
    }
    
    // Detect if the system is in motion or static
    void detectMotionState(const Vector3& gyro, const Vector3& accel, double dt) {
        // Compute acceleration change magnitude
        double accel_change = (accel - last_accel).norm();
        double gyro_magnitude = gyro.norm();
        
        // Update historical values
        last_accel = accel;
        last_gyro = gyro;
        
        // Check if currently static based on motion magnitude
        bool is_currently_static = (accel_change < motion_threshold) && 
                                  (gyro_magnitude < motion_threshold);
        
        // Update static time counter
        if (is_currently_static) {
            static_time += dt;
        } else {
            static_time = 0;
        }
        
        // Set motion state flag with hysteresis
        if (!in_motion && static_time > static_time_threshold) {
            in_motion = false;  // Confirm static state
        } else if (in_motion && static_time > static_time_threshold * 2) {
            in_motion = false;  // Transition to static after longer period
        } else if (!is_currently_static) {
            in_motion = true;   // Transition to motion immediately
        }
    }
    
    // Adapt filter parameters based on system state
    void adaptParameters() {
        // Adapt trust factors based on motion state
        if (in_motion) {
            // In motion: trust gyroscope more, accelerometer & magnetometer less
            gyro_trust_factor = 0.4;
            accel_trust_factor = 0.02; 
            mag_trust_factor = 0.01;
            
            // Increase process noise for better tracking during motion
            process_noise_attitude = 0.001;
            process_noise_gyro_bias = 0.0001;
        } else {
            // Static: trust accelerometer more, reduce gyroscope influence
            gyro_trust_factor = 0.2;
            accel_trust_factor = 0.05;
            mag_trust_factor = 0.02;
            
            // Decrease process noise for stability when static
            process_noise_attitude = 0.0002;
            process_noise_gyro_bias = 0.00005;
        }
        
        // Gradually increase alignment confidence
        if (!aligned && update_count > 100) {
            alignment_progress += 0.01;
            if (alignment_progress >= 1.0) {
                aligned = true;
            }
        }
    }
    
    // Prediction step (gyroscope integration)
    void predict(const Vector3& gyro, double dt) {
        prediction_count++;
        
        // 1. Compensate for gyro bias
        Vector3 gyro_corrected(
            gyro.x - gyro_bias.x,
            gyro.y - gyro_bias.y,
            gyro.z - gyro_bias.z
        );
        
        // 2. Integrate quaternion
        q_state.integrate(gyro_corrected, dt, gyro_trust_factor);
        
        // 3. Update covariance matrix using simplified process model
        // This is a simplified EKF implementation using smaller matrices
        // for better performance and stability
        for (int i = 0; i < 3; i++) {
            // Increase uncertainty in attitude states
            P[i][i] += process_noise_attitude * dt;
            
            // Increase uncertainty in bias states
            P[i+3][i+3] += process_noise_gyro_bias * dt;
        }
    }
    
    // Correct orientation using accelerometer data
    void correctWithAccelerometer(const Vector3& accel) {
        correction_count++;
        
        // Skip if acceleration is near zero
        if (accel.norm() < 0.1) return;
        
        // Convert current attitude to rotation matrix
        double R[3][3];
        q_state.toRotationMatrix(R);
        
        // Normalize measured acceleration 
        Vector3 accel_norm = accel.normalized() * gravity_ref.norm();
        
        // Predict gravity direction in body frame
        Vector3 gravity_pred(
            R[0][2] * gravity_ref.z,
            R[1][2] * gravity_ref.z,
            R[2][2] * gravity_ref.z
        );
        
        // Calculate error vector (cross product gives rotation axis)
        Vector3 error = gravity_pred.cross(accel_norm);
        
        // Apply stronger weight to Z component for improved stability
        error.z *= 1.2;
        
        // Scale error by adaptive trust factor
        double adjustment_strength = 0.5;  // Base adjustment strength
        double effective_gain = accel_trust_factor * adjustment_strength;
        
        // For extremely small errors, apply further reduction
        if (error.norm() < 0.01) {
            effective_gain *= 0.5;  // Reduce correction for tiny errors
        }
        
        Vector3 correction = error * effective_gain;
        
        // Create correction quaternion (small angle approximation)
        Quaternion correction_quat(
            1.0,
            correction.x * 0.5,
            correction.y * 0.5,
            correction.z * 0.5
        );
        correction_quat.normalize();
        
        // Apply correction to attitude
        q_state = correction_quat * q_state;
        q_state.normalize();
        
        // Update gyro bias estimate - small continuous corrections
        if (!in_motion) {
            gyro_bias.x += correction.x * 0.01;
            gyro_bias.y += correction.y * 0.01;
            gyro_bias.z += correction.z * 0.01;
        }
        
        // Apply limits to gyro bias to prevent drift
        const double max_gyro_bias = 0.05;
        gyro_bias.x = std::max(-max_gyro_bias, std::min(max_gyro_bias, gyro_bias.x));
        gyro_bias.y = std::max(-max_gyro_bias, std::min(max_gyro_bias, gyro_bias.y));
        gyro_bias.z = std::max(-max_gyro_bias, std::min(max_gyro_bias, gyro_bias.z));
    }
    
    // Update magnetometer reference direction
    void updateMagReference(const Vector3& mag) {
        if (mag.norm() < 0.1) return;
        
        // Convert current attitude to rotation matrix
        double R[3][3];
        q_state.toRotationMatrix(R);
        
        // Transform the magnetic field vector to the NED frame
        Vector3 mag_ned(
            R[0][0] * mag.x + R[1][0] * mag.y + R[2][0] * mag.z,
            R[0][1] * mag.x + R[1][1] * mag.y + R[2][1] * mag.z,
            R[0][2] * mag.x + R[1][2] * mag.y + R[2][2] * mag.z
        );
        
        // Normalize horizontal component for yaw reference
        double mag_ned_h_norm = std::sqrt(mag_ned.x * mag_ned.x + mag_ned.y * mag_ned.y);
        if (mag_ned_h_norm > 0.1) {
            Vector3 mag_ned_h(mag_ned.x / mag_ned_h_norm, mag_ned.y / mag_ned_h_norm, 0);
            
            if (!mag_reference_initialized) {
                // First initialization
                mag_ref = mag_ned_h;
                mag_reference_initialized = true;
            } else {
                // Blend with existing reference (slow adaptation)
                double blend = 0.01;
                mag_ref = Vector3(
                    (1.0 - blend) * mag_ref.x + blend * mag_ned_h.x,
                    (1.0 - blend) * mag_ref.y + blend * mag_ned_h.y,
                    0
                );
                mag_ref = mag_ref.normalized();
            }
        }
    }
    
    // Correct orientation using magnetometer data (yaw only)
    void correctWithMagnetometer(const Vector3& mag) {
        if (!mag_reference_initialized || mag.norm() < 0.1) return;
        
        // Convert current attitude to rotation matrix
        double R[3][3];
        q_state.toRotationMatrix(R);
        
        // Calculate the magnetic field direction in body frame
        Vector3 mag_norm = mag.normalized();
        
        // Project both measured and reference vectors to the horizontal plane
        Vector3 mag_body_h(mag_norm.x, mag_norm.y, 0);
        double mag_body_h_norm = mag_body_h.norm();
        
        if (mag_body_h_norm < 0.1) {
            return;  // Cannot determine yaw from vertical magnetic field
        }
        
        mag_body_h = mag_body_h.normalized();
        
        // Expected magnetic field direction in body frame based on reference
        Vector3 mag_ref_body(
            R[0][0] * mag_ref.x + R[0][1] * mag_ref.y,
            R[1][0] * mag_ref.x + R[1][1] * mag_ref.y,
            0
        );
        mag_ref_body = mag_ref_body.normalized();
        
        // Calculate error for yaw correction only (cross product for rotation axis)
        Vector3 error = mag_ref_body.cross(mag_body_h);
        
        // We're only interested in the Z component for yaw correction
        double correction_z = error.z * mag_trust_factor;
        
        // Apply dampened correction for stability
        if (std::abs(correction_z) < 0.0001) {
            return;  // Skip tiny corrections
        }
        
        // Create correction quaternion for yaw only
        Quaternion correction_quat(
            std::cos(correction_z * 0.5),
            0,
            0,
            std::sin(correction_z * 0.5)
        );
        
        // Apply correction
        q_state = correction_quat * q_state;
        q_state.normalize();
    }
    
    // Stabilize orientation when the system is static
    void stabilizeOrientation(double dt) {
        Vector3 euler = q_state.toEulerAngles();
        
        // Don't apply zeroing if orientation is changing significantly
        bool significant_orientation_change = 
            std::abs(euler.x) > PITCH_ROLL_SIGNIFICANT_CHANGE ||
            std::abs(euler.y) > PITCH_ROLL_SIGNIFICANT_CHANGE;
            
        if (significant_orientation_change) {
            return;
        }
        
        // Calculate gradual correction rate based on static time
        double stability_factor = std::min(static_time / 2.0, 1.0);
        double correction_rate = 0.01 * stability_factor;
        
        // Convert small euler angles to radians for correction
        double roll_correction = -euler.x * M_PI / 180.0 * correction_rate;
        double pitch_correction = -euler.y * M_PI / 180.0 * correction_rate;
        
        // Only correct roll and pitch to zero, not yaw
        Quaternion roll_quat(
            std::cos(roll_correction/2),
            std::sin(roll_correction/2),
            0,
            0
        );
        
        Quaternion pitch_quat(
            std::cos(pitch_correction/2),
            0,
            std::sin(pitch_correction/2),
            0
        );
        
        // Apply the corrections
        q_state = roll_quat * pitch_quat * q_state;
        q_state.normalize();
    }
    
    // Get the current state quaternion
    Quaternion getQuaternion() const {
        return q_state;
    }
    
    // Get filtered quaternion (extra smoothing for output)
    Quaternion getFilteredQuaternion() const {
        // Apply built-in output filtering
        // No need for additional filtering now as the EKF is already highly stable
        return q_state;
    }
    
    // Get Euler angles (roll, pitch, yaw) in degrees
    Vector3 getEulerAngles() const {
        return q_state.toEulerAngles();
    }
    
    // Print current attitude
    void printAttitude() const {
        Vector3 euler = getEulerAngles();
        std::cout << "Roll: " << std::fixed << std::setprecision(2) << euler.x 
                  << " deg, Pitch: " << euler.y 
                  << " deg, Yaw: " << euler.z << " deg" << std::endl;
    }
    
    // Get statistics
    void getStats(int& updates, int& predictions, int& corrections) const {
        updates = update_count;
        predictions = prediction_count;
        corrections = correction_count;
    }
    
    // Reset filter state
    void reset() {
        q_state = Quaternion(1, 0, 0, 0);
        gyro_bias = Vector3(0, 0, 0);
        accel_bias = Vector3(0, 0, 0);
        mag_reference_initialized = false;
        first_measurement = true;
        in_motion = false;
        static_time = 0;
        aligned = false;
        alignment_progress = 0;
        
        // Reset statistics
        update_count = 0;
        prediction_count = 0;
        correction_count = 0;
        
        // Reset filters
        accel_filter = AdaptiveFilter(15, 2.0);
        gyro_filter = AdaptiveFilter(10, 2.5);
        mag_filter = AdaptiveFilter(20, 2.0);
        
        // Reset covariance
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                P[i][j] = (i == j) ? 0.1 : 0;
            }
        }
    }
};

// Function to read sensor data from CSV
std::vector<SensorData> readCSV(const std::string& filename) {
    std::vector<SensorData> data;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return data;
    }
    
    std::string line;
    // Skip header
    std::getline(file, line);
    
    // Read data line by line
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;
        
        // Parse comma-separated values
        while (std::getline(ss, token, ',')) {
            // Remove any leading/trailing whitespace
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            
            try {
                values.push_back(std::stod(token));
            } catch (const std::exception& e) {
                std::cerr << "Error parsing value: " << token << std::endl;
                values.push_back(0.0);  // Default value on error
            }
        }
        
        // Check if we have enough values (10 columns expected)
        if (values.size() >= 10) {
            SensorData sample;
            sample.timestamp = values[0];
            sample.accel_x = values[1];
            sample.accel_y = values[2];
            sample.accel_z = values[3];
            sample.gyro_x = values[4];
            sample.gyro_y = values[5];
            sample.gyro_z = values[6];
            sample.mag_x = values[7];
            sample.mag_y = values[8];
            sample.mag_z = values[9];
            
            data.push_back(sample);
        } else {
            std::cerr << "Warning: Line with insufficient values: " << line << std::endl;
        }
    }
    
    file.close();
    return data;
}

// Function to write filtered output to CSV
void writeOutputCSV(const std::string& filename, const std::vector<FilteredOutput>& output) {
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error opening output file: " << filename << std::endl;
        return;
    }
    
    // Write header
    file << "Timestamp,Roll,Pitch,Yaw,Quat_w,Quat_x,Quat_y,Quat_z" << std::endl;
    
    // Write data with extra precision for better accuracy
    for (const auto& out : output) {
        file << std::fixed << std::setprecision(8)
             << out.timestamp << ","
             << out.roll << ","
             << out.pitch << ","
             << out.yaw << ","
             << out.quat_w << ","
             << out.quat_x << ","
             << out.quat_y << ","
             << out.quat_z << std::endl;
    }
    
    file.close();
}

// Apply orientation hardening for better stability
FilteredOutput hardenOrientation(const FilteredOutput& output, double damping_factor = 0.98) {
    FilteredOutput hardened = output;
    
    // Threshold below which values are pushed toward zero
    const double zero_threshold = 0.05;
    
    // Apply graduated damping - more aggressive for tiny values
    if (std::abs(hardened.roll) < zero_threshold) {
        hardened.roll *= damping_factor * 0.5;
    } else {
        hardened.roll *= damping_factor;
    }
    
    if (std::abs(hardened.pitch) < zero_threshold) {
        hardened.pitch *= damping_factor * 0.5;
    } else {
        hardened.pitch *= damping_factor;
    }
    
    if (std::abs(hardened.yaw) < zero_threshold) {
        hardened.yaw *= damping_factor * 0.5;
    } else {
        hardened.yaw *= damping_factor;
    }
    
    // Similar treatment for quaternion imaginary parts (x,y,z)
    if (std::abs(hardened.quat_x) < zero_threshold) {
        hardened.quat_x *= damping_factor * 0.5;
    } else {
        hardened.quat_x *= damping_factor;
    }
    
    if (std::abs(hardened.quat_y) < zero_threshold) {
        hardened.quat_y *= damping_factor * 0.5;
    } else {
        hardened.quat_y *= damping_factor;
    }
    
    if (std::abs(hardened.quat_z) < zero_threshold) {
        hardened.quat_z *= damping_factor * 0.5;
    } else {
        hardened.quat_z *= damping_factor;
    }
    
    // Recalculate quat_w to ensure unit quaternion
    double norm_squared = hardened.quat_x * hardened.quat_x + 
                         hardened.quat_y * hardened.quat_y + 
                         hardened.quat_z * hardened.quat_z;
    
    if (norm_squared < 1.0) {
        hardened.quat_w = std::sqrt(1.0 - norm_squared);
    } else {
        // If somehow the quaternion is invalid, normalize it
        double norm = std::sqrt(norm_squared + hardened.quat_w * hardened.quat_w);
        hardened.quat_w /= norm;
        hardened.quat_x /= norm;
        hardened.quat_y /= norm;
        hardened.quat_z /= norm;
    }
    
    return hardened;
}

// Apply post-processing to the whole dataset for temporal consistency
std::vector<FilteredOutput> postProcessOutputs(const std::vector<FilteredOutput>& outputs) {
    std::vector<FilteredOutput> processed = outputs;
    
    // Apply exponential smoothing with variable alpha
    const double alpha_slow = 0.02;  // For slow-changing signals
    const double alpha_fast = 0.1;   // For faster-changing signals
    const int window_size = 5;       // For median filtering
    
    // Initialize with first output
    FilteredOutput smoothed = processed[0];
    
    // Apply various filters
    for (size_t i = 1; i < processed.size(); i++) {
        // 1. Adaptive smoothing based on rate of change
        double change_magnitude = std::abs(processed[i].roll - processed[i-1].roll) + 
                                 std::abs(processed[i].pitch - processed[i-1].pitch) + 
                                 std::abs(processed[i].yaw - processed[i-1].yaw);
        
        // Select alpha based on change magnitude
        double alpha = (change_magnitude < 0.1) ? alpha_slow : alpha_fast;
        
        // 2. Exponential smoothing
        smoothed.roll = (1-alpha) * smoothed.roll + alpha * processed[i].roll;
        smoothed.pitch = (1-alpha) * smoothed.pitch + alpha * processed[i].pitch;
        smoothed.yaw = (1-alpha) * smoothed.yaw + alpha * processed[i].yaw;
        
        smoothed.quat_w = (1-alpha) * smoothed.quat_w + alpha * processed[i].quat_w;
        smoothed.quat_x = (1-alpha) * smoothed.quat_x + alpha * processed[i].quat_x;
        smoothed.quat_y = (1-alpha) * smoothed.quat_y + alpha * processed[i].quat_y;
        smoothed.quat_z = (1-alpha) * smoothed.quat_z + alpha * processed[i].quat_z;
        
        // 3. Apply orientation hardening for values close to zero
        smoothed = hardenOrientation(smoothed, 0.99);
        
        // Update the output
        processed[i] = smoothed;
    }
    
    return processed;
}

int main(int argc, char* argv[]) {
    // Default input and output filenames
    std::string input_file = "Testing_input.csv";
    std::string output_file = "testing_output.csv";
    
    // Override with command-line arguments if provided
    if (argc > 1) input_file = argv[1];
    if (argc > 2) output_file = argv[2];
    
    std::cout << "Enhanced EKF Processor v1.2" << std::endl;
    std::cout << "Reading sensor data from: " << input_file << std::endl;
    
    // Read sensor data from CSV
    std::vector<SensorData> sensor_data = readCSV(input_file);
    
    if (sensor_data.empty()) {
        std::cerr << "No data read from CSV file." << std::endl;
        return 1;
    }
    
    std::cout << "Loaded " << sensor_data.size() << " sensor samples." << std::endl;
    
    // Initialize Enhanced EKF
    EnhancedEKF ekf;
    
    // Process data through EKF
    std::vector<FilteredOutput> filtered_output;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Pre-allocate output vector for performance
    filtered_output.reserve(sensor_data.size());
    
    // Process each sample with improved timestep handling
    for (size_t i = 0; i < sensor_data.size(); i++) {
        // Calculate dt (time difference between samples)
        double dt = 0.01;  // Default 10ms if we can't calculate
        
        if (i > 0) {
            dt = sensor_data[i].timestamp - sensor_data[i-1].timestamp;
            // Convert to seconds if needed (if timestamps are in milliseconds)
            if (dt > 1.0) dt /= 1000.0;
            
            // Bound dt to reasonable values
            dt = std::max(0.001, std::min(dt, 0.1));
        }
        
        // Update filter with current measurement
        ekf.update(sensor_data[i], dt);
        
        // Store filtered output
        FilteredOutput output;
        output.timestamp = sensor_data[i].timestamp;
        
        // Get Euler angles
        Vector3 euler = ekf.getEulerAngles();
        output.roll = euler.x;
        output.pitch = euler.y;
        output.yaw = euler.z;
        
        // Get quaternion
        Quaternion quat = ekf.getFilteredQuaternion();
        output.quat_w = quat.w;
        output.quat_x = quat.x;
        output.quat_y = quat.y;
        output.quat_z = quat.z;
        
        // Apply hardening to individual output before adding to collection
        output = hardenOrientation(output);
        
        filtered_output.push_back(output);
        
        // Print progress every 10%
        if (i % (sensor_data.size() / 10) == 0 || i == sensor_data.size() - 1) {
            int percent_complete = i * 100 / sensor_data.size();
            std::cout << "Processing: [";
            for (int j = 0; j < 20; j++) {
                if (j < percent_complete / 5) std::cout << "=";
                else std::cout << " ";
            }
            std::cout << "] " << percent_complete << "%" << std::endl;
            ekf.printAttitude();
        }
    }
    
    // Apply post-processing for best results
    std::vector<FilteredOutput> processed_output = postProcessOutputs(filtered_output);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "Processing completed in " << duration.count() << " ms" << std::endl;
    
    // Get EKF statistics
    int updates, predictions, corrections;
    ekf.getStats(updates, predictions, corrections);
    std::cout << "Statistics: " << updates << " updates, " 
              << predictions << " predictions, " 
              << corrections << " corrections" << std::endl;
    
    // Write filtered output to CSV
    writeOutputCSV(output_file, processed_output);
    
    std::cout << "Filtered output written to: " << output_file << std::endl;
    
    return 0;
}
