/**
 * High-Accuracy EKF2 CSV Processor - Precision Tuned for Original Output Matching
 * 
 * This version is specifically tuned to match the original filtered output
 * with maximum accuracy by carefully analyzing the differences and adjusting
 * the algorithms accordingly.
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

// Class for high-precision vector operations
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
        if (n > 1e-15) {  // High precision normalization
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
};

// Class for high-precision quaternion operations
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
        if (n > 1e-12) {
            w /= n;
            x /= n;
            y /= n;
            z /= n;
        } else {
            w = 1;
            x = y = z = 0;
        }
    }
    
    // Get rotation matrix for high-precision calculations
    void toRotationMatrix(double R[3][3]) const {
        double w2 = w*w, x2 = x*x, y2 = y*y, z2 = z*z;
        double wx = w*x, wy = w*y, wz = w*z;
        double xy = x*y, xz = x*z, yz = y*z;
        
        R[0][0] = w2 + x2 - y2 - z2;
        R[0][1] = 2*(xy - wz);
        R[0][2] = 2*(xz + wy);
        
        R[1][0] = 2*(xy + wz);
        R[1][1] = w2 - x2 + y2 - z2;
        R[1][2] = 2*(yz - wx);
        
        R[2][0] = 2*(xz - wy);
        R[2][1] = 2*(yz + wx);
        R[2][2] = w2 - x2 - y2 + z2;
    }
    
    // Get Euler angles with high precision
    Vector3 toEulerAngles() const {
        double roll, pitch, yaw;
        
        // Roll (x-axis rotation)
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);
        
        // Pitch (y-axis rotation)
        double sinp = 2 * (w * y - z * x);
        if (std::abs(sinp) >= 1) {
            pitch = std::copysign(M_PI / 2, sinp);
        } else {
            pitch = std::asin(sinp);
        }
        
        // Yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
        
        // Convert to degrees with full precision
        return Vector3(
            roll * 180.0 / M_PI,
            pitch * 180.0 / M_PI,
            yaw * 180.0 / M_PI
        );
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
    
    // Quaternion addition for small corrections
    Quaternion operator+(const Quaternion& q) const {
        return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z);
    }
    
    // Update quaternion with angular velocity
    void integrate(const Vector3& omega, double dt) {
        // High-precision integration using Rodrigues formula
        double angle = omega.norm() * dt;
        
        // Very small angle approximation
        if (angle < 1e-10) {
            *this = *this + Quaternion(0, 
                                     omega.x * dt * 0.5,
                                     omega.y * dt * 0.5,
                                     omega.z * dt * 0.5);
            normalize();
            return;
        }
        
        // Full integration for larger angles
        double s = std::sin(angle * 0.5) / (omega.norm() + 1e-15);
        double c = std::cos(angle * 0.5);
        
        Quaternion q_delta(
            c,
            omega.x * s,
            omega.y * s,
            omega.z * s
        );
        
        *this = *this * q_delta;
        normalize();
    }
};

// High-precision median filter for outlier rejection
class MedianFilter {
private:
    std::deque<Vector3> history;
    size_t max_size;
    
public:
    MedianFilter(size_t size = 3) : max_size(size) {}
    
    Vector3 filter(const Vector3& input) {
        history.push_back(input);
        if (history.size() > max_size) {
            history.pop_front();
        }
        
        if (history.size() < 3) {
            return input;
        }
        
        // Get median for each component
        std::vector<double> x_vals, y_vals, z_vals;
        for (const auto& v : history) {
            x_vals.push_back(v.x);
            y_vals.push_back(v.y);
            z_vals.push_back(v.z);
        }
        
        std::sort(x_vals.begin(), x_vals.end());
        std::sort(y_vals.begin(), y_vals.end());
        std::sort(z_vals.begin(), z_vals.end());
        
        size_t mid = x_vals.size() / 2;
        
        return Vector3(
            x_vals[mid],
            y_vals[mid],
            z_vals[mid]
        );
    }
};

// High-accuracy EKF implementation tuned to match original output
class HighAccuracyEKF {
private:
    Quaternion q_state;
    Vector3 gyro_bias;
    
    // Precision-tuned parameters
    double process_noise_attitude;
    double process_noise_gyro_bias;
    double accel_noise;
    double mag_noise;
    
    // Adaptive trust factors
    double accel_trust;
    double mag_trust;
    double gyro_trust;
    
    // High-precision filters
    MedianFilter accel_filter;
    MedianFilter gyro_filter;
    MedianFilter mag_filter;
    
    // State tracking
    bool is_static;
    double static_time;
    int frame_count;
    bool initialized;
    
    // Reference vectors
    Vector3 gravity_ref;
    Vector3 mag_ref;
    bool mag_ref_established;
    
    // Covariance matrix for precision tracking
    double P[6][6];
    
public:
    HighAccuracyEKF() 
        : gyro_bias(0, 0, 0),
          process_noise_attitude(0.0001),
          process_noise_gyro_bias(0.00001),
          accel_noise(0.01),
          mag_noise(0.05),
          accel_trust(0.05),
          mag_trust(0.02),
          gyro_trust(0.1),
          accel_filter(5),
          gyro_filter(3),
          mag_filter(7),
          is_static(false),
          static_time(0),
          frame_count(0),
          initialized(false),
          gravity_ref(0, 0, 9.81),
          mag_ref_established(false) {
        
        // Initialize quaternion and covariance
        q_state = Quaternion(1, 0, 0, 0);
        
        // Initialize covariance matrix
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                P[i][j] = (i == j) ? 0.01 : 0;
            }
        }
    }
    
    void update(const SensorData& data, double dt) {
        frame_count++;
        
        // Apply high-precision filtering
        Vector3 gyro(data.gyro_x, data.gyro_y, data.gyro_z);
        Vector3 accel(data.accel_x, data.accel_y, data.accel_z);
        Vector3 mag(data.mag_x, data.mag_y, data.mag_z);
        
        gyro = gyro_filter.filter(gyro);
        accel = accel_filter.filter(accel);
        mag = mag_filter.filter(mag);
        
        // Static detection for parameter adaptation
        detectStatic(gyro, accel, dt);
        
        // Adaptive parameter tuning
        adaptParameters();
        
        // Initialize on first frame
        if (!initialized) {
            initializeOrientation(accel, mag);
            initialized = true;
            return;
        }
        
        // Prediction step
        predict(gyro, dt);
        
        // Correction steps
        correctWithAccelerometer(accel);
        
        if (mag.norm() > 0.1 && mag_ref_established) {
            correctWithMagnetometer(mag);
        }
        
        // High-precision stabilization
        if (is_static) {
            stabilizeQuaternion(dt);
        }
        
        // Ensure quaternion normalization
        q_state.normalize();
    }
    
    void initializeOrientation(const Vector3& accel, const Vector3& mag) {
        // High-precision initial attitude determination
        if (accel.norm() < 0.1) {
            q_state = Quaternion(1, 0, 0, 0);
            return;
        }
        
        // Normalize accelerometer to get gravity direction
        Vector3 gravity_body = accel.normalized();
        
        // Calculate roll and pitch from accelerometer
        double roll = std::atan2(-gravity_body.y, -gravity_body.z);
        double pitch = std::atan2(gravity_body.x, 
                                 std::sqrt(gravity_body.y*gravity_body.y + gravity_body.z*gravity_body.z));
        
        // Initialize quaternion with roll and pitch
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        
        q_state = Quaternion(
            cr * cp,
            sr * cp,
            cr * sp,
            -sr * sp
        );
        q_state.normalize();
        
        // Initialize magnetic reference if available
        if (mag.norm() > 0.1) {
            establishMagReference(mag);
        }
    }
    
    void establishMagReference(const Vector3& mag) {
        // Get current rotation matrix
        double R[3][3];
        q_state.toRotationMatrix(R);
        
        // Transform magnetic field to earth frame
        Vector3 mag_earth(
            R[0][0]*mag.x + R[1][0]*mag.y + R[2][0]*mag.z,
            R[0][1]*mag.x + R[1][1]*mag.y + R[2][1]*mag.z,
            R[0][2]*mag.x + R[1][2]*mag.y + R[2][2]*mag.z
        );
        
        // Use horizontal component as reference
        double mag_h_norm = std::sqrt(mag_earth.x*mag_earth.x + mag_earth.y*mag_earth.y);
        if (mag_h_norm > 0.1) {
            mag_ref = Vector3(mag_earth.x/mag_h_norm, mag_earth.y/mag_h_norm, 0);
            mag_ref_established = true;
        }
    }
    
    void detectStatic(const Vector3& gyro, const Vector3& accel, double dt) {
        // Precise static detection
        double gyro_mag = gyro.norm();
        
        // Update static timer
        if (gyro_mag < 0.02) {
            static_time += dt;
        } else {
            static_time = 0;
        }
        
        // Set static flag with hysteresis
        if (!is_static && static_time > 0.5) {
            is_static = true;
        } else if (is_static && static_time < 0.2) {
            is_static = false;
        }
    }
    
    void adaptParameters() {
        // Adjust trust factors based on state
        if (is_static) {
            accel_trust = 0.02;
            mag_trust = 0.01;
            gyro_trust = 0.01;
        } else {
            accel_trust = 0.001;
            mag_trust = 0.0005;
            gyro_trust = 0.5;
        }
        
        // Gradually increase trust after initialization
        if (frame_count < 100) {
            accel_trust *= 0.5;
            mag_trust *= 0.3;
        }
    }
    
    void predict(const Vector3& gyro, double dt) {
        // Compensate for gyro bias
        Vector3 gyro_corrected = gyro - gyro_bias;
        
        // High-precision integration
        q_state.integrate(gyro_corrected, dt);
        
        // Update covariance
        for (int i = 0; i < 3; i++) {
            P[i][i] += process_noise_attitude * dt;
            P[i+3][i+3] += process_noise_gyro_bias * dt;
        }
    }
    
    void correctWithAccelerometer(const Vector3& accel) {
        if (accel.norm() < 0.1) return;
        
        // Current rotation matrix
        double R[3][3];
        q_state.toRotationMatrix(R);
        
        // Predict gravity in body frame
        Vector3 gravity_pred(
            R[0][2] * gravity_ref.z,
            R[1][2] * gravity_ref.z,
            R[2][2] * gravity_ref.z
        );
        
        // Normalize measured acceleration
        Vector3 accel_norm = accel.normalized() * gravity_ref.norm();
        
        // Calculate error
        Vector3 error = gravity_pred.cross(accel_norm);
        
        // Apply correction with adaptive gain
        double gain = accel_trust;
        Vector3 correction = error * gain;
        
        // Create small correction quaternion
        double norm = correction.norm() * 0.5;
        if (norm > 1e-10) {
            double scale = (norm < 0.1) ? 1.0 : std::sin(norm) / norm;
            Quaternion q_correction(
                std::cos(norm),
                correction.x * scale,
                correction.y * scale,
                correction.z * scale
            );
            
            q_state = q_correction * q_state;
            q_state.normalize();
        }
        
        // Update gyro bias estimate
        if (is_static) {
            gyro_bias.x += error.x * 0.0001;
            gyro_bias.y += error.y * 0.0001;
            gyro_bias.z += error.z * 0.0001;
        }
    }
    
    void correctWithMagnetometer(const Vector3& mag) {
        if (!mag_ref_established || mag.norm() < 0.1) return;
        
        // Current rotation matrix
        double R[3][3];
        q_state.toRotationMatrix(R);
        
        // Project measured magnetic field to earth frame
        Vector3 mag_earth(
            R[0][0]*mag.x + R[1][0]*mag.y + R[2][0]*mag.z,
            R[0][1]*mag.x + R[1][1]*mag.y + R[2][1]*mag.z,
            R[0][2]*mag.x + R[1][2]*mag.y + R[2][2]*mag.z
        );
        
        // Get horizontal component
        double mag_h_norm = std::sqrt(mag_earth.x*mag_earth.x + mag_earth.y*mag_earth.y);
        if (mag_h_norm < 0.1) return;
        
        Vector3 mag_h_earth = Vector3(
            mag_earth.x / mag_h_norm,
            mag_earth.y / mag_h_norm,
            0
        );
        
        // Calculate yaw error
        Vector3 error = mag_ref.cross(mag_h_earth);
        double yaw_error = error.z * mag_trust;
        
        // Apply yaw correction
        if (std::abs(yaw_error) > 1e-10) {
            Quaternion q_yaw(
                std::cos(yaw_error * 0.5),
                0,
                0,
                std::sin(yaw_error * 0.5)
            );
            
            q_state = q_yaw * q_state;
            q_state.normalize();
        }
    }
    
    void stabilizeQuaternion(double dt) {
        if (static_time < 1.0) return;
        
        // Get current Euler angles
        Vector3 euler = q_state.toEulerAngles();
        
        // Stabilize roll and pitch towards zero
        double stabilization_rate = 0.001 * std::min(static_time, 10.0);
        
        // Create stabilization rotation
        Quaternion q_stabilize(
            1,
            -euler.x * M_PI / 180.0 * stabilization_rate * 0.5,
            -euler.y * M_PI / 180.0 * stabilization_rate * 0.5,
            0
        );
        q_stabilize.normalize();
        
        // Apply stabilization
        q_state = q_stabilize * q_state;
        q_state.normalize();
    }
    
    Quaternion getQuaternion() const {
        return q_state;
    }
    
    Vector3 getEulerAngles() const {
        return q_state.toEulerAngles();
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
            try {
                values.push_back(std::stod(token));
            } catch (const std::exception& e) {
                std::cerr << "Error parsing value: " << token << std::endl;
                values.push_back(0.0);
            }
        }
        
        // Check if we have enough values
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
    
    // Write data with maximum precision
    for (const auto& out : output) {
        file << std::fixed << std::setprecision(12)
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

int main(int argc, char* argv[]) {
    // Default input and output filenames
    std::string input_file = "Testing_input.csv";
    std::string output_file = "testing_output.csv";
    
    // Override with command-line arguments if provided
    if (argc > 1) input_file = argv[1];
    if (argc > 2) output_file = argv[2];
    
    std::cout << "High-Accuracy EKF Processor" << std::endl;
    std::cout << "Reading sensor data from: " << input_file << std::endl;
    
    // Read sensor data from CSV
    std::vector<SensorData> sensor_data = readCSV(input_file);
    
    if (sensor_data.empty()) {
        std::cerr << "No data read from CSV file." << std::endl;
        return 1;
    }
    
    std::cout << "Loaded " << sensor_data.size() << " sensor samples." << std::endl;
    
    // Initialize High-Accuracy EKF
    HighAccuracyEKF ekf;
    
    // Process data through EKF
    std::vector<FilteredOutput> filtered_output;
    filtered_output.reserve(sensor_data.size());
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Process each sample with high precision
    for (size_t i = 0; i < sensor_data.size(); i++) {
        // Calculate precise dt
        double dt = 0.01;  // Default
        if (i > 0) {
            dt = sensor_data[i].timestamp - sensor_data[i-1].timestamp;
            if (dt > 1.0) dt /= 1000.0;  // Convert ms to s if needed
            dt = std::max(0.0001, std::min(dt, 0.1));
        }
        
        // Update filter
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
        Quaternion quat = ekf.getQuaternion();
        output.quat_w = quat.w;
        output.quat_x = quat.x;
        output.quat_y = quat.y;
        output.quat_z = quat.z;
        
        filtered_output.push_back(output);
        
        // Print progress
        if (i % (sensor_data.size() / 10) == 0 || i == sensor_data.size() - 1) {
            int percent_complete = i * 100 / sensor_data.size();
            std::cout << "Processing: " << percent_complete << "%" << std::endl;
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "Processing completed in " << duration.count() << " ms" << std::endl;
    
    // Write filtered output to CSV
    writeOutputCSV(output_file, filtered_output);
    
    std::cout << "Filtered output written to: " << output_file << std::endl;
    
    return 0;
}
