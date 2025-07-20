/**
 * EKF2 CSV Processor - Main Implementation
 * 
 * This program reads sensor data from a CSV file, processes it through a
 * simplified EKF implementation based on PX4's EKF2, and outputs the filtered results.
 */

 #include <iostream>
 #include <fstream>
 #include <sstream>
 #include <vector>
 #include <string>
 #include <cmath>
 #include <chrono>
 #include <iomanip>
 
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
     
     // Get Euler angles (roll, pitch, yaw) in degrees
     Vector3 toEulerAngles() const {
         double roll, pitch, yaw;
         
         // Roll (x-axis rotation)
         double sinr_cosp = 2 * (w * x + y * z);
         double cosr_cosp = 1 - 2 * (x * x + y * y);
         roll = std::atan2(sinr_cosp, cosr_cosp);
         
         // Pitch (y-axis rotation)
         double sinp = 2 * (w * y - z * x);
         if (std::abs(sinp) >= 1)
             pitch = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
         else
             pitch = std::asin(sinp);
         
         // Yaw (z-axis rotation)
         double siny_cosp = 2 * (w * z + x * y);
         double cosy_cosp = 1 - 2 * (y * y + z * z);
         yaw = std::atan2(siny_cosp, cosy_cosp);
         
         // Convert to degrees
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
     
     // Update quaternion with angular velocity (gyro reading)
     void integrate(const Vector3& omega, double dt) {
         // Create a quaternion representing the rotation
         double magnitude = omega.norm() * dt * 0.5;
         
         if (magnitude < 1e-10) {
             return;  // No rotation
         }
         
         double s = std::sin(magnitude);
         
         Quaternion q_delta(
             std::cos(magnitude),
             omega.x / omega.norm() * s,
             omega.y / omega.norm() * s,
             omega.z / omega.norm() * s
         );
         
         // Apply the rotation
         *this = *this * q_delta;
         normalize();
     }
 };
 
 // Simplified Extended Kalman Filter implementation
 class SimpleEKF {
 private:
     // State quaternion
     Quaternion q_state;
     
     // Reference vectors
     Vector3 gravity_ref;  // Reference gravity vector in NED frame
     Vector3 mag_ref;      // Reference magnetic field vector in NED frame
     
     // Filter parameters
     double gyro_bias[3] = {0, 0, 0};
     double accel_bias[3] = {0, 0, 0};
     
     // Process and measurement noise parameters
     double process_noise = 0.01;
     double accel_meas_noise = 0.5;
     double mag_meas_noise = 0.5;
     
     // Feedback gains (simplified from full EKF)
     double accel_gain = 0.01;
     double mag_gain = 0.005;
     
     bool mag_reference_initialized = false;
 
 public:
     SimpleEKF() {
         // Initialize with identity quaternion
         q_state = Quaternion(1, 0, 0, 0);
         
         // Set reference vectors
         gravity_ref = Vector3(0, 0, 9.81);  // Gravity points down in NED
         mag_ref = Vector3(1, 0, 0);         // Initial magnetic reference (will be updated)
     }
     
     // Process one step of sensor data
     void update(const SensorData& data, double dt) {
         // 1. State prediction using gyroscope
         Vector3 gyro(data.gyro_x, data.gyro_y, data.gyro_z);
         predict(gyro, dt);
         
         // 2. Measurement update using accelerometer and magnetometer
         Vector3 accel(data.accel_x, data.accel_y, data.accel_z);
         Vector3 mag(data.mag_x, data.mag_y, data.mag_z);
         
         updateFromAccel(accel);
         updateFromMag(mag);
     }
     
     // Prediction step (gyroscope integration)
     void predict(const Vector3& gyro, double dt) {
         // Compensate for gyro bias (if estimated)
         Vector3 gyro_corrected(
             gyro.x - gyro_bias[0],
             gyro.y - gyro_bias[1],
             gyro.z - gyro_bias[2]
         );
         
         // Integrate quaternion
         q_state.integrate(gyro_corrected, dt);
     }
     
     // Update from accelerometer
     void updateFromAccel(const Vector3& accel) {
         // Skip if acceleration is zero
         if (accel.norm() < 1e-10) return;
         
         // Convert current attitude to rotation matrix
         double R[3][3];
         q_state.toRotationMatrix(R);
         
         // Normalize measured acceleration
         Vector3 accel_norm = accel.normalized() * gravity_ref.norm();
         
         // Predict gravity direction in body frame using current attitude
         Vector3 gravity_pred(
             R[0][2] * gravity_ref.z,
             R[1][2] * gravity_ref.z,
             R[2][2] * gravity_ref.z
         );
         
         // Calculate error (innovation)
         Vector3 error = accel_norm - gravity_pred;
         
         // Create correction quaternion (simplified)
         double correction_vector[3] = {
             error.x * accel_gain,
             error.y * accel_gain,
             error.z * accel_gain
         };
         
         // Apply attitude correction (small angle approximation)
         Quaternion correction(
             1.0,
             correction_vector[0] * 0.5,
             correction_vector[1] * 0.5,
             correction_vector[2] * 0.5
         );
         correction.normalize();
         
         // Apply correction
         q_state = correction * q_state;
         q_state.normalize();
     }
     
     // Update from magnetometer
     void updateFromMag(const Vector3& mag) {
         // Skip if magnetic field is zero
         if (mag.norm() < 1e-10) return;
         
         // Convert current attitude to rotation matrix
         double R[3][3];
         q_state.toRotationMatrix(R);
         
         // Normalize measured magnetic field
         Vector3 mag_norm = mag.normalized();
         
         // Initialize or update magnetic field reference
         if (!mag_reference_initialized) {
             // Transform the first measurement to the NED frame to use as reference
             Vector3 mag_ned(
                 R[0][0] * mag_norm.x + R[1][0] * mag_norm.y + R[2][0] * mag_norm.z,
                 R[0][1] * mag_norm.x + R[1][1] * mag_norm.y + R[2][1] * mag_norm.z,
                 R[0][2] * mag_norm.x + R[1][2] * mag_norm.y + R[2][2] * mag_norm.z
             );
             
             // Normalize horizontal component to use for yaw reference
             double mag_ned_h_norm = std::sqrt(mag_ned.x * mag_ned.x + mag_ned.y * mag_ned.y);
             if (mag_ned_h_norm > 1e-10) {
                 mag_ref = Vector3(mag_ned.x, mag_ned.y, 0).normalized();
                 mag_reference_initialized = true;
             }
             return;
         }
         
         // Predict magnetic field direction in body frame
         Vector3 mag_pred(
             R[0][0] * mag_ref.x + R[0][1] * mag_ref.y,
             R[1][0] * mag_ref.x + R[1][1] * mag_ref.y,
             R[2][0] * mag_ref.x + R[2][1] * mag_ref.y
         );
         mag_pred = mag_pred.normalized();
         
         // Project both vectors to horizontal plane for yaw correction
         Vector3 mag_body_h(mag_norm.x, mag_norm.y, 0);
         Vector3 mag_pred_h(mag_pred.x, mag_pred.y, 0);
         
         double mag_body_h_norm = mag_body_h.norm();
         double mag_pred_h_norm = mag_pred_h.norm();
         
         if (mag_body_h_norm < 1e-10 || mag_pred_h_norm < 1e-10) {
             return;  // Cannot determine yaw from vertical magnetic field
         }
         
         mag_body_h = mag_body_h.normalized();
         mag_pred_h = mag_pred_h.normalized();
         
         // Calculate error (innovation) - use cross product for rotation direction
         Vector3 error = mag_body_h.cross(mag_pred_h);
         
         // For yaw correction, we only use the z component
         double correction_z = error.z * mag_gain;
         
         // Create correction quaternion
         Quaternion correction(
             1.0,
             0.0,
             0.0,
             correction_z * 0.5
         );
         correction.normalize();
         
         // Apply correction
         q_state = correction * q_state;
         q_state.normalize();
     }
     
     // Get the current state quaternion
     Quaternion getQuaternion() const {
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
     
     // Write data
     for (const auto& out : output) {
         file << std::fixed << std::setprecision(6)
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
     std::string input_file = "data.csv";
     std::string output_file = "filtered_output.csv";
     
     // Override with command-line arguments if provided
     if (argc > 1) input_file = argv[1];
     if (argc > 2) output_file = argv[2];
     
     std::cout << "Reading sensor data from: " << input_file << std::endl;
     
     // Read sensor data from CSV
     std::vector<SensorData> sensor_data = readCSV(input_file);
     
     if (sensor_data.empty()) {
         std::cerr << "No data read from CSV file." << std::endl;
         return 1;
     }
     
     std::cout << "Loaded " << sensor_data.size() << " sensor samples." << std::endl;
     
     // Initialize EKF
     SimpleEKF ekf;
     
     // Process data through EKF
     std::vector<FilteredOutput> filtered_output;
     
     auto start_time = std::chrono::high_resolution_clock::now();
     
     // Process each sample
     for (size_t i = 0; i < sensor_data.size(); i++) {
         // Calculate dt (time difference between samples)
         double dt = 0.01;  // Default 10ms if we can't calculate
         
         if (i > 0) {
             dt = sensor_data[i].timestamp - sensor_data[i-1].timestamp;
             // Convert to seconds if needed (if timestamps are in milliseconds)
             if (dt > 1.0) dt /= 1000.0;
         }
         
         // Bound dt to reasonable values
         if (dt <= 0 || dt > 1.0) dt = 0.01;
         
         // Update filter with current measurement
         ekf.update(sensor_data[i], dt);
         
         // Store filtered output
         FilteredOutput output;
         output.timestamp = sensor_data[i].timestamp;
         
         Vector3 euler = ekf.getEulerAngles();
         output.roll = euler.x;
         output.pitch = euler.y;
         output.yaw = euler.z;
         
         Quaternion quat = ekf.getQuaternion();
         output.quat_w = quat.w;
         output.quat_x = quat.x;
         output.quat_y = quat.y;
         output.quat_z = quat.z;
         
         filtered_output.push_back(output);
         
         // Print progress every 10%
         if (i % (sensor_data.size() / 10) == 0 || i == sensor_data.size() - 1) {
             std::cout << "Processing: " << (i * 100 / sensor_data.size()) << "% complete" << std::endl;
             ekf.printAttitude();
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