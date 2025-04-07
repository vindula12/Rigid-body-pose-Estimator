/**
 * EKF2 CSV Processing Application
 * Modified from PX4 EKF2 implementation to work with CSV data
 */

 #include <iostream>
 #include <fstream>
 #include <sstream>
 #include <vector>
 #include <string>
 #include <cmath>
 #include <Eigen/Dense>
 #include <chrono>
 
 using namespace std;
 using namespace Eigen;
 
 // Simplified structure to match CSV data format
 struct SensorData {
     double timestamp;
     Vector3d accel;
     Vector3d gyro;
     Vector3d mag;
 };
 
 // Simplified EKF implementation for attitude estimation
 class SimpleEKF {
 public:
     SimpleEKF() {
         // Initialize state (quaternion [w,x,y,z])
         x_ = Vector4d(1, 0, 0, 0);
         
         // Initialize covariance
         P_ = Matrix4d::Identity() * 0.1;
         
         // Process noise
         gyro_noise_ = 0.01;
         accel_noise_ = 0.1;
         mag_noise_ = 0.1;
         
         // Measurement noise
         R_accel_ = Matrix3d::Identity() * 0.5;
         R_mag_ = Matrix3d::Identity() * 0.5;
         
         // Earth's magnetic field reference (will be calibrated)
         mag_earth_ = Vector3d(1, 0, 0);
         
         // Gravity reference
         gravity_ = Vector3d(0, 0, 9.81);
     }
     
     // Process one timestep
     void update(const SensorData& data, double dt) {
         // 1. State prediction using gyroscope
         Vector3d gyro = data.gyro;
         predict(gyro, dt);
         
         // 2. Correction using accelerometer and magnetometer
         updateAccel(data.accel);
         updateMag(data.mag);
     }
     
     // Get the current attitude as Euler angles (roll, pitch, yaw) in degrees
     Vector3d getEulerAngles() const {
         double w = x_(0);
         double x = x_(1);
         double y = x_(2);
         double z = x_(3);
         
         // Roll (x-axis rotation)
         double sinr_cosp = 2 * (w * x + y * z);
         double cosr_cosp = 1 - 2 * (x * x + y * y);
         double roll = atan2(sinr_cosp, cosr_cosp);
         
         // Pitch (y-axis rotation)
         double sinp = 2 * (w * y - z * x);
         double pitch;
         if (abs(sinp) >= 1)
             pitch = copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
         else
             pitch = asin(sinp);
         
         // Yaw (z-axis rotation)
         double siny_cosp = 2 * (w * z + x * y);
         double cosy_cosp = 1 - 2 * (y * y + z * z);
         double yaw = atan2(siny_cosp, cosy_cosp);
         
         return Vector3d(roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
     }
     
     // Get the current quaternion
     Vector4d getQuaternion() const {
         return x_;
     }
     
     // Print the current state
     void printState() const {
         Vector3d euler = getEulerAngles();
         cout << "Roll: " << euler(0) << " deg, ";
         cout << "Pitch: " << euler(1) << " deg, ";
         cout << "Yaw: " << euler(2) << " deg" << endl;
     }
 
 private:
     // State vector (quaternion)
     Vector4d x_;
     
     // State covariance matrix
     Matrix4d P_;
     
     // Process noise parameters
     double gyro_noise_;
     double accel_noise_;
     double mag_noise_;
     
     // Measurement noise matrices
     Matrix3d R_accel_;
     Matrix3d R_mag_;
     
     // Earth's magnetic field reference in NED
     Vector3d mag_earth_;
     
     // Gravity reference in NED
     Vector3d gravity_;
     
     // State prediction step
     void predict(const Vector3d& gyro, double dt) {
         // Convert gyro measurements to quaternion rate
         Vector4d q = x_;
         
         // Simplified quaternion integration from angular velocity
         double wx = gyro(0);
         double wy = gyro(1);
         double wz = gyro(2);
         
         Matrix4d Omega;
         Omega << 0, -wx, -wy, -wz,
                  wx, 0, wz, -wy,
                  wy, -wz, 0, wx,
                  wz, wy, -wx, 0;
         
         // Integrate quaternion
         x_ = x_ + 0.5 * dt * Omega * x_;
         
         // Normalize quaternion
         x_.normalize();
         
         // Update covariance
         // (Simplified - would normally include proper Jacobians)
         Matrix4d Q = Matrix4d::Identity() * gyro_noise_ * dt;
         P_ = P_ + Q;
     }
     
     // Quaternion to rotation matrix
     Matrix3d quaternionToRotationMatrix(const Vector4d& q) const {
         double w = q(0);
         double x = q(1);
         double y = q(2);
         double z = q(3);
         
         Matrix3d R;
         
         R << 1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
              2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
              2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y);
              
         return R;
     }
     
     // Update using accelerometer measurements
     void updateAccel(const Vector3d& accel) {
         // Current rotation matrix from NED to body
         Matrix3d R = quaternionToRotationMatrix(x_);
         
         // Predicted acceleration in body frame (gravity rotated)
         Vector3d accel_pred = R.transpose() * gravity_;
         
         // Normalized measured acceleration
         Vector3d accel_norm = accel.normalized() * gravity_.norm();
         
         // Innovation (difference between measurement and prediction)
         Vector3d y = accel_norm - accel_pred;
         
         // Measurement Jacobian
         Matrix<double, 3, 4> H = Matrix<double, 3, 4>::Zero();
         // (In a full implementation, would compute proper Jacobian here)
         
         // Simplified approach: use a simple gain for correction
         double gain = 0.01;
         
         // Compute correction (simplified)
         Vector3d correction = gain * y;
         
         // Convert to quaternion correction (simplified)
         Vector4d q_correction;
         q_correction << 0, correction(0), correction(1), correction(2);
         
         // Apply correction to state
         x_ = x_ + q_correction;
         x_.normalize();
     }
     
     // Update using magnetometer measurements
     void updateMag(const Vector3d& mag) {
         // Simplified implementation for now
         // In a full EKF, would properly account for magnetic distortions
         
         // For simplicity, only use magnetometer for yaw correction
         // (would normally integrate with accelerometer for complete attitude)
         
         // Current rotation matrix from NED to body
         Matrix3d R = quaternionToRotationMatrix(x_);
         
         // Use mag readings to slowly update mag_earth_ reference
         if (mag_earth_.norm() < 0.1) {
             // Initialize on first update
             mag_earth_ = R * mag.normalized();
         } else {
             // Slowly update reference direction
             mag_earth_ = 0.999 * mag_earth_ + 0.001 * (R * mag.normalized());
             mag_earth_.normalize();
         }
         
         // Simplified correction approach for yaw
         double gain = 0.005;
         
         // Projected magnetic field in horizontal plane
         Vector3d mag_h = mag;
         mag_h(2) = 0;
         mag_h.normalize();
         
         // Predicted direction based on current attitude
         Vector3d mag_pred = R.transpose() * mag_earth_;
         mag_pred(2) = 0;
         mag_pred.normalize();
         
         // Compute correction (cross product for rotation error)
         Vector3d correction = gain * mag_h.cross(mag_pred);
         
         // Apply only yaw correction
         Vector4d q_correction;
         q_correction << 0, 0, 0, correction(2);
         
         // Apply correction to state
         x_ = x_ + q_correction;
         x_.normalize();
     }
 };
 
 // Simplified structure for filtered output
 struct FilteredOutput {
     double timestamp;
     Vector3d euler_angles;  // roll, pitch, yaw in degrees
     Vector4d quaternion;    // w, x, y, z
 };
 
 // Function to read sensor data from CSV
 vector<SensorData> readCSV(const string& filename) {
     vector<SensorData> data;
     ifstream file(filename);
     
     if (!file.is_open()) {
         cerr << "Error opening file: " << filename << endl;
         return data;
     }
     
     string line;
     // Skip header
     getline(file, line);
     
     // Read data line by line
     while (getline(file, line)) {
         stringstream ss(line);
         string token;
         vector<double> values;
         
         // Parse comma-separated values
         while (getline(ss, token, ',')) {
             values.push_back(stod(token));
         }
         
         // Check if we have enough values (10 columns)
         if (values.size() >= 10) {
             SensorData sample;
             sample.timestamp = values[0];
             sample.accel = Vector3d(values[1], values[2], values[3]);
             sample.gyro = Vector3d(values[4], values[5], values[6]);
             sample.mag = Vector3d(values[7], values[8], values[9]);
             
             data.push_back(sample);
         }
     }
     
     file.close();
     return data;
 }
 
 // Function to write filtered output to CSV
 void writeOutputCSV(const string& filename, const vector<FilteredOutput>& output) {
     ofstream file(filename);
     
     if (!file.is_open()) {
         cerr << "Error opening output file: " << filename << endl;
         return;
     }
     
     // Write header
     file << "Timestamp,Roll,Pitch,Yaw,Quat_w,Quat_x,Quat_y,Quat_z" << endl;
     
     // Write data
     for (const auto& out : output) {
         file << out.timestamp << ","
              << out.euler_angles(0) << ","
              << out.euler_angles(1) << ","
              << out.euler_angles(2) << ","
              << out.quaternion(0) << ","
              << out.quaternion(1) << ","
              << out.quaternion(2) << ","
              << out.quaternion(3) << endl;
     }
     
     file.close();
 }
 
 int main(int argc, char* argv[]) {
     // Default input and output filenames
     string input_file = "data.csv";
     string output_file = "filtered_output.csv";
     
     // Override with command-line arguments if provided
     if (argc > 1) input_file = argv[1];
     if (argc > 2) output_file = argv[2];
     
     cout << "Reading sensor data from: " << input_file << endl;
     
     // Read sensor data from CSV
     vector<SensorData> sensor_data = readCSV(input_file);
     
     if (sensor_data.empty()) {
         cerr << "No data read from CSV file." << endl;
         return 1;
     }
     
     cout << "Loaded " << sensor_data.size() << " sensor samples." << endl;
     
     // Initialize EKF
     SimpleEKF ekf;
     
     // Process data through EKF
     vector<FilteredOutput> filtered_output;
     
     auto start_time = chrono::high_resolution_clock::now();
     
     // Process each sample
     for (size_t i = 0; i < sensor_data.size(); i++) {
         // Calculate dt (time difference between samples)
         double dt = 0.01;  // Default 10ms if we can't calculate
         
         if (i > 0) {
             dt = (sensor_data[i].timestamp - sensor_data[i-1].timestamp);
             // Convert to seconds if needed (if timestamps are in milliseconds)
             if (dt > 1) dt /= 1000.0;
         }
         
         // Bound dt to reasonable values
         if (dt <= 0 || dt > 1.0) dt = 0.01;
         
         // Update filter with current measurement
         ekf.update(sensor_data[i], dt);
         
         // Store filtered output
         FilteredOutput output;
         output.timestamp = sensor_data[i].timestamp;
         output.euler_angles = ekf.getEulerAngles();
         output.quaternion = ekf.getQuaternion();
         
         filtered_output.push_back(output);
         
         // Print progress every 10%
         if (i % (sensor_data.size() / 10) == 0) {
             cout << "Processing: " << (i * 100 / sensor_data.size()) << "% complete" << endl;
             ekf.printState();
         }
     }
     
     auto end_time = chrono::high_resolution_clock::now();
     auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);
     
     cout << "Processing completed in " << duration.count() << " ms" << endl;
     
     // Write filtered output to CSV
     writeOutputCSV(output_file, filtered_output);
     
     cout << "Filtered output written to: " << output_file << endl;
     
     return 0;
 }