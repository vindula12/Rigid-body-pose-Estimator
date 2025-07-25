#!/usr/bin/env python3
"""
Fixed High-Performance EKF Backend Server
Debugging version with proper error handling and logging
"""

import asyncio
import websockets
import json
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
import numba
from numba import jit
import threading
import time
import queue
import logging
from dataclasses import dataclass
from typing import Optional, List, Dict, Any
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, UploadFile, File
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
import io
import subprocess
import os
import tempfile
from pathlib import Path
import shutil
import signal

# Configure logging with more detail
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class EKF21Wrapper:
    """Wrapper for EKF21 executable with proper error handling"""
    
    def __init__(self, executable_path="./EKF21"):
        self.executable_path = executable_path
        self.temp_dir = tempfile.mkdtemp()
        self.input_file = os.path.join(self.temp_dir, "Input.csv")  # Exact filename EKF21 expects
        self.output_file = os.path.join(self.temp_dir, "testing_output.csv")  # Exact filename from your command
        
        # Check if executable exists and is executable
        if not os.path.exists(self.executable_path):
            logger.error(f"EKF21 executable not found: {self.executable_path}")
            raise FileNotFoundError(f"EKF21 executable not found: {self.executable_path}")
        
        if not os.access(self.executable_path, os.X_OK):
            logger.error(f"EKF21 executable is not executable: {self.executable_path}")
            raise PermissionError(f"EKF21 executable is not executable: {self.executable_path}")
        
        logger.info(f"EKF21Wrapper initialized with executable: {self.executable_path}")
        logger.info(f"Temp directory: {self.temp_dir}")
        
    def __del__(self):
        """Cleanup temporary files"""
        if hasattr(self, 'temp_dir') and os.path.exists(self.temp_dir):
            try:
                shutil.rmtree(self.temp_dir)
                logger.info(f"Cleaned up temp directory: {self.temp_dir}")
            except Exception as e:
                logger.warning(f"Failed to cleanup temp directory: {e}")
    
    def prepare_input_file(self, csv_content: str) -> int:
        """Prepare input file for EKF21 with exact format matching"""
        try:
            logger.info("Preparing input file for EKF21...")
            
            # Parse CSV
            df = pd.read_csv(io.StringIO(csv_content))
            logger.info(f"Original CSV shape: {df.shape}")
            logger.info(f"Original columns: {list(df.columns)}")
            
            # Required columns in exact order for EKF21
            required_columns = ['Timestamp', 'Accel_X', 'Accel_Y', 'Accel_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z']
            
            # Column mapping for different naming conventions
            column_mapping = {
                'timestamp': 'Timestamp', 'time': 'Timestamp', 'Time': 'Timestamp',
                'accel_x': 'Accel_X', 'AccelX': 'Accel_X', 'ax': 'Accel_X', 'Accel_x': 'Accel_X',
                'accel_y': 'Accel_Y', 'AccelY': 'Accel_Y', 'ay': 'Accel_Y', 'Accel_y': 'Accel_Y',
                'accel_z': 'Accel_Z', 'AccelZ': 'Accel_Z', 'az': 'Accel_Z', 'Accel_z': 'Accel_Z',
                'gyro_x': 'Gyro_X', 'GyroX': 'Gyro_X', 'gx': 'Gyro_X', 'Gyro_x': 'Gyro_X',
                'gyro_y': 'Gyro_Y', 'GyroY': 'Gyro_Y', 'gy': 'Gyro_Y', 'Gyro_y': 'Gyro_Y',
                'gyro_z': 'Gyro_Z', 'GyroZ': 'Gyro_Z', 'gz': 'Gyro_Z', 'Gyro_z': 'Gyro_Z',
            }
            
            # Create a copy and rename columns
            df_renamed = df.copy()
            df_renamed = df_renamed.rename(columns=column_mapping)
            
            # Check if we have all required columns
            missing_cols = [col for col in required_columns if col not in df_renamed.columns]
            
            if missing_cols:
                # Try to infer columns by position if we have at least 7 columns
                if len(df.columns) >= 7:
                    logger.warning("Missing columns, attempting to infer by position...")
                    df_output = df.iloc[:, :7].copy()
                    df_output.columns = required_columns
                else:
                    raise ValueError(f"Insufficient columns. Need {required_columns}, missing: {missing_cols}")
            else:
                # Use renamed columns in correct order
                df_output = df_renamed[required_columns].copy()
            
            # Validate data types and ranges
            for col in required_columns:
                if col != 'Timestamp':
                    df_output[col] = pd.to_numeric(df_output[col], errors='coerce')
            
            # Check for NaN values
            if df_output.isnull().any().any():
                logger.warning("Found NaN values in data, filling with zeros...")
                df_output = df_output.fillna(0)
            
            # Ensure timestamp is properly formatted
            if df_output['Timestamp'].dtype == 'object':
                try:
                    df_output['Timestamp'] = pd.to_numeric(df_output['Timestamp'])
                except:
                    # Create synthetic timestamps if parsing fails
                    logger.warning("Failed to parse timestamps, creating synthetic ones...")
                    df_output['Timestamp'] = np.arange(len(df_output)) * 0.01  # 100Hz assumption
            
            # Save input file with exact format EKF21 expects
            df_output.to_csv(self.input_file, index=False, float_format='%.6f')
            
            # Log first few rows for debugging
            logger.info(f"Prepared input file with {len(df_output)} rows")
            logger.info(f"Sample data:\n{df_output.head(3)}")
            logger.info(f"Input file saved to: {self.input_file}")
            
            # Verify file was written correctly
            if not os.path.exists(self.input_file):
                raise RuntimeError("Failed to write input file")
            
            file_size = os.path.getsize(self.input_file)
            logger.info(f"Input file size: {file_size} bytes")
            
            return len(df_output)
            
        except Exception as e:
            logger.error(f"Error preparing input file: {str(e)}")
            raise Exception(f"Error preparing input file: {str(e)}")
    
    async def run_ekf21(self, progress_callback=None):
        """Run EKF21 executable asynchronously with timeout and better error handling"""
        try:
            if progress_callback:
                await progress_callback("Starting EKF21 processing...")
            
            logger.info(f"Running EKF21: {self.executable_path}")
            logger.info(f"Input file: {self.input_file}")
            logger.info(f"Output file: {self.output_file}")
            
            # Verify input file exists
            if not os.path.exists(self.input_file):
                raise FileNotFoundError(f"Input file not found: {self.input_file}")
            
            # Copy EKF21 executable to temp directory to avoid path issues
            temp_ekf21 = os.path.join(self.temp_dir, "EKF21")
            
            # Copy the executable
            shutil.copy2(self.executable_path, temp_ekf21)
            os.chmod(temp_ekf21, 0o755)  # Make sure it's executable
            
            # Run EKF21 executable with exact format you specified
            cmd = ["./EKF21", "Input.csv", "testing_output.csv", "--no-test"]
            logger.info(f"Executing command: {' '.join(cmd)}")
            logger.info(f"Working directory: {self.temp_dir}")
            logger.info(f"EKF21 executable copied to: {temp_ekf21}")
            
            if progress_callback:
                await progress_callback("EKF21 processing in progress...")
            
            # Use asyncio subprocess with timeout and stdin support
            process = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                stdin=asyncio.subprocess.PIPE,  # Enable stdin for sending Enter
                cwd=self.temp_dir  # Run in temp directory where everything is located
            )
            
            # Send Enter keypress to handle interactive prompt
            try:
                # Send multiple newlines to handle any interactive prompts
                await process.stdin.write(b'\n\n\n')
                await process.stdin.drain()
                process.stdin.close()
            except Exception as e:
                logger.warning(f"Could not send input to EKF21: {e}")
            
            # Wait for process completion with timeout (60 seconds max for larger files)
            try:
                stdout, stderr = await asyncio.wait_for(
                    process.communicate(), 
                    timeout=60.0
                )
            except asyncio.TimeoutError:
                logger.error("EKF21 process timed out")
                process.kill()
                await process.wait()
                raise RuntimeError("EKF21 process timed out after 60 seconds")
            
            # Check return code
            if process.returncode != 0:
                error_msg = stderr.decode() if stderr else "Unknown error"
                stdout_msg = stdout.decode() if stdout else "No output"
                logger.error(f"EKF21 failed with return code {process.returncode}")
                logger.error(f"Stderr: {error_msg}")
                logger.error(f"Stdout: {stdout_msg}")
                raise RuntimeError(f"EKF21 execution failed: {error_msg}")
            
            # Log success
            if stdout:
                logger.info(f"EKF21 stdout: {stdout.decode()}")
            
            if progress_callback:
                await progress_callback("EKF21 processing completed!")
            
            logger.info("EKF21 execution completed successfully")
            
            # Check if output file was created
            if not os.path.exists(self.output_file):
                raise FileNotFoundError(f"EKF21 did not create output file: {self.output_file}")
            
            output_size = os.path.getsize(self.output_file)
            logger.info(f"Output file size: {output_size} bytes")
            
            return self.load_results()
            
        except Exception as e:
            logger.error(f"Error running EKF21: {str(e)}")
            raise Exception(f"Error running EKF21: {str(e)}")
    
    def load_results(self):
        """Load results from EKF21 output file with translation support"""
        try:
            logger.info(f"Loading results from: {self.output_file}")
            
            if not os.path.exists(self.output_file):
                raise FileNotFoundError(f"EKF21 output file not found: {self.output_file}")
            
            # Read the output file
            df = pd.read_csv(self.output_file)
            logger.info(f"Loaded output with shape: {df.shape}")
            logger.info(f"Output columns: {list(df.columns)}")
            
            # Expected columns from EKF21 (with translation)
            expected_columns = [
                'Timestamp', 'Roll', 'Pitch', 'Yaw', 
                'Quat_w', 'Quat_x', 'Quat_y', 'Quat_z',
                'Pos_X', 'Pos_Y', 'Pos_Z',  # Position columns
                'Vel_X', 'Vel_Y', 'Vel_Z'   # Velocity columns
            ]
            
            # Check for missing columns
            missing_cols = [col for col in expected_columns if col not in df.columns]
            has_translation = len(missing_cols) == 0
            
            if missing_cols:
                logger.warning(f"Missing translation columns: {missing_cols}")
                logger.info("Will use fallback translation calculation")
            
            # Convert to list of dictionaries for JSON serialization
            results = []
            for i, row in df.iterrows():
                try:
                    result = {
                        'timestamp': float(row['Timestamp']),
                        'roll': float(row['Roll']),
                        'pitch': float(row['Pitch']),
                        'yaw': float(row['Yaw']),
                        'quat_w': float(row['Quat_w']),
                        'quat_x': float(row['Quat_x']),
                        'quat_y': float(row['Quat_y']),
                        'quat_z': float(row['Quat_z'])
                    }
                    
                    # Add translation data if available
                    if has_translation:
                        result.update({
                            'pos_x': float(row['Pos_X']),
                            'pos_y': float(row['Pos_Y']),
                            'pos_z': float(row['Pos_Z']),
                            'vel_x': float(row['Vel_X']),
                            'vel_y': float(row['Vel_Y']),
                            'vel_z': float(row['Vel_Z'])
                        })
                    else:
                        # Fallback: simple numerical integration
                        result.update({
                            'pos_x': 0.0,
                            'pos_y': 0.0, 
                            'pos_z': 0.0,
                            'vel_x': 0.0,
                            'vel_y': 0.0,
                            'vel_z': 0.0
                        })
                    
                    results.append(result)
                    
                except Exception as e:
                    logger.warning(f"Error processing row {i}: {e}")
                    continue
            
            # Enhanced velocity/position calculation if not provided by EKF21
            if not has_translation:
                results = self.calculate_translation_fallback(results)
            
            logger.info(f"Successfully loaded {len(results)} results")
            
            # Log sample results
            if results:
                logger.info(f"Sample result: {results[0]}")
            
            return results
            
        except Exception as e:
            logger.error(f"Error loading EKF21 results: {str(e)}")
            raise Exception(f"Error loading EKF21 results: {str(e)}")
# End
    def calculate_translation_fallback(self, results):
            """Calculate position and velocity using numerical integration fallback"""
            logger.info("Calculating translation using numerical integration fallback")
            
            # Simple integration (this is basic - EKF21 should provide better data)
            pos_x, pos_y, pos_z = 0.0, 0.0, 0.0
            
            for i in range(1, len(results)):
                dt = results[i]['timestamp'] - results[i-1]['timestamp']
                if dt > 0 and dt < 0.1:  # Reasonable dt
                    
                    # Simple dead reckoning (very basic)
                    # In reality, this should use IMU data and orientation
                    
                    # Estimate velocity from orientation changes (placeholder)
                    vel_x = 0.0  # Would need IMU acceleration data
                    vel_y = 0.0
                    vel_z = 0.0
                    
                    # Update position
                    pos_x += vel_x * dt
                    pos_y += vel_y * dt  
                    pos_z += vel_z * dt
                    
                    # Update results
                    results[i]['pos_x'] = pos_x
                    results[i]['pos_y'] = pos_y
                    results[i]['pos_z'] = pos_z
                    results[i]['vel_x'] = vel_x
                    results[i]['vel_y'] = vel_y
                    results[i]['vel_z'] = vel_z
            
            return results


class EKFProcessor:
    """Main EKF processing class with improved error handling"""
    
    def __init__(self, ekf21_path="./EKF21"):
        self.ekf21_wrapper = EKF21Wrapper(ekf21_path)
        self.is_processing = False
        self.results = []
        self.input_data = None
        
    async def process_csv_data(self, csv_content: str, progress_callback=None):
        """Process CSV data using EKF21 with comprehensive error handling"""
        try:
            self.is_processing = True
            logger.info("Starting CSV data processing...")
            
            if progress_callback:
                await progress_callback("Validating input data...")
            
            # Store input data for visualization
            try:
                df = pd.read_csv(io.StringIO(csv_content))
                self.input_data = df.to_dict('records')
                logger.info(f"Stored {len(self.input_data)} input records")
            except Exception as e:
                logger.warning(f"Failed to store input data: {e}")
                self.input_data = []
            
            if progress_callback:
                await progress_callback("Preparing input file...")
            
            # Prepare input file
            sample_count = self.ekf21_wrapper.prepare_input_file(csv_content)
            logger.info(f"Prepared {sample_count} samples for EKF21")
            
            if progress_callback:
                await progress_callback(f"Prepared {sample_count} samples, starting EKF21...")
            
            # Run EKF21
            results = await self.ekf21_wrapper.run_ekf21(progress_callback)
            
            if not results:
                raise RuntimeError("EKF21 returned no results")
            
            logger.info(f"EKF21 processing completed with {len(results)} results")
            
            self.results = results
            self.is_processing = False
            
            if progress_callback:
                await progress_callback(f"Processing completed! Generated {len(results)} results.")
            
            return results
            
        except Exception as e:
            self.is_processing = False
            logger.error(f"Error in process_csv_data: {str(e)}")
            if progress_callback:
                await progress_callback(f"Error: {str(e)}")
            raise e
    
    def cancel_processing(self):
        """Cancel current processing"""
        self.is_processing = False
        logger.info("Processing cancelled by user")
    
    def get_results(self):
        """Get processing results"""
        return self.results
    
    def get_input_data(self):
        """Get input data for visualization"""
        return self.input_data

# FastAPI application
app = FastAPI(title="High-Performance EKF API", version="1.0.0")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Serve static files
app.mount("/static", StaticFiles(directory="static"), name="static")

# Global EKF processor
ekf_processor = EKFProcessor()

class ConnectionManager:
    """WebSocket connection manager with better error handling"""
    
    def __init__(self):
        self.active_connections: List[WebSocket] = []
    
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        logger.info(f"Client connected. Total connections: {len(self.active_connections)}")
    
    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        logger.info(f"Client disconnected. Total connections: {len(self.active_connections)}")
    
    async def send_personal_message(self, message: str, websocket: WebSocket):
        try:
            await websocket.send_text(message)
        except Exception as e:
            logger.error(f"Error sending message: {e}")
            self.disconnect(websocket)
    
    async def broadcast(self, message: str):
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except Exception as e:
                logger.error(f"Error broadcasting message: {e}")
                disconnected.append(connection)
        
        # Remove disconnected clients
        for conn in disconnected:
            self.disconnect(conn)

manager = ConnectionManager()

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            
            try:
                message = json.loads(data)
            except json.JSONDecodeError as e:
                logger.error(f"Invalid JSON received: {e}")
                await websocket.send_text(json.dumps({
                    'type': 'error',
                    'message': 'Invalid JSON format'
                }))
                continue
            
            if message['type'] == 'process_csv':
                csv_content = message['data']
                ekf21_path = message.get('ekf21_path', './EKF21')
                
                logger.info(f"Processing CSV data with EKF21 path: {ekf21_path}")
                
                # Update EKF processor path
                ekf_processor.ekf21_wrapper.executable_path = ekf21_path
                
                async def progress_callback(msg):
                    try:
                        await websocket.send_text(json.dumps({
                            'type': 'progress',
                            'message': msg
                        }))
                    except Exception as e:
                        logger.error(f"Error sending progress: {e}")
                
                try:
                    results = await ekf_processor.process_csv_data(csv_content, progress_callback)
                    
                    # Send results in chunks to avoid large messages
                    chunk_size = 100
                    total_chunks = (len(results) + chunk_size - 1) // chunk_size
                    
                    for i in range(0, len(results), chunk_size):
                        chunk = results[i:i+chunk_size]
                        await websocket.send_text(json.dumps({
                            'type': 'results_chunk',
                            'data': chunk,
                            'chunk_index': i // chunk_size,
                            'total_chunks': total_chunks
                        }))
                        
                        # Small delay to prevent overwhelming
                        await asyncio.sleep(0.01)
                    
                    # Send input data for IMU visualization
                    input_data = ekf_processor.get_input_data()
                    if input_data:
                        await websocket.send_text(json.dumps({
                            'type': 'input_data',
                            'data': input_data[:1000]  # Limit for performance
                        }))
                    
                    await websocket.send_text(json.dumps({
                        'type': 'complete',
                        'total_samples': len(results)
                    }))
                    
                    logger.info(f"Successfully processed and sent {len(results)} results")
                    
                except Exception as e:
                    logger.error(f"Processing error: {e}")
                    await websocket.send_text(json.dumps({
                        'type': 'error',
                        'message': str(e)
                    }))
            
            elif message['type'] == 'cancel':
                ekf_processor.cancel_processing()
                await websocket.send_text(json.dumps({
                    'type': 'cancelled'
                }))
                
    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        manager.disconnect(websocket)

@app.post("/upload")
async def upload_file(file: UploadFile = File(...)):
    """Upload CSV file endpoint with better validation"""
    try:
        content = await file.read()
        csv_content = content.decode('utf-8')
        
        # Quick validation
        lines = csv_content.split('\n')
        if len(lines) < 2:
            raise ValueError("CSV file must have at least header and one data row")
        
        # Validate CSV structure
        df = pd.read_csv(io.StringIO(csv_content))
        
        logger.info(f"Uploaded file: {file.filename}, Shape: {df.shape}")
        logger.info(f"Columns: {list(df.columns)}")
        
        # Check minimum requirements
        if len(df.columns) < 7:
            return JSONResponse(
                status_code=400,
                content={
                    "status": "error",
                    "message": f"Insufficient columns. Need at least 7 columns (timestamp + 6 IMU), got {len(df.columns)}"
                }
            )
        
        if len(df) < 10:
            return JSONResponse(
                status_code=400,
                content={
                    "status": "error",
                    "message": f"Insufficient data. Need at least 10 rows, got {len(df)}"
                }
            )
        
        return {
            "status": "success",
            "filename": file.filename,
            "size": len(content),
            "rows": len(df),
            "columns": list(df.columns)
        }
        
    except Exception as e:
        logger.error(f"Upload error: {e}")
        return JSONResponse(
            status_code=400,
            content={
                "status": "error",
                "message": str(e)
            }
        )

@app.get("/")
async def get_dashboard():
    """Serve the dashboard"""
    try:
        with open("static/index.html", "r") as f:
            return HTMLResponse(content=f.read())
    except FileNotFoundError:
        return HTMLResponse(content="<h1>Dashboard not found. Please ensure static/index.html exists.</h1>")

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy", 
        "timestamp": time.time(),
        "ekf21_path": ekf_processor.ekf21_wrapper.executable_path,
        "ekf21_exists": os.path.exists(ekf_processor.ekf21_wrapper.executable_path)
    }

@app.get("/api/results")
async def get_results():
    """Get current processing results"""
    return {
        "results": ekf_processor.get_results(),
        "input_data": ekf_processor.get_input_data(),
        "is_processing": ekf_processor.is_processing
    }

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="EKF Backend Server")
    parser.add_argument("--host", default="localhost", help="Host to bind to")
    parser.add_argument("--port", type=int, default=8000, help="Port to bind to")
    parser.add_argument("--ekf21-path", default="./EKF21", help="Path to EKF21 executable")
    
    args = parser.parse_args()
    
    # Update global EKF processor path
    ekf_processor.ekf21_wrapper.executable_path = args.ekf21_path
    
    # Check if EKF21 executable exists
    if not os.path.exists(args.ekf21_path):
        print(f"❌ ERROR: EKF21 executable not found at: {args.ekf21_path}")
        print("   Please compile EKF21.cpp first or provide correct path with --ekf21-path")
        exit(1)
    
    if not os.access(args.ekf21_path, os.X_OK):
        print(f"❌ ERROR: EKF21 executable is not executable: {args.ekf21_path}")
        print("   Try: chmod +x EKF21")
        exit(1)
    
    print(f"🚀 Starting EKF Backend Server on {args.host}:{args.port}")
    print(f"📁 EKF21 executable path: {args.ekf21_path}")
    print(f"📁 EKF21 executable exists: {os.path.exists(args.ekf21_path)}")
    print(f"📁 EKF21 executable is executable: {os.access(args.ekf21_path, os.X_OK)}")
    print(f"🌐 Dashboard: http://{args.host}:{args.port}")
    print(f"📡 WebSocket: ws://{args.host}:{args.port}/ws")
    print("📝 Logs will show detailed processing information")
    
    uvicorn.run(app, host=args.host, port=args.port)