#!/bin/bash

echo "🔧 EKF21 Input Test and Fix"
echo "==========================="

# Create test input
cat > Input.csv << 'EOF'
Timestamp,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z
0.00,0.1,-0.05,9.81,0.01,0.02,-0.01
0.01,0.1,-0.05,9.81,0.01,0.02,-0.01
0.02,0.1,-0.05,9.81,0.01,0.02,-0.01
0.03,0.1,-0.05,9.81,0.01,0.02,-0.01
0.04,0.1,-0.05,9.81,0.01,0.02,-0.01
EOF

echo "✅ Created Input.csv with 5 samples"

# Test 1: Run EKF21 and automatically provide Enter input
echo ""
echo "🧪 Test 1: Running EKF21 with automatic Enter input"
echo "=================================================="

echo "Sending Enter keypress automatically..."
echo "" | timeout 10s ./EKF21 Input.csv output2.csv

RETURN_CODE=$?
echo "Return code: $RETURN_CODE"

if [ -f "output2.csv" ]; then
    echo "✅ SUCCESS! output2.csv was created"
    echo "📊 File size: $(wc -c < output2.csv) bytes"
    echo "📏 Number of lines: $(wc -l < output2.csv)"
    echo ""
    echo "📋 Contents of output2.csv:"
    cat output2.csv
    SUCCESS=true
else
    echo "❌ output2.csv still not created"
    SUCCESS=false
fi

# Test 2: Run with multiple Enter inputs
if [ "$SUCCESS" = "false" ]; then
    echo ""
    echo "🧪 Test 2: Multiple Enter inputs"
    echo "==============================="
    
    printf "\n\n\n\n\n" | timeout 15s ./EKF21 Input.csv output2.csv
    
    if [ -f "output2.csv" ]; then
        echo "✅ SUCCESS with multiple Enter inputs!"
        echo "📋 Contents:"
        cat output2.csv
        SUCCESS=true
    else
        echo "❌ Still no output file"
    fi
fi

# Test 3: Check what EKF21 is actually doing
if [ "$SUCCESS" = "false" ]; then
    echo ""
    echo "🧪 Test 3: Verbose execution trace"
    echo "=================================="
    
    echo "Running with strace to see file operations..."
    echo "" | timeout 10s strace -e trace=openat,write,close ./EKF21 Input.csv output2.csv 2>&1 | grep -E "(openat|write|close|output2|csv)"
fi

# Cleanup test files
rm -f Input.csv output2.csv

echo ""
echo "🔍 Analysis and Fixes:"
echo "====================="

if [ "$SUCCESS" = "true" ]; then
    echo "✅ SOLUTION FOUND: EKF21 works but needs Enter input"
    echo ""
    echo "💡 The issue is that EKF21 waits for user input before processing."
    echo "   This is why the backend times out - it's waiting for input!"
    echo ""
    echo "🔧 Backend Fix Needed:"
    echo "   The backend needs to send an Enter keypress to EKF21"
    echo "   OR modify EKF21.cpp to remove the interactive prompt"
    echo ""
    echo "Quick backend fix: Send '\\n' to the process stdin"
else
    echo "❌ EKF21 has a more complex issue"
    echo ""
    echo "🔧 Possible issues in EKF21.cpp:"
    echo "1. Not using command line arguments correctly"
    echo "2. Not writing to the correct output file"
    echo "3. Logic error preventing file creation"
    echo "4. Exception/error occurring silently"
    echo ""
    echo "💡 Recommended fixes:"
    echo "1. Add debug output to EKF21.cpp to see what's happening"
    echo "2. Check if writeOutputCSV() function is being called"
    echo "3. Verify file paths and permissions in the code"
    echo "4. Remove or skip the interactive testing mode"
fi

echo ""
echo "🚀 For immediate backend fix:"
echo "   Modify the backend to send Enter input to EKF21 process"
echo "   OR compile EKF21 without the interactive testing mode"