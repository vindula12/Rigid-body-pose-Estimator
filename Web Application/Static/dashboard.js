/**
 * Main Dashboard JavaScript
 * Handles WebSocket communication, UI updates, and data visualization
 */

class EKFDashboard {
    constructor() {
        this.ws = null;
        this.isConnected = false;
        this.isProcessing = false;
        this.results = [];
        this.inputData = [];
        this.threeJSViz = null;
        this.processingStartTime = null;
        
        this.init();
    }
    
    init() {
        this.setupEventListeners();
        this.initializePlots();
        this.initializeThreeJS();
        this.connectWebSocket();
    }
    
    setupEventListeners() {
        // File input
        document.getElementById('fileInput').addEventListener('change', (e) => {
            this.handleFileSelect(e.target.files[0]);
        });
        
        // Control buttons
        document.getElementById('processBtn').addEventListener('click', () => this.processData());
        document.getElementById('cancelBtn').addEventListener('click', () => this.cancelProcessing());
        document.getElementById('clearBtn').addEventListener('click', () => this.clearResults());
        
        // Window resize
        window.addEventListener('resize', () => this.handleResize());
        
        // Keyboard shortcuts
        document.addEventListener('keydown', (e) => this.handleKeyboard(e));
    }
    
    connectWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/ws`;
        
        this.updateConnectionStatus('connecting');
        
        try {
            this.ws = new WebSocket(wsUrl);
            
            this.ws.onopen = () => {
                this.isConnected = true;
                this.updateConnectionStatus('connected');
                this.showStatus('Connected to EKF processor', 'success');
                console.log('WebSocket connected');
            };
            
            this.ws.onmessage = (event) => {
                try {
                    const message = JSON.parse(event.data);
                    this.handleWebSocketMessage(message);
                } catch (error) {
                    console.error('Error parsing WebSocket message:', error);
                }
            };
            
            this.ws.onclose = () => {
                this.isConnected = false;
                this.updateConnectionStatus('disconnected');
                this.showStatus('Disconnected from server', 'error');
                console.log('WebSocket disconnected');
                
                // Attempt to reconnect after 5 seconds
                setTimeout(() => {
                    if (!this.isConnected) {
                        this.connectWebSocket();
                    }
                }, 5000);
            };
            
            this.ws.onerror = (error) => {
                console.error('WebSocket error:', error);
                this.showStatus('Connection error', 'error');
                this.updateConnectionStatus('disconnected');
            };
            
        } catch (error) {
            console.error('Failed to create WebSocket:', error);
            this.showStatus('Failed to connect to server', 'error');
            this.updateConnectionStatus('disconnected');
        }
    }
    
    handleWebSocketMessage(message) {
        switch (message.type) {
            case 'progress':
                this.updateProgress(message.message);
                break;
                
            case 'results_chunk':
                this.handleResultsChunk(message.data, message.chunk_index, message.total_chunks);
                break;
                
            case 'input_data':
                this.inputData = message.data;
                this.updateIMUPlot();
                break;
                
            case 'complete':
                this.handleProcessingComplete(message.total_samples);
                break;
                
            case 'error':
                this.handleError(message.message);
                break;
                
            case 'cancelled':
                this.handleCancelled();
                break;
                
            default:
                console.warn('Unknown message type:', message.type);
        }
    }
    
    handleResultsChunk(data, chunkIndex, totalChunks) {
        // Add data to results
        this.results.push(...data);
        
        // Update progress
        const progress = ((chunkIndex + 1) / totalChunks) * 100;
        this.updateProgressBar(progress);
        
        // Update visualizations in real-time
        this.updatePlots();
        this.updateStatistics();
        
        // Update 3D visualization
        if (this.threeJSViz) {
            this.threeJSViz.setData(this.results);
        }
        
        // Show processing progress
        this.showStatus(`Processing chunk ${chunkIndex + 1}/${totalChunks}...`, 'info');
    }
    
    handleProcessingComplete(totalSamples) {
        this.isProcessing = false;
        this.updateButtons();
        this.hideLoadingOverlay();
        
        const processingTime = (Date.now() - this.processingStartTime) / 1000;
        const processingRate = totalSamples / processingTime;
        
        this.showStatus(
            `✅ Processing complete! ${totalSamples} samples processed in ${processingTime.toFixed(2)}s (${processingRate.toFixed(1)} Hz)`,
            'success'
        );
        
        // Update processing rate display
        document.getElementById('processingHz').textContent = `${processingRate.toFixed(1)} Hz`;
        
        // Final update of all visualizations
        this.updatePlots();
        this.updateStatistics();
        
        if (this.threeJSViz) {
            this.threeJSViz.setData(this.results);
        }
        
        console.log('Processing completed:', {
            totalSamples,
            processingTime,
            processingRate,
            resultsLength: this.results.length
        });
    }
    
    handleError(message) {
        this.isProcessing = false;
        this.updateButtons();
        this.hideLoadingOverlay();
        this.showStatus(`❌ Error: ${message}`, 'error');
        console.error('Processing error:', message);
    }
    
    handleCancelled() {
        this.isProcessing = false;
        this.updateButtons();
        this.hideLoadingOverlay();
        this.showStatus('⏹️ Processing cancelled', 'info');
        console.log('Processing cancelled');
    }
    
    handleFileSelect(file) {
        if (!file) return;
        
        const fileLabel = document.querySelector('.file-text');
        if (fileLabel) {
            fileLabel.textContent = file.name;
        }
        
        // Validate file type
        if (!file.name.toLowerCase().endsWith('.csv')) {
            this.showStatus('Please select a CSV file', 'error');
            return;
        }
        
        // Check file size (limit to 10MB)
        if (file.size > 10 * 1024 * 1024) {
            this.showStatus('File too large. Please select a file smaller than 10MB', 'error');
            return;
        }
        
        this.showStatus(`File selected: ${file.name} (${(file.size / 1024).toFixed(1)} KB)`, 'info');
    }
    
    async processData() {
        if (!this.isConnected) {
            this.showStatus('Not connected to server', 'error');
            return;
        }
        
        const fileInput = document.getElementById('fileInput');
        const file = fileInput.files[0];
        
        if (!file) {
            this.showStatus('Please select a CSV file', 'error');
            return;
        }
        
        try {
            // Read file content
            const csvContent = await this.readFile(file);
            
            // Get EKF21 path
            const ekfPath = document.getElementById('ekfPath').value || './EKF21';
            
            // Start processing
            this.isProcessing = true;
            this.processingStartTime = Date.now();
            this.updateButtons();
            this.showLoadingOverlay();
            this.clearResults();
            
            // Send data to server
            this.ws.send(JSON.stringify({
                type: 'process_csv',
                data: csvContent,
                ekf21_path: ekfPath
            }));
            
            this.showStatus('Sending data to EKF processor...', 'info');
            
        } catch (error) {
            this.isProcessing = false;
            this.updateButtons();
            this.hideLoadingOverlay();
            this.showStatus(`Error reading file: ${error.message}`, 'error');
            console.error('File reading error:', error);
        }
    }
    
    cancelProcessing() {
        if (this.ws && this.isProcessing) {
            this.ws.send(JSON.stringify({ type: 'cancel' }));
        }
    }
    
    clearResults() {
        this.results = [];
        this.inputData = [];
        
        // Clear plots
        this.initializePlots();
        
        // Clear 3D visualization
        if (this.threeJSViz) {
            this.threeJSViz.setData([]);
        }
        
        // Reset statistics
        this.resetStatistics();
        
        // Reset progress
        this.updateProgressBar(0);
        this.showStatus('Results cleared', 'info');
    }
    
    readFile(file) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.onload = (e) => resolve(e.target.result);
            reader.onerror = (e) => reject(new Error('Failed to read file'));
            reader.readAsText(file);
        });
    }
    
    updateConnectionStatus(status) {
        const statusElement = document.getElementById('connectionStatus');
        if (!statusElement) return;
        
        statusElement.className = `status-${status}`;
        
        switch (status) {
            case 'connected':
                statusElement.textContent = '⚡ Connected';
                break;
            case 'connecting':
                statusElement.textContent = '🔄 Connecting...';
                break;
            case 'disconnected':
                statusElement.textContent = '❌ Disconnected';
                break;
        }
    }
    
    updateButtons() {
        const processBtn = document.getElementById('processBtn');
        const cancelBtn = document.getElementById('cancelBtn');
        
        if (processBtn) {
            processBtn.disabled = this.isProcessing || !this.isConnected;
        }
        
        if (cancelBtn) {
            cancelBtn.disabled = !this.isProcessing;
        }
    }
    
    updateProgress(message) {
        this.showStatus(message, 'info');
        
        // Extract progress percentage if available
        const progressMatch = message.match(/(\d+\.?\d*)%/);
        if (progressMatch) {
            const progress = parseFloat(progressMatch[1]);
            this.updateProgressBar(progress);
        }
    }
    
    updateProgressBar(progress) {
        const progressFill = document.getElementById('progressFill');
        if (progressFill) {
            progressFill.style.width = `${Math.min(100, Math.max(0, progress))}%`;
        }
    }
    
    showStatus(message, type) {
        const statusElement = document.getElementById('statusMessage');
        if (!statusElement) return;
        
        statusElement.textContent = message;
        statusElement.className = `status-message ${type}`;
        
        // Auto-hide success messages after 5 seconds
        if (type === 'success') {
            setTimeout(() => {
                if (statusElement.textContent === message) {
                    statusElement.textContent = 'Ready to process IMU data...';
                    statusElement.className = 'status-message info';
                }
            }, 5000);
        }
    }
    
    showLoadingOverlay() {
        const overlay = document.getElementById('loadingOverlay');
        if (overlay) {
            overlay.classList.add('show');
        }
    }
    
    hideLoadingOverlay() {
        const overlay = document.getElementById('loadingOverlay');
        if (overlay) {
            overlay.classList.remove('show');
        }
    }
    
    initializePlots() {
        // Initialize Euler angles plot
        const eulerLayout = {
            title: 'Euler Angles (Real-time)',
            xaxis: { title: 'Time (s)' },
            yaxis: { title: 'Angle (degrees)' },
            showlegend: true,
            plot_bgcolor: 'rgba(240,240,240,0.1)',
            paper_bgcolor: 'rgba(0,0,0,0)',
            font: { color: '#2c3e50' },
            margin: { l: 50, r: 50, t: 50, b: 50 }
        };
        
        Plotly.newPlot('eulerPlot', [], eulerLayout, { responsive: true });
        
        // Initialize quaternion plot
        const quaternionLayout = {
            title: 'Quaternion Components (Real-time)',
            xaxis: { title: 'Time (s)' },
            yaxis: { title: 'Value' },
            showlegend: true,
            plot_bgcolor: 'rgba(240,240,240,0.1)',
            paper_bgcolor: 'rgba(0,0,0,0)',
            font: { color: '#2c3e50' },
            margin: { l: 50, r: 50, t: 50, b: 50 }
        };
        
        Plotly.newPlot('quaternionPlot', [], quaternionLayout, { responsive: true });
        
        // Initialize IMU plot
        const imuLayout = {
            title: 'IMU Data (Accelerometer & Gyroscope)',
            xaxis: { title: 'Time (s)' },
            yaxis: { title: 'Acceleration (m/s²)' },
            yaxis2: { 
                title: 'Angular Velocity (rad/s)',
                overlaying: 'y',
                side: 'right'
            },
            showlegend: true,
            plot_bgcolor: 'rgba(240,240,240,0.1)',
            paper_bgcolor: 'rgba(0,0,0,0)',
            font: { color: '#2c3e50' },
            margin: { l: 50, r: 50, t: 50, b: 50 }
        };
        
        Plotly.newPlot('imuPlot', [], imuLayout, { responsive: true });
    }
    
    initializeThreeJS() {
        try {
            this.threeJSViz = new ThreeJSVisualization('threejsContainer');
            console.log('Three.js visualization initialized');
        } catch (error) {
            console.error('Error initializing Three.js:', error);
            this.showStatus('3D visualization initialization failed', 'error');
        }
    }
    
    updatePlots() {
        if (this.results.length === 0) return;
        
        this.updateEulerPlot();
        this.updateQuaternionPlot();
    }
    
    updateEulerPlot() {
        const times = this.results.map(r => r.timestamp);
        const rolls = this.results.map(r => r.roll);
        const pitches = this.results.map(r => r.pitch);
        const yaws = this.results.map(r => r.yaw);
        
        const eulerData = [
            {
                x: times,
                y: rolls,
                name: 'Roll',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#e74c3c', width: 2 }
            },
            {
                x: times,
                y: pitches,
                name: 'Pitch',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#2ecc71', width: 2 }
            },
            {
                x: times,
                y: yaws,
                name: 'Yaw',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#3498db', width: 2 }
            }
        ];
        
        Plotly.react('eulerPlot', eulerData);
    }
    
    updateQuaternionPlot() {
        const times = this.results.map(r => r.timestamp);
        const quat_w = this.results.map(r => r.quat_w);
        const quat_x = this.results.map(r => r.quat_x);
        const quat_y = this.results.map(r => r.quat_y);
        const quat_z = this.results.map(r => r.quat_z);
        
        const quaternionData = [
            {
                x: times,
                y: quat_w,
                name: 'W (scalar)',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#9b59b6', width: 2 }
            },
            {
                x: times,
                y: quat_x,
                name: 'X',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#e74c3c', width: 2 }
            },
            {
                x: times,
                y: quat_y,
                name: 'Y',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#2ecc71', width: 2 }
            },
            {
                x: times,
                y: quat_z,
                name: 'Z',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#3498db', width: 2 }
            }
        ];
        
        Plotly.react('quaternionPlot', quaternionData);
    }
    
    updateIMUPlot() {
        if (this.inputData.length === 0) return;
        
        const times = this.inputData.map(d => d.Timestamp);
        const accel_x = this.inputData.map(d => d.Accel_X);
        const accel_y = this.inputData.map(d => d.Accel_Y);
        const accel_z = this.inputData.map(d => d.Accel_Z);
        const gyro_x = this.inputData.map(d => d.Gyro_X);
        const gyro_y = this.inputData.map(d => d.Gyro_Y);
        const gyro_z = this.inputData.map(d => d.Gyro_Z);
        
        const imuData = [
            // Accelerometer data
            {
                x: times,
                y: accel_x,
                name: 'Accel X',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#e74c3c', width: 1 },
                yaxis: 'y'
            },
            {
                x: times,
                y: accel_y,
                name: 'Accel Y',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#2ecc71', width: 1 },
                yaxis: 'y'
            },
            {
                x: times,
                y: accel_z,
                name: 'Accel Z',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#3498db', width: 1 },
                yaxis: 'y'
            },
            // Gyroscope data
            {
                x: times,
                y: gyro_x,
                name: 'Gyro X',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#e74c3c', width: 1, dash: 'dash' },
                yaxis: 'y2'
            },
            {
                x: times,
                y: gyro_y,
                name: 'Gyro Y',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#2ecc71', width: 1, dash: 'dash' },
                yaxis: 'y2'
            },
            {
                x: times,
                y: gyro_z,
                name: 'Gyro Z',
                type: 'scatter',
                mode: 'lines',
                line: { color: '#3498db', width: 1, dash: 'dash' },
                yaxis: 'y2'
            }
        ];
        
        Plotly.react('imuPlot', imuData);
    }
    
    updateStatistics() {
        if (this.results.length === 0) return;
        
        // Calculate statistics
        const rolls = this.results.map(r => r.roll);
        const pitches = this.results.map(r => r.pitch);
        const yaws = this.results.map(r => r.yaw);
        
        const maxRoll = Math.max(...rolls.map(Math.abs));
        const maxPitch = Math.max(...pitches.map(Math.abs));
        const maxYaw = Math.max(...yaws.map(Math.abs));
        
        // Calculate quaternion norm
        const quatNorms = this.results.map(r => 
            Math.sqrt(r.quat_w*r.quat_w + r.quat_x*r.quat_x + r.quat_y*r.quat_y + r.quat_z*r.quat_z)
        );
        const avgQuatNorm = quatNorms.reduce((a, b) => a + b, 0) / quatNorms.length;
        
        // Update displays
        this.updateStatElement('samplesProcessed', this.results.length);
        this.updateStatElement('maxRoll', `${maxRoll.toFixed(1)}°`);
        this.updateStatElement('maxPitch', `${maxPitch.toFixed(1)}°`);
        this.updateStatElement('maxYaw', `${maxYaw.toFixed(1)}°`);
        this.updateStatElement('quatNorm', avgQuatNorm.toFixed(3));
    }
    
    updateStatElement(id, value) {
        const element = document.getElementById(id);
        if (element) {
            element.textContent = value;
        }
    }
    
    resetStatistics() {
        this.updateStatElement('samplesProcessed', 0);
        this.updateStatElement('processingHz', '0 Hz');
        this.updateStatElement('maxRoll', '0°');
        this.updateStatElement('maxPitch', '0°');
        this.updateStatElement('maxYaw', '0°');
        this.updateStatElement('quatNorm', '1.000');
    }
    
    handleResize() {
        // Resize plots
        if (document.getElementById('eulerPlot')) {
            Plotly.Plots.resize('eulerPlot');
        }
        if (document.getElementById('quaternionPlot')) {
            Plotly.Plots.resize('quaternionPlot');
        }
        if (document.getElementById('imuPlot')) {
            Plotly.Plots.resize('imuPlot');
        }
        
        // Resize Three.js visualization
        if (this.threeJSViz) {
            this.threeJSViz.onWindowResize();
        }
    }
    
    handleKeyboard(event) {
        // Keyboard shortcuts
        if (event.ctrlKey || event.metaKey) {
            switch (event.key) {
                case 'o':
                    event.preventDefault();
                    document.getElementById('fileInput').click();
                    break;
                case 'p':
                    event.preventDefault();
                    if (!this.isProcessing) {
                        this.processData();
                    }
                    break;
                case 'r':
                    event.preventDefault();
                    this.clearResults();
                    break;
            }
        }
        
        // Animation controls
        if (this.threeJSViz) {
            switch (event.key) {
                case ' ':
                    event.preventDefault();
                    if (this.threeJSViz.isAnimationPlaying()) {
                        this.threeJSViz.pause();
                    } else {
                        this.threeJSViz.play();
                    }
                    break;
                case 'r':
                    if (!event.ctrlKey && !event.metaKey) {
                        this.threeJSViz.reset();
                    }
                    break;
            }
        }
    }
    
    // Public API methods
    getResults() {
        return this.results;
    }
    
    getInputData() {
        return this.inputData;
    }
    
    isConnectedToServer() {
        return this.isConnected;
    }
    
    isCurrentlyProcessing() {
        return this.isProcessing;
    }
    
    exportResults() {
        if (this.results.length === 0) {
            this.showStatus('No results to export', 'error');
            return;
        }
        
        try {
            const csvContent = this.convertToCSV(this.results);
            const blob = new Blob([csvContent], { type: 'text/csv' });
            const url = window.URL.createObjectURL(blob);
            
            const a = document.createElement('a');
            a.href = url;
            a.download = `ekf_results_${new Date().getTime()}.csv`;
            document.body.appendChild(a);
            a.click();
            document.body.removeChild(a);
            window.URL.revokeObjectURL(url);
            
            this.showStatus('Results exported successfully', 'success');
        } catch (error) {
            console.error('Export error:', error);
            this.showStatus('Failed to export results', 'error');
        }
    }
    
    convertToCSV(data) {
        if (data.length === 0) return '';
        
        const headers = Object.keys(data[0]);
        const csvRows = [headers.join(',')];
        
        for (const row of data) {
            const values = headers.map(header => {
                const value = row[header];
                return typeof value === 'string' ? `"${value}"` : value;
            });
            csvRows.push(values.join(','));
        }
        
        return csvRows.join('\n');
    }
    
    // Debug methods
    getDebugInfo() {
        return {
            isConnected: this.isConnected,
            isProcessing: this.isProcessing,
            resultsCount: this.results.length,
            inputDataCount: this.inputData.length,
            threeJSInitialized: !!this.threeJSViz,
            processingStartTime: this.processingStartTime
        };
    }
    
    // Cleanup
    destroy() {
        if (this.ws) {
            this.ws.close();
        }
        
        if (this.threeJSViz) {
            this.threeJSViz.dispose();
        }
        
        // Remove event listeners
        window.removeEventListener('resize', this.handleResize);
        document.removeEventListener('keydown', this.handleKeyboard);
    }
}

// Initialize dashboard when DOM is loaded
document.addEventListener('DOMContentLoaded', () => {
    window.ekfDashboard = new EKFDashboard();
    console.log('EKF Dashboard initialized');
    
    // Add export button functionality
    const exportBtn = document.createElement('button');
    exportBtn.textContent = '💾 Export Results';
    exportBtn.className = 'btn btn-success';
    exportBtn.onclick = () => window.ekfDashboard.exportResults();
    
    const controlButtons = document.querySelector('.control-buttons');
    if (controlButtons) {
        controlButtons.appendChild(exportBtn);
    }
    
    // Add keyboard shortcuts info
    console.log('Keyboard shortcuts:');
    console.log('- Ctrl+O: Open file');
    console.log('- Ctrl+P: Process data');
    console.log('- Ctrl+R: Clear results');
    console.log('- Space: Play/Pause 3D animation');
    console.log('- R: Reset 3D animation');
});

// Handle page unload
window.addEventListener('beforeunload', () => {
    if (window.ekfDashboard) {
        window.ekfDashboard.destroy();
    }
});