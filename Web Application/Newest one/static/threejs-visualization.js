/**
 * Advanced Three.js Visualization for EKF Data
 * Features: Animated rigid body, coordinate frames, trajectory visualization
 */

class ThreeJSVisualization {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.rigidBody = null;
        this.coordinateFrames = null;
        this.trajectory = null;
        this.controls = null;
        this.interpolationEnabled = true;
        this.smoothingLevel = 1; // 0=none, 1=medium, 2=high
        this.frameSkip = 1;
        this.showPerformanceStats = false;
        this.lastFrameTime = 0;
        this.frameCount = 0;
        this.performanceStats = null;
        
        // Animation data
        this.data = [];
        this.currentIndex = 0;
        this.isPlaying = false;
        this.animationSpeed = 1.0;
        this.animationId = null;
        
        // Translation-specific elements
        this.positionMarker = null;
        this.trajectoryPath = null;
        this.translationInfo = null;
        this.hasTranslationData = false;
        
        // Enhanced visualization controls
        this.showTranslation = true;
        this.showTrajectoryPath = true;
        this.trajectoryOpacity = 0.8;
        
        // Performance tracking
        this.frameCount = 0;
        this.lastFpsUpdate = Date.now();
        this.fps = 0;
        
        this.init();
    }


    // Enhanced updateVisualization method
    updateVisualization() {
        if (this.data.length === 0 || this.currentIndex >= this.data.length) return;
        
        const now = performance.now();
        const deltaTime = now - this.lastFrameTime;
        this.lastFrameTime = now;
        
        // Performance tracking
        if (this.showPerformanceStats) {
            this.updatePerformanceStats(deltaTime);
        }
        
        const currentData = this.data[this.currentIndex];
        let targetData = currentData;
        
        // Smart interpolation for smoother animation
        if (this.interpolationEnabled && this.currentIndex < this.data.length - 1) {
            const nextData = this.data[this.currentIndex + 1];
            const progress = (this.currentIndex % 1); // Get fractional part
            targetData = this.interpolateData(currentData, nextData, progress);
        }
        
        // Enhanced rigid body animation with smooth transitions
        if (this.rigidBody) {
            this.updateRigidBodySmooth(targetData);
        }
        
        // Enhanced trajectory with motion trails
        this.updateTrajectoryEnhanced(targetData);
        
        // Update UI elements
        this.updateTimeDisplay(currentData);
        this.updateTranslationInfo(targetData);
    }

    // Smooth rigid body updates
    updateRigidBodySmooth(data) {
        const quaternion = new THREE.Quaternion(
            data.quat_x, data.quat_y, data.quat_z, data.quat_w
        );
        
        // Smooth rotation with SLERP
        if (this.smoothingLevel > 0) {
            const smoothingFactor = this.smoothingLevel === 2 ? 0.05 : 0.1;
            this.rigidBody.quaternion.slerp(quaternion, 1 - smoothingFactor);
        } else {
            this.rigidBody.setRotationFromQuaternion(quaternion);
        }
        
        // Smooth position updates
        const targetPosition = new THREE.Vector3(
            data.pos_x || 0, data.pos_y || 0, data.pos_z || 0
        );
        
        if (this.smoothingLevel > 0) {
            const positionSmoothing = this.smoothingLevel === 2 ? 0.03 : 0.08;
            this.rigidBody.position.lerp(targetPosition, 1 - positionSmoothing);
        } else {
            this.rigidBody.position.copy(targetPosition);
        }
        
        // Enhanced velocity vector with better scaling
        this.updateVelocityVectorEnhanced(data);
    }

    // Enhanced velocity vector
    updateVelocityVectorEnhanced(data) {
        if (!this.velocityVector || !data.vel_x) return;
        
        const velocity = new THREE.Vector3(data.vel_x, data.vel_y, data.vel_z);
        const speed = velocity.length();
        
        if (speed > 0.01) {
            this.velocityVector.visible = true;
            this.velocityVector.position.copy(this.rigidBody.position);
            
            // Enhanced direction and scaling
            const direction = velocity.normalize();
            const vectorScale = document.getElementById('vectorScale')?.value || 2;
            const scaledLength = Math.min(speed * vectorScale, 10);
            
            // Smooth vector orientation
            const targetQuaternion = new THREE.Quaternion().setFromUnitVectors(
                new THREE.Vector3(0, 1, 0), direction
            );
            
            if (this.smoothingLevel > 0) {
                this.velocityVector.quaternion.slerp(targetQuaternion, 0.2);
            } else {
                this.velocityVector.quaternion.copy(targetQuaternion);
            }
            
            this.velocityVector.scale.setScalar(scaledLength);
        } else {
            this.velocityVector.visible = false;
        }
    }

    // Data interpolation for smooth animation
    interpolateData(data1, data2, t) {
        return {
            timestamp: data1.timestamp + (data2.timestamp - data1.timestamp) * t,
            roll: data1.roll + (data2.roll - data1.roll) * t,
            pitch: data1.pitch + (data2.pitch - data1.pitch) * t,
            yaw: data1.yaw + (data2.yaw - data1.yaw) * t,
            quat_w: data1.quat_w + (data2.quat_w - data1.quat_w) * t,
            quat_x: data1.quat_x + (data2.quat_x - data1.quat_x) * t,
            quat_y: data1.quat_y + (data2.quat_y - data1.quat_y) * t,
            quat_z: data1.quat_z + (data2.quat_z - data1.quat_z) * t,
            pos_x: (data1.pos_x || 0) + ((data2.pos_x || 0) - (data1.pos_x || 0)) * t,
            pos_y: (data1.pos_y || 0) + ((data2.pos_y || 0) - (data1.pos_y || 0)) * t,
            pos_z: (data1.pos_z || 0) + ((data2.pos_z || 0) - (data1.pos_z || 0)) * t,
            vel_x: (data1.vel_x || 0) + ((data2.vel_x || 0) - (data1.vel_x || 0)) * t,
            vel_y: (data1.vel_y || 0) + ((data2.vel_y || 0) - (data1.vel_y || 0)) * t,
            vel_z: (data1.vel_z || 0) + ((data2.vel_z || 0) - (data1.vel_z || 0)) * t
        };
    }

    // Performance monitoring
    updatePerformanceStats(deltaTime) {
        this.frameCount++;
        const fps = 1000 / deltaTime;
        
        if (!this.performanceStats) {
            this.performanceStats = document.createElement('div');
            this.performanceStats.className = 'performance-stats';
            this.container.appendChild(this.performanceStats);
        }
        
        if (this.frameCount % 10 === 0) { // Update every 10 frames
            this.performanceStats.innerHTML = `
                FPS: ${fps.toFixed(1)}<br>
                Frame: ${this.currentIndex}/${this.data.length}<br>
                Interpolation: ${this.interpolationEnabled ? 'ON' : 'OFF'}<br>
                Smoothing: ${['None', 'Medium', 'High'][this.smoothingLevel]}
            `;
        }
    }

    // Enhanced camera views
    setCameraView(view) {
        const distance = 15;
        switch (view) {
            case 'front':
                this.camera.position.set(0, 0, distance);
                break;
            case 'side':
                this.camera.position.set(distance, 0, 0);
                break;
            case 'top':
                this.camera.position.set(0, distance, 0);
                break;
            case 'isometric':
                this.camera.position.set(distance * 0.7, distance * 0.7, distance * 0.7);
                break;
            default:
                this.camera.position.set(8, 8, 8);
        }
        
        // Smooth camera transition
        const targetPosition = this.camera.position.clone();
        this.animateCamera(targetPosition);
    }

    // Smooth camera animation
    animateCamera(targetPosition) {
        const startPosition = this.camera.position.clone();
        const startTime = performance.now();
        const duration = 1000; // 1 second
        
        const animateStep = () => {
            const elapsed = performance.now() - startTime;
            const progress = Math.min(elapsed / duration, 1);
            
            // Smooth easing
            const eased = 1 - Math.pow(1 - progress, 3);
            
            this.camera.position.lerpVectors(startPosition, targetPosition, eased);
            this.camera.lookAt(0, 0, 0);
            
            if (progress < 1) {
                requestAnimationFrame(animateStep);
            }
        };
        
        requestAnimationFrame(animateStep);
    }

    stepForward() {
        if (this.currentIndex < this.data.length - 1) {
            this.currentIndex++;
            this.updateVisualization();
        }
    }

    stepBackward() {
        if (this.currentIndex > 0) {
            this.currentIndex--;
            this.updateVisualization();
        }
    }

    // Enhanced animation loop
    animate() {
        this.animationId = requestAnimationFrame(() => this.animate());
        
        // Frame skipping for performance
        if (this.frameCount % this.frameSkip !== 0) {
            this.frameCount++;
            this.renderer.render(this.scene, this.camera);
            return;
        }
        
        // Update animation with smoother increments
        if (this.isPlaying && this.data.length > 0) {
            const increment = this.animationSpeed * 0.5; // Smaller increments for smoothness
            this.currentIndex += increment;
            
            if (this.currentIndex >= this.data.length) {
                this.currentIndex = 0; // Loop animation
            }
            
            this.updateVisualization();
        }
        
        // Render
        this.renderer.render(this.scene, this.camera);
        this.frameCount++;
    }
    
    init() {
        this.setupScene();
        this.setupCamera();
        this.setupRenderer();
        this.setupLighting();
        this.setupControls();
        this.createRigidBody();
        this.createCoordinateFrames();
        this.createTrajectory();
        this.createVectors();
        this.setupEventListeners();
        this.animate();
    }
    
    setupScene() {
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0f0f0f);
        
        // Add fog for depth perception
        this.scene.fog = new THREE.Fog(0x0f0f0f, 10, 100);
        
        // Add grid
        const gridHelper = new THREE.GridHelper(20, 20, 0x444444, 0x222222);
        this.scene.add(gridHelper);
        
        // Add axes helper
        const axesHelper = new THREE.AxesHelper(2);
        this.scene.add(axesHelper);
    }
    
    setupCamera() {
        this.camera = new THREE.PerspectiveCamera(
            75, 
            this.container.clientWidth / this.container.clientHeight, 
            0.1, 
            1000
        );
        this.camera.position.set(5, 5, 5);
        this.camera.lookAt(0, 0, 0);
    }
    
    setupRenderer() {
        this.renderer = new THREE.WebGLRenderer({ 
            antialias: true,
            alpha: true
        });
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.renderer.gammaOutput = true;
        this.renderer.gammaFactor = 2.2;
        
        this.container.appendChild(this.renderer.domElement);
    }
    
    setupLighting() {
        // Ambient light
        const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
        this.scene.add(ambientLight);
        
        // Directional light (main)
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(10, 10, 5);
        directionalLight.castShadow = true;
        directionalLight.shadow.mapSize.width = 2048;
        directionalLight.shadow.mapSize.height = 2048;
        directionalLight.shadow.camera.near = 0.5;
        directionalLight.shadow.camera.far = 50;
        directionalLight.shadow.camera.left = -10;
        directionalLight.shadow.camera.right = 10;
        directionalLight.shadow.camera.top = 10;
        directionalLight.shadow.camera.bottom = -10;
        this.scene.add(directionalLight);
        
        // Point light for additional illumination
        const pointLight = new THREE.PointLight(0x3498db, 0.5, 100);
        pointLight.position.set(-5, 5, 5);
        this.scene.add(pointLight);
        
        // Hemisphere light for natural ambiance
        const hemisphereLight = new THREE.HemisphereLight(0x87ceeb, 0x2c3e50, 0.3);
        this.scene.add(hemisphereLight);
    }
    
    setupControls() {
        // Simple orbit controls (manual implementation)
        this.controls = {
            isMouseDown: false,
            mouseX: 0,
            mouseY: 0,
            rotationSpeed: 0.005,
            zoomSpeed: 0.1,
            panSpeed: 0.01
        };
        
        // Mouse event listeners
        this.renderer.domElement.addEventListener('mousedown', this.onMouseDown.bind(this));
        this.renderer.domElement.addEventListener('mousemove', this.onMouseMove.bind(this));
        this.renderer.domElement.addEventListener('mouseup', this.onMouseUp.bind(this));
        this.renderer.domElement.addEventListener('wheel', this.onMouseWheel.bind(this));
    }
    
    createRigidBody() {
        // Create a detailed aircraft-like rigid body
        const bodyGroup = new THREE.Group();
        
        // Main fuselage
        const fuselageGeometry = new THREE.CylinderGeometry(0.1, 0.15, 2, 8);
        const fuselageMaterial = new THREE.MeshPhongMaterial({ 
            color: 0x3498db,
            shininess: 100
        });
        const fuselage = new THREE.Mesh(fuselageGeometry, fuselageMaterial);
        fuselage.rotation.z = Math.PI / 2;
        fuselage.castShadow = true;
        bodyGroup.add(fuselage);
        
        // Wings
        const wingGeometry = new THREE.BoxGeometry(0.05, 1.5, 0.3);
        const wingMaterial = new THREE.MeshPhongMaterial({ 
            color: 0x2ecc71,
            shininess: 50
        });
        const leftWing = new THREE.Mesh(wingGeometry, wingMaterial);
        leftWing.position.set(-0.2, 0.75, 0);
        leftWing.castShadow = true;
        bodyGroup.add(leftWing);
        
        const rightWing = new THREE.Mesh(wingGeometry, wingMaterial);
        rightWing.position.set(-0.2, -0.75, 0);
        rightWing.castShadow = true;
        bodyGroup.add(rightWing);
        
        // Tail
        const tailGeometry = new THREE.BoxGeometry(0.03, 0.4, 0.6);
        const tailMaterial = new THREE.MeshPhongMaterial({ 
            color: 0xe74c3c,
            shininess: 50
        });
        const tail = new THREE.Mesh(tailGeometry, tailMaterial);
        tail.position.set(-0.9, 0, 0.3);
        tail.castShadow = true;
        bodyGroup.add(tail);
        
        // Propeller
        const propellerGeometry = new THREE.CylinderGeometry(0.02, 0.02, 0.6, 4);
        const propellerMaterial = new THREE.MeshPhongMaterial({ 
            color: 0xf39c12,
            shininess: 100
        });
        const propeller = new THREE.Mesh(propellerGeometry, propellerMaterial);
        propeller.position.set(1.1, 0, 0);
        propeller.rotation.z = Math.PI / 2;
        propeller.castShadow = true;
        bodyGroup.add(propeller);
        
        // Add glow effect
        const glowGeometry = new THREE.SphereGeometry(1.2, 16, 16);
        const glowMaterial = new THREE.MeshBasicMaterial({
            color: 0x3498db,
            transparent: true,
            opacity: 0.1,
            side: THREE.BackSide
        });
        const glow = new THREE.Mesh(glowGeometry, glowMaterial);
        bodyGroup.add(glow);
        
        this.rigidBody = bodyGroup;
        this.scene.add(this.rigidBody);
    }
    
    createCoordinateFrames() {
        // World coordinate frame (NED)
        this.worldFrame = new THREE.Group();
        
        // X-axis (North) - Red
        const xGeometry = new THREE.CylinderGeometry(0.02, 0.02, 2, 8);
        const xMaterial = new THREE.MeshPhongMaterial({ color: 0xff0000 });
        const xAxis = new THREE.Mesh(xGeometry, xMaterial);
        xAxis.rotation.z = -Math.PI / 2;
        xAxis.position.x = 1;
        this.worldFrame.add(xAxis);
        
        // Y-axis (East) - Green
        const yGeometry = new THREE.CylinderGeometry(0.02, 0.02, 2, 8);
        const yMaterial = new THREE.MeshPhongMaterial({ color: 0x00ff00 });
        const yAxis = new THREE.Mesh(yGeometry, yMaterial);
        yAxis.position.y = 1;
        this.worldFrame.add(yAxis);
        
        // Z-axis (Down) - Blue
        const zGeometry = new THREE.CylinderGeometry(0.02, 0.02, 2, 8);
        const zMaterial = new THREE.MeshPhongMaterial({ color: 0x0000ff });
        const zAxis = new THREE.Mesh(zGeometry, zMaterial);
        zAxis.rotation.x = Math.PI / 2;
        zAxis.position.z = 1;
        this.worldFrame.add(zAxis);
        
        // Add labels
        this.addAxisLabels(this.worldFrame);
        
        this.scene.add(this.worldFrame);
        
        // Body coordinate frame (FRD)
        this.bodyFrame = new THREE.Group();
        
        // X-axis (Forward) - Red
        const bxGeometry = new THREE.CylinderGeometry(0.015, 0.015, 1.5, 8);
        const bxMaterial = new THREE.MeshPhongMaterial({ color: 0xff4444 });
        const bxAxis = new THREE.Mesh(bxGeometry, bxMaterial);
        bxAxis.rotation.z = -Math.PI / 2;
        bxAxis.position.x = 0.75;
        this.bodyFrame.add(bxAxis);
        
        // Y-axis (Right) - Green
        const byGeometry = new THREE.CylinderGeometry(0.015, 0.015, 1.5, 8);
        const byMaterial = new THREE.MeshPhongMaterial({ color: 0x44ff44 });
        const byAxis = new THREE.Mesh(byGeometry, byMaterial);
        byAxis.position.y = 0.75;
        this.bodyFrame.add(byAxis);
        
        // Z-axis (Down) - Blue
        const bzGeometry = new THREE.CylinderGeometry(0.015, 0.015, 1.5, 8);
        const bzMaterial = new THREE.MeshPhongMaterial({ color: 0x4444ff });
        const bzAxis = new THREE.Mesh(bzGeometry, bzMaterial);
        bzAxis.rotation.x = Math.PI / 2;
        bzAxis.position.z = 0.75;
        this.bodyFrame.add(bzAxis);
        
        this.rigidBody.add(this.bodyFrame);
    }
    
    addAxisLabels(frame) {
        // Create text labels for axes (simplified)
        const loader = new THREE.FontLoader();
        // Note: In a real implementation, you would load a font and create text geometry
        // For now, we'll use simple geometric shapes as labels
        
        // X label
        const xLabelGeometry = new THREE.SphereGeometry(0.05, 8, 8);
        const xLabelMaterial = new THREE.MeshPhongMaterial({ color: 0xff0000 });
        const xLabel = new THREE.Mesh(xLabelGeometry, xLabelMaterial);
        xLabel.position.set(2.2, 0, 0);
        frame.add(xLabel);
        
        // Y label
        const yLabelGeometry = new THREE.SphereGeometry(0.05, 8, 8);
        const yLabelMaterial = new THREE.MeshPhongMaterial({ color: 0x00ff00 });
        const yLabel = new THREE.Mesh(yLabelGeometry, yLabelMaterial);
        yLabel.position.set(0, 2.2, 0);
        frame.add(yLabel);
        
        // Z label
        const zLabelGeometry = new THREE.SphereGeometry(0.05, 8, 8);
        const zLabelMaterial = new THREE.MeshPhongMaterial({ color: 0x0000ff });
        const zLabel = new THREE.Mesh(zLabelGeometry, zLabelMaterial);
        zLabel.position.set(0, 0, 2.2);
        frame.add(zLabel);
    }
    
    createTrajectory() {
        // Create trajectory line (your existing code)
        const trajectoryGeometry = new THREE.BufferGeometry();
        const trajectoryMaterial = new THREE.LineBasicMaterial({ 
            color: 0xffff00,
            linewidth: 3,
            opacity: 0.8,
            transparent: true
        });
        
        this.trajectoryLine = new THREE.Line(trajectoryGeometry, trajectoryMaterial);
        this.scene.add(this.trajectoryLine);
        
        // NEW: Add position marker for current location
        const markerGeometry = new THREE.SphereGeometry(0.1, 16, 12);
        const markerMaterial = new THREE.MeshPhongMaterial({ 
            color: 0xff0000,
            emissive: 0x330000,
            shininess: 100
        });
        this.positionMarker = new THREE.Mesh(markerGeometry, markerMaterial);
        this.positionMarker.castShadow = true;
        this.scene.add(this.positionMarker);
        
        // NEW: Create enhanced trajectory path with multiple colors
        this.trajectoryPath = new THREE.Group();
        this.scene.add(this.trajectoryPath);
        
        // Create trajectory points array
        this.trajectoryPoints = [];
    }
    
    createVectors() {
        // Velocity vector
        this.velocityVector = this.createArrow(0xff00ff, 2);
        this.scene.add(this.velocityVector);
        
        // Acceleration vector
        this.accelerationVector = this.createArrow(0x00ffff, 1.5);
        this.scene.add(this.accelerationVector);
    }
    
    createArrow(color, length) {
        const group = new THREE.Group();
        
        // Arrow shaft
        const shaftGeometry = new THREE.CylinderGeometry(0.02, 0.02, length, 8);
        const shaftMaterial = new THREE.MeshPhongMaterial({ color: color });
        const shaft = new THREE.Mesh(shaftGeometry, shaftMaterial);
        shaft.position.y = length / 2;
        group.add(shaft);
        
        // Arrow head
        const headGeometry = new THREE.ConeGeometry(0.08, 0.2, 8);
        const headMaterial = new THREE.MeshPhongMaterial({ color: color });
        const head = new THREE.Mesh(headGeometry, headMaterial);
        head.position.y = length + 0.1;
        group.add(head);
        
        group.visible = false;
        return group;
    }
    
    setupEventListeners() {
        window.addEventListener('resize', this.onWindowResize.bind(this));
        
        // Animation control listeners
        const playBtn = document.getElementById('playBtn');
        const pauseBtn = document.getElementById('pauseBtn');
        const resetBtn = document.getElementById('resetBtn');
        const speedSlider = document.getElementById('speedSlider');
        const timeSlider = document.getElementById('timeSlider');
        
        if (playBtn) playBtn.addEventListener('click', () => this.play());
        if (pauseBtn) pauseBtn.addEventListener('click', () => this.pause());
        if (resetBtn) resetBtn.addEventListener('click', () => this.reset());
        if (speedSlider) speedSlider.addEventListener('input', (e) => this.setSpeed(e.target.value));
        if (timeSlider) timeSlider.addEventListener('input', (e) => this.setTime(e.target.value));
    }
    
    onMouseDown(event) {
        this.controls.isMouseDown = true;
        this.controls.mouseX = event.clientX;
        this.controls.mouseY = event.clientY;
    }
    
    onMouseMove(event) {
        if (!this.controls.isMouseDown) return;
        
        const deltaX = event.clientX - this.controls.mouseX;
        const deltaY = event.clientY - this.controls.mouseY;
        
        // Rotate camera around origin
        const spherical = new THREE.Spherical();
        spherical.setFromVector3(this.camera.position);
        
        spherical.theta -= deltaX * this.controls.rotationSpeed;
        spherical.phi += deltaY * this.controls.rotationSpeed;
        
        // Limit phi to prevent flipping
        spherical.phi = Math.max(0.1, Math.min(Math.PI - 0.1, spherical.phi));
        
        this.camera.position.setFromSpherical(spherical);
        this.camera.lookAt(0, 0, 0);
        
        this.controls.mouseX = event.clientX;
        this.controls.mouseY = event.clientY;
    }
    
    onMouseUp(event) {
        this.controls.isMouseDown = false;
    }
    
    onMouseWheel(event) {
        const scale = event.deltaY > 0 ? 1.1 : 0.9;
        this.camera.position.multiplyScalar(scale);
        
        // Limit zoom
        const distance = this.camera.position.length();
        if (distance < 2) this.camera.position.normalize().multiplyScalar(2);
        if (distance > 50) this.camera.position.normalize().multiplyScalar(50);
        
        event.preventDefault();
    }
    
    onWindowResize() {
        this.camera.aspect = this.container.clientWidth / this.container.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    }
    
    setData(data) {
        console.log('Setting visualization data:', data.length, 'points');
        this.data = data;
        this.currentIndex = 0;
        
        // Check if data contains translation information
        this.hasTranslationData = data.length > 0 && 
            (data[0].pos_x !== undefined || data[0].vel_x !== undefined);
        
        console.log('Has translation data:', this.hasTranslationData);
        
        this.updateTrajectory();
        this.updateTimeSlider();
        
        // NEW: Update translation-specific elements
        this.updateTranslationVisualization();
    }

    updateTranslationVisualization() {
        if (!this.hasTranslationData) {
            console.log('No translation data available, using rotation-only mode');
            return;
        }
        
        // Clear existing trajectory path
        this.trajectoryPath.clear();
        
        // Create enhanced trajectory with color coding based on velocity
        const positions = [];
        const colors = [];
        
        for (let i = 0; i < this.data.length; i++) {
            const point = this.data[i];
            const x = point.pos_x || 0;
            const y = point.pos_y || 0;
            const z = point.pos_z || 0;
            
            positions.push(x, y, z);
            
            // Color based on velocity magnitude
            const vel_mag = Math.sqrt(
                (point.vel_x || 0) * (point.vel_x || 0) + 
                (point.vel_y || 0) * (point.vel_y || 0) + 
                (point.vel_z || 0) * (point.vel_z || 0)
            );
            
            // Color from blue (slow) to red (fast)
            const maxVel = 5.0; // Adjust based on your data
            const normalizedVel = Math.min(vel_mag / maxVel, 1.0);
            
            colors.push(
                normalizedVel,      // Red component
                0.2,                // Green component
                1.0 - normalizedVel // Blue component
            );
        }
        
        // Create colored trajectory line
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
        geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
        
        const material = new THREE.LineBasicMaterial({ 
            vertexColors: true,
            linewidth: 3,
            opacity: this.trajectoryOpacity,
            transparent: true
        });
        
        const coloredTrajectory = new THREE.Line(geometry, material);
        this.trajectoryPath.add(coloredTrajectory);
        
        console.log('Enhanced trajectory created with', positions.length / 3, 'points');
    }
    
    updateTrajectory() {
        if (this.data.length === 0) return;
        
        const positions = [];
        this.trajectoryPoints = [];
        
        for (let i = 0; i < this.data.length; i++) {
            const point = this.data[i];
            // Use position data if available, otherwise use index-based positioning
            const x = point.pos_x || 0;
            const y = point.pos_y || 0;
            const z = point.pos_z || 0;
            
            positions.push(x, y, z);
            this.trajectoryPoints.push(new THREE.Vector3(x, y, z));
        }
        
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
        
        this.trajectoryLine.geometry.dispose();
        this.trajectoryLine.geometry = geometry;
    }
    
    updateTimeSlider() {
        const timeSlider = document.getElementById('timeSlider');
        const totalTime = document.getElementById('totalTime');
        
        if (timeSlider && this.data.length > 0) {
            timeSlider.max = this.data.length - 1;
            timeSlider.value = this.currentIndex;
        }
        
        if (totalTime && this.data.length > 0) {
            const duration = this.data[this.data.length - 1].timestamp - this.data[0].timestamp;
            totalTime.textContent = duration.toFixed(2) + 's';
        }
    }
    
    play() {
        this.isPlaying = true;
        const playBtn = document.getElementById('playBtn');
        const pauseBtn = document.getElementById('pauseBtn');
        
        if (playBtn) playBtn.style.display = 'none';
        if (pauseBtn) pauseBtn.style.display = 'inline-block';
    }
    
    pause() {
        this.isPlaying = false;
        const playBtn = document.getElementById('playBtn');
        const pauseBtn = document.getElementById('pauseBtn');
        
        if (playBtn) playBtn.style.display = 'inline-block';
        if (pauseBtn) pauseBtn.style.display = 'none';
    }
    
    reset() {
        this.currentIndex = 0;
        this.isPlaying = false;
        this.updateTimeSlider();
        
        const playBtn = document.getElementById('playBtn');
        const pauseBtn = document.getElementById('pauseBtn');
        
        if (playBtn) playBtn.style.display = 'inline-block';
        if (pauseBtn) pauseBtn.style.display = 'none';
    }
    
    setSpeed(speed) {
        this.animationSpeed = parseFloat(speed);
        const speedLabel = document.getElementById('speedLabel');
        if (speedLabel) speedLabel.textContent = speed + 'x';
    }
    
    setTime(index) {
        this.currentIndex = parseInt(index);
        this.updateVisualization();
    }
    
    updateVisualization() {
        if (this.data.length === 0 || this.currentIndex >= this.data.length) return;
        
        const currentData = this.data[this.currentIndex];
        
        // Update rigid body orientation (your existing code)
        if (this.rigidBody) {
            const quaternion = new THREE.Quaternion(
                currentData.quat_x,
                currentData.quat_y,
                currentData.quat_z,
                currentData.quat_w
            );
            this.rigidBody.setRotationFromQuaternion(quaternion);
            
            // ENHANCED: Update position with smooth interpolation
            const targetPosition = new THREE.Vector3(
                currentData.pos_x || 0,
                currentData.pos_y || 0,
                currentData.pos_z || 0
            );
            
            // Smooth position transition for better visualization
            this.rigidBody.position.lerp(targetPosition, 0.1);
            
            // Update position marker to exact position
            if (this.positionMarker) {
                this.positionMarker.position.copy(targetPosition);
            }
        }
        
        // ENHANCED: Update velocity vector with better scaling
        if (this.velocityVector && currentData.vel_x !== undefined) {
            const velocity = new THREE.Vector3(currentData.vel_x, currentData.vel_y, currentData.vel_z);
            const speed = velocity.length();
            
            if (speed > 0.1) {
                this.velocityVector.visible = true;
                this.velocityVector.position.copy(this.rigidBody.position);
                
                // Better velocity vector direction and scaling
                const direction = velocity.normalize();
                this.velocityVector.lookAt(
                    this.rigidBody.position.x + direction.x,
                    this.rigidBody.position.y + direction.y,
                    this.rigidBody.position.z + direction.z
                );
                
                // Scale vector based on speed (with reasonable limits)
                const vectorScale = Math.min(speed * 2, 5);
                this.velocityVector.scale.setScalar(vectorScale);
            } else {
                this.velocityVector.visible = false;
            }
        }
        
        // NEW: Update translation information display
        this.updateTranslationInfo(currentData);
        
        // Update time display (your existing code)
        const currentTime = document.getElementById('currentTime');
        const timeSlider = document.getElementById('timeSlider');
        
        if (currentTime) {
            currentTime.textContent = currentData.timestamp.toFixed(2) + 's';
        }
        
        if (timeSlider) {
            timeSlider.value = this.currentIndex;
        }
    }
    
    updateTranslationInfo(currentData) {
        // Create or update translation info panel
        let infoPanel = document.getElementById('translation-info');
        if (!infoPanel) {
            infoPanel = document.createElement('div');
            infoPanel.id = 'translation-info';
            infoPanel.style.cssText = `
                position: absolute;
                top: 10px;
                right: 10px;
                background: rgba(0,0,0,0.8);
                color: white;
                padding: 15px;
                border-radius: 5px;
                font-family: 'Courier New', monospace;
                font-size: 12px;
                z-index: 1000;
                min-width: 200px;
            `;
            this.container.appendChild(infoPanel);
        }
        
        // Update info content
        const pos_x = currentData.pos_x || 0;
        const pos_y = currentData.pos_y || 0;
        const pos_z = currentData.pos_z || 0;
        const vel_x = currentData.vel_x || 0;
        const vel_y = currentData.vel_y || 0;
        const vel_z = currentData.vel_z || 0;
        
        const speed = Math.sqrt(vel_x*vel_x + vel_y*vel_y + vel_z*vel_z);
        const distance = Math.sqrt(pos_x*pos_x + pos_y*pos_y + pos_z*pos_z);
        
        infoPanel.innerHTML = `
            <h4 style="margin: 0 0 10px 0; color: #00ff00;">6-DOF Motion Data</h4>
            <div style="margin-bottom: 8px;">
                <strong>Translation:</strong><br>
                Position: (${pos_x.toFixed(2)}, ${pos_y.toFixed(2)}, ${pos_z.toFixed(2)}) m<br>
                Distance: ${distance.toFixed(2)} m<br>
                Velocity: (${vel_x.toFixed(2)}, ${vel_y.toFixed(2)}, ${vel_z.toFixed(2)}) m/s<br>
                Speed: ${speed.toFixed(2)} m/s
            </div>
            <div>
                <strong>Rotation:</strong><br>
                Roll: ${((currentData.roll || 0) * 180/Math.PI).toFixed(1)}°<br>
                Pitch: ${((currentData.pitch || 0) * 180/Math.PI).toFixed(1)}°<br>
                Yaw: ${((currentData.yaw || 0) * 180/Math.PI).toFixed(1)}°
            </div>
        `;
    }    

    animate() {
        this.animationId = requestAnimationFrame(() => this.animate());
        
        // Update animation
        if (this.isPlaying && this.data.length > 0) {
            this.currentIndex += this.animationSpeed;
            
            if (this.currentIndex >= this.data.length) {
                this.currentIndex = 0; // Loop animation
            }
            
            this.updateVisualization();
        }
        
        // Update FPS counter
        this.frameCount++;
        const now = Date.now();
        if (now - this.lastFpsUpdate > 1000) {
            this.fps = this.frameCount;
            this.frameCount = 0;
            this.lastFpsUpdate = now;
        }
        
        // Render
        this.renderer.render(this.scene, this.camera);
    }
    
    dispose() {
        if (this.animationId) {
            cancelAnimationFrame(this.animationId);
        }
        
        // Clean up Three.js objects
        if (this.renderer) {
            this.renderer.dispose();
        }
        
        if (this.scene) {
            this.scene.traverse((object) => {
                if (object.geometry) object.geometry.dispose();
                if (object.material) {
                    if (Array.isArray(object.material)) {
                        object.material.forEach(material => material.dispose());
                    } else {
                        object.material.dispose();
                    }
                }
            });
        }
        
        // Remove event listeners
        window.removeEventListener('resize', this.onWindowResize.bind(this));
        if (this.renderer && this.renderer.domElement) {
            this.renderer.domElement.removeEventListener('mousedown', this.onMouseDown.bind(this));
            this.renderer.domElement.removeEventListener('mousemove', this.onMouseMove.bind(this));
            this.renderer.domElement.removeEventListener('mouseup', this.onMouseUp.bind(this));
            this.renderer.domElement.removeEventListener('wheel', this.onMouseWheel.bind(this));
        }
    }
    
    // Public API methods
    getCurrentFrame() {
        return this.currentIndex;
    }
    
    getTotalFrames() {
        return this.data.length;
    }
    
    isAnimationPlaying() {
        return this.isPlaying;
    }
    
    getAnimationSpeed() {
        return this.animationSpeed;
    }
    
    getFPS() {
        return this.fps;
    }
    
    // Camera controls
    resetCamera() {
        this.camera.position.set(5, 5, 5);
        this.camera.lookAt(0, 0, 0);
    }
    
    setCameraView(view) {
        switch (view) {
            case 'front':
                this.camera.position.set(0, 0, 10);
                break;
            case 'side':
                this.camera.position.set(10, 0, 0);
                break;
            case 'top':
                this.camera.position.set(0, 10, 0);
                break;
            default:
                this.resetCamera();
        }
        this.camera.lookAt(0, 0, 0);
    }
    
    // Utility methods
    exportFrame() {
        return this.renderer.domElement.toDataURL('image/png');
    }
    
    setBackground(color) {
        this.scene.background = new THREE.Color(color);
    }
    
    toggleTrajectory() {
        this.trajectoryLine.visible = !this.trajectoryLine.visible;
    }
    
    toggleVectors() {
        this.velocityVector.visible = !this.velocityVector.visible;
        this.accelerationVector.visible = !this.accelerationVector.visible;
    }
    
    toggleCoordinateFrames() {
        this.worldFrame.visible = !this.worldFrame.visible;
        this.bodyFrame.visible = !this.bodyFrame.visible;
    }

    // Enhanced camera control for translation viewing
    setCameraToFollowRigidBody(follow = true) {
        this.followRigidBody = follow;
        if (follow && this.rigidBody) {
            // Position camera relative to rigid body
            const offset = new THREE.Vector3(5, 5, 5);
            this.camera.position.copy(this.rigidBody.position).add(offset);
            this.camera.lookAt(this.rigidBody.position);
        }
    }

    // Toggle translation-specific visualizations
    toggleTranslationElements() {
        if (this.positionMarker) {
            this.positionMarker.visible = !this.positionMarker.visible;
        }
        if (this.trajectoryPath) {
            this.trajectoryPath.visible = !this.trajectoryPath.visible;
        }
        this.showTranslation = !this.showTranslation;
    }

    // Set trajectory opacity
    setTrajectoryOpacity(opacity) {
        this.trajectoryOpacity = opacity;
        if (this.trajectoryLine) {
            this.trajectoryLine.material.opacity = opacity;
        }
        if (this.trajectoryPath) {
            this.trajectoryPath.children.forEach(child => {
                if (child.material) {
                    child.material.opacity = opacity;
                }
            });
        }
    }

    // Get translation statistics
    getTranslationStats() {
        if (!this.hasTranslationData || this.data.length === 0) return null;
        
        let maxSpeed = 0;
        let totalDistance = 0;
        let maxDistance = 0;
        
        for (let i = 0; i < this.data.length; i++) {
            const point = this.data[i];
            const vel_mag = Math.sqrt(
                (point.vel_x || 0) * (point.vel_x || 0) + 
                (point.vel_y || 0) * (point.vel_y || 0) + 
                (point.vel_z || 0) * (point.vel_z || 0)
            );
            
            const distance = Math.sqrt(
                (point.pos_x || 0) * (point.pos_x || 0) + 
                (point.pos_y || 0) * (point.pos_y || 0) + 
                (point.pos_z || 0) * (point.pos_z || 0)
            );
            
            maxSpeed = Math.max(maxSpeed, vel_mag);
            maxDistance = Math.max(maxDistance, distance);
            
            if (i > 0) {
                const prevPoint = this.data[i-1];
                const dx = (point.pos_x || 0) - (prevPoint.pos_x || 0);
                const dy = (point.pos_y || 0) - (prevPoint.pos_y || 0);
                const dz = (point.pos_z || 0) - (prevPoint.pos_z || 0);
                totalDistance += Math.sqrt(dx*dx + dy*dy + dz*dz);
            }
        }
        
        return {
            maxSpeed,
            totalDistance,
            maxDistance,
            hasTranslation: this.hasTranslationData
        };
    }

}

// Export for use in other scripts
window.ThreeJSVisualization = ThreeJSVisualization;