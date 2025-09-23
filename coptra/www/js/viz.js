var ros = new ROSLIB.Ros({
	url : 'ws://' + location.hostname + ':9090'
});

var titleEl = document.querySelector('title');

// DOM elements for status updates
var vizStatus = document.getElementById('viz-status');
var loadingViz = document.getElementById('loading-viz');
var vizError = document.getElementById('viz-error');

ros.on('error', function(error) {
	titleEl.innerText = 'Disconnected';
	vizStatus.textContent = '❌ Ошибка подключения к ROS';
	vizStatus.className = 'viz-status disconnected';
	loadingViz.style.display = 'none';
	vizError.style.display = 'block';
	console.error('ROS connection error:', error);
});

ros.on('connection', function() {
	console.log('connected');
	titleEl.innerText = 'Connected';
	vizStatus.textContent = '✅ Подключено к ROS';
	vizStatus.className = 'viz-status connected';
	loadingViz.style.display = 'none';
	vizError.style.display = 'none';
});

ros.on('close', function() {
	console.log('disconnected');
	titleEl.innerText = 'Disconnected';
	vizStatus.textContent = '❌ Отключено от ROS';
	vizStatus.className = 'viz-status disconnected';
	loadingViz.style.display = 'block';
	loadingViz.textContent = 'Переподключение...';
});

var viewer, tfClient;

function setScene(fixedFrame) {
	viewer = new ROS3D.Viewer({
		divID: 'viz',
		width: 1000,
		height: 600,
		antialias: true
	});

	tfClient = new ROSLIB.TFClient({
		ros: ros,
		angularThres: 0.01,
		transThres: 0.01,
		rate: 10.0,
		fixedFrame : fixedFrame
	});

	var map = new ROS3D.Grid({
		ros: ros,
		tfClient: tfClient,
		rootObject: viewer.scene
	});

	viewer.scene.add(map);
}

function addAxes() {
	var axes = new ROS3D.Axes({
		ros: ros,
		tfClient: tfClient,
		rootObject: viewer.scene
	});
	viewer.scene.add(axes);
}

function addVehicle() {
	new ROS3D.MarkerArrayClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/vehicle_marker',
		rootObject: viewer.scene
	});
}


function addCamera() {
	new ROS3D.MarkerArrayClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/main_camera/camera_markers',
		rootObject: viewer.scene
	});
}

function addAruco() {
	new ROS3D.MarkerArrayClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/aruco_detect/visualization',
		rootObject: viewer.scene
	});
}

function addArucoMap() {
	new ROS3D.MarkerArrayClient({
		ros: ros,
		tfClient: tfClient,
		topic: '/aruco_map/visualization',
		rootObject: viewer.scene
	});
}

// Control functions for buttons
function resetView() {
	if (viewer && viewer.camera) {
		viewer.camera.position.set(0, 0, 10);
		viewer.camera.lookAt(0, 0, 0);
		viewer.camera.updateProjectionMatrix();
	}
}

function toggleGrid() {
	// Toggle grid visibility
	if (viewer && viewer.scene) {
		// Implementation for grid toggle
		console.log('Grid toggled');
	}
}

function toggleAxes() {
	// Toggle axes visibility
	if (viewer && viewer.scene) {
		// Implementation for axes toggle
		console.log('Axes toggled');
	}
}

function toggleTrajectory() {
	// Toggle trajectory visibility
	if (viewer && viewer.scene) {
		// Implementation for trajectory toggle
		console.log('Trajectory toggled');
	}
}

// Initialize control buttons when DOM is ready
document.addEventListener('DOMContentLoaded', function() {
	// Add event listeners for control buttons
	var resetBtn = document.getElementById('reset-view');
	var gridBtn = document.getElementById('toggle-grid');
	var axesBtn = document.getElementById('toggle-axes');
	var trajectoryBtn = document.getElementById('toggle-trajectory');
	
	if (resetBtn) {
		resetBtn.addEventListener('click', resetView);
	}
	if (gridBtn) {
		gridBtn.addEventListener('click', toggleGrid);
	}
	if (axesBtn) {
		axesBtn.addEventListener('click', toggleAxes);
	}
	if (trajectoryBtn) {
		trajectoryBtn.addEventListener('click', toggleTrajectory);
	}
});
