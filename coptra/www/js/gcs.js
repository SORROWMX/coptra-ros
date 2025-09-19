var body = document.querySelector('body');
var titleEl = document.querySelector('title');
var modeEl = document.querySelector('#flight-mode');
var batteryEl = document.querySelector('#battery-status');
var connectionStatusEl = document.querySelector('#connection-status');

var url = 'ws://' + location.hostname + ':9090';
var ros = new ROSLIB.Ros({ url: url });

function speak(txt) {
	var utterance = new SpeechSynthesisUtterance(txt);
	window.speechSynthesis.speak(utterance);
}

ros.on('connection', function () {
	body.classList.add('connected');
	titleEl.innerText = 'Coptra GCS - Connected';
	connectionStatusEl.innerHTML = '<span class="status-indicator status-connected"></span>Connected';
});

ros.on('close', function () {
	titleEl.innerText = 'Coptra GCS - Disconnected';
	modeEl.innerHTML = '---';
	batteryEl.innerHTML = '---';
	connectionStatusEl.innerHTML = '<span class="status-indicator status-disconnected"></span>Disconnected';
	body.classList.remove('connected');
	setTimeout(function() {
		titleEl.innerText = 'Coptra GCS - Reconnecting';
		connectionStatusEl.innerHTML = '<span class="status-indicator status-warning"></span>Reconnecting...';
		ros.connect(url);
	}, 2000);
});

var fcuState = {};

new ROSLIB.Topic({
	ros: ros,
	name: '/mavros/state',
	messageType: 'mavros_msgs/State'
}).subscribe(function(msg) {
	modeEl.innerHTML = msg.mode;
	if (fcuState.mode != msg.mode) {
		// mode changed
		speak(msg.mode + ' flight mode');
	}
	fcuState = msg;
});

new ROSLIB.Topic({
	ros: ros,
	name: '/mavros/statustext/recv',
	messageType: 'mavros_msgs/StatusText'
}).subscribe(function(message) {
	var BLACKLIST = [];
	if (message.severity <= 4) {
		if (BLACKLIST.some(function(e) {
				return message.text.indexOf(e) != -1;
			})) {
			console.log('Filtered out message ' + message.text);
			return;
		}
		speak(message.text);
	}
});

new ROSLIB.Topic({
	ros: ros,
	name: '/mavros/battery',
	messageType: 'sensor_msgs/BatteryState',
	throttle_rate: 5000
}).subscribe(function(message) {
	var LOW_BATTERY = 3.8;
	batteryEl.innerHTML = (message.cell_voltage[0].toFixed(2) + ' V') || '';
});
