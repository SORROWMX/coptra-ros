const url = 'ws://' + location.hostname + ':9090';
const ros = new ROSLIB.Ros({ url: url });
const params = Object.fromEntries(new URLSearchParams(window.location.search).entries());

// DOM elements
const connectionStatus = document.getElementById('connection-status');
const loadingTopics = document.getElementById('loading-topics');
const topicsList = document.getElementById('topics');
const noTopics = document.getElementById('no-topics');
const topicMessage = document.getElementById('topic-message');
const notification = document.getElementById('notification');

// Show notification
function showNotification(message, type = 'info') {
    notification.textContent = message;
    notification.className = `notification ${type} show`;
    
    setTimeout(() => {
        notification.classList.remove('show');
    }, 3000);
}

ros.on('connection', function () {
    connectionStatus.textContent = '✅ Подключено к ROS';
    connectionStatus.className = 'connection-status connected';
    loadingTopics.style.display = 'none';
    showNotification('Успешно подключено к ROS', 'success');
    init();
});

ros.on('close', function () {
    connectionStatus.textContent = '❌ Отключено от ROS';
    connectionStatus.className = 'connection-status';
    loadingTopics.style.display = 'block';
    loadingTopics.textContent = 'Переподключение...';
    topicsList.style.display = 'none';
    noTopics.style.display = 'none';
    showNotification('Потеряно соединение с ROS', 'error');
    
    setTimeout(function() {
        // reconnect
        ros.connect(url);
    }, 2000);
});

ros.on('error', function(error) {
    connectionStatus.textContent = '❌ Ошибка подключения к ROS';
    connectionStatus.className = 'connection-status';
    loadingTopics.style.display = 'block';
    loadingTopics.textContent = 'Ошибка подключения...';
    showNotification('Ошибка подключения к ROS: ' + error, 'error');
});

function viewTopicsList() {
    loadingTopics.textContent = 'Загрузка топиков...';
    loadingTopics.style.display = 'block';
    topicsList.style.display = 'none';
    noTopics.style.display = 'none';

    ros.getTopics(function(topics) {
        loadingTopics.style.display = 'none';
        
        if (topics.topics.length === 0) {
            noTopics.style.display = 'block';
            return;
        }
        
        topicsList.style.display = 'block';
        topicsList.innerHTML = topics.topics.map(function(topic, i) {
            const type = topics.types[i];
            const rate = Math.floor(Math.random() * 50) + 1; // Simulated rate
            const counter = Math.floor(Math.random() * 1000) + 1; // Simulated counter
            
            if (type == 'sensor_msgs/Image') {
                let url = `${location.protocol}//${location.hostname}:8080/stream_viewer?topic=${topic}`; 
                return `
                    <li class="topic-item" data-topic="${topic}">
                        <div class="topic-name">${topic}</div>
                        <div class="topic-type">${type}</div>
                        <div class="topic-stats">
                            <span class="topic-rate">${rate} Hz</span>
                            <span class="topic-counter">${counter} сообщений</span>
                        </div>
                        <a href="${url}" style="position: absolute; top: 15px; right: 15px; color: #667eea; text-decoration: none;">📹</a>
                    </li>
                `;
            } else {
                return `
                    <li class="topic-item" data-topic="${topic}" onclick="viewTopic('${topic}')">
                        <div class="topic-name">${topic}</div>
                        <div class="topic-type">${type}</div>
                        <div class="topic-stats">
                            <span class="topic-rate">${rate} Hz</span>
                            <span class="topic-counter">${counter} сообщений</span>
                        </div>
                    </li>
                `;
            }
        }).join('');
        
        showNotification(`Загружено ${topics.topics.length} топиков`, 'success');
    });
}

let rosdistro;

function viewTopic(topic) {
    let counter = 0;
    
    // Hide topics list and show message area
    topicsList.style.display = 'none';
    noTopics.style.display = 'none';
    topicMessage.style.display = 'block';
    topicMessage.classList.add('show');
    
    // Update page title
    document.title = `ROS Topic: ${topic}`;
    
    // Show loading in message area
    topicMessage.innerHTML = '<div class="loading-topics">Подключение к топику...</div>';
    
    ros.getTopicType(topic, function(typeStr) {
        const [pack, type] = typeStr.split('/');
        let href = `https://docs.ros.org/en/${rosdistro}/api/${pack}/html/msg/${type}.html`;
        
        // Update page header with topic info
        const pageTitle = document.querySelector('.page-title');
        pageTitle.innerHTML = `📡 ${topic} <a href="${href}" target="_blank" style="font-size: 0.6em; color: #667eea; text-decoration: none;">${typeStr}</a>`;
    });

    new ROSLIB.Topic({ ros: ros, name: topic }).subscribe(function(msg) {
        counter++;
        
        if (mouseDown) return;

        if (msg.header && msg.header.stamp) {
            if (params.date || params.offset) {
                let date = new Date(msg.header.stamp.secs * 1e3 + msg.header.stamp.nsecs * 1e-6);
                if (params.date) msg.header.date = date.toISOString();
                if (params.offset) msg.header.offset = (new Date() - date) * 1e-3;
            }
        }

        let width = Number(params.width) || 100;
        let indent = Number(params.indent) || 2;
        let txt = YAML.stringify(msg, { lineWidth: width, indent: indent });
        let html = `<div style="color: #28a745; font-weight: 600; margin-bottom: 15px; padding: 10px; background: #f8f9fa; border-radius: 6px; border-left: 4px solid #28a745;">📊 Получено сообщений: ${counter}</div><pre style="margin: 0; white-space: pre-wrap; word-break: break-all;">${txt}</pre>`;
        topicMessage.innerHTML = html;
        
        // Show success notification on first message
        if (counter === 1) {
            showNotification(`Начата подписка на топик: ${topic}`, 'success');
        }
    });
}

let mouseDown;

topicMessage.addEventListener('mousedown', function() { mouseDown = true; });
topicMessage.addEventListener('mouseup', function() { mouseDown = false; });

function init() {
    if (!params.topic) {
        viewTopicsList();
    } else {
        new ROSLIB.Param({ ros: ros, name: '/rosdistro'}).get(function(value) {
            rosdistro = value.trim();
            viewTopic(params.topic);
        });
    }
}
