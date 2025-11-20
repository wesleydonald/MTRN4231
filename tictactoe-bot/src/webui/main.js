// Wait for the DOM to be fully loaded
window.addEventListener('load', function() {

    // --- 1. ROS Bridge Connection ---
    const ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090' // Default rosbridge WebSocket port
    });

    const statusElement = document.getElementById('ros-status');

    ros.on('connection', function() {
        console.log('Connected to websocket server.');
        statusElement.innerText = 'Connected';
        statusElement.className = 'connected';
    });

    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
        statusElement.innerText = 'Error';
        statusElement.className = 'disconnected';
    });

    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
        statusElement.innerText = 'Disconnected';
        statusElement.className = 'disconnected';
    });

    // --- 2. GUI -> ROS (Publishers & Service Calls) ---
    // From brain.cpp: create_subscription<std_msgs::msg::String>("/game/control")
    const controlTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/game/control',
        messageType: 'std_msgs/msg/String'
    });

    // From brain.cpp: create_service<std_srvs::srv::Trigger>("/ur5e/trigger_estop")
    const estopClient = new ROSLIB.Service({
        ros: ros,
        name: '/ur5e/trigger_estop',
        serviceType: 'std_srvs/srv/Trigger'
    });

    // Button Click Handlers
    document.getElementById('start-btn').onclick = function() {
        const message = new ROSLIB.Message({ data: 'start' });
        controlTopic.publish(message);
        console.log('Published "start" to /game/control');
    };

    document.getElementById('end-btn').onclick = function() {
        const message = new ROSLIB.Message({ data: 'end' });
        controlTopic.publish(message);
        console.log('Published "end" to /game/control');
    };

    document.getElementById('estop-btn').onclick = function() {
        const request = new ROSLIB.ServiceRequest({}); // Trigger request is empty
        estopClient.callService(request, function(result) {
            console.log('E-Stop service called. Success: ' + result.success);
            alert('E-STOP Confirmed: ' + result.message);
        });
    };

    // --- 3. ROS -> GUI (Subscribers) ---
    // Helper function to update text content
    function updateText(elementId, text) {
        const el = document.getElementById(elementId);
        if (el) { el.innerText = text; }
    }

    // From brain.cpp: create_publisher<std_msgs::msg::String>("/game/status")
    const statusListener = new ROSLIB.Topic({
        ros: ros,
        name: '/game/status',
        messageType: 'std_msgs/msg/String'
    });
    statusListener.subscribe(function(message) {
        console.log('Received status: ' + message.data);
        updateText('game-status', message.data);
    });

    // From brain.cpp: create_publisher<std_msgs::msg::Int32>("/game/metrics/wins")
    const winsListener = new ROSLIB.Topic({
        ros: ros,
        name: '/game/metrics/wins',
        messageType: 'std_msgs/msg/Int32'
    });
    winsListener.subscribe(function(message) {
        console.log('Received wins: ' + message.data);
        updateText('wins', message.data);
    });

    // From brain.cpp: create_publisher<std_msgs::msg::Int32>("/game/metrics/losses")
    const lossesListener = new ROSLIB.Topic({
        ros: ros,
        name: '/game/metrics/losses',
        messageType: 'std_msgs/msg/Int32'
    });
    lossesListener.subscribe(function(message) {
        console.log('Received losses: ' + message.data);
        // Note: brain.cpp calls human wins "session_losses_" (a loss for the robot)
        updateText('losses', message.data);
    });

    // From brain.cpp: create_publisher<std_msgs::msg::Int32>("/game/metrics/ties")
    const tiesListener = new ROSLIB.Topic({
        ros: ros,
        name: '/game/metrics/ties',
        messageType: 'std_msgs/msg/Int32'
    });
    tiesListener.subscribe(function(message) {
        console.log('Received ties: ' + message.data);
        updateText('ties', message.data);
    });

});