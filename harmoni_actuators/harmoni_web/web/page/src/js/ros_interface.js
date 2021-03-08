function rosInit(ros_master_uri = '') {

    if (ros_master_uri == '') {
        ros_master_uri = 'ws://' + location.hostname + ':9090'
    }

    console.log('ROS master URI: ' + ros_master_uri)
    ros = new ROSLIB.Ros({
        url: ros_master_uri
    });

    // Once connected, setup the ROS network
    ros.on('connection', function() {
        console.log('Connected to websocket server!');
        setupRosNetwork()
    });

    // If unable to connect or the connection closes, refresh the page
    // to try to reconnect
    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
        reload_page_to_retry_connecting();
    });
    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
        reload_page_to_retry_connecting();
    });
}

function reload_page_to_retry_connecting(wait_seconds = 2) {
    sleep(wait_seconds).then(function() {
        document.location.reload(true);
    });
}

function sleep(seconds) {
    return new Promise(resolve => setTimeout(resolve, seconds * 1000));
}

function setupRosNetwork() {

    display_view_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/harmoni/actuating/web/default/set_view',
        messageType: 'std_msgs/String'
    });
    display_view_listener.subscribe(viewListener);

    display_request_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/harmoni/actuating/web/default/set_view/request',
        messageType: 'std_msgs/String'
    });
    display_request_listener.subscribe(requestListener);

    user_response_publisher = new ROSLIB.Topic({
        ros: ros,
        name: '/harmoni/actuating/web/default/listen_click_event',
        queue_size: 1,
        messageType: 'std_msgs/String'
    });

    is_connected_client = new ROSLIB.Service({
        ros: ros,
        name: '/harmoni/actuating/web/default/is_connected',
        serviceType: 'std_srvs/Trigger'
    });

    is_connected_client.advertise(function(_, response) {
        console.log('is_connected_client received service request');
        response['success'] = true;
        response['message'] = 'Harmoni Web is connected';
        return true;
    });
}

