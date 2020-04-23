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
        setup_mouse_and_keypress_event_publishers();
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

    display_listener = new ROSLIB.Topic({
        ros: ros,
        name: 'cordial/gui/display',
        messageType: 'cordial_gui/Display'
    });
    display_listener.subscribe(make_display);

    user_response_publisher = new ROSLIB.Topic({
        ros: ros,
        name: 'cordial/gui/user_response',
        queue_size: 1,
        messageType: 'std_msgs/String'
    });

    mouse_event_publisher = new ROSLIB.Topic({
        ros: ros,
        name: 'cordial/gui/event/mouse',
        queue_size: 1,
        messageType: 'cordial_gui/MouseEvent'
    });

    keypress_event_publisher = new ROSLIB.Topic({
        ros: ros,
        name: 'cordial/gui/event/keypress',
        queue_size: 1,
        messageType: 'std_msgs/String'
    });

    is_connected_client = new ROSLIB.Service({
        ros: ros,
        name: '/cordial/gui/is_connected',
        serviceType: 'std_srvs/Trigger'
    });
    is_connected_client.advertise(function(_, response) {
        console.log('is_connected_client received service request');
        response['success'] = true;
        response['message'] = 'GUI is connected';
        return true;
    });

    is_started_publisher = new ROSLIB.Topic({
        ros: ros,
        name: 'cordial/gui/event/new_server',
        queue_size: 1,
        messageType: 'std_msgs/Empty'
    });
    is_started_publisher.publish({})
}

function setup_mouse_and_keypress_event_publishers() {

    var enable_mouse_move_callback = true;
    var enable_times_per_second = 10;

    var ms_before_enable = 1000 / enable_times_per_second
    window.setInterval(function() {
            enable_mouse_move_callback = true;
        },
        ms_before_enable
    );
    $(this).mousemove(function(e) {
        if (enable_mouse_move_callback) {
            _publish_mouse_event(e, false);
            enable_mouse_move_callback = false;
        }
    })
    $(this).click(function(e) {
        _publish_mouse_event(e, true);
    })
    $(this).keypress(function(e) {
        _publish_key_press(e);
    });
}


function make_display(display_msg) {

    console.log("Message received:", display_msg)

    var display_type = display_msg.type
    var content = display_msg.content
    var buttons = display_msg.buttons
    var args = display_msg.args
    var buttons_delay_seconds = display_msg.buttons_delay_seconds
    var callback_fn = publish_user_response

    switch (display_type) {
        case 'black':
            show_black_screen();
            break;
        case 'multiple choice':
            multiple_choice_prompt(content, buttons, callback_fn, args, buttons_delay_seconds);
            break;
        case 'text entry':
            text_entry_prompt(content, buttons, callback_fn, args, buttons_delay_seconds);
            break;
        case 'time entry':
            time_entry_prompt(content, buttons, callback_fn, args, buttons_delay_seconds);
            break;
        case 'slider':
            slider_prompt(content, buttons, callback_fn, args, buttons_delay_seconds);
            break;
        default:
            var message = "'" + display_type + "' not implemented";
            console.log(message);
            alert(message);
            break;
    }
}

function publish_user_response(value) {
    console.log("Publishing '" + value + "'");
    user_response_publisher.publish({ data: value });
}

function _publish_key_press(event) {
    var out_dict = { data: event.code };
    keypress_event_publisher.publish(out_dict);;
    console.log("Publishing keypress event: ", out_dict)
}

function _publish_mouse_event(event, is_click) {
    var normalized_pos = _get_normalized_mouse_position(event);
    var out_dict = {
        percentage_width_x: normalized_pos["x"],
        percentage_height_y: normalized_pos["y"],
        is_click: is_click
    }
    mouse_event_publisher.publish(out_dict);
    console.log("Publishing mouse event: ", out_dict);
}

function _get_normalized_mouse_position(event) {
    var height = $(document).height();
    var width = $(document).width();
    var x_pos = Math.min(100, Math.max(0, _round_percentage(event.clientX / width)));
    var y_pos = Math.min(100, Math.max(0, _round_percentage(event.clientY / height)));
    return {
        x: x_pos,
        y: y_pos
    };
}

function _round_percentage(value) {
    return Math.round(value * 100);
}