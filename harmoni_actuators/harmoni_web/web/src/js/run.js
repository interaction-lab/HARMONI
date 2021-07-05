var IS_DEBUG_DISPLAY = false;
var ROS_MASTER_URI = '';
var display_idx = 0;
var view = "test_container";
var show = false

if (!IS_DEBUG_DISPLAY) {
    rosInit(ROS_MASTER_URI);
} else {
    setup_cycle_through_displays(display_idx);
}

function btnChrome_onclick() {
    document.documentElement.webkitRequestFullScreen();
  }

btnChrome_onclick()