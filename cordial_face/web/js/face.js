
//------------------------------------------------------------------------------
// Face for SPRITE robot (to be run on mobile phone)
// Copyright (C) 2017 Elaine Schaertl Short
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//------------------------------------------------------------------------------



var listener,ros, latency_listener, latency_publisher;
var container, stats;

var camera, scene, renderer;
var tweens;

var  text, plane;

var parts;
var aus_l;
var aus_r;
var n_aus = 43;

var targetRotation = 0;
var targetRotationOnMouseDown = 0;

var mouseX = 0;
var mouseXOnMouseDown = 0;

var windowHalfX = window.innerWidth / 2;
var windowHalfY = window.innerHeight / 2;

var camera_depth = 550;

// Idle behavior
var last_blink;
var last_glance;
var looking;
var poked, poke_end, poke_time;
var blinking, blink_end, blink_time;

var background_color;
var cm_per_px;
var ros_master_uri;
var viseme_adjustment;
var viseme_buffer, viseme_time_buffer, viseme_dur, viseme_start;
var startup_time, prev_frame_time;

function startFace(bkgd_color,
           robot_name,
           ros_uri,
           cm_per_pixel,
           viseme_adj,
           eye_white_color,
           eye_iris_color,
           eye_size,
           eye_height,
           eye_separation,
           eye_iris_size,
           eye_pupil_scale,
           eye_pupil_shape,
           eyelid_width,
           eyelid_height,
           eyelid_arch,
           nose_color,
           nose_x,
           nose_y,
           nose_width,
           nose_height,
           mouth_color,
           mouth_x,
           mouth_y,
           mouth_width,
           mouth_height,
           mouth_thickness,
           mouth_opening,
           mouth_width_scale,
           upper_lip_height_scale,
           lower_lip_height_scale,
           brow_color,
           brow_width,
           brow_height,
           brow_thickness,
           brow_arch){
    d = new Date();
    startup_time = d.getTime()
    prev_frame_time = startup_time
    viseme_buffer = []
    viseme_time_buffer = []
    poked = false
    poke_start_time=0

    container = document.createElement( 'div' );
    document.body.appendChild( container );
    viseme_adjustment=viseme_adj
    // camera = new THREE.PerspectiveCamera( 50, window.innerWidth / window.innerHeight, 1, 2000 );
    camera = new THREE.OrthographicCamera(window.innerWidth/-2, window.innerWidth/2, window.innerHeight/2, window.innerHeight/-2, 1, 2000)

    camera.position.set( 0, -eye_height, camera_depth );

    scene = new THREE.Scene();
    parts = []

    background_color = bkgd_color;
    cm_per_px = cm_per_pixel;
    ros_master_uri = ros_uri;


    // addEyes(white_color, iris_color, size, height, separation, iris_size, pupil_scale, pupil_shape)
    addEyes(eye_white_color,
        eye_iris_color,
        eye_size,
        eye_height,
        eye_separation,
        eye_iris_size,
        eye_pupil_scale,
        eye_pupil_shape);

    // addLids(color, width, height, arch)
    addLids(background_color,
        eyelid_width,
        eyelid_height,
        eyelid_arch)

    // addNose(color, x, y, width, height)
    addNose(nose_color,
        nose_x,
        nose_y,
        nose_width,
        nose_height)

    //addNose(0x000000, 0,0,10,10)
    //addNose(0x000000, 0,100,10,10)

    // addMouth(color, x, y, width, height, thickness, opening, width_scale, ulip_h_scale, llip_h_scale)
    addMouth( mouth_color,
          mouth_x,
          mouth_y,
          mouth_width,
          mouth_height,
          mouth_thickness,
          mouth_opening,
          mouth_width_scale,
          upper_lip_height_scale,
          lower_lip_height_scale)

    // addBrows(color, width, height, thickness, arch)
    addBrows(brow_color,
         brow_width,
         brow_height,
         brow_thickness,
         brow_arch)

    last_blink = 0;
    last_gaze = 0;
    looking = false;

    aus_l = []
    aus_r = []
    for(i=0;i<=n_aus+1;i++){
    aus_l.push(0)
    aus_r.push(0)
    }

    lookat(0,0,500);
    zeroFace(500);

    renderer = new THREE.CanvasRenderer();
    renderer.setClearColor( background_color );
    renderer.setSize( window.innerWidth, window.innerHeight );
    renderer.sortElements = false;
    container.appendChild( renderer.domElement );

    document.addEventListener( 'mousedown', onDocumentMouseDown, false );
    document.addEventListener( 'touchstart', onDocumentTouchStart, false );
    document.addEventListener( 'touchmove', onDocumentTouchMove, false );

    //

    window.addEventListener( 'resize', onWindowResize, false );

    // Connecting to ROS
    // -----------------

    if(ros_master_uri == ""){
    ros_master_uri = "ws://" + location.hostname + ":9090"
    }
    console.log("ROS master URI: " + ros_master_uri)

    ros = new ROSLIB.Ros({
    url : ros_master_uri
    });

    ros.on('connection', function() {
    console.log('Connected to websocket server.');
    });

    ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', function() {
    console.log('Connection to websocket server closed.');
    });

    // Subscribing to a Topic
    // ----------------------

    listener = new ROSLIB.Topic({
    ros : ros,
    name : robot_name+'/face',
    messageType : 'cordial_face/FaceRequest'
    });

    listener.subscribe(get_goal);

    latency_listener = new ROSLIB.Topic({
    ros : ros,
    name : robot_name+'/latency/ROS',
    messageType : 'std_msgs/String'
    });
    latency_listener.subscribe(get_latency);

    latency_publisher = new ROSLIB.Topic({
    ros : ros,
    name : robot_name+'/latency/js',
    messageType : 'std_msgs/String'
    });

    animate();
}

function get_latency(msg){
    console.log(msg)
    d = new Date();
    time = d.getTime()-startup_time
    latency_publisher.publish({data:msg.data+":"+time})
}


function facePart(name, x, y, z, xs, ys, zs){
    this.name = name;
    this.idle_pos = {x:x, y:y, z:z};
    this.idle_rot = {x:0,y:0,z:0};
    this.idle_scale = {x:1,y:1,z:1};
    //below MUST be set after assembling the part
    this.idle_size = {x:0, y:0, z:0};

    this.threedee = new THREE.Object3D();
    this.threedee.position.x = x;
    this.threedee.position.y = y;
    if(typeof(z) == 'undefined'){
    this.threedee.position.z = 0;
    } else {
    this.threedee.position.z = z;
    }
    this.threedee.name = name
    parts.push(this)
    scene.add(this.threedee)
    this.rot = function(goal, t){
    TWEEN.remove(this.threedee.rotation);

    var goal = goal;
    var target = this.threedee.rotation;
    var tween = new TWEEN.Tween(target, {override:true}).to(goal,t);
    tween.easing(TWEEN.Easing.Quadratic.InOut);
    tween.start();
    }
    this.pos = function(goal, t){
    TWEEN.remove(this.threedee.position);

    var goal = goal;
    var target = this.threedee.position;
    var tween = new TWEEN.Tween(target, {override:true}).to(goal,t);
    tween.easing(TWEEN.Easing.Quadratic.InOut);
    tween.start();
    }
    this.scale = function(goal, t){
    TWEEN.remove(this.threedee.scale);

    var goal = goal;
    var target = this.threedee.scale;
    var tween = new TWEEN.Tween(target, {override:true}).to(goal,t);
    tween.easing(TWEEN.Easing.Quadratic.InOut);
    tween.start();
    }
    this.size = function(){
    var helper = new THREE.BoundingBoxHelper(this.threedee, 0xff0000);
    helper.update();
    var ret = new THREE.Vector3(helper.box.max.x-helper.box.min.x, helper.box.max.y-helper.box.min.y, helper.box.max.z-helper.box.min.z);
    return(ret);
    }

}

function getPart(name){
    for(i in parts){
    if(parts[i].name==name){
        return parts[i];
    }
    }
    return null;
}

function zeroFace(t){
    for(a in aus_l){
    au(a,0)
    }
    move_face(t)
}

function oneFace(t){
    for(a in aus_l){
    au(a,1.0)
    }
    move_face(t)
}


function lookat(x,y,z,t){
    //console.log("Looking at: " + x + "," + y + "," + z)

    var leye = getPart("leye").threedee

    var leyex = x-leye.position.x
    var leyey = y-leye.position.y
    var leyez = z-leye.position.z

    var xgoal = -Math.atan2(leyey, leyez)

    var ygoal = Math.atan2(leyex*Math.cos(xgoal), leyez)

    getPart("leye").rot({x: xgoal, y:ygoal}, t);

    var reye = getPart("reye").threedee

    var reyex = x-reye.position.x
    var reyey = y-reye.position.y
    var reyez = z-reye.position.z

    xgoal = -Math.atan2(reyey, reyez)
    ygoal = Math.atan2(reyex*Math.cos(xgoal), reyez)

    getPart("reye").rot({x: xgoal, y:ygoal}, t);
}

// for looking at things IRL; use avg velocity rather than time
// velocity is in radians/sec
function lookat_real_world(realx,realy,realz,vel){
    x = realx/cm_per_px;
    y = realy/cm_per_px;
    z = realz/cm_per_px;

    var leye = getPart("leye").threedee

    var leyex = x-leye.position.x
    var leyey = y-leye.position.y
    var leyez = z-leye.position.z

    var xgoal = -Math.atan2(leyey, leyez)
    var ygoal = Math.atan2(leyex*Math.cos(xgoal), leyez)

    var xdist = Math.abs(leye.rotation.x-xgoal)
    var ydist = Math.abs(leye.rotation.y-ygoal)

    var reye = getPart("reye").threedee

    var reyex = x-reye.position.x
    var reyey = y-reye.position.y
    var reyez = z-reye.position.z

    xgoal = -Math.atan2(reyey, reyez)
    ygoal = Math.atan2(reyex*Math.cos(xgoal), reyez)

    xdist = Math.max(Math.abs(reye.rotation.x-xgoal),xdist)
    ydist = Math.max(Math.abs(reye.rotation.y-ygoal),ydist)

    t = Math.max(xdist/vel, ydist/vel)

    lookat(x,y,z,t*1000)
}


function zero_aus(to_zero, t){
    for(a in to_zero){
    au(to_zero[a],0)
    }
    move_face(t)
}

function zero_aus_no_move(to_zero){
    for(a in to_zero){
    au(to_zero[a],0)
    }
}

function play_visemes(visemes, time_per, times){
    viseme_buffer = visemes
    viseme_time_buffer = times
    viseme_dur = time_per
    d = new Date();
    viseme_start = d.getTime();
}

function check_and_play_visemes(){
    if(viseme_buffer.length==0){return}
    d = new Date();
    elapsed = d.getTime()-viseme_start+viseme_dur;
    elapsed=elapsed/1000.0
    if(viseme_time_buffer[0]>elapsed){return};
    while(viseme_time_buffer[0]<elapsed){
    play_viseme = viseme_buffer.shift()
    viseme_time = viseme_time_buffer.shift()
    }
    console.log(elapsed, viseme_time)
    viseme(play_viseme,viseme_dur);
}


function old_visemes(visemes, time_per, times, start){
    please_stop = false
    adj = 1.0-viseme_adjustment
    d = new Date();
    start = d.getTime();
    i = 0
    interval_ms = 10
    function check_and_play(){
    d = new Date();
    elapsed = d.getTime()-start;
    elapsed=elapsed/1000.0
    if(times[i]/adj<=elapsed){
        viseme(visemes[i],time_per);
        i++;
        while(times[i]/adj<elapsed+interval_ms/1000){i++};
    }
    if(i >= visemes.length || please_stop){return}
    else {setTimeout(check_and_play, interval_ms)};
    }
    setTimeout(check_and_play, interval_ms)
}

function stop_visemes(){
    please_stop = true
}

function viseme(viseme_name, t){
    console.log("Viseme: " + viseme_name)
    switch(viseme_name){

    case "M_B_P": //au 23, 24?, 14?,
    au(23, .75)
    au(14, .25)

    zero_aus_no_move([10,11,16,17,25,26,27])
    move_face(t)
    break;

    case "AA_AH": //au 25, 26, 14
    au(26, 1)
    au(25, .5)
    au(14, .5)

    zero_aus_no_move([11,12,16,17,18,20,27])
    move_face(t)
    break;

    case "AO_AW": //au 25, 26, 27
    au(26, .75)
    au(27, 1)
    zero_aus_no_move([11,12,16,17,18,20])
    move_face(t)


    break;

    case "EH_AE_AY": //au 25, 26, 14
    au(14, .75)
    au(26, .75)

    zero_aus_no_move([11,12,16,17,18,20,27])
    move_face(t)

    break;

    case "CH_SH_ZH": //au 18, 25, 10
    au(10, .75)
    au(18, 1)
    zero_aus_no_move([11,12,14,16,17,20,26,27])
    move_face(t)
    break;

    case "N_NG_D_Z": //au 10,
    au(10,.6)
    au(18,.5)

    zero_aus_no_move([11,16,17,25,26,27])
    move_face(t)

    break;

    case "R_ER": //au 10
    au(10,1)
    au(18,.7)

    zero_aus_no_move([11,12,14,16,17,20,26,27])
    move_face(t)

    break;

    case "EY": //au 25, 26, 14
    au(26,1)
    zero_aus_no_move([11,12,16,17,18,20,27])
    move_face(t)
    break;

    case "L": //au 25
    au(10,.5)
    au(18,.5)

    zero_aus_no_move([11,12,14,16,17,20,26,27])
    move_face(t)

    break;

    // "you" "too" "moo"
    case "OO": //au 10, 25,
    au(10,1)
    au(25,1)
    au(18,1)

    zero_aus_no_move([11,12,14,16,17,20,26,27])
    move_face(t)

    break;

    // --------------- CONSONANTS ---------------------//

    // M,B,P -> My, Buy, Pie
    case 'BILABIAL':
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(23, .75)
    au(14, .25)
    au(24, .7)

    move_face(t)
    break;

    // F,V -> oFFer, Vest
    case "LABIODENTAL":
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(10,0.5);
    au(20,0.4);
    au(25,.8);

    move_face(t)
    break;

    // TH, TH - THin, THis
    case "INTERDENTAL":
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(10,.6)
    au(18,.75)
    au(25,.5)

    move_face(t)
    break;

    // L,T,D,Z,S,N -> Light, Top, DaD, Zebra, Sad, Nope
    case "DENTAL_ALVEOLAR":
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(25,.65)

    move_face(t)
    break;

    // R,SH,ZH,CH -> Red, SHould, aSia, CHart
    case "POSTALVEOLAR":
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(10, .75)
    au(18, 1)
    au(25, 1)

    move_face(t)
    break;

    // K,G,NG -> Cat, Game, thiNG
    case "VELAR_GLOTTAL":
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(10,.6)
    // au(18,.5)
    au(26,.5)

    move_face(t)
    break;

    // ------------------ VOWELS ------------------------//
    // EE, I -> flEEce, bIt
    case "CLOSE_FRONT_VOWEL":
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(26,1)
    au(20,1)
    au(10,.4)
    move_face(t)
    break;

    // OO -> bOOt
    case "CLOSE_BACK_VOWEL":
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(10,.5)
    au(13,.8)
    au(16,.6)
    au(18,1)
    au(23,1)
    au(24,1)
    au(25,1)
    au(26,.4)

    move_face(t)

    break;

    // schwa -> ArenA
    case "MID_CENTRAL_VOWEL":
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(26, 1)
    au(25, .5)
    au(23,1)

    move_face(t)
    break;

    // AE,AU,A,AY,EH -> trAp, mOUth, fAther, fAce, drEss
    case "OPEN_FRONT_VOWEL":
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(14, 1)
    au(20, 1)
    au(25, .7)
    au(26, .75)

    move_face(t)
    break;

    // AW,OI,O -> thOUght, chOIce, gOAt
    case "OPEN_BACK_VOWEL":
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(26, .5)
    au(27, 1)

    move_face(t)
    break;
    
    case "IDLE":
    zeroFace(t)
    break;
    }
}

function blink(t){
    au(43, 1.0, t/2);
    blink_time=t

    d = new Date()
    blink_end = d.getTime()+t/2;
    blinking=true
}


function au(id, degree, side){
    //console.log("Playing AU: " + id + " degree: " + degree + " time: " + t + " side: " + side);

    if(side == "r"){
    aus_r[id] = degree
    } else if(side == "l"){
    aus_l[id] = degree
    } else{
    aus_l[id] = degree
    aus_r[id] = degree
    }
}

function move_face(t){
    // ***** BROWS *****

    //brow rotation (au 1)
    brow_rot_targetl = (Math.PI/8)*aus_l[1];
    brow_rot_targetr = (Math.PI/8)*aus_r[1];
    rbrow = getPart("rbrow");
    lbrow = getPart("lbrow");

    rbrow.rot({z:rbrow.idle_rot.z+brow_rot_targetr}, t);
    lbrow.rot({z:lbrow.idle_rot.z-brow_rot_targetl}, t);

    //brow height (aus 2,4)
    brow_raise_targetr = 1 + .75*aus_r[2] - .5*aus_r[4]
    brow_raise_targetl = 1 + .75*aus_l[2] - .5*aus_l[4]

    rbrow.scale({y:rbrow.idle_scale.y*brow_raise_targetr},t);
    lbrow.scale({y:lbrow.idle_scale.y*brow_raise_targetl},t);

    //brow lowering (au 4)
    brow_height_targetl = -20*aus_l[4]
    brow_height_targetr = -20*aus_r[4]

    rbrow.pos({y:rbrow.idle_pos.y+brow_height_targetr}, t);
    lbrow.pos({y:lbrow.idle_pos.y+brow_height_targetl}, t);


    // ***** EYELIDS ******
    closure = -.5
    urlid = getPart("urlid");
    ullid = getPart("ullid");
    lrlid = getPart("lrlid");
    lllid = getPart("lllid");


    // eyelid flattening (au 7)
    lid_flattenr = .6*aus_r[7]
    lid_flattenl = .6*aus_l[7]

    lrlid.scale({y:lrlid.idle_scale.y*(1-lid_flattenr)},t);
    lllid.scale({y:lllid.idle_scale.y*(1-lid_flattenl)},t);

    // eyelid closing (aus 5,7,43)
    urlid_p = urlid.idle_pos.y;
    lrlid_p = lrlid.idle_pos.y;
    ullid_p = ullid.idle_pos.y;
    lllid_p = lllid.idle_pos.y;
    lid_width = ullid.idle_size.x/4;

    r_eye_width = (urlid_p-lrlid_p)+urlid.threedee.scale.y*lid_width+lrlid.threedee.scale.y*lid_width;
    l_eye_width = (ullid_p-lllid_p)+ullid.threedee.scale.y*lid_width+lllid.threedee.scale.y*lid_width;

    //[-.5,0]
    openr = -.5*aus_r[5]
    openl = -.5*aus_l[5]

    //[0,.6]
    closer = .6*aus_r[7]
    closel = .6*aus_l[7]

    //[-.5,1]
    closurer = (openr+closer)+aus_r[43]*(1-(openr+closer))
    closurel = (openl+closel)+aus_l[43]*(1-(openl+closel))

    urlid.pos({y:urlid_p-r_eye_width/2*(closurer)},t);
    lrlid.pos({y:lrlid_p+r_eye_width/2*(closurer)},t);
    ullid.pos({y:ullid_p-l_eye_width/2*(closurel)},t);
    lllid.pos({y:lllid_p+l_eye_width/2*(closurel)},t);

    // ***** NOSE *****

    // nose wrinkle (raise) (au 9)
    wrinkle_dist = 30*aus_l[9];
    nose = getPart("nose");
    nose.pos({y:nose.idle_pos.y+wrinkle_dist}, t);

    // nose width (aus 38,39)
    width_scale = 1+.35*aus_l[38]-.35*aus_l[39];
    nose = getPart("nose");
    nose.scale({x:nose.idle_scale.x*width_scale}, t);


        var circleShape = new THREE.Shape();
        circleShape.moveTo(0,0)
        circleShape.arc(0,0,20,0,6.6, true)

    // rbrow.threedee.children[0].geometry.vertices = new THREE.ShapeGeometry( circleShape )
        // rbrow.threedee.children[0].geometry.verticesNeedUpdate = true
        // rbrow.threedee.children[0].geometry.elementsNeedUpdate = true;
        // rbrow.threedee.children[0].geometry.morphTargetsNeedUpdate = true;
        // rbrow.threedee.children[0].geometry.uvsNeedUpdate = true;
        // rbrow.threedee.children[0].geometry.normalsNeedUpdate = true;
        // rbrow.threedee.children[0].geometry.colorsNeedUpdate = true;
        // rbrow.threedee.children[0].geometry.tangentsNeedUpdate = true;



    // ***** MOUTH *****
    ulip=getPart("ulip");
    llip=getPart("llip");

    max_up_dist = (nose.idle_pos.y-ulip.idle_pos.y)/1.5
    max_down_dist = (nose.idle_pos.y-llip.idle_pos.y)/1.5

    // upper lip position (aus 10, 11)

    upper_dist = max_up_dist*(.55*aus_l[10]+.25*aus_l[11]+.35*aus_l[17]+.2*aus_l[25]+.1*aus_l[26]+.75*aus_l[27])/2.2

    // upper lip width (aus 11, 12, 14)

    upper_w_scale = 1+.1*aus_l[11]+.2*aus_l[12]+.25*aus_l[14]+.25*aus_l[17]-.7*aus_l[18]+.35*aus_l[20]

    // upper lip height (aus 10, 11, 12)

    upper_h_scale = 1-1.5*aus_l[10]-.2*aus_l[11]+.2*aus_l[12]+.9*aus_l[13]-.3*aus_l[14]-2*aus_l[15]-aus_l[17]-.25*aus_l[20]-.5*aus_l[23]-.5*aus_l[24]-.25*aus_l[27]

    // lower lip position

    lower_dist = max_down_dist*(-.5*aus_l[16]+.35*aus_l[17]-.2*aus_l[25]-.5*aus_l[26]-.75*aus_l[27])/2.3

    // lower lip width (aus 12, 14)

    lower_w_scale = 1+.2*aus_l[12]+.25*aus_l[14]+.25*aus_l[16]+.25*aus_l[17]-.7*aus_l[18]+.35*aus_l[20]

    // lower lip height (aus 12, 13, 14)

    lower_h_scale = 1+.2*aus_l[12]+.9*aus_l[13]-.3*aus_l[14]-2*aus_l[15]-aus_l[17]-.25*aus_l[20]-.5*aus_l[23]-.5*aus_l[24]-.25*aus_l[27]


    //adjust zero scales:
    if(upper_w_scale == 0) upper_w_scale += 0.001;
    if(upper_h_scale == 0) upper_h_scale += 0.001;
    if(lower_w_scale == 0) lower_w_scale += 0.001;
    if(lower_h_scale == 0) lower_h_scale += 0.001;

    // move mouth
    ulip.scale({x:ulip.idle_scale.x*upper_w_scale, y:ulip.idle_scale.y*upper_h_scale},t);
    llip.scale({x:llip.idle_scale.x*lower_w_scale, y:llip.idle_scale.y*lower_h_scale},t);
    ulip.pos({y:ulip.idle_pos.y+upper_dist},t);
    llip.pos({y:llip.idle_pos.y+lower_dist},t);

        console.log(rbrow.threedee.children[0].geometry)

}



function addEyes(white_color, iris_color, size, height, separation, iris_size, pupil_scale, pupil_shape){
    var circleRadius = iris_size;
    var circleShape = new THREE.Shape();
    circleShape.moveTo(0,0)
    circleShape.arc(0,0,iris_size,0,6.6, true)

    if(pupil_shape=="round"){
    pupilShape = circleShape;
    } else if(pupil_shape="cat"){
    pupilShape = new THREE.Shape();
    pupilShape.moveTo(0,0);
    //pupilShape.arc(0,0,iris_size,0,6.6, true);
    pupilShape.ellipse(0,0,iris_size/3,iris_size,0,6.6,true);
    } else {
    pupilShape = circleShape;
    }

    var x_adj = (separation)*(size/camera_depth);
    var y_adj = height * (size/camera_depth);

     reye = new facePart("reye", -(separation/2)-x_adj, y_adj, -size);
     addSphere(reye.threedee, size, white_color,0 , 0, 0, 0, 0, 0, 1 );
     addShape(reye.threedee,circleShape, iris_color, 0, 0, size, 0, 0, 0, 1 );
     addShape(reye.threedee,pupilShape, 0x000000, 0, 0, size, 0, 0, 0, pupil_scale );
     reye.idle_size = reye.size()
    
     leye = new facePart("leye", (separation/2)+x_adj, y_adj, -size);
     addSphere(leye.threedee, size, white_color, 0, 0, 0, 0, 0, 0, 1 );
     addShape(leye.threedee,circleShape, iris_color, 0, 0, size, 0, 0, 0, 1 );
     addShape(leye.threedee,pupilShape, 0x000000, 0, 0, size, 0, 0, 0, pupil_scale );
     leye.idle_size = leye.size()
}


function addMouth(color, x, y, width, height, thickness, opening, width_scale, ulip_h_scale, llip_h_scale){

    var arcShape = new THREE.Shape();

    arcShape.absarc( 0, width, width, 0, -3.23, true );
    arcShape.absarc( 0, width, width, -.15, -3.23, false );

    llip = new facePart("llip", x,y-(height-width));
    addLine(llip.threedee, arcShape, color, thickness, 0,0,0,0,0,.11,1);

    ulip = new facePart("ulip", x,y);
    addLine(ulip.threedee, arcShape, color, thickness, 0,0,0,0,0,.11,1);

    mouth_fix = llip.idle_pos.y + (ulip.idle_pos.y-llip.idle_pos.y)/2;

    ulip.idle_scale.x = width_scale;
    llip.idle_scale.x = width_scale;
    ulip.idle_scale.y = ulip_h_scale;
    llip.idle_scale.y = llip_h_scale;

    ulip.idle_pos.y = mouth_fix+opening/2;
    llip.idle_pos.y = mouth_fix-opening/2;

    ulip.idle_size.x = width_scale*ulip.size().x;
    ulip.idle_size.y = ulip_h_scale*ulip.size().y;
    llip.idle_size.x = width_scale*llip.size().x;
    llip.idle_size.y = llip_h_scale*llip.size().y;

}


function addNose(color, x, y, width, height){
    var xc = width/2;
    var yc = height/2

    var triangleShape = new THREE.Shape();
    triangleShape.moveTo(  -xc, yc );
    triangleShape.lineTo(  xc, yc );
    triangleShape.lineTo( 0, -yc );
    triangleShape.lineTo(  -xc, yc );

    nose = new facePart("nose", x, y);
    addShape(nose.threedee, triangleShape, color, 0,0,0,0,0,0,1);
    nose.idle_size = nose.size();
}

function addLids(color, width, height, arch){
    var arcShape = new THREE.Shape();

    arcShape.moveTo( -width,-width*10);
    arcShape.moveTo( -2*width,-width*10);
    arcShape.moveTo( -2*width, width*10);
    arcShape.moveTo( 2*width, width*10);
    arcShape.moveTo( 2*width, -width*10);
    arcShape.moveTo( width, -width*10);

    arcShape.absarc( 0, 0, width, 0, 3.23, true );
    //arcShape.absarc( 0, 0, width, -.15, 3.23, false );

    var leye = getPart("leye");
    var reye = getPart("reye");

    //var xl = (camera_depth)*leye.threedee.position.x/((camera_depth)-leye.threedee.position.z);
    //var xr = camera_depth*reye.threedee.position.x/(camera_depth-reye.threedee.position.z);

    var xl = leye.threedee.position.x
    var xr = reye.threedee.position.x

    var y = height //+ (camera_depth*leye.threedee.position.y/(camera_depth-leye.threedee.position.z));

    ullid = new facePart("ullid", xl,y)
    addShape(ullid.threedee, arcShape, color,0,0,0,0,0,0,1);

    urlid = new facePart("urlid", xr,y)
    addShape(urlid.threedee, arcShape, color, 0,0,0,0,0,0,1);

    ullid.scale({y:arch}, 1);
    urlid.scale({y:arch}, 1);

    arcShape = new THREE.Shape();

    arcShape.moveTo( -width,width*10);
    arcShape.moveTo( -2*width,width*10);
    arcShape.moveTo( -2*width, -width*10);
    arcShape.moveTo( 2*width, -width*10);
    arcShape.moveTo( 2*width, width*10);
    arcShape.moveTo( width, width*10);

    arcShape.absarc( 0, 0, width, 0, -3.23, true );

    y = y - 2*height;

    lllid = new facePart("lllid", xl,y)
    addShape(lllid.threedee, arcShape, color,0,0,0,0,0,0,1);

    lrlid = new facePart("lrlid", xr,y)
    addShape(lrlid.threedee, arcShape, color, 0,0,0,0,0,0,1);

    lllid.scale({y:arch}, 1);
    lrlid.scale({y:arch}, 1);

    //lid_width = width;
    ullid.idle_scale.y = arch;
    urlid.idle_scale.y = arch;
    lllid.idle_scale.y = arch;
    lrlid.idle_scale.y = arch;

    ullid.idle_size = ullid.size();
    urlid.idle_size = urlid.size();
    lllid.idle_size = lllid.size();
    lrlid.idle_size = lrlid.size();
}


function addBrows(color, width, height, thickness, arch){

      //original arc eyebrow
        var arcShape = new THREE.Shape();

    arcShape.absarc( 0, 0, width, -.15, 3.23, false );
        arcShape.absarc( 0, 0, width, 0, 3.23, true );

        //left eyebrow shape
        var leftBrowShape = new THREE.Shape();
        var upperPoints = [new THREE.Vector2(0, 140), new THREE.Vector2(width+45, 30)];
        var lowerPoints = [new THREE.Vector2(0, 80), new THREE.Vector2(-width,0)];

        leftBrowShape.moveTo(-width-20,0);
        leftBrowShape.quadraticCurveTo(-width-30, 0, -width-45, 60);
        leftBrowShape.splineThru(upperPoints);
        leftBrowShape.quadraticCurveTo(width+50, 20, width+45, 10);
        leftBrowShape.splineThru(lowerPoints);

        //right eyebrow shape
        var rightBrowShape = new THREE.Shape();
        var upperPoints = [new THREE.Vector2(0, 140), new THREE.Vector2(-width-45, 30)];
        var lowerPoints = [new THREE.Vector2(0, 80), new THREE.Vector2(width,0)];

        rightBrowShape.moveTo(width+10,0);
        rightBrowShape.quadraticCurveTo(width+30, 0, width+45, 60);
        rightBrowShape.splineThru(upperPoints);
        rightBrowShape.quadraticCurveTo(-width-50, 20, -width-45, 10);
        rightBrowShape.splineThru(lowerPoints);


    var leye = getPart("leye");
    var reye = getPart("reye");

    //var xl = camera_depth*leye.threedee.position.x/(camera_depth-leye.threedee.position.z);
    //var xr = camera_depth*reye.threedee.position.x/(camera_depth-reye.threedee.position.z);
    var xl = leye.threedee.position.x
    var xr = reye.threedee.position.x

    var y = height //+ (camera_depth*leye.threedee.position.y/(camera_depth-leye.threedee.position.z));

        lbrow = new facePart("lbrow", xl,y)
    addLine(lbrow.threedee, arcShape, color, thickness, 0,0,0,0,0,.11,1);

    rbrow = new facePart("rbrow", xr,y)
        addLine(rbrow.threedee, arcShape, color, thickness, 0,0,0,0,0,.11,1);

    lbrow.scale({y:arch}, 1);
    rbrow.scale({y:arch}, 1);

    lbrow.idle_scale.y = arch;
    rbrow.idle_scale.y = arch;

    lbrow.idle_size = lbrow.size();
    rbrow.idle_size = rbrow.size();

}

function addLine(threedee, shape, color, width,  x, y, z, rx, ry, rz, s) {
    var geometry = shape.createPointsGeometry();
    var material = new THREE.LineBasicMaterial( { linewidth:width, color: color, transparent: true } );

    var line = new THREE.Line( geometry, material);
    line.position.set( x, y, z );
    line.rotation.set( rx, ry, rz );
    line.scale.set( s, s, s );
    threedee.add( line );
}


function addShape( threedee, shape, color, x, y, z, rx, ry, rz, s ) {
    // flat shape
    var geometry = new THREE.ShapeGeometry( shape );
    var material = new THREE.MeshBasicMaterial( { color: color, overdraw: 0.5 } );

    var mesh = new THREE.Mesh( geometry, material );
    mesh.position.set( x, y, z );
    mesh.rotation.set( rx, ry, rz );
    mesh.scale.set( s, s, s );
    threedee.add( mesh );

    // line
    // addLine(threedee, shape, 0x333333, 5, x, y, z, rx, ry, rz, s);
}

function addSphere( threedee, radius, color, x, y, z, rx, ry, rz, s ) {
    var geometry = new THREE.SphereGeometry( radius , 15, 15);
    var material = new THREE.MeshBasicMaterial( { color: color, overdraw: 0.5 } );
    var mesh = new THREE.Mesh( geometry, material );
    mesh.position.set( x, y, z );
    mesh.rotation.set( rx, ry, rz );
    mesh.scale.set( s, s, s );
    threedee.add( mesh );
}

/**

   Moving the face

**/

function get_goal(message) {
    if(message.visemes.length!=0){
    console.log('Message received: visemes: ' + message.visemes);
    stop_visemes()
    play_visemes(message.visemes, message.viseme_ms, message.times, message.start)
    }
    if(message.aus.length!=0){
    console.log('Message received: aus: ' + message.aus + " degrees: " + message.au_degrees + " side: " + message.side)
    side = "b"
    if(message.side == 1){
        side = 'r'
    }
    if(message.side == 2){
        side = 'l'
    }
    for(a in message.aus){
        this_au = parseInt(message.aus[a])
        if(message.au_ms[a]<0){
        console.log("Time cannot be less than zero!")
        } else {
        if(this_au == 1 ||this_au == 4||this_au==2||this_au==5 || this_au == 7 || this_au == 43){
            au(this_au, message.au_degrees[a], side)
        }
        else {
            au(this_au, message.au_degrees[a], "b")
        }
        }
    }
    move_face(message.au_ms)
    }
    if(message.hold_gaze==1){
    looking = true
    }
    if(message.hold_gaze==2){
    looking = false
    }
    if(message.retarget_gaze){
    console.log("Message received: gaze: " + message.gaze_target.x + "," + message.gaze_target.y + "," +  message.gaze_target.z)
    x = message.gaze_target.x
    y = message.gaze_target.y
    z = message.gaze_target.z
    gaze_vel = message.gaze_vel
    if(gaze_vel > 0){
        lookat_real_world(x, y, z, gaze_vel)
    }
    else {
        lookat_real_world(x, y, z, 1.7) //1.7 rad/s is an average human eye saccade speed
    }
    }
}


function doIdle(){

    var d = new Date();
    var now = d.getTime();
    if(poked){
    if(now-poke_end >= 0){
        zeroFace(poke_time/2)
        poked=false
    }
    }

    if(blinking){
    if(now-blink_end >= 0){
        au(43, 0)
        move_face(blink_time/2)
        blinking=false
    }
    }
    if((Math.floor((Math.random() * 500))==0 && now-last_blink > 2000)|| now-last_blink > 8000){
    blink(300);
    last_blink = now;
    }


    if(!looking){
    var xrange = windowHalfX;
    var yrange = windowHalfY;
    var zrange = 1000;

    if((Math.floor((Math.random() * 500))==0 && now-last_gaze > 2000)|| now-last_gaze > 5000){
        var xgoal = -xrange/2 + Math.floor((Math.random() * xrange));
        var ygoal = -yrange/2 + Math.floor((Math.random() * yrange));
        var zgoal = 2000;
        lookat(xgoal,ygoal,zgoal, 700);
        last_gaze = now
    }
    }
}

function doPoke(){
    poke_time=600
    au(9,1);
    au(43,1);
    au(18,1);
    move_face(300)

    d = new Date()
    poke_end = d.getTime()+500;
    poked=true;
}



function onWindowResize() {
    //TODO: make this look good

    windowHalfX = window.innerWidth / 2;
    windowHalfY = window.innerHeight / 2;

    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();

    renderer.setSize( window.innerWidth, window.innerHeight );

}

//


function onDocumentMouseMove( event ) {
/**
    mouseX = event.clientX - windowHalfX;

    targetRotation = targetRotationOnMouseDown + ( mouseX - mouseXOnMouseDown ) * 0.02;
    //update_goal((event.clientX)/100)
**/
}

function onDocumentMouseDown( event ) {

    event.preventDefault();

    //document.addEventListener( 'mousemove', onDocumentMouseMove, false );
    //document.addEventListener( 'mouseup', onDocumentMouseUp, false );
    //document.addEventListener( 'mouseout', onDocumentMouseOut, false );

    mouseX = event.clientX - windowHalfX;
    mouseY = windowHalfY - event.clientY;
    clickOrTouch(mouseX, mouseY)

}

function clickOrTouch( x, y) {
    if (typeof gui === "undefined"){
        doPoke();
    }
    lookat_real_world(0, 0, 60, 1.7);
}


function onDocumentMouseUp( event ) {
/**
    document.removeEventListener( 'mousemove', onDocumentMouseMove, false );
    document.removeEventListener( 'mouseup', onDocumentMouseUp, false );
    document.removeEventListener( 'mouseout', onDocumentMouseOut, false );
**/
}

function onDocumentMouseOut( event ) {
/**
    document.removeEventListener( 'mousemove', onDocumentMouseMove, false );
    document.removeEventListener( 'mouseup', onDocumentMouseUp, false );
    document.removeEventListener( 'mouseout', onDocumentMouseOut, false );
**/
}

function onDocumentTouchStart( event ) {
/**
    if ( event.touches.length == 1 ) {

    event.preventDefault();

    mouseXOnMouseDown = event.touches[ 0 ].pageX - windowHalfX;
    update_goal((event.touches[ 0 ].pageX)/100)
    targetRotationOnMouseDown = targetRotation;

    }
**/
    mouseX = event.touches[0].pageX - windowHalfX;
    mouseY = windowHalfY - event.touches[0].pageY;
    clickOrTouch(mouseX, mouseY)


}

function onDocumentTouchMove( event ) {
/**
    if ( event.touches.length == 1 ) {

    event.preventDefault();

    mouseX = event.touches[ 0 ].pageX - windowHalfX;
    update_goal((event.touches[ 0 ].pageX)/100)
    targetRotation = targetRotationOnMouseDown + ( mouseX - mouseXOnMouseDown ) * 0.05;

    }
**/
}

//

function print_elapsed() {
    d = new Date();
    time  = d.getTime()
    elapsed = time-prev_frame_time;
    console.log(elapsed)
}

function set_time(){
    d = new Date();
    time  = d.getTime()
    prev_frame_time=time
}

function animate() {
    //print_elapsed()
    //set_time()
    requestAnimationFrame( animate );
    //set_time()
    doIdle();
    //print_elapsed()
    check_and_play_visemes()
    TWEEN.update();
    renderer.render( scene, camera );
    //print_elapsed()
    //set_time()
}
