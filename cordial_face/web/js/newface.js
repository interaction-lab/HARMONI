
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

var upperLipControlPoints = []; lowerLipControlPoints = [];
var lBrowControlPoints = []; rBrowControlPoints = [];

var lockIdleUntil = 0; //variable to lock idling, if a behavioral blink is occuring

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

/*
This is the first function that is called by the HTML Script.
It is responsible for the creation of all the face parts, and defines color, sizes, orientations, etc.
**See kiwi.html or chili.html for types of the input arguments.**
*/
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
       mouth_dimple_size,
       upper_lip_height_scale,
       lower_lip_height_scale,
       brow_color,
       brow_width,
       brow_height,
       brow_thickness,
       brow_innersize){

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
          var SCALING_FACTOR = 1.2
          camera = new THREE.OrthographicCamera(window.innerWidth/-2 * SCALING_FACTOR,
            window.innerWidth/2 * SCALING_FACTOR, window.innerHeight/2, window.innerHeight/-2 * SCALING_FACTOR, 300, 1000)

          camera.position.set( 0, window.innerHeight/4*(1- SCALING_FACTOR), camera_depth );

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

          // addMouth(color, x, y, width, height, thickness, opening, dimple_size, ulip_h_scale, llip_h_scale)
          addMouth( mouth_color,
              mouth_x,
              mouth_y,
              mouth_width,
              mouth_height,
              mouth_thickness,
              mouth_opening,
              mouth_dimple_size,
              upper_lip_height_scale,
              lower_lip_height_scale)

          // addBrows(color, width, height, thickness, arch)
          addBrows(brow_color,
             brow_width,
             brow_height,
             brow_thickness,
             brow_innersize)

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

          renderer = new THREE.WebGLRenderer({antialias:true, precision:'lowp'});
          renderer.setClearColor( background_color );
          renderer.setSize( window.innerWidth, window.innerHeight + 30);
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

          //finally, start the animation loop.
          animate();
}

function get_latency(msg){
    console.log(msg)
    d = new Date();
    time = d.getTime()-startup_time
    latency_publisher.publish({data:msg.data+":"+time})
}

/*
Create a new face part object that tracks the metadata for the THREE.Object3D, which is the part that is actually drawn.
Lip objects are created with a different constructor that enables control points rather than scale, rotate, translate
name - a string that represents the name of the face part
x - float representing x offset
y - float representing y offset
z - float representing z offset
*/
function facePart(name, x, y, z){
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
      var box = new THREE.Box3().setFromObject( this.threedee );
      var ret = new THREE.Vector3();
      box.getSize(ret);
      return ret;
    }

}

/*
  Constructs a set of lips from give control points, and places them at the given xx,y values.
  name - a string - the name (either ulip or llip)
  x - an integer - x offset
  y - an integer - y offset
  control points  - an array of 4 vector3's - defines mouth control points from right to left
*/
function constructLipPoints(name, x, y, controlPoints){
  this.name = name;
  this.id = name == 'llip' ? 0 : 1;
  this.idle_pos = {x:x, y:y, z:33};

  this.threedee = new THREE.Object3D();
  this.threedee.position.x = x;
  this.threedee.position.y = y;

  this.points= {x0:controlPoints[0].x, y0:controlPoints[0].y,
                x1:controlPoints[1].x, y1:controlPoints[1].y,
                x2:controlPoints[2].x, y2:controlPoints[2].y,
                x3:controlPoints[3].x, y3:controlPoints[3].y, lip:this.id};

  this.threedee.name = name;
  parts.push(this);
  scene.add(this.threedee);


  this.moveToLocation = function(newPoints, time){
    var goal =   {x0:newPoints[0].x, y0:newPoints[0].y,
                  x1:newPoints[1].x, y1:newPoints[1].y,
                  x2:newPoints[2].x, y2:newPoints[2].y,
                  x3:newPoints[3].x, y3:newPoints[3].y, lip:this.id};
    var tween = new TWEEN.Tween(this.points).to(goal, time);
    tween.easing(TWEEN.Easing.Quadratic.InOut);
    tween.onUpdate(function() {
         drawMouth(this.x0, this.y0, this.x1, this.y1, this.x2, this.y2, this.x3, this.y3, this.lip)
          });
    tween.start();
    }
  }

/*
This is a helper method to update the geometry of one of the lips.
This creates a Catmull Rom spline that interpolates between the control points.
x0,y0,...x3,y3 - floats that represent the location of the control points in local space (relative to the lip object)
id - in integer that is either 0 for lower lip or 1 for upper lip
*/
function drawMouth(x0,y0,x1,y1,x2,y2,x3,y3, id){
  var lipObject = getPart(id == 0 ? 'llip' : 'ulip')
  var pointsArray = [new THREE.Vector3(x0,y0,0), new THREE.Vector3(x1,y1,0),
                     new THREE.Vector3(x2,y2,0), new THREE.Vector3(x3,y3,0)];

  var curve = new THREE.CatmullRomCurve3( pointsArray);
  curve.curveType = 'catmullrom';

  var splinePoints = curve.getPoints( 10 );
  var splineGeometry = new THREE.Geometry().setFromPoints( splinePoints );
  var line = new MeshLine();
  line.setGeometry( splineGeometry);

  lipObject.threedee.children[0].geometry = line.geometry;
  if(id == 0){
    //move the circles at the ends of the lips
    lipObject.threedee.children[1].position.x = x3
    lipObject.threedee.children[1].position.y = y3
    lipObject.threedee.children[2].position.x = x0
    lipObject.threedee.children[2].position.y = y0
  }
}

/*
  Constructs a set of lips from give control points, and places them at the given xx,y values.
  name - a string - the name (either ulip or llip)
  x - an integer - x offset
  y - an integer - y offset
  control points  - an array of 4 vector3's - defines mouth control points from right to left
*/
function constructBrowPoints(name, x, y, controlPoints){
  this.name = name;
  this.id = name == 'lbrow' ? 0 : 1;
  this.idle_pos = {x:x, y:y, z:33};

  this.threedee = new THREE.Object3D();
  this.threedee.position.x = x;
  this.threedee.position.y = y;

  this.points= {x0:controlPoints[0].x, y0:controlPoints[0].y,
                x1:controlPoints[1].x, y1:controlPoints[1].y,
                x2:controlPoints[2].x, y2:controlPoints[2].y, brow:this.id};

  this.threedee.name = name;
  parts.push(this);
  scene.add(this.threedee);


  this.moveToLocation = function(newPoints, time){
    var goal =   {x0:newPoints[0].x, y0:newPoints[0].y,
                  x1:newPoints[1].x, y1:newPoints[1].y,
                  x2:newPoints[2].x, y2:newPoints[2].y, brow:this.id};
    var tween = new TWEEN.Tween(this.points).to(goal, time);
    tween.easing(TWEEN.Easing.Quadratic.InOut);
    tween.onUpdate(function() {
         drawBrows(this.x0, this.y0, this.x1, this.y1, this.x2, this.y2, this.brow)
          });
    tween.start();
    }
  }

  /*
  This is a helper method to update the geometry of one of the lips.
  This creates a Catmull Rom spline that interpolates between the control points.
  x0,y0,...x3,y3 - floats that represent the location of the control points in local space (relative to the lip object)
  id - in integer that is either 0 for lower lip or 1 for upper lip
  */
  function drawBrows(x0,y0,x1,y1,x2,y2, id){
    var browObject = getPart(id == 0 ? 'lbrow' : 'rbrow')
    var pointsArray = [new THREE.Vector3(x0,y0,0), new THREE.Vector3(x1,y1,0),
                       new THREE.Vector3(x2,y2,0)];

    var curve = new THREE.CatmullRomCurve3( pointsArray);
    curve.curveType = 'catmullrom';

    var splinePoints = curve.getSpacedPoints( 50 );
    var splineGeometry = new THREE.Geometry().setFromPoints( splinePoints );
    var line = new MeshLine();
    line.setGeometry( splineGeometry, function(p){return p*0.55 + 0.45;});

    browObject.threedee.children[0].geometry = line.geometry;
    browObject.threedee.children[1].position.x = x2
    browObject.threedee.children[1].position.y = y2
  }

/*
returns the part from the list of parts in the scene based on name.
*/
function getPart(name){
    for(i in parts){
  if(parts[i].name==name){
      return parts[i];
  }
    }
    return null;
}

/*
initializes the eye objects. Handles the placement within the facePart reference frame.
Eyes are spheres with circular irises, circular pupils and a circular catchlight (the white shine)
*/
function addEyes(white_color, iris_color, size, height, separation, iris_size, pupil_scale, pupil_shape){
    var circleRadius = iris_size;
    var circleShape = new THREE.Shape();
    circleShape.moveTo(0,0)
    circleShape.arc(0,0,iris_size,0,6.6, true)

    if(pupil_shape=="round"){
      pupilShape = circleShape;
    } else if(pupil_shape=="cat"){
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
   addSphere(reye.threedee, size-2, white_color,0 , 0, 0, 0, 0, 0, 1 );
   addShape(reye.threedee,circleShape, iris_color, 0, 0, size, 0, 0, 0, 1 );
   addShape(reye.threedee,pupilShape, 0x000000, 0, 0, size + 1, 0, 0, 0, pupil_scale);
   if(pupil_shape=="round"){
    addShape(reye.threedee,circleShape, 0xffffff, -iris_size*pupil_scale/2, iris_size*pupil_scale/1.6, size + 2, 0, 0, 0, 0.15);
   }
   reye.idle_size = reye.size()

   leye = new facePart("leye", (separation/2)+x_adj, y_adj, -size);
   addSphere(leye.threedee, size, white_color, 0, 0, 0, 0, 0, 0, 1 );
   addShape(leye.threedee,circleShape, iris_color, 0, 0, size, 0, 0, 0, 1 );
   addShape(leye.threedee,pupilShape, 0x000000, 0, 0, size + 1, 0, 0, 0, pupil_scale);
   if(pupil_shape=="round"){
   addShape(leye.threedee,circleShape, 0xffffff, -iris_size*pupil_scale/2, iris_size*pupil_scale/1.6, size + 2, 0, 0, 0, 0.15 );
   }
   leye.idle_size = leye.size()
}

/*
Initializes the mouth object, which consists of two lip objects.
The lip objects each consist of 4 control points: two mouth corners and two intermediate control points.
The lips share the same lip corners
*/
function addMouth(color, x, y, width, height, thickness, opening, dimple_size, ulip_h_scale, llip_h_scale){

    upperLipControlPoints = [new THREE.Vector3(width/2, 0, 0),new THREE.Vector3(width/5, -height, 0), new THREE.Vector3(-width/5, -height, 0),new THREE.Vector3(-width/2, 0, 0)]
    lowerLipControlPoints = [new THREE.Vector3(width/2, 0, 0),new THREE.Vector3(width/5,-height, 0), new THREE.Vector3(-width/5, -height, 0),new THREE.Vector3(-width/2, 0, 0)]

    var upperCurve = new THREE.CatmullRomCurve3( upperLipControlPoints);
    var lowerCurve = new THREE.CatmullRomCurve3( lowerLipControlPoints);

    var circleShape = new THREE.Shape();
    circleShape.moveTo(0,0)
    circleShape.arc(0,0,dimple_size,0,6.6, true)

    llip = new constructLipPoints("llip", x,y, lowerLipControlPoints);
    addLine(llip.threedee, lowerCurve, color, thickness, 0,0,53,0,0,0,1);
    addShape(llip.threedee,circleShape, color, width/2, 0, 53, 0, 0, 0, 1 );
    addShape(llip.threedee, circleShape, color, -width/2, 0, 53, 0, 0, 0, 1 );


    ulip = new constructLipPoints("ulip", x,y, upperLipControlPoints);
    addLine(ulip.threedee, upperCurve, color, thickness, 0,0,55,0,0,0,1);
}

/*
Initializes the nose object. Its shape is an upside down triangle of size width and height
*/
function addNose(color, x, y, width, height){
    var xc = width/2;
    var yc = height/2

    var triangleShape = new THREE.Shape();
    triangleShape.moveTo(  -xc, yc );
    triangleShape.lineTo(  xc, yc );
    triangleShape.lineTo( 0, -yc );
    triangleShape.lineTo(  -xc, yc );

    nose = new facePart("nose", x, y);
    addShape(nose.threedee, triangleShape, color, 0,0,55,0,0,0,1);
    nose.idle_size = nose.size();
}

/*
Adds the lid objects. These are actually very large rectangles with a "U" cutout.
Blinking moves them in the y direction.
*/
function addLids(color, width, height, arch){
    var upperShape = new THREE.Shape();

    upperShape.moveTo( -width,-width*10);
    upperShape.lineTo( -2*width,-width*10);
    upperShape.lineTo( -2*width, width*10);
    upperShape.lineTo( 2*width, width*10);
    upperShape.lineTo( 2*width, -width*10);
    upperShape.lineTo( width, -width*10);

    upperShape.absarc( 0, 0, width, 0, 3.23, false );
    //arcShape.absarc( 0, 0, width, -.15, 3.23, false );

    var leye = getPart("leye");
    var reye = getPart("reye");

    //var xl = (camera_depth)*leye.threedee.position.x/((camera_depth)-leye.threedee.position.z);
    //var xr = camera_depth*reye.threedee.position.x/(camera_depth-reye.threedee.position.z);

    var xl = leye.threedee.position.x
    var xr = reye.threedee.position.x

    var y = height //+ (camera_depth*leye.threedee.position.y/(camera_depth-leye.threedee.position.z));

    ullid = new facePart("ullid", xl,y)
    addShape(ullid.threedee, upperShape, color,0,0,50,0,0,0,1);

    urlid = new facePart("urlid", xr,y)
    addShape(urlid.threedee, upperShape, color, 0,0,50,0,0,0,1);

    ullid.scale({y:arch}, 1);
    urlid.scale({y:arch}, 1);

    arcShape = new THREE.Shape();

    arcShape.moveTo( -width,width*10);
    arcShape.lineTo( -2*width,width*10);
    arcShape.lineTo( -2*width, -width*10);
    arcShape.lineTo( 2*width, -width*10);
    arcShape.lineTo( 2*width, width*10);
    arcShape.lineTo( width, width*10);

    arcShape.absarc( 0, 0, width, 0, -3.23, true );

    y = y - 2*height;

    lllid = new facePart("lllid", xl,y)
    addShape(lllid.threedee, arcShape, color,0,0,50,0,0,0,1);

    lrlid = new facePart("lrlid", xr,y)
    addShape(lrlid.threedee, arcShape, color, 0,0,50,0,0,0,1);

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


/*
Adds both brow objects. The brows are controlled by three points and taper at the ends
*/
function addBrows(color, width, height, thickness, innerSize){

    rBrowControlPoints = [new THREE.Vector3(-1.2*width, 0, 0),new THREE.Vector3(-width/2, 3*thickness, 0), new THREE.Vector3(width, 2.2*thickness, 0)]
    lBrowControlPoints = [new THREE.Vector3(1.2*width, 0, 0),new THREE.Vector3(width/2,3*thickness, 0), new THREE.Vector3(-width, 2.2*thickness, 0)]

    var lCurve = new THREE.CatmullRomCurve3( lBrowControlPoints);
    var rCurve = new THREE.CatmullRomCurve3( rBrowControlPoints);

    var leye = getPart("leye");
    var reye = getPart("reye");

    var xl = leye.threedee.position.x
    var xr = reye.threedee.position.x

    var y = height

    var circleShape = new THREE.Shape();
    circleShape.moveTo(0,0)
    circleShape.arc(0,0,innerSize,0,6.6, true)

    lbrow = new constructBrowPoints("lbrow", xl,y,lBrowControlPoints)
    addLine(lbrow.threedee, lCurve, color, thickness,0,0,55,0,0,0,1);
    addShape(lbrow.threedee,circleShape, color,lBrowControlPoints[2].x, lBrowControlPoints[2].y, 53, 0, 0, 0, 1 );

    rbrow = new constructBrowPoints("rbrow", xr,y,rBrowControlPoints)
    addLine(rbrow.threedee, rCurve, color, thickness,0,0,55,0,0,0,1);
    addShape(rbrow.threedee,circleShape, color, rBrowControlPoints[2].x, rBrowControlPoints[2].y, 53, 0, 0, 0, 1 );

}

/*
Method for adding a generic line to the threedee object in a face part.
threedee - object3d from the face part
shape - the curve object to get he geometry from
color - the color of the material to form the mesh with
x,y,z - float representing the coordiantes of the line in the face part reference frame
rx,ry,rz - float representing rotations about the various axes.
s - float representing the scale of the object
*/
function addLine(threedee, shape, color, width,  x, y, z, rx, ry, rz, s) {
    var points = shape.getPoints( 10 );
    var geometry = new THREE.Geometry().setFromPoints( points );

    var line = new MeshLine();
    line.setGeometry( geometry);

    var material = new MeshLineMaterial({lineWidth:width/300, color:new THREE.Color( color )});

    var mesh = new THREE.Mesh( line.geometry, material ); // this syntax could definitely be improved!
    mesh.position.set( x, y, z );
    mesh.rotation.set( rx, ry, rz );
    mesh.scale.set( s, s, s );
    threedee.add( mesh);
}

/*
method for adding a shape to a threedee object in a face part
*/
function addShape( threedee, shape, color, x, y, z, rx, ry, rz, s ) {
    // flat shape
    var geometry = new THREE.ShapeGeometry( shape );
    var material = new THREE.MeshBasicMaterial( { color: color, overdraw: 0.5 } );

    var mesh = new THREE.Mesh( geometry, material );
    mesh.position.set( x, y, z );
    mesh.rotation.set( rx, ry, rz );
    mesh.scale.set( s, s, s );
    threedee.add( mesh );

    // line (uncomment to see how the parts are drawn)
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

/*
Zeros all the AU's and moves the face to its neutral position.
t - time to move in milliseconds
*/
function zeroFace(t){
    for(a in aus_l){
        au(a,0)
    }
    move_face(t)
}

/*
Sets all AUs to One and moves to it.
t - time to move in milliseconds
*/
function oneFace(t){
    for(a in aus_l){
       au(a,1.0)
    }
    move_face(t)
}

/*
looks at a given point in the x,y,z space
x,y,z - floats representing to coordinates of the point to look at
t - float representing time to move to the location
*/
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

/*
zeros an array of aus, and moves to the face.
to_zero - an array of integers representing the number of the aus to zero
t -  time to move to the face (in milliseconds)
*/
function zero_aus(to_zero, t){
    for(a in to_zero){
  au(to_zero[a],0)
    }
    move_face(t)
}

/*
Same as above, but it does not move the face.
*/
function zero_aus_no_move(to_zero){
    for(a in to_zero){
      au(to_zero[a],0)
    }
}

/*
Checks to see if any visemes have been added to the buffer.
If they have, it then plays the viseme for the indicated time
*/
function check_and_play_visemes(){
    if(viseme_buffer.length==0){return}
    d = new Date();
    elapsed = d.getTime()-viseme_start+viseme_dur;
    elapsed = elapsed/1000.0
    if(viseme_time_buffer[0]>elapsed){return};
    while(viseme_time_buffer[0]<elapsed){
      play_viseme = viseme_buffer.shift()
      viseme_time = viseme_time_buffer.shift()
    }
    // console.log(elapsed, viseme_time)
    console.log(play_viseme, viseme_dur)
    viseme(play_viseme,viseme_dur);
}

/*
adds a viseme information to the buffers.
visemes - a string representing the type of viseme to play
time_per - the length of the viseme to play
times - the time at which the viseme should be played
*/
function play_visemes(visemes, time_per, times){
    viseme_buffer = visemes
    console.log(times)
    for(time in times){
      times[time]-=.035;
    }
    console.log(times)
    viseme_time_buffer = times
    viseme_dur = time_per
    d = new Date();
    viseme_start = d.getTime();
}

/*
Stops the face from speaking
*/
function stop_visemes(){
    please_stop = true
}

/*
Sets the AUs for the respective viseme.
viseme_name - a string that explains the viseme
t - a float to represent the time to move the face to the new position
*/
function viseme(viseme_name, t){
    console.log("Viseme: " + viseme_name)
    switch(viseme_name){

    // --------------- CONSONANTS ---------------------//

    // M,B,P -> My, Buy, Pie
    case 'BILABIAL':
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(23, .75)
    au(14, .25)
    au(24, .7)

    move_face(t, false)
    break;

    // F,V -> oFFer, Vest
    case "LABIODENTAL":
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(10,0.5);
    au(20,0.4);
    au(25,.8);

    move_face(t, false)
    break;

    // TH, TH - THin, THis
    case "INTERDENTAL":
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(10,.6)
    au(18,.75)
    au(25,.5)

    move_face(t, false)
    break;

    // L,T,D,Z,S,N -> Light, Top, DaD, Zebra, Sad, Nope
    case "DENTAL_ALVEOLAR":
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(25,.65)

    move_face(t, false)
    break;

    // R,SH,ZH,CH -> Red, SHould, aSia, CHart
    case "POSTALVEOLAR":
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(10, .75)
    au(18, 1)
    au(25, 1)

    move_face(t, false)
    break;

    // K,G,NG -> Cat, Game, thiNG
    case "VELAR_GLOTTAL":
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(10,.6)
    // au(18,.5)
    au(26,.5)

    move_face(t, false)
    break;

    // ------------------ VOWELS ------------------------//
    // EE, I -> flEEce, bIt
    case "CLOSE_FRONT_VOWEL":
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(26,1)
    au(20,1)
    au(10,.4)
    move_face(t, false)
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

    move_face(t, false)

    break;

    // schwa -> ArenA
    case "MID_CENTRAL_VOWEL":
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(26, 1)
    au(25, .5)
    au(23,1)

    move_face(t, false)
    break;

    // AE,AU,A,AY,EH -> trAp, mOUth, fAther, fAce, drEss
    case "OPEN_FRONT_VOWEL":
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(14, 1)
    au(20, 1)
    au(25, .7)
    au(26, .75)

    move_face(t, false)
    break;

    // AW,OI,O -> thOUght, chOIce, gOAt
    case "OPEN_BACK_VOWEL":
    zero_aus_no_move([10,13,14,16,18,20,23,24,25,26,27])
    au(26, .5)
    au(27, 1)

    move_face(t, false)
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

/*
Sets AU on a certain side to the given degree.
id - an integer representing the AU to change
degree - a float from 0 to 1 to control the intensity of the AU
side - a string indicating the side of the AU to modify (does both if left blank)
*/
function au(id, degree, side){

    if(side == "r"){
  aus_r[id] = degree
    } else if(side == "l"){
  aus_l[id] = degree
    } else{
  aus_l[id] = degree
  aus_r[id] = degree
    }
}
/*
Calculates the positions of all the facial features based on the current value of all the aus
t - float representing the milliseconds it should take to move to the current AU set
notViseme - flag to determine if the movement is a viseme. If it is, we only will modify the mouth shape
*/
function move_face(t, notViseme){
    if(notViseme == undefined){
      notViseme = true
    }

    if(notViseme){
      // ***** BROWS ***** (AU 1, 2, 4)
      lbrow = getPart("lbrow")

      var max_x = lBrowControlPoints[1].x/2
      var max_y = lbrow.idle_pos.y/3

      lInner = lBrowControlPoints[2].clone();
      rInner = rBrowControlPoints[2].clone();

      lInner.y += max_y * (1.5*aus_l[1] + .1*aus_l[2] - 1.2*aus_l[4])
      lInner.x -= max_x * (0.15*aus_l[1] + 0.5*aus_l[4])
      rInner.y += max_y * (1.5*aus_r[1] + .1*aus_r[2] - 1.2*aus_r[4])
      rInner.x += max_x * (0.15*aus_r[1]  + 0.5*aus_r[4])

      lMid = lBrowControlPoints[1].clone();
      rMid = rBrowControlPoints[1].clone();

      lMid.y += max_y * (0.1*aus_l[1] + aus_l[2] - 0.6*aus_l[4])
      lMid.x -= max_x * (aus_l[1] + -0.5*aus_l[2] + aus_l[4])
      rMid.y += max_y * (0.1*aus_r[1] + aus_r[2]  - 0.6*aus_r[4])
      rMid.x += max_x * (aus_r[1] + -0.5*aus_r[2] + aus_r[4])

      lOuter = lBrowControlPoints[0].clone();
      rOuter = rBrowControlPoints[0].clone();

      lOuter.y += max_y * (0.5*aus_l[2] + 0.1*aus_l[4])
      lOuter.x -= max_x * (-.2*aus_l[2]  + 0.2*aus_l[4])
      rOuter.y += max_y * (0.5*aus_r[2] + 0.1*aus_r[4])
      rOuter.x += max_x * (-.2*aus_r[2]  + 0.2*aus_l[4])


      rBrow = [rOuter,rMid, rInner]
      lBrow = [lOuter,lMid, lInner]

      rbrow.moveToLocation(rBrow, t);
      lbrow.moveToLocation(lBrow, t);

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

    }
    // ***** MOUTH *****
    ulip=getPart("ulip");
    llip=getPart("llip");

    max_up_dist = (nose.idle_pos.y-ulip.idle_pos.y)/1.5
    max_down_dist = (nose.idle_pos.y-llip.idle_pos.y)/1.5
    max_x_variation = (upperLipControlPoints[0].x - upperLipControlPoints[3].x) / 4; //should be width divided by 2 I think

    lcorner = upperLipControlPoints[0].clone();
    rcorner = upperLipControlPoints[3].clone();

    lcorner.x += max_x_variation*(.2*aus_l[12] + .05*aus_l[13] + .25*aus_l[14] -.1*aus_l[26] -.3*aus_l[27] +.35*aus_l[17]-.7*aus_l[18] + .25*aus_l[20] -.2*aus_l[23] -.1*aus_l[24])/1.1
    lcorner.y += max_down_dist*(-.2*aus_l[25] -.2*aus_l[26] + .7*aus_l[13]-1.5*aus_l[15]-.5*aus_l[27] - .2*aus_l[20] -.3*aus_l[23] -.5*aus_l[24])/3.4
    rcorner.x -= max_x_variation*(.2*aus_r[12] + .05*aus_r[13] + .25*aus_r[14]-.1*aus_r[26] -.3*aus_r[27] +.35*aus_r[17]-.7*aus_r[18] + .25*aus_r[20] -.2*aus_r[23] -.1*aus_r[24])/1.1
    rcorner.y += max_down_dist*(-.2*aus_r[25] -.2*aus_r[26] + .7*aus_r[13]-1.5*aus_r[15]-.5*aus_r[27] - .2*aus_r[20] -.3*aus_r[23] -.5*aus_r[24])/3.4


    upperl = upperLipControlPoints[1].clone();
    upperr = upperLipControlPoints[2].clone();

    upperl.x += max_x_variation*(.55*aus_l[10] + .25*aus_l[14]-.4*aus_l[18]+ .25*aus_l[20] -.1*aus_l[23])/1.05
    upperl.y += max_up_dist*(.1*aus_l[25] +.3*aus_l[26] +.6*aus_l[27] + .55*aus_l[10]+.35*aus_l[17])/2.2
    upperr.x -= max_x_variation*(.55*aus_r[10] + .25*aus_r[14]-.4*aus_r[18] + .25*aus_r[20] -.1*aus_r[23])/1.05
    upperr.y += max_up_dist*(.1*aus_r[25] +.3*aus_r[26] +.6*aus_r[27] + .55*aus_r[10]+.35*aus_r[17])/2.2



    lowerl = lowerLipControlPoints[1].clone();
    lowerr = lowerLipControlPoints[2].clone();

    lowerl.x += max_x_variation*(.25*aus_l[14] + .5*aus_l[16] + .2*aus_l[26]-.4*aus_l[18]+ .25*aus_l[20] -.2*aus_l[23])/1.05
    lowerl.y += max_down_dist*(-.4*aus_l[25] -.7*aus_l[26] -1.6*aus_l[27]+ .55*aus_l[10] -.2*aus_l[16] +.45*aus_l[17])/2.2
    lowerr.x -= max_x_variation*(.25*aus_r[14] + .5*aus_r[16] + .2*aus_r[26]-.4*aus_r[18] + .25*aus_r[20] -.2*aus_r[23])/1.05
    lowerr.y += max_down_dist*(-.4*aus_r[25] -.7*aus_r[26] -1.6*aus_r[27] + .55*aus_r[10] -.2*aus_r[16] +.45*aus_r[17])/2.2

    upperLip = [rcorner, upperr, upperl, lcorner];
    lowerLip = [rcorner, lowerr, lowerl, lcorner];


    ulip.moveToLocation(upperLip, t);
    llip.moveToLocation(lowerLip, t);
}

/**
Receives and processes the FaceRequest Message sent from the face_keyframe_server
message - a FaceRequest Message, which contains visemes and/or aus to send to the face.
          the message also contains timing information and magnitude information.
**/
function get_goal(message) {
  if(message.visemes.length!=0){
    console.log('Message received: visemes: ' + message.visemes);
    console.log(message)
    stop_visemes()
    // play_visemes(message.visemes, message.viseme_ms, message.times, message.start)
    play_visemes(message.visemes, 35, message.times, message.start)
  }

  if(message.aus.length!=0){
    console.log('Message received: aus: ' + message.aus + " degrees: " + message.au_degrees + " side: " + message.side)
    for(a in message.aus){
      this_au = message.aus[a]
      console.log(this_au)
      if(message.au_ms[a]<0){
        console.log("Time cannot be less than zero!")

      } else if(this_au == 'ZeroFace'){
        zeroFace(message.au_ms)
        console.log('clearing AUS')

      } else {
        //if the au is unilateral, it will have an 'r' or 'l' at the end to indicate the side to move
        if(this_au.slice(-1) == 'l' || this_au.slice(-1) == 'r'){
          au(parseInt(this_au.slice(0,-1)), message.au_degrees[a], this_au.slice(-1))
        }
        //otherwise assume bilateral movement
        else {
          au(parseInt(this_au), message.au_degrees[a])
        } if(parseInt(this_au) == 43){
          lockIdleUntil = new Date().getTime() + message.au_ms + 500
          console.log(new Date().getTime())
          console.log(lockIdleUntil)
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

/*
function to perform idle animation, which consists of looking around the room at random
*/
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

/*
Puckers the lips, blinks and raises the nose
*/
function doPoke(){
    poke_time=600
    au(9,1);
    au(43,1);
    au(18,1);
    au(2,1)
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
    //don't do the poke animation if the  gui is up or else dragging the controllers makes the poke happen
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
    if (typeof gui === "undefined" && new Date().getTime() > lockIdleUntil){
      doIdle();
    }
    //print_elapsed()
    check_and_play_visemes()
    TWEEN.update();



    renderer.render( scene, camera );
    //print_elapsed()
    //set_time()
}
