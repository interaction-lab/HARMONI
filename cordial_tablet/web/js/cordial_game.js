//------------------------------------------------------------------------------
// Tablet Game Interface for use with CoRDial
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



var ros, setup_srv;
var token_listener,area_listener,banner_listener;
var pose_publisher,location_publisher;

var instance_name;

var environment, game_layer,renderer, graphics;
var area_layer, sprite_layer;

var is_banner, banner_graphics, banner_text, banner_dismiss_text;

// global variable for message_text
var message_text;

var setup;

var windowW = window.innerWidth;
var windowH = window.innerHeight;

var sprites = [];
var areas = [];
var texts = [];

var bkgd_texture;
//var logo_sprite;
var go_fish_arrange;

var ros_connected;

function ros_init(ros_master_uri){
    ros_connected=false;
    if(ros_master_uri == ""){
	ros_master_uri = "ws://" + location.hostname + ":9090"
    }
    console.log("ROS master URI: " + ros_master_uri)
    ros = new ROSLIB.Ros({
	url : ros_master_uri
    });

    ros.on('connection', function() {
	ros_connected = true;
	console.log('Connected to websocket server.');
    });

    ros.on('error', function(error) {
	ros_connected=false;
	console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', function() {
	ros_connected = false;
	console.log('Connection to websocket server closed.');
    });

    // Subscribing to a Topic
    // ----------------------

    token_listener = new ROSLIB.Topic({
        ros : ros,
	name : '/CoRDial/tablet/tokens/',
	messageType : 'cordial_tablet/ChangeToken'
    });

    token_listener.subscribe(req_token);
    
    refresh_listener = new ROSLIB.Topic({
        ros : ros,
	name : '/CoRDial/tablet/reload/',
	messageType : 'cordial_tablet/Reload'
    });

    refresh_listener.subscribe(refresh_screen);
    
    area_listener = new ROSLIB.Topic({
        ros : ros,
	name : '/CoRDial/tablet/areas',
	messageType : 'cordial_tablet/AddRemoveArea'
    });

    area_listener.subscribe(req_area);

    banner_listener = new ROSLIB.Topic({
        ros : ros,
	name : '/CoRDial/tablet/banner',
	messageType : 'cordial_tablet/DispBanner'
    });

    is_banner=false;
    banner_listener.subscribe(req_banner);

    go_fish_arrange=false;

    pose_publisher = new ROSLIB.Topic({
	ros : ros,
	name : '/CoRDial/tablet/tokens/position',
	queue_size: 3,
	messageType: 'cordial_tablet/TokenPose'
    });

    location_publisher = new ROSLIB.Topic({
	ros : ros,
	name : '/CoRDial/tablet/tokens/location',
	queue_size: 3,
	messageType: 'cordial_tablet/TokenLocation'
    });

    setup_srv = new ROSLIB.Service({
	ros : ros,
	name : '/CoRDial/tablet/setup',
	serviceType : 'cordial_tablet/Setup'
    });
}

function refresh_screen(msg){
    if(msg.s==""||msg.s==instance_name){
	location.reload()
    }
}

function add_token(name, img_location, x, y, type, text,textsize){
    var sprite = new PIXI.Sprite.fromImage(img_location);

    if(textsize==0){textsize=60}

    //var sprite = new PIXI.Sprite(texture);
    sprite.interactive=false;
    if(type=="interactive"){
	sprite.interactive=true;
	sprite.on('mouseup', mouseup_cb)
	sprite.on('touchend', touchend_cb)
	sprite.on('touchstart', touchstart_cb)
	sprite.on('touchmove', touchmove_cb)
	sprite.on('mousedown', mousedown_cb)
	sprite.on('mousemove', mousemove_cb)
    }
 //    else if(type=="card"){
	// sprite.interactive=true;
	// sprite.on('mouseup', mouseupbutton_cb)
	// sprite.on('touchend', mouseupbutton_cb)
	// sprite.on('mousedown', mousedownbutton_cb)
	// sprite.on('touchstart', mousedownbutton_cb)
 //    }
    else if(type=="button"){
	sprite.interactive=true;
	sprite.on('mousedown', mousedownbutton_cb)
	sprite.on('mousemove', mousemovebutton_cb)
	sprite.on('mouseup', mouseupbutton_cb)
    	sprite.on('touchend', mouseupbutton_cb)	
    	sprite.on('touchstart', mousedownbutton_cb)
    	sprite.on('touchmove', mousemovebutton_cb)
    }
    else if(type=="text"){
	padding = 10

	sprite.graphics =  new PIXI.Graphics();
	sprite.addChild(sprite.graphics)

	sprite_text= new PIXI.Text(text, {font : textsize+'px Helvetica'});
	sprite_text.x = (sprite_text.width+padding)/2
	sprite_text.y = (sprite_text.height+padding)/2
	sprite_text.anchor.x = 0.5;
	sprite_text.anchor.y = 0.5;


	
	sprite.graphics.beginFill(0xFFFFFF, 0);
	sprite.graphics.drawRect(0,0,sprite_text.width+padding,sprite_text.height+padding);
	sprite.graphics.endFill();

	// center the sprites anchor point
	sprite.anchor.x = 0.5;
	sprite.anchor.y = 0.5;

	sprite.addChild(sprite_text);

	// move the sprite to position
	sprite.position.x = x;
	sprite.position.y = y;
	sprite.name = name

	sprite_layer.addChild(sprite)
	sprites.push(sprite)
   
	return sprite
    }
    //EVENT CALLED "click"
    sprite.click = false;

    // center the sprites anchor point
    sprite.anchor.x = 0.5;
    sprite.anchor.y = 0.5;

    // move the sprite to position
    sprite.position.x = x;
    sprite.position.y = y;
    sprite.name = name
    sprite.status_cb = send_sprite_pose

    sprite_layer.addChild(sprite)
    
    sprite_text= new PIXI.Text(text, {font : textsize+'px Helvetica'});
    sprite_text.x = 0
    sprite_text.y = 0
    sprite_text.anchor.x = 0.5;
    sprite_text.anchor.y = 0.5;
    sprite.addChild(sprite_text);

    sprites.push(sprite)
   
    setTimeout(function(){update_locations(sprite,false)},300);
    return sprite
};

//cb is from sprite -- in this function "this" = "the sprite"
function touchend_cb(touchData){
    //console.log(this.name)
    //console.log(touchData)
    //this.tint = 0xFFFFFF
    this.click=false
    update_locations(this,true)
    this.status_cb()
};

function touchstart_cb(touchData){
    //console.log(this.name)
    //console.log(touchData)
    //this.tint = 0xBBBBBB
    createjs.Tween.removeTweens(this.position);
    this.click=true
    this.status_cb()

    // displays the image as "top" image
    sprite_layer.removeChild(this);
    sprite_layer.addChild(this);
};

function touchmove_cb(touchData){
    //console.log(this)
    //console.log("touched..")
    //console.log(touchData.data.originalEvent.touches)
    if(this.click){
	var touches = touchData.data.originalEvent.touches
	for (i=0;i< touches.length;i++){
	    touch_loc = new PIXI.Point(touches[i].pageX,touches[i].pageY)
	    if(this.containsPoint(touch_loc)){
		this.position.x = touch_loc.x
		this.position.y = touch_loc.y
		this.status_cb()
	    }
	}
    };
};

function add_area(name, x, y,width, height, drawn, arrange, text, textx, texty, textsize, xpadding, ypadding,is_filled, bkgd_color,img){
    var area = new PIXI.Rectangle(x,y,width,height);

    if(textsize==0){textsize=60}

    area.name = name
    area.arrange = arrange
    area.xpadding = xpadding
    area.ypadding = ypadding
    areas.push(area)
    if(drawn&&!img){
    	console.log("drawing rectangle")
	graphics.cacheAsBitmap=true
    	graphics.lineStyle(5, 0x000000, 10);
        
	if(is_filled){
	    graphics.beginFill(bkgd_color);
	}
		
    	graphics.drawRect(x,y,width,height);
    	var area_text= new PIXI.Text(text,{font : textsize+'px Helvetica'});
            
    	area_text.x = x+textx+5;
    	area_text.y = y+texty+5;

    	area_layer.addChild(area_text);

	if(is_filled){
	    graphics.endFill();
	}
    }
    if(drawn&&img){
	var area_bkgd = PIXI.Sprite.fromImage('img/'+img)
	area_bkgd.width = width
	area_bkgd.height = height
	area_bkgd.x=x
	area_bkgd.y=y
	area_layer.addChild(area_bkgd)
	var area_text= new PIXI.Text(text,{font : textsize+'px Helvetica'});
            
    	area_text.x = x+textx+5;
    	area_text.y = y+texty+5;

    	area_layer.addChild(area_text);
    }
    
}

function send_locations(sprite, area_names){
    location_publisher.publish({instanceid: instance_name, tokenid:sprite.name, areas:area_names, x:sprite.position.x, y:sprite.position.y})
}

function arrange_areas(relevant_areas){
    for(var j=0;j<relevant_areas.length;j++){
	var area = relevant_areas[j]
	var contained_sprites=[]
	for(var i=0;i<sprites.length;i++){
	    var bounds = sprites[i].getBounds()
	    /*if(area.contains(bounds.x,bounds.y)||
	       area.contains(bounds.x+bounds.width,bounds.y)||
	       area.contains(bounds.x+bounds.width,bounds.y+bounds.height)||
	       area.contains(bounds.x,bounds.y+bounds.height)){*/
	    if(area.contains(bounds.x+bounds.width/2,bounds.y+bounds.height/2)){
		contained_sprites.push(sprites[i])
	    }
	}

	function sort_by_pose(sprite1, sprite2){
	    var bounds1 = sprite1.getBounds();
	    var bounds2 = sprite2.getBounds();
	    
	    if(bounds1.y==bounds2.y){
		return bounds1.x-bounds2.x
	    }else{
		return bounds1.y-bounds2.y
	    }
	}
	
	function sort_by_xpose(sprite1, sprite2){
	    var bounds1 = sprite1.getBounds();
	    var bounds2 = sprite2.getBounds();
	    
	    return bounds1.x-bounds2.x
	}

	
	function sort_by_ypose(sprite1, sprite2){
	    var bounds1 = sprite1.getBounds();
	    var bounds2 = sprite2.getBounds();
	    
	    return bounds1.y-bounds2.y
	}

	if(area.arrange=="tile"){
	    //console.log(contained_sprites)
	    contained_sprites.sort(sort_by_pose)
	    
	    var maxw = 0
	    var maxh = 0
	    for(var i=0;i<contained_sprites.length;i++){
		var b = contained_sprites[i].getBounds()
		maxw = Math.max(maxw, b.width)
		maxh = Math.max(maxh, b.height)
	    }

	    var nsprites = contained_sprites.length
	    
	    var preferred_spacing = 10
	    var xpadding = area.xpadding
	    var ypadding = area.ypadding
	    var done = false
	    while(!done){
		ncols = Math.max(1,Math.floor((area.width-2*xpadding)/(maxw+preferred_spacing)))
		nrows = Math.max(1,Math.ceil(nsprites/ncols))
		if(nrows*(maxh+preferred_spacing)+2*ypadding<area.height){
		    done = true
		} else if(nrows > 1){
		    if(preferred_spacing + maxw > 5){
			preferred_spacing=preferred_spacing-5
		    } else {
			preferred_spacing = -maxw
			done = true
		    }
		} else if(nrows == 1){    
		    if(ncols*(maxw+preferred_spacing)+2*xpadding<area.width){
			done = true
		    } else if(preferred_spacing + maxw > 5){
			preferred_spacing = preferred_spacing-5
		    } else {
			preferred_spacing = -maxw
			done = true
		    }
		} else {
		    console.log("There's an error in calculating the spacing; time to debug!")
		}
	    }

	    
	    ncols = contained_sprites.length
	    nrows = 1
	    if(maxw+preferred_spacing > 0){
		ncols = Math.max(1,Math.floor((area.width-2*xpadding)/(maxw+preferred_spacing)))
		nrows = Math.max(1,Math.ceil(nsprites/ncols))
	    } 
	    
	    for(var i= 0; i < contained_sprites.length; i++){
		sprite = contained_sprites[i]
		col = i%ncols
		row = Math.floor(i/ncols)
		pose = {x:area.x+xpadding+maxw/2+col*(maxw+preferred_spacing), y:area.y+ypadding+maxh/2+row*(maxh+preferred_spacing)}
		move_token(sprite, pose, 0.01, false, false)
	    }
	}

	if(area.arrange=="center"){
	    targetx = (area.width-2*area.xpadding)/2.0
	    targety = (area.height-2*area.ypadding)/2.0
	    for(var i= 0; i < contained_sprites.length; i++){
		sprite = contained_sprites[i]
		pose = {x:area.x+area.xpadding+targetx, y:area.y+area.ypadding+targety}
		move_token(sprite, pose, 0.01, false, false)
	    }
	}


	if(area.arrange=="horizontal"){
	    if(contained_sprites.length>1){
		contained_sprites.sort(sort_by_xpose)
		targety = (area.height-2*area.ypadding)/2.0
		spacing = (area.width-2*area.xpadding)/(contained_sprites.length-1)
		for(var i= 0; i < contained_sprites.length; i++){
		    sprite = contained_sprites[i]
		    pose = {x:area.x+area.xpadding+spacing*i, y:area.y+area.ypadding+targety}
		    move_token(sprite, pose, 0.01, false, false)
		}
	    }else{
		targetx = (area.width-2*area.xpadding)/2.0
		targety = (area.height-2*area.ypadding)/2.0
		for(var i= 0; i < contained_sprites.length; i++){
		    sprite = contained_sprites[i]
		    pose = {x:area.x+area.xpadding+targetx, y:area.y+area.ypadding+targety}
		    move_token(sprite, pose, 0.01, false, false)
		}
	    }
	}

	if(area.arrange=="left"){
	    console.log("left align")
	    if(contained_sprites.length>1){
		contained_sprites.sort(sort_by_xpose)
		targety = (area.height-2*area.ypadding)/2.0
		spacing = (area.width-2*area.xpadding)/(contained_sprites.length-1)
		next_spot = area.x+area.xpadding
		for(var i= 0; i < contained_sprites.length; i++){
		    sprite = contained_sprites[i]
		    pose = {x:next_spot, y:area.y+area.ypadding+targety}
		    move_token(sprite, pose, 0.01, false, false)
		    next_spot = next_spot + Math.min(sprite.width, spacing)
		    console.log(next_spot)
		}
	    }else{
		targetx = area.x+area.xpadding
		targety = (area.height-2*area.ypadding)/2.0
		for(var i= 0; i < contained_sprites.length; i++){
		    sprite = contained_sprites[i]
		    pose = {x:targetx, y:area.y+area.ypadding+targety}
		    move_token(sprite, pose, 0.01, false, false)
		}
	    }
	}

	if(area.arrange=="right"){
	    console.log("right align")
	    if(contained_sprites.length>1){
		contained_sprites.sort(sort_by_xpose)
		targety = (area.height-2*area.ypadding)/2.0

		spacing = (area.width-2*area.xpadding)/(contained_sprites.length-1)
		console.log(spacing)
		next_spot = area.x+area.width-area.xpadding
		for(var i= contained_sprites.length-1; i >= 0; i--){
		    sprite = contained_sprites[i]
		    pose = {x:next_spot, y:area.y+area.ypadding+targety}
		    move_token(sprite, pose, 0.01, false, false)
		    next_spot = next_spot - Math.min(sprite.width, spacing)
		    console.log(next_spot)
		}
	    }else{
		targetx = area.x+area.width-area.xpadding
		targety = (area.height-2*area.ypadding)/2.0
		for(var i= 0; i < contained_sprites.length; i++){
		    sprite = contained_sprites[i]
		    pose = {x:targetx, y:area.y+area.ypadding+targety}
		    move_token(sprite, pose, 0.01, false, false)
		}
	    }
	}
	if(area.arrange=="position"){
	    targetx = 0
	    targety = 0
	    for(var i= 0; i < contained_sprites.length; i++){
		sprite = contained_sprites[i]
		pose = {x:area.x+area.xpadding+targetx, y:area.y+area.ypadding+targety}
		move_token(sprite, pose, 0.01, false, false)
	    }
	}

	if(area.arrange=="vertical"){
	    if(contained_sprites.length>1){
		contained_sprites.sort(sort_by_ypose)
		targetx = (area.width-2*area.xpadding)/2.0
		spacing = (area.height-2*area.ypadding)/(contained_sprites.length-1)
	    	for(var i= 0; i < contained_sprites.length; i++){
		    sprite = contained_sprites[i]
		    pose = {x:area.x+area.xpadding+targetx, y:area.y+area.ypadding+spacing*i}
		    move_token(sprite, pose, 0.01, false, false)
		}
	    }else{
		targetx = (area.width-2*area.xpadding)/2.0
		targety = (area.height-2*area.ypadding)/2.0
		for(var i= 0; i < contained_sprites.length; i++){
		    sprite = contained_sprites[i]
		    pose = {x:area.x+area.xpadding+targetx, y:area.y+area.ypadding+targety}
		    move_token(sprite, pose, 0.01, false, false)
		}

	    }
	}
    }
}


function update_locations(sprite, send_update){
    var relevant_areas=[]
    var area_names = []
    for(var j=0;j<areas.length;j++){
	var area = areas[j]
	var bounds = sprite.getBounds()
	/*if(area.contains(bounds.x,bounds.y)||
	   area.contains(bounds.x+bounds.width,bounds.y)||
	   area.contains(bounds.x+bounds.width,bounds.y+bounds.height)||
	   area.contains(bounds.x,bounds.y+bounds.height)){*/
	if(area.contains(bounds.x+bounds.width/2,bounds.y+bounds.height/2)){  
	    relevant_areas.push(areas[j])
	    area_names.push(areas[j].name)
	}
    }

    arrange_areas(relevant_areas)
    if(send_update){
	send_locations(sprite, area_names)
    }
}



function move_token_to_area(sprite,area,t){
    var goal = {x:area.x,y:area.y}
    move_token(sprite, goal, t, true, true)
}


function move_token(sprite,goal,t,publish_after,arrange_after){
    //sprite.interactive=false

    // displays the image as "top" image
    sprite_layer.removeChild(sprite);
    sprite_layer.addChild(sprite);

    createjs.Tween.removeTweens(sprite.position);
    var goal = goal;
    var target = sprite.position;
    var t = createjs.Tween.get(target, {override:true}).to(goal,t*1000, createjs.Ease.getPowInOut(2))
    t.on("change", sprite_pose_update, null, false, {sprite:sprite})
    t.call(sprite_move_done, params=[sprite])
    
    function sprite_pose_update(evt,data){
	//console.log(evt)
	data.sprite.status_cb()
    }

    function sprite_move_done(sprite){
	if(arrange_after){
	    update_locations(sprite,publish_after)
	}
    }

}

function rotate_token(sprite,goal,t){
//    sprite.interactive=false

    // displays the image as "top" image
    sprite_layer.removeChild(sprite);
    sprite_layer.addChild(sprite);

    createjs.Tween.removeTweens(sprite.rotation);
    var goal = goal;
    var target = sprite;
    var t = createjs.Tween.get(target, {override:true}).to({rotation:goal},t*1000, createjs.Ease.getPowInOut(2))
    t.on("change", sprite_pose_update, null, false, {sprite:sprite})
    t.call(sprite_move_done, params=[sprite])
    
    function sprite_pose_update(evt,data){
	//console.log(target)
	//data.sprite.rotation=target
	//console.log(data.sprite.rotation)
    }

    function sprite_move_done(sprite){
	//console.log(sprite.name)
	//console.log(sprite.rotation)
    }
}

//cb is from sprite -- in this function "this" = "the sprite"
function mouseup_cb(mouseData){
    console.log(this.name)
    //this.tint = 0xFFFFFF
    this.click=false
    update_locations(this,true)
    this.status_cb()
};

function mousedown_cb(mouseData){
    //this.tint = 0xBBBBBB
    createjs.Tween.removeTweens(this.position);
    this.click=true    
    this.status_cb()

    // displays the image as "top" image
    sprite_layer.removeChild(this);
    sprite_layer.addChild(this);
};

// Buttons topic and mouse implementation
// roslib.js and pixi.js
function mousemove_cb(mouseData){
    //console.log(mouseData)
    if(this.click){
	var clickx = mouseData.data.originalEvent.layerX;
	var clicky =  mouseData.data.originalEvent.layerY;
	this.position.x = clickx;
	this.position.y = clicky;
	this.status_cb()
    };
};

function display_message(msg,h){
    var action="none"
    if(msg.action.indexOf('add')>-1){
	action="add"
    }else if(msg.action.indexOf('delete')>-1){
	action="delete"
    }

    // Sets the message
    if(action == "add")
        message_text.text = msg.s;
    // Deletes the message
    else if(action == "delete")
        message_text.text = "";
}

function display_banner(text,h){
    if(is_banner){
        banner_text.text = text;
	return;
    }

    is_banner=true;
    game_layer.visible=false;

    banner_graphics = new PIXI.Graphics()
    // Change bankground color of the banner
    //banner_graphics.beginFill(0xFFB5C5);
    banner_graphics.lineStyle(2, 0xFFFFFF, 10);
    banner_graphics.drawRect(0,(windowH-h)/2,windowW,h);
    banner_graphics.endFill();
    environment.addChild(banner_graphics)

    banner_text= new PIXI.Text(text);
    banner_text.x = 5;
    banner_text.y = (windowH-h)/2+20;
    environment.addChild(banner_text);

    banner_dismiss_text = new PIXI.Text("(Click banner to dismiss...)")
    banner_dismiss_text.x = windowW-400;
    banner_dismiss_text.y = (windowH+h)/2-40;
    environment.addChild(banner_dismiss_text);

    environment.interactive=true;
    environment.on("click", clear_banner);
    environment.on("tap", clear_banner);

}

function clear_banner(){
    environment.removeChild(banner_graphics);
    environment.removeChild(banner_text);
    environment.removeChild(banner_dismiss_text);

    game_layer.visible=true;

    environment.interactive=false;
    environment.on("click", null);
    environment.on("tap", null);
    is_banner=false;
}

//cb is from sprite -- in this function "this" = "the sprite"
function mouseupbutton_cb(mouseData){
    console.log(this)
    if(this.click){
	this.status_cb()
	update_locations(this,true)
    };
    this.tint = 0xFFFFFF
    this.click=false
};

function mousedownbutton_cb(mouseData){
    this.tint = 0xBBBBBB
    this.click=true
};

function mousemovebutton_cb(mouseData){
    
};

function send_sprite_pose(){
    //console.log(this)
    pose_publisher.publish({instanceid: instance_name, tokenid:this.name, x:this.position.x, y:this.position.y})
};

// check version of this pixi with current version
function game_init(name){
    // create an new instance of a pixi stage
    //stage = new PIXI.Stage(0x66FF99);
    //container = new PIXI.Container();
    //container.interactive=true
    //container.on('mouseup', mousedown_cb)
    //container.on('mouseout', mouseout_cb)
    //container.on('mousemove',mousemove_cb)

    console.log(name);

    environment = new PIXI.Container();

    instance_name=name

    // create a renderer instance
    renderer = PIXI.autoDetectRenderer(windowW,windowH);

    // Make the renderer fill out entire window
    renderer.view.style.position = "absolute";
    renderer.view.style.display = "block";
    renderer.autoResize = true;
    
    //TODO: allow for choice of tiles or color for background?
    //renderer.backgroundColor=0x008000

    var texture = PIXI.Texture.fromImage('bkgd/default_background.png')
    bkgd_texture = new PIXI.extras.TilingSprite(texture, renderer.width,renderer.height)
    environment.addChild(bkgd_texture)

    /*// add USC Viterbi logo on top left corner
    var logo_texture = PIXI.Texture.fromImage('img/war.png');
    // var temp_texture = PIXI.Texture(viterbi_texture, new PIXI.Rectangle(125,125,125,125));
    logo_sprite = new PIXI.Sprite(logo_texture);
    //logo_sprite.x = 1;
    //logo_sprite.y = 1;
    environment.addChild(logo_sprite);*/

    game_layer = new PIXI.Container();

    environment.addChild(game_layer);

    message_text= new PIXI.Text("");
    message_text.x = 20;
    message_text.y = (windowH-160)/2+20;
    environment.addChild(message_text);

    graphics = new PIXI.Graphics();

    game_layer.addChild(graphics)

    // holds the areas
    area_layer = new PIXI.Container();
    game_layer.addChild(area_layer);

    // holds the sprites
    sprite_layer = new PIXI.Container();
    game_layer.addChild(sprite_layer);

    
    // add the renderer view element to the DOM
    document.body.appendChild(renderer.view);
 
    var request = new ROSLIB.ServiceRequest({
	id : instance_name,
	window_w : windowW,
	window_h : windowH
	});

    function setup_game(setup){
	for(var i = 0; i<setup.areas.length;i++){
	    var area = setup.areas[i]
	    area.id = instance_name
	    req_area(area)
	}
	
	for(var i=0;i<setup.tokens.length;i++){
	    var token=setup.tokens[i]
	    token.id = instance_name
	    if (!token.action.indexOf("add")>-1){
		token.action = "add;"+token.action
	    }
	    req_token(token)
	}

	console.log(setup)
	if(setup.background.length > 0){
	    change_background(setup.background)
	}
    }

    createjs.Ticker.setFPS(60)
    requestAnimationFrame( animate );

    setup_srv.callService(request, 
			  setup_game,
			  function(error){
			      console.log(error)
			      display_banner("Setup error: " + error,200)
			  }
			 )
}


function req_area(msg){
    if(msg.id!=instance_name){
	return
    }
    var exists=false
    var i = 0
    for(;i<areas.length;i++){
	if(areas[i].name==msg.name){
	    exists=true
	    break
	}
    }

    if(exists){
        // Used to update how many cards player has via "x"
        for(var j=0;j<areas.length;j++){
            if(areas[j].name == msg.name)
                 areas[j].number_text.text = "" + msg.x;
        }
	console.log("Error: area with that name already exists! Removal not implemented.")
    }else{
	add_area(msg.name, msg.x, msg.y, msg.width, msg.height, msg.drawn, msg.arrange, msg.text, msg.textx,msg.texty, msg.textsize, msg.xpadding,msg.ypadding, msg.filled, msg.bcolor,msg.img);
    }
}

function req_token(msg){
    if(msg.id!=instance_name){
	return
    }
    var sprite
    var exists=false
    var i = 0
    for(;i<sprites.length;i++){
	if(sprites[i].name==msg.name){
	    sprite = sprites[i]
	    exists=true
	    break
	}
    }
    
    function msg_contains(str){
	return msg.action.indexOf(str)>-1;
    }
    
    if(msg_contains("add")){
	if (!exists){
	    console.log("Adding token: " + msg.name)
	    sprite = add_token(msg.name,"img/"+msg.img_loc, msg.x, msg.y, msg.type, msg.text, msg.textsize);
	    exists=true
	}else{
	    console.log("Token already exists; not adding. Name: " + msg.name)
	}
    }else if(!exists){
	console.log("Error: no such token: " + msg.name)
	return
    }
    
    /*while(!sprite.texture.baseTexture.hasLoaded){
	console.log("Waiting on texture load...")
    }*/
    console.log("Texture status:")
    console.log(sprite.texture.baseTexture.hasLoaded)

    if(msg_contains("delete")){
	console.log("Deleting sprite")
	var doomed_sprite = sprites.splice(i,1)[0]
	sprite_layer.removeChild(doomed_sprite)
    }
    if(msg_contains("retexture")){
	console.log("Retexturing sprite")
	console.log("retexturing")
	sprite.texture=new PIXI.Texture.fromImage('img/'+msg.img_loc)
    }
    if(msg_contains("move")){
	console.log("Moving sprite")
	move_token(sprite, {x:msg.x,y:msg.y}, 0.5, false, true)
    }
    if(msg_contains("disable")){
	console.log("Disabling sprite")
	sprite.interactive=false
    }
    if(msg_contains("enable")){
	console.log("Enabling sprite")
	sprite.interactive=true
    }
    if(msg_contains("tint")){
	console.log("Tinting sprite")
	sprite.tint=msg.tint
    }
    if(msg_contains("scale")){
	console.log("Scaling sprite")
	sprite.scale=new PIXI.Point(msg.scalex,msg.scaley)
    }
    if(msg_contains("rotate")){
	console.log("Rotate sprite")
	rotate_token(sprite, msg.angle, 0.5)
    }
}

function change_background(texture_img){
    var num = environment.getChildIndex(bkgd_texture);
    environment.removeChild(bkgd_texture);
    
    // new background
    var texture = PIXI.Texture.fromImage('bkgd/'+texture_img)
    bkgd_texture = new PIXI.extras.TilingSprite(texture, renderer.width,renderer.height);

    environment.addChildAt(bkgd_texture, num);

    // new logo
    /*num = environment.getChildIndex(logo_sprite);
    environment.removeChild(logo_sprite);

    var logo_texture = PIXI.Texture.fromImage('img/'+logo_img);
    logo_sprite = new PIXI.Sprite(logo_texture);

    logo_sprite.position.x = windowW * 3 / 4;

    environment.addChildAt(logo_sprite, num);*/

}


function req_banner(msg){
	// Dynamically change game background and logo. Checks if the game is "go_fish"
    /*if(msg.s2 == "go_fish") {
	console.log(msg.s2)
	change_background("background2.png","goFish.png")
       	go_fish_arrange = true;

    }
    else*/ 
    if(msg.action != "")
        display_message(msg, 180)
    else
        display_banner(msg.s,200)
}

function animate() {
    //TWEEN.update()
    requestAnimationFrame( animate );
    renderer.render(environment);
}

//more about "window":
// http://www.w3schools.com/jsref/obj_window.asp
window.onresize = function() {
    windowW = window.innerWidth;
    windowH = window.innerHeight;
    renderer.resize(windowW,windowH)
}
