//initialize ros library

function close_accordion_section() {
			jQuery('.accordion .accordion-section-title').removeClass('active');
			jQuery('.accordion .accordion-section-content').slideUp(300).removeClass('open');
}

var switching = false;
var updated = false;
var right_action_frame = 'default';
var left_action_frame = 'default';
var usingright = true;
var usingleft = true;




var current_action = "none";

var ros = new ROSLIB.Ros({
	url : 'ws://localhost:9090'
  });

var usingright_param = new ROSLIB.Param({
	ros : ros,
    name : 'use_right_arm'
});

usingright_param.set(usingright);

var usingleft_param = new ROSLIB.Param({
	ros : ros,
    name : 'use_left_arm'
});

usingleft_param.set(usingleft);

var right_param = new ROSLIB.Param({
	ros : ros,
	name : 'right_action_tf_frame'
});

right_param.set(right_action_frame);

var left_param = new ROSLIB.Param({
	ros : ros,
	name : 'left_action_tf_frame'
});

left_param.set(left_action_frame);

var speechPub = new ROSLIB.Topic({
	ros : ros,
	name : '/recognized_command',
	messageType : 'pr2_pbd_speech_recognition/Command'
});

var guiPub = new ROSLIB.Topic({
	ros : ros,
	name : '/gui_command',
	messageType : 'pr2_pbd_interaction/GuiCommand'
});

var expListener = new ROSLIB.Topic({
	ros : ros,
	name : '/experiment_state',
	messageType : 'pr2_pbd_interaction/ExperimentState'
});

var resultListener = new ROSLIB.Topic({
	ros : ros,
	name : '/execution_result',
	messageType : 'pr2_pbd_interaction/ExecutionResult'
});

// var tfListener = new ROSLIB.Topic({
// 	ros : ros,
// 	name : '/current_tf_transforms_throttle',
// 	messageType : 'tf/tfMessage'
// });

var expListenerSrvCli = new ROSLIB.Service({
	ros : ros,
	name : '/get_experiment_state',
	serviceType : 'pr2_pbd_interaction/GetExperimentState'
});

var tfSrv = new ROSLIB.Service({
	ros : ros,
	name : '/tf_service',
	serviceType : 'pr2_pbd_interaction/CurrentTf'
});

var actionFrameSrv = new ROSLIB.Service({
	ros : ros,
	name : '/set_action_frame',
	serviceType : 'pr2_pbd_interaction/SetActionFrame'
});

window.lockUpdate = false;

var get_arm_step_id = function (step_number, arm_index) {
            return (2 * step_number + arm_index)
        }

window.addEventListener("load", function() {

    ros.on("error", function() {
        alert("Error connecting to the ROS server. App will not work.")
    });

    ros.on('connection', function() {
	    console.log('Connected to websocket server.');
    });

    ros.on('close', function() {
	    console.log('Connection to websocket server closed.');
    });

    // Create the main viewer.
    var viewer = new ROS3D.Viewer({
      divID : 'visualWindow',
      width : 800,
      height : 600,
      antialias : true
    });

    // Add a grid.
//    viewer.addObject(new ROS3D.Grid());

    // Setup a client to listen to TFs.
    var tfClient = new ROSLIB.TFClient({
      ros : ros,
      angularThres : 0.01,
      transThres : 0.01,
      rate : 10.0
    });

    // Setup the marker client for world objects.
    var objectsClient = new ROS3D.InteractiveMarkerClient({
        ros : ros,
        tfClient : tfClient,
        topic : '/world_objects',
        camera : viewer.camera,
        rootObject : viewer.selectableObjects
    });

    // Setup the marker client.
    var markerClient = new ROS3D.MarkerClient({
      ros : ros,
      tfClient : tfClient,
      topic : '/visualization_marker',
      rootObject : viewer.scene
    });

    // Setup the marker array client..
    var markerClient = new ROS3D.MarkerArrayClient({
      ros : ros,
      tfClient : tfClient,
      topic : '/visualization_marker_array',
      rootObject : viewer.scene
    });

    // Setup the marker client for saved poses.
    var poseClient = new ROS3D.InteractiveMarkerClient({
        ros : ros,
        tfClient : tfClient,
        topic : '/programmed_actions',
        camera : viewer.camera,
        path : 'http://resources.robotwebtools.org/',
        rootObject : viewer.selectableObjects
    });

    // Setup the URDF client.
    var urdfClient = new ROS3D.UrdfClient({
      ros : ros,
      tfClient : tfClient,
      path : 'http://resources.robotwebtools.org/',
      rootObject : viewer.scene
    });

	//hook up buttons with com attribute to speech commands
	[].slice.call(document.querySelectorAll("button[com]")).forEach(function(el) {
		el.addEventListener("click", function() {
			var relCom = new ROSLIB.Message({
				command : el.getAttribute("com")
			});
			speechPub.publish(relCom);
		});
	});

	var actsListCont = document.querySelector("#actsList");
	// var stepsSpan = document.querySelector("#curAct");
	// var frameSelector = document.querySelector("#framesList");
	// var rightFrameOptions = document.createElement("select");
	// rightFrameOptions.id = "rightFrameOptions";
	// frameSelector.appendChild(rightFrameOptions);

	var roboState = {
		recording: false,
		rightRelaxed: false,
		leftRelaxed: false
	};

	var relRightBut = document.querySelector('button[com="relax-right-arm"]');
	var frzRightBut = document.querySelector('button[com="freeze-right-arm"]');
	var relLeftBut = document.querySelector('button[com="relax-left-arm"]');
	var frzLeftBut = document.querySelector('button[com="freeze-left-arm"]');

	//update freeze state ui
	var onStateUpdate = function() {
		relRightBut.style.display = roboState.rightRelaxed ? "none" : "inline-block";
		frzRightBut.style.display = roboState.rightRelaxed ? "inline-block" : "none";
		relLeftBut.style.display = roboState.leftRelaxed ? "none" : "inline-block";
		frzLeftBut.style.display = roboState.leftRelaxed ? "inline-block" : "none";
	};
	onStateUpdate();

	relRightBut.addEventListener("click", function() {
		roboState.rightRelaxed = true;
		onStateUpdate();
	});

	frzRightBut.addEventListener("click", function() {
		roboState.rightRelaxed = false;
		onStateUpdate();
	});

	relLeftBut.addEventListener("click", function() {
		roboState.leftRelaxed = true;
		onStateUpdate();
	});

	frzLeftBut.addEventListener("click", function() {
		roboState.leftRelaxed = false;
		onStateUpdate();
	});



	//code for drawing the current action and actions
	var drawState = function(state) {
		// if (switching === true) {
		// 	console.log("Skipping draw state");
		// 	switching = false;
		// 	return;
		// }
		console.log("Drawing state");
		if (window.lockUpdate) 
			return;
		//draw action list
		actsListCont.innerHTML = "";
		state.action_names.forEach(function(act_n) {
			var ac_section = document.createElement("div");
			
			
			ac_section.className = "accordion-section";
			var ac_section_title = document.createElement("a");
			ac_section_title.className = "accordion-section-title";
			ac_section_title.href = "#" + act_n.replace(/\s/g, '');
			ac_section_title.innerHTML = act_n;
			actsListCont.appendChild(ac_section);
			ac_section.appendChild(ac_section_title);
			var toggle = document.createElement("div");
			var dv = document.createElement("div");
			toggle.appendChild(dv);
			
			toggle.id = act_n.replace(/\s/g, '');
			toggle.style.display = "none";
			
			// dv.innerHTML = act_n;
			ac_section_title.addEventListener("click", function(e) {
				console.log("Just started!");
				if (current_action === e.currentTarget.innerHTML){
					switching = true;
				}
				else {
					switching = false;
					updated = false;
				}
				
				console.log(e.currentTarget.innerHTML);

				current_action = e.currentTarget.innerHTML;
				// removeChildrenFromNode(dv);

			   	var len = dv.childNodes.length;

			    while (dv.hasChildNodes())
			    {
			      dv.removeChild(dv.firstChild);
			    }  

				// Grab current anchor value
				var currentAttrValue = jQuery(this).attr('href');

				if(jQuery(e.target).is('.active')) {
					close_accordion_section();
				}else {
					close_accordion_section();

					// Add active class to section title
					jQuery(this).addClass('active');
					// Open up the hidden content panel
					jQuery('.accordion ' + currentAttrValue).slideDown(300).addClass('open');

				}

				if (!updated){
					toggle.style.display = "none";
				}

				e.preventDefault();

		        console.log("Almost at end!");
		        if (switching) {
		        	switching = false;
		        }
		        else {     
			  		speechPub.publish(new ROSLIB.Message({
						command: "switch-to-action " + act_n
					}));
				}
		  		

				var rightFrameOptions = document.createElement("select");
				rightFrameOptions.id = "rightFrameOptions";
				rightFrameOptions.onchange = function() {
					var index = this.selectedIndex;
					right_action_frame = this.children[index].innerHTML.trim();
					// actionFrameSrv.callService(new ROSLIB.ServiceRequest({}), function(result) {

					// });
					var right_param = new ROSLIB.Param({
					    ros : ros,
					    name : 'right_action_tf_frame'
					  });

					  right_param.set(right_action_frame);
					//console.log(inputText);
				}
				dv.appendChild(rightFrameOptions);

				var rightCheck = document.createElement("input");
				rightCheck.type = "checkbox";
				rightCheck.name = "Right Arm";
				rightCheck.value = "0";
				rightCheck.id = "rightCheck";
				rightCheck.onchange = function() {
					if (usingright){
						usingright = false;
						var usingright_param = new ROSLIB.Param({
							ros : ros,
						    name : 'use_right_arm'
						});

					  	usingright_param.set(usingright);
					}
					else{
						usingright = true;
						var usingright_param = new ROSLIB.Param({
							ros : ros,
						    name : 'use_right_arm'
						});

					  	usingright_param.set(usingright);
					}
				}
				rightCheck.checked = usingright;
				var rightLabel = document.createElement("label");
				var rightDesc = document.createTextNode("Right Arm");
				rightLabel.appendChild(rightCheck);
				rightLabel.appendChild(rightDesc);
				dv.appendChild(rightLabel);
				console.log("Done with click event");

				var leftFrameOptions = document.createElement("select");
				leftFrameOptions.id = "lefttFrameOptions";
				leftFrameOptions.onchange = function() {
					var index = this.selectedIndex;
					left_action_frame = this.children[index].innerHTML.trim();
					// actionFrameSrv.callService(new ROSLIB.ServiceRequest({}), function(result) {
					// 	console.log("Frame service returned");
					// });
					var left_param = new ROSLIB.Param({
					    ros : ros,
					    name : 'left_action_tf_frame'
					  });

					  left_param.set(left_action_frame);
					console.log("changing selector");
				}
				dv.appendChild(leftFrameOptions);

		        tfSrv.callService(new ROSLIB.ServiceRequest({}), function(result) {
					console.log("Tf service called");

					
					console.log("Processing tf result");

					while( rightFrameOptions.firstChild ) {
					    rightFrameOptions.removeChild( rightFrameOptions.firstChild );
					}

					for (var i = 0; i < result.current_transforms.transforms.length; i++) {
					    var option = document.createElement("option");
					    option.value = result.current_transforms.transforms[i].header.frame_id;
					    option.text = result.current_transforms.transforms[i].header.frame_id;
					    rightFrameOptions.appendChild(option);
					    // console.log(result.current_transforms.transforms[i].header.frame_id);
					}
					var option = document.createElement("option");
				    option.value = "default";
				    option.text = "default";
					rightFrameOptions.appendChild(option);
					for(var i, j = 0; i = rightFrameOptions.options[j]; j++) {
					    if(i.value == right_action_frame) {
					        rightFrameOptions.selectedIndex = j;
					        break;
					    }
					}

					while( leftFrameOptions.firstChild ) {
					    leftFrameOptions.removeChild( leftFrameOptions.firstChild );
					}

					for (var i = 0; i < result.current_transforms.transforms.length; i++) {
					    var option = document.createElement("option");
					    option.value = result.current_transforms.transforms[i].header.frame_id;
					    option.text = result.current_transforms.transforms[i].header.frame_id;
					    leftFrameOptions.appendChild(option);
					    // console.log(result.current_transforms.transforms[i].header.frame_id);
					}
					var option = document.createElement("option");
				    option.value = "default";
				    option.text = "default";
					leftFrameOptions.appendChild(option);

					for(var i, j = 0; i = leftFrameOptions.options[j]; j++) {
					    if(i.value == left_action_frame) {
					        leftFrameOptions.selectedIndex = j;
					        break;
					    }
					}


				});

				var leftCheck = document.createElement("input");
				leftCheck.type = "checkbox";
				leftCheck.name = "Left Arm";
				leftCheck.value = "0";
				leftCheck.id = "leftCheck";
				leftCheck.onchange = function() {
					if (usingleft){
						usingleft = false;
						var usingleft_param = new ROSLIB.Param({
							ros : ros,
						    name : 'use_left_arm'
						});

					  	usingleft_param.set(usingleft);
					}
					else{
						usingleft = true;
						var usingleft_param = new ROSLIB.Param({
							ros : ros,
						    name : 'use_left_arm'
						});

					  	usingleft_param.set(usingleft);
					}
				}
				leftCheck.checked = usingleft;
				var leftLabel = document.createElement("label");
				var leftDesc = document.createTextNode("Left Arm");
				leftLabel.appendChild(leftCheck);
				leftLabel.appendChild(leftDesc);
				dv.appendChild(leftLabel);
				console.log("Done with click event");


			});
			
			toggle.className = "accordion-section-content";
			
			ac_section.appendChild(toggle);
			//current action:
			stepsSpan = document.createElement("div");

			toggle.appendChild(stepsSpan);
			if (true){ 
				var action = jsyaml.load(state.action_str);
				//html for one step
				var dispStep = function(step_act, i) {
					console.log(step_act);
					console.log(step_act.armTarget.state[0].state[0]);

					step_uses_left = false;
					step_uses_right = false;
					if (step_act.armTarget.state[4]){
						step_uses_right = true;
					}

					if (step_act.armTarget.state[5]){
						step_uses_left = true;
					}


					if (step_act.armTarget.state[0].state[0] === 0){
						right_frame = "base_link";
					}
					else{
						right_frame = step_act.armTarget.state[0].state[4];
					}
					console.log(right_frame);
					if (step_act.armTarget.state[1].state[0] === 0){
						left_frame = "base_link";
					}
					else{
						left_frame = step_act.armTarget.state[1].state[4];
					}
					console.log(left_frame);
		

		            var stepRow = document.createElement("tr");
		            var stepIndexCol = document.createElement("td");
		            
		            stepRow.appendChild(stepIndexCol);
		            var stepIndexNode = document.createTextNode(i+1);
		            stepIndexCol.appendChild(stepIndexNode);

		            if (step_uses_right){
			            var rightFrameCol = document.createElement("td");
			            stepRow.appendChild(rightFrameCol);
			            rightFrameNode = document.createTextNode("Right frame: " + right_frame);
			        
			            rightFrameCol.appendChild(rightFrameNode);
			        }
			        if (step_uses_left){
			        	var leftFrameCol = document.createElement("td");
			            stepRow.appendChild(leftFrameCol);
			            leftFrameNode = document.createTextNode("Left frame: " + left_frame);
			           
			            leftFrameCol.appendChild(leftFrameNode);
			        }
			        if (step_uses_right){
			            var rightCol = document.createElement("td")
			            stepRow.appendChild(rightCol);
			        }
			        if (step_uses_left){
		            	var leftCol = document.createElement("td")
		            	stepRow.appendChild(leftCol);
		            }
		            if (step_uses_right){
			            var selectRightBut = document.createElement("button");
			            selectRightBut.innerHTML = "Select right";
			            selectRightBut.addEventListener("click", function() {
			                guiPub.publish(new ROSLIB.Message({
			                    command: "select-step",
			                    param: get_arm_step_id(i, 0)
			                }));
			             });
			            rightCol.appendChild(selectRightBut)
			        }
			        if (step_uses_left){
			            var selectLeftBut = document.createElement("button");
			            selectLeftBut.innerHTML = "Select left";
			            selectLeftBut.addEventListener("click", function() {
			                guiPub.publish(new ROSLIB.Message({
			                    command: "select-step",
			                    param: get_arm_step_id(i, 1)
			                }));
			             });
			            leftCol.appendChild(selectLeftBut)
			        }
		            var delCol = document.createElement("td")
		            stepRow.appendChild(delCol);
		            var delBut = document.createElement("button");
		            delBut.innerHTML = "Delete";
		            delBut.addEventListener("click", function() {
		                guiPub.publish(new ROSLIB.Message({
		                    command: "delete-step",
		                    param: i
		                }));
		            });
		            delCol.appendChild(delBut);
		            return stepRow;
				};

				//go though action steps and add them all to the gui
				action.arm_steps.forEach(function(step_act, i) {
					stepsSpan.appendChild(dispStep(step_act, i));
				});
				if (ac_section_title.innerHTML === current_action){
					ac_section_title.click();
				}
				console.log("Done with action");

			}

			console.log("Done drawing state");

		});
		// actsListCont.querySelectorAll("div")[state.selected_action].className = 
		// 	"selected";

		newNameInp.value = state.action_names[state.selected_action];

		

		// var action = jsyaml.load(state.action_str);
		// //html for one step
		// var dispStep = function(step_act, i) {
  //           var stepRow = document.createElement("tr");
  //           var stepIndexCol = document.createElement("td")
  //           stepRow.appendChild(stepIndexCol);
  //           var stepIndexNode = document.createTextNode(i+1);
  //           stepIndexCol.appendChild(stepIndexNode);
  //           var rightCol = document.createElement("td")
  //           stepRow.appendChild(rightCol);
  //           var leftCol = document.createElement("td")
  //           stepRow.appendChild(leftCol);
  //           var selectRightBut = document.createElement("button");
  //           selectRightBut.innerHTML = "Select right";
  //           selectRightBut.addEventListener("click", function() {
  //               guiPub.publish(new ROSLIB.Message({
  //                   command: "select-step",
  //                   param: get_arm_step_id(i, 0)
  //               }));
  //            });
  //           rightCol.appendChild(selectRightBut)
  //           var selectLeftBut = document.createElement("button");
  //           selectLeftBut.innerHTML = "Select left";
  //           selectLeftBut.addEventListener("click", function() {
  //               guiPub.publish(new ROSLIB.Message({
  //                   command: "select-step",
  //                   param: get_arm_step_id(i, 1)
  //               }));
  //            });
  //           leftCol.appendChild(selectLeftBut)
  //           var delCol = document.createElement("td")
  //           stepRow.appendChild(delCol);
  //           var delBut = document.createElement("button");
  //           delBut.innerHTML = "Delete";
  //           delBut.addEventListener("click", function() {
  //               guiPub.publish(new ROSLIB.Message({
  //                   command: "delete-step",
  //                   param: i
  //               }));
  //           });
  //           delCol.appendChild(delBut);
  //           return stepRow;
		// };

		// //go though action steps and add them all to the gui
		// action.arm_steps.forEach(function(step_act, i) {
		// 	stepsSpan.appendChild(dispStep(step_act, i));
		// });
		

		
	};

	expListener.subscribe(function(state) {
		console.log("Listener");
		updated = true;
		drawState(state);
	});

	var processResult = function(result) {
	    console.log('Execution status: ' + result.status.status + '. Message: ' + result.error_msg)
	    if (result.status.status !== 1) {
	        if (result.error_msg !== '') {
	            alert('Execution of an action failed with the following error: ' + result.error_msg);
	        }
	    }
	};

	resultListener.subscribe(function(result) {
		processResult(result);
	});

	

	// tfListener.subscribe(function(result) {
	// 	addTfOptions(result);
	// });

	expListenerSrvCli.callService(new ROSLIB.ServiceRequest({}), function(result) {
		console.log("Service client call");
		drawState(result.state);
	});

	var overlayDiv = document.querySelector("#overlay");
	var newNameInp = document.querySelector("#newName");

	document.querySelector("#renPopup").addEventListener("click", function() {
		overlayDiv.style.display = "";
	});
	document.querySelector("#doRename").addEventListener("click", function() {
		speechPub.publish(new ROSLIB.Message({
			command: "name-action " + newNameInp.value
		}));
		overlayDiv.style.display = "none";
	});
	document.querySelector("#cancelRename").addEventListener("click", function() {
		overlayDiv.style.display = "none";
	});
});