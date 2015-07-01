//initialize ros library

function close_accordion_section() {
			jQuery('.accordion .accordion-section-title').removeClass('active');
			jQuery('.accordion .accordion-section-content').slideUp(300).removeClass('open');
}

var switching = false;

var current_action = "none";

var ros = new ROSLIB.Ros({
	url : 'ws://localhost:9090'
  });

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
	// var frameOptions = document.createElement("select");
	// frameOptions.id = "frameOptions";
	// frameSelector.appendChild(frameOptions);

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
		if (switching === true) {
			console.log("Skipping draw state");
			switching = false;
			return;
		}
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
				current_action = act_n;
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

				e.preventDefault();

		        console.log("Almost at end!");     
		  		speechPub.publish(new ROSLIB.Message({
					command: "switch-to-action " + act_n
				}));

				switching = true;

				var frameOptions = document.createElement("select");
				frameOptions.id = "frameOptions";
				dv.appendChild(frameOptions);

		        tfSrv.callService(new ROSLIB.ServiceRequest({}), function(result) {
					console.log("Tf service called");

					
					console.log("Processing tf result");

					while( frameOptions.firstChild ) {
					    frameOptions.removeChild( frameOptions.firstChild );
					}

					for (var i = 0; i < result.current_transforms.transforms.length; i++) {
					    var option = document.createElement("option");
					    option.value = result.current_transforms.transforms[i].header.frame_id;
					    option.text = result.current_transforms.transforms[i].header.frame_id;
					    frameOptions.appendChild(option);
					    // console.log(result.current_transforms.transforms[i].header.frame_id);
					}
					
					
				});

				var rightCheck = document.createElement("input");
				rightCheck.type = "checkbox";
				rightCheck.name = "Right Arm";
				rightCheck.value = "0";
				rightCheck.id = "rightCheck";
				var rightLabel = document.createElement("label");
				var rightDesc = document.createTextNode("Right Arm");
				rightLabel.appendChild(rightCheck);
				rightLabel.appendChild(rightDesc);
				dv.appendChild(rightLabel);

				var leftCheck = document.createElement("input");
				leftCheck.type = "checkbox";
				leftCheck.name = "Left Arm";
				leftCheck.value = "0";
				leftCheck.id = "leftCheck";
				var leftLabel = document.createElement("label");
				var leftDesc = document.createTextNode("Left Arm");
				leftLabel.appendChild(leftCheck);
				leftLabel.appendChild(leftDesc);
				dv.appendChild(leftLabel);


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
		            var stepRow = document.createElement("tr");
		            var stepIndexCol = document.createElement("td")
		            stepRow.appendChild(stepIndexCol);
		            var stepIndexNode = document.createTextNode(i+1);
		            stepIndexCol.appendChild(stepIndexNode);
		            var rightCol = document.createElement("td")
		            stepRow.appendChild(rightCol);
		            var leftCol = document.createElement("td")
		            stepRow.appendChild(leftCol);
		            var selectRightBut = document.createElement("button");
		            selectRightBut.innerHTML = "Select right";
		            selectRightBut.addEventListener("click", function() {
		                guiPub.publish(new ROSLIB.Message({
		                    command: "select-step",
		                    param: get_arm_step_id(i, 0)
		                }));
		             });
		            rightCol.appendChild(selectRightBut)
		            var selectLeftBut = document.createElement("button");
		            selectLeftBut.innerHTML = "Select left";
		            selectLeftBut.addEventListener("click", function() {
		                guiPub.publish(new ROSLIB.Message({
		                    command: "select-step",
		                    param: get_arm_step_id(i, 1)
		                }));
		             });
		            leftCol.appendChild(selectLeftBut)
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
			}

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