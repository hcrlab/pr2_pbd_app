//initialize ros library
var ros = new ROSLIB.Ros({
	url : 'ws://' + window.location.hostname + ':9090'
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

var expListenerSrvCli = new ROSLIB.Service({
	ros : ros,
	name : '/get_experiment_state',
	serviceType : 'pr2_pbd_interaction/GetExperimentState'
});

window.lockUpdate = false;

window.addEventListener("load", function() {
	var loadPageVar = function (sVar) {
	  return decodeURI(window.location.search.replace(new RegExp("^(?:.*[&\\?]" + encodeURI(sVar).replace(/[\.\+\*]/g, "\\$&") + "(?:\\=([^&]*))?)?.*$", "i"), "$1"));
	};

	if (loadPageVar("visual").toLowerCase() == "true") {
		var iFrame = document.createElement("iframe");
		iFrame.src = "visual.html";
		iFrame.style.width = "100%";
		iFrame.style.height = "600px";
		document.querySelector("#visualSection").appendChild(iFrame);
	}


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
	var addButSpan = document.querySelector("#addButs");
	var delButSpan = document.querySelector("#delButs");
	var stepsSpan = document.querySelector("#curAct");

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
		if (window.lockUpdate) 
			return;
		//draw action list
		actsListCont.innerHTML = "";
		state.action_names.forEach(function(act_n) {
			var dv = document.createElement("div");
			dv.innerHTML = act_n;
			dv.addEventListener("click", function() {
				speechPub.publish(new ROSLIB.Message({
					command: "switch-to-action " + act_n
				}));
			});
			actsListCont.appendChild(dv);
		});
		actsListCont.querySelectorAll("div")[state.selected_action].className = 
			"selected";

		newNameInp.value = state.action_names[state.selected_action];

		//current action:
		addButSpan.innerHTML = "";
		delButSpan.innerHTML = "";
		stepsSpan.innerHTML = "";

		var action = jsyaml.load(state.action_str);
		//html for one step
		var dispStep = function(step_act, i) {

			var outerCont = document.createElement("div");

			var sel = document.createElement("select");
			sel.innerHTML = "<option value='0'>Action</option>" + 
				"<option value='1'>Pose</option><option value='3'>Trajectory</option>";
			outerCont.appendChild(sel);

			//html for stuff besides +/-, and type buttons
			var genHtml = function(actType) {
				var stepCont = document.createElement("span");

				switch (actType) {
					case 0://action
						sel.selectedIndex = 0;
						var actsSelect = document.createElement("select");
						actsSelect.innerHTML = state.action_names.reduce(function(c, nm) {
							return c + "<option value='" + nm + "'>" + nm + "</option>";
						}, "");
						actsSelect.selectedIndex = state.action_ids.indexOf(parseInt(step_act.id));
						stepCont.appendChild(actsSelect);

						var saveBut = document.createElement("button");
						saveBut.innerHTML = "save";
						stepCont.appendChild(saveBut);
						saveBut.addEventListener("click", function() {
							guiPub.publish(new ROSLIB.Message({
								command: "select-action-step",
								param: (i + 1)
							}));
							speechPub.publish(new ROSLIB.Message({
								command: "delete-last-step"
							}));
							speechPub.publish(new ROSLIB.Message({
								command: "add-action-step " + actsSelect.value
							}));
						});
						break;
					case 1://pose
						sel.selectedIndex = 1;
						var recPose = document.createElement("button");
						recPose.innerHTML = "record";
						stepCont.appendChild(recPose);
						recPose.addEventListener("click", function() {
							guiPub.publish(new ROSLIB.Message({
								command: "select-action-step",
								param: (i + 1)
							}));
							speechPub.publish(new ROSLIB.Message({
								command: "delete-last-step"
							}));
							speechPub.publish(new ROSLIB.Message({
								command: "save-pose"
							}));
						});
						break;
					case 2://gripper
						
						break;
					case 3://trajectory
						sel.selectedIndex = 2;
						var recBut = document.createElement("button");
						recBut.innerHTML = "Record";
						stepCont.appendChild(recBut);
						recBut.addEventListener("click", function() {
							if (roboState.recording) {
								recBut.innerHTML = "Record";
								speechPub.publish(new ROSLIB.Message({
									command: "stop-recording-motion"
								}));
							} else {
								window.lockUpdate = true;
								recBut.innerHTML = "Stop";
								guiPub.publish(new ROSLIB.Message({
									command: "select-action-step",
									param: (i + 1)
								}));
								speechPub.publish(new ROSLIB.Message({
									command: "delete-last-step"
								}));
								speechPub.publish(new ROSLIB.Message({
									command: "start-recording-motion"
								}));
							}
							roboState.recording = !roboState.recording;
						});
						break;
				}

				["right", "left"].forEach(function(arm_name, arm_ind) {
					var selRefFrame = document.createElement("select");
					state.object_names.forEach(function(oName) {
						var opt = document.createElement("option");
						opt.value = oName;
						opt.innerHTML = oName;
						selRefFrame.appendChild(opt);
					});
					var spanSideName = document.createElement("span");
					spanSideName.appendChild(document.createTextNode(arm_name + " reference frame"));
					stepCont.appendChild(spanSideName);

					stepCont.appendChild(selRefFrame);
					selRefFrame.addEventListener("change", function() {
						guiPub.publish(new ROSLIB.Message({
							command: "select-action-step",
							param: (i)
						}));
						speechPub.publish(new ROSLIB.Message({
							command: "set-step-relativity-" + arm_name + " " + selRefFrame.value
						}));
					});
					selRefFrame.selectedIndex = state.object_names.indexOf(step_act.landmark_types[arm_ind].friendly_name);
				});

				var selScanMode = document.createElement("select");
				["set-step-to-no-scan", "set-step-to-scan", "set-step-to-scan-move-arms"].forEach(function(oName) {
					var opt = document.createElement("option");
					opt.value = oName;
					opt.innerHTML = oName;
					selScanMode.appendChild(opt);
				});
				stepCont.appendChild(selScanMode);
				selScanMode.addEventListener("change", function() {
					guiPub.publish(new ROSLIB.Message({
						command: "select-action-step",
						param: (i)
					}));
					speechPub.publish(new ROSLIB.Message({
						command: selScanMode.value
					}));
				});
				selScanMode.selectedIndex = step_act.scan_code;

				return stepCont;
			};

			sel.addEventListener("change", function () {
				outerCont.querySelector("span").remove();
				outerCont.appendChild(genHtml(parseInt(sel.value)));
			})

			outerCont.appendChild(genHtml(step_act.type));

			return outerCont;
		};

		//button to add a step
		var addBut = document.createElement("button");
		addBut.innerHTML = "+";
		addButSpan.appendChild(addBut);
		addBut.addEventListener("click", function() {
			guiPub.publish(new ROSLIB.Message({
				command: "select-action-step",
				param: 0
			}));
			speechPub.publish(new ROSLIB.Message({
				command: "save-pose"
			}));
		});

		//go though action steps and add them all to the gui
		action.actions.forEach(function(step_act, i) {
			var delBut = document.createElement("button");
			delBut.innerHTML = "-";
			delButSpan.appendChild(delBut);
			delBut.addEventListener("click", function() {
				guiPub.publish(new ROSLIB.Message({
					command: "select-action-step",
					param: (i + 1)
				}));
				speechPub.publish(new ROSLIB.Message({
					command: "delete-last-step"
				}));
			});

			var addBut = document.createElement("button");
			addBut.innerHTML = "+";
			addButSpan.appendChild(addBut);
			addBut.addEventListener("click", function() {
				guiPub.publish(new ROSLIB.Message({
					command: "select-action-step",
					param: (i + 1)
				}));
				speechPub.publish(new ROSLIB.Message({
					command: "save-pose"
				}));
			});
			stepsSpan.appendChild(dispStep(step_act, i));
		});
	};

	expListener.subscribe(function(state) {
		drawState(state);
	});


	expListenerSrvCli.callService(new ROSLIB.ServiceRequest({}), function(result) {
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