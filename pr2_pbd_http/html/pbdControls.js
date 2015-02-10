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

//	if (loadPageVar("visual").toLowerCase() == "true") {
//		var iFrame = document.createElement("iframe");
//		iFrame.src = "visual.html";
//		iFrame.style.width = "100%";
//		iFrame.style.height = "600px";
//		document.querySelector("#visualSection").appendChild(iFrame);
//	}


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

//			var sel = document.createElement("select");
//			sel.innerHTML = "<option value='Action'>Action</option>" +
//				"<option value='ManipulationStep'>ManipulationStep</option>";
//			outerCont.appendChild(sel);

			//html for stuff besides +/-, and type buttons
			var genHtml = function(step_act) {
			    var actType = step_act.step_type
				var stepCont = document.createElement("span");

                var typeLabel = document.createElement("span");
                typeLabel.innerHTML = actType;
                stepCont.appendChild(typeLabel);

                var delBut = document.createElement("button");
                delBut.innerHTML = "Delete step";
                delButSpan.appendChild(delBut);
                delBut.addEventListener("click", function() {
                    guiPub.publish(new ROSLIB.Message({
                        command: "delete-step",
                        param: i
                    }));
                });
                stepCont.appendChild(delBut);

				switch (actType) {
					case "Action"://action
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
					case "ManipulationStep"://manipulation
					    var stepTable = document.createElement("table")
					    step_act.arm_steps.forEach(function(step, arm_step_index) {
					            var stepRow = document.createElement("tr")
                                var stepIndexNode = document.createTextNode(arm_step_index);
                                stepRow.appendChild(stepIndexNode);
					            var selectButton = document.createElement("button");
                                selectButton.innerHTML = "Select";
                                stepRow.appendChild(selectButton);
                                selectButton.addEventListener("click", function() {
                                    guiPub.publish(new ROSLIB.Message({
                                        command: "select-arm-step",
                                        param: arm_step_index
                                    }));
				                 });
                                stepTable.appendChild(stepRow);
				        });
                        stepCont.appendChild(stepTable);
						break;
					case 2://gripper
						
						break;
					default:
					    break;
				}

				return stepCont;
			};

//			sel.addEventListener("change", function () {
//				outerCont.querySelector("span").remove();
//				outerCont.appendChild(genHtml(sel.value));
//			})

			outerCont.appendChild(genHtml(step_act));

			return outerCont;
		};

		//go though action steps and add them all to the gui
		action.steps.forEach(function(step_act, i) {
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