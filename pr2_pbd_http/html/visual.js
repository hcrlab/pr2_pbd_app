window.addEventListener("load", function() {
	document.querySelector("#visualSection").style.display = "block";
	var ros = new ROSLIB.Ros({
	  url : 'ws://localhost:9090'
	});

	// Create the main viewer.
	var viewer = new ROS3D.Viewer({
	  divID : 'visualWindow',
	  width : 800,
	  height : 600,
	  antialias : true
	});

	// Add a grid.
	viewer.addObject(new ROS3D.Grid());

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

    // Setup the marker client for saved poses.
    var poseClient = new ROS3D.InteractiveMarkerClient({
        ros : ros,
        tfClient : tfClient,
        topic : '/programmed_actions',
        camera : viewer.camera,
        rootObject : viewer.selectableObjects
    });

    // Setup the marker client for robot interaction.
    var imClient = new ROS3D.InteractiveMarkerClient({
      ros : ros,
      tfClient : tfClient,
      topic : '/interactive_controller_markers',
      camera : viewer.camera,
      rootObject : viewer.selectableObjects
    });

	// Setup the URDF client.
	var urdfClient = new ROS3D.UrdfClient({
	  ros : ros,
	  tfClient : tfClient,
	  path : 'http://resources.robotwebtools.org/',
	  rootObject : viewer.scene
	});

	var intSetsSrv = new ROSLIB.Service({
		ros : ros,
		name : '/interactive_controller_settings',
		serviceType : 'pr2_interactive_controller/InteractionSettings'
	});

	var conChb = document.querySelector("#contUpd");
	var updBut = document.querySelector("#updatePose");

	var comPub = new ROSLIB.Topic({
		ros : ros,
		name : '/interactive_controller_command',
		messageType : 'pr2_interactive_controller/InteractionCommand'
	});

	intSetsSrv.callService(new ROSLIB.ServiceRequest({}), function(result) {
		 conChb.checked = result.continuos_update;
	});

	conChb.addEventListener("change", function() {
		comPub.publish(new ROSLIB.Message({
			command: conChb.checked ? 2 : 3
		}));
	});

	updBut.addEventListener("click", function() {
		comPub.publish(new ROSLIB.Message({
			command: 0
		}));
	});
});