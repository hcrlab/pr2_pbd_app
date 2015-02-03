#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerControl.h>


//#include <string>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <pr2_interactive_controller/InteractionCommand.h>
#include <pr2_interactive_controller/InteractionSettings.h>

#define NUMBER_OF_ARMS 2
#define ARM_ARRAY {"r_wrist_roll_link", "l_wrist_roll_link"}
#define UPDATE_SECONDS 0.1
#define LOOP_RATE 0.05

class InteractionData
{
public:
  std::vector<geometry_msgs::Pose> markerPoses;
  bool updateMarker;
  bool continuousUpdate;
  bool updatePose;
  bool newPoses;
};

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, 
    InteractionData * interaction, int arm_ind)
{
  // ROS_INFO_STREAM( feedback->marker_name << " is now at "
  //     << feedback->pose.position.x << ", " << feedback->pose.position.y
  //     << ", " << feedback->pose.position.z );

  interaction->markerPoses[arm_ind] = feedback->pose;

  interaction->newPoses = true;
  // ros::Duration lapsed = ros::Time::now() - interaction->lastSynced;
  // if (lapsed.toSec() > 1)
  // {
  //   interaction->lastSynced = ros::Time::now();
  //   interaction->arm->setPoseTarget(feedback->pose);
  //   interaction->arm->move();
  // }
}

bool getSettings(pr2_interactive_controller::InteractionSettings::Request  &req,
         pr2_interactive_controller::InteractionSettings::Response &res, InteractionData * interaction)
{
  res.continuos_update = interaction->continuousUpdate;
  return true;
}

void handleCommand(const pr2_interactive_controller::InteractionCommand &com, InteractionData * interaction)
{
  switch (com.command)
  {
    case pr2_interactive_controller::InteractionCommand::UPDATE_POSE:
      interaction->updatePose = true;
      break;
    case pr2_interactive_controller::InteractionCommand::UPDATE_MARKER:
      interaction->updateMarker = true;
      break;
    case pr2_interactive_controller::InteractionCommand::CONTINUOUS_POSE_ON:
      interaction->continuousUpdate = true;
      break;
    case pr2_interactive_controller::InteractionCommand::CONTINUOUS_POSE_OFF:
      interaction->continuousUpdate = false;
      break;
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_controller");

  ros::NodeHandle n;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  moveit::planning_interface::MoveGroup arms("arms");
  std::string armNames[NUMBER_OF_ARMS] = ARM_ARRAY;




  // group.setPoseTarget(pose);
  // group.move();
  // geometry_msgs::Pose pose1;
  // pose1.orientation.w = 1.0;
  // pose1.position.x = 0.28;
  // pose1.position.y = -0.7;
  // pose1.position.z = 1.0;


  // ROS_INFO("sleeping");
  // ros::Duration(1).sleep();
  // ROS_INFO("done");


  // geometry_msgs::PoseStamped stampedPose = r_arm.getCurrentPose();
  // geometry_msgs::Pose pose = stampedPose.pose;
  //ROS_INFO("%s", r_arm.getEndEffectorLink().c_str());// r_wrist_roll_link
  // ROS_INFO("%s", r_arm.getEndEffector().c_str());
  // ROS_INFO("%s", r_arm.getPoseReferenceFrame().c_str());


  InteractionData id;
  for (int i = 0; i < NUMBER_OF_ARMS; i++)
    id.markerPoses.push_back(geometry_msgs::Pose());
  id.updatePose = false;
  id.updateMarker = false;
  id.continuousUpdate = false;







  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("interactive_controller_markers");

  std::vector<visualization_msgs::InteractiveMarker> markers;

  for (int arm_ind = 0; arm_ind < NUMBER_OF_ARMS; arm_ind++) 
  {
    geometry_msgs::PoseStamped stampedPose = arms.getCurrentPose(armNames[arm_ind]);
    geometry_msgs::Pose pose = stampedPose.pose;
    id.markerPoses[arm_ind] = pose;
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = stampedPose.header.frame_id;
    int_marker.name = arm_ind == 0 ? "right_arm" : "left_arm";
    int_marker.description = "Simple 1-DOF Control";
    int_marker.pose = pose;
    int_marker.scale = 0.3;

    // create a grey box marker
    visualization_msgs::Marker sphere_marker;
    sphere_marker.type = visualization_msgs::Marker::SPHERE;
    sphere_marker.scale.x = 0.15;
    sphere_marker.scale.y = 0.15;
    sphere_marker.scale.z = 0.15;
    sphere_marker.color.r = 0.5;
    sphere_marker.color.g = 0.8;
    sphere_marker.color.b = 1;
    sphere_marker.color.a = 0.3;

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl sphere_control;
    sphere_control.always_visible = true;
    sphere_control.markers.push_back( sphere_marker );

    // add the control to the interactive marker
    int_marker.controls.push_back( sphere_control );

    
    visualization_msgs::InteractiveMarkerControl control;


    if (true)//(interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    {
        std::string mode_text;
        mode_text = "MOVE_ROTATE_3D";
        int_marker.name += "_" + mode_text;
        int_marker.description = std::string("3D Control") + (true ? " + 6-DOF controls" : "") + "\n" + mode_text;
    }

    if (true)//(show_6dof)
    {
      control.orientation.w = 1;
      control.orientation.x = 1;
      control.orientation.y = 0;
      control.orientation.z = 0;
      control.name = "rotate_x";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_x";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 1;
      control.orientation.z = 0;
      control.name = "rotate_z";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_z";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);

      control.orientation.w = 1;
      control.orientation.x = 0;
      control.orientation.y = 0;
      control.orientation.z = 1;
      control.name = "rotate_y";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
      int_marker.controls.push_back(control);
      control.name = "move_y";
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
      int_marker.controls.push_back(control);
    }

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, boost::bind(processFeedback, _1, &id, arm_ind));
  }


  


  // 'commit' changes and send to all clients
  server.applyChanges();

  ros::Subscriber sub = n.subscribe<pr2_interactive_controller::InteractionCommand>("interactive_controller_command", 1000, 
      boost::function<void(const pr2_interactive_controller::InteractionCommand)>(boost::bind(handleCommand, _1, &id)));

  ros::ServiceServer service = n.advertiseService("interactive_controller_settings",
    boost::function<bool(pr2_interactive_controller::InteractionSettings::Request&,
         pr2_interactive_controller::InteractionSettings::Response&)>(boost::bind(getSettings, _1, _2, &id)));



  ros::Duration loopRate(LOOP_RATE);

  ros::Time lastUpdate = ros::Time::now();
  while (ros::ok())
  {

    if (id.updatePose)
    {
      if (id.newPoses)
      {
        for (int i = 0; i < NUMBER_OF_ARMS; i++)
          arms.setPoseTarget(id.markerPoses[i], armNames[i]);
        arms.move();
        id.updatePose = false;
        id.newPoses = false;
      }
    }
    if (id.updateMarker)
    {
      id.updateMarker = false;
    }
    if (id.continuousUpdate)
    {
      if (((ros::Time::now() - lastUpdate).toSec() > UPDATE_SECONDS) &&
          id.newPoses)
      {
        lastUpdate = ros::Time::now();
        id.newPoses = false;
        for (int i = 0; i < NUMBER_OF_ARMS; i++)
          arms.setPoseTarget(id.markerPoses[i], armNames[i]);
        arms.move();
      }
    }

    loopRate.sleep();
    // start the ROS main loop
    ros::spinOnce();
  }
}