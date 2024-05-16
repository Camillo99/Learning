# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/home/camillo/noetic_ws/src/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/home/camillo/noetic_ws/src/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/camillo/noetic_ws/devel_isolated/xacro;/home/camillo/noetic_ws/devel_isolated/visualization_marker_tutorials;/home/camillo/noetic_ws/devel_isolated/urdf_tutorial;/home/camillo/noetic_ws/devel_isolated/rviz_python_tutorial;/home/camillo/noetic_ws/devel_isolated/rviz_plugin_tutorials;/home/camillo/noetic_ws/devel_isolated/rqt_rviz;/home/camillo/noetic_ws/devel_isolated/librviz_tutorial;/home/camillo/noetic_ws/devel_isolated/rviz;/home/camillo/noetic_ws/devel_isolated/robot_state_publisher;/home/camillo/noetic_ws/devel_isolated/kdl_parser;/home/camillo/noetic_ws/devel_isolated/urdf;/home/camillo/noetic_ws/devel_isolated/turtle_tf2;/home/camillo/noetic_ws/devel_isolated/turtle_tf;/home/camillo/noetic_ws/devel_isolated/turtle_actionlib;/home/camillo/noetic_ws/devel_isolated/turtlesim;/home/camillo/noetic_ws/devel_isolated/tf_conversions;/home/camillo/noetic_ws/devel_isolated/tf2_kdl;/home/camillo/noetic_ws/devel_isolated/laser_geometry;/home/camillo/noetic_ws/devel_isolated/interactive_marker_tutorials;/home/camillo/noetic_ws/devel_isolated/interactive_markers;/home/camillo/noetic_ws/devel_isolated/tf2_geometry_msgs;/home/camillo/noetic_ws/devel_isolated/tf;/home/camillo/noetic_ws/devel_isolated/tf2_ros;/home/camillo/noetic_ws/devel_isolated/stereo_msgs;/home/camillo/noetic_ws/devel_isolated/rqt_image_view;/home/camillo/noetic_ws/devel_isolated/map_msgs;/home/camillo/noetic_ws/devel_isolated/image_transport;/home/camillo/noetic_ws/devel_isolated/cv_bridge;/home/camillo/noetic_ws/devel_isolated/sensor_msgs;/home/camillo/noetic_ws/devel_isolated/rqt_launch;/home/camillo/noetic_ws/devel_isolated/roswtf;/home/camillo/noetic_ws/devel_isolated/nodelet_topic_tools;/home/camillo/noetic_ws/devel_isolated/diagnostic_analysis;/home/camillo/noetic_ws/devel_isolated/actionlib_tutorials;/home/camillo/noetic_ws/devel_isolated/actionlib;/home/camillo/noetic_ws/devel_isolated/rosbag;/home/camillo/noetic_ws/devel_isolated/topic_tools;/home/camillo/noetic_ws/devel_isolated/tf2_py;/home/camillo/noetic_ws/devel_isolated/smach_ros;/home/camillo/noetic_ws/devel_isolated/self_test;/home/camillo/noetic_ws/devel_isolated/rqt_reconfigure;/home/camillo/noetic_ws/devel_isolated/rqt_gui_py;/home/camillo/noetic_ws/devel_isolated/rqt_gui_cpp;/home/camillo/noetic_ws/devel_isolated/rostopic;/home/camillo/noetic_ws/devel_isolated/rospy_tutorials;/home/camillo/noetic_ws/devel_isolated/rosnode;/home/camillo/noetic_ws/devel_isolated/rosmsg;/home/camillo/noetic_ws/devel_isolated/rosbag_storage;/home/camillo/noetic_ws/devel_isolated/message_filters;/home/camillo/noetic_ws/devel_isolated/joint_state_publisher;/home/camillo/noetic_ws/devel_isolated/filters;/home/camillo/noetic_ws/devel_isolated/dynamic_reconfigure;/home/camillo/noetic_ws/devel_isolated/diagnostic_common_diagnostics;/home/camillo/noetic_ws/devel_isolated/diagnostic_updater;/home/camillo/noetic_ws/devel_isolated/diagnostic_aggregator;/home/camillo/noetic_ws/devel_isolated/rosout;/home/camillo/noetic_ws/devel_isolated/roscpp_tutorials;/home/camillo/noetic_ws/devel_isolated/pluginlib_tutorials;/home/camillo/noetic_ws/devel_isolated/nodelet_tutorial_math;/home/camillo/noetic_ws/devel_isolated/nodelet;/home/camillo/noetic_ws/devel_isolated/bondpy;/home/camillo/noetic_ws/devel_isolated/bondcpp;/home/camillo/noetic_ws/devel_isolated/roscpp;/home/camillo/noetic_ws/devel_isolated/xmlrpcpp;/home/camillo/noetic_ws/devel_isolated/webkit_dependency;/home/camillo/noetic_ws/devel_isolated/viz;/home/camillo/noetic_ws/devel_isolated/visualization_tutorials;/home/camillo/noetic_ws/devel_isolated/visualization_msgs;/home/camillo/noetic_ws/devel_isolated/urdf_parser_plugin;/home/camillo/noetic_ws/devel_isolated/control_msgs;/home/camillo/noetic_ws/devel_isolated/trajectory_msgs;/home/camillo/noetic_ws/devel_isolated/tf2;/home/camillo/noetic_ws/devel_isolated/tf2_msgs;/home/camillo/noetic_ws/devel_isolated/std_srvs;/home/camillo/noetic_ws/devel_isolated/smach_msgs;/home/camillo/noetic_ws/devel_isolated/shape_msgs;/home/camillo/noetic_ws/devel_isolated/rqt_py_common;/home/camillo/noetic_ws/devel_isolated/rosgraph_msgs;/home/camillo/noetic_ws/devel_isolated/nav_msgs;/home/camillo/noetic_ws/devel_isolated/kdl_conversions;/home/camillo/noetic_ws/devel_isolated/eigen_conversions;/home/camillo/noetic_ws/devel_isolated/geometry_msgs;/home/camillo/noetic_ws/devel_isolated/diagnostic_msgs;/home/camillo/noetic_ws/devel_isolated/bond;/home/camillo/noetic_ws/devel_isolated/actionlib_msgs;/home/camillo/noetic_ws/devel_isolated/std_msgs;/home/camillo/noetic_ws/devel_isolated/smclib;/home/camillo/noetic_ws/devel_isolated/smach;/home/camillo/noetic_ws/devel_isolated/rqt_web;/home/camillo/noetic_ws/devel_isolated/rqt_topic;/home/camillo/noetic_ws/devel_isolated/rqt_top;/home/camillo/noetic_ws/devel_isolated/rqt_tf_tree;/home/camillo/noetic_ws/devel_isolated/rqt_srv;/home/camillo/noetic_ws/devel_isolated/rqt_shell;/home/camillo/noetic_ws/devel_isolated/rqt_service_caller;/home/camillo/noetic_ws/devel_isolated/rqt_runtime_monitor;/home/camillo/noetic_ws/devel_isolated/rqt_robot_steering;/home/camillo/noetic_ws/devel_isolated/rqt_robot_plugins;/home/camillo/noetic_ws/devel_isolated/rqt_robot_monitor;/home/camillo/noetic_ws/devel_isolated/rqt_robot_dashboard;/home/camillo/noetic_ws/devel_isolated/rqt_py_console;/home/camillo/noetic_ws/devel_isolated/rqt_publisher;/home/camillo/noetic_ws/devel_isolated/rqt_pose_view;/home/camillo/noetic_ws/devel_isolated/rqt_plot;/home/camillo/noetic_ws/devel_isolated/rqt_nav_view;/home/camillo/noetic_ws/devel_isolated/rqt_msg;/home/camillo/noetic_ws/devel_isolated/rqt_moveit;/home/camillo/noetic_ws/devel_isolated/rqt_logger_level;/home/camillo/noetic_ws/devel_isolated/rqt_gui;/home/camillo/noetic_ws/devel_isolated/rqt_graph;/home/camillo/noetic_ws/devel_isolated/rqt_dep;/home/camillo/noetic_ws/devel_isolated/rqt_console;/home/camillo/noetic_ws/devel_isolated/rqt_common_plugins;/home/camillo/noetic_ws/devel_isolated/rqt_bag_plugins;/home/camillo/noetic_ws/devel_isolated/rqt_bag;/home/camillo/noetic_ws/devel_isolated/rqt_action;/home/camillo/noetic_ws/devel_isolated/rostest;/home/camillo/noetic_ws/devel_isolated/roslz4;/home/camillo/noetic_ws/devel_isolated/rosconsole_bridge;/home/camillo/noetic_ws/devel_isolated/resource_retriever;/home/camillo/noetic_ws/devel_isolated/qt_gui_cpp;/home/camillo/noetic_ws/devel_isolated/pluginlib;/home/camillo/noetic_ws/devel_isolated/rosconsole;/home/camillo/noetic_ws/devel_isolated/angles;/home/camillo/noetic_ws/devel_isolated/rosunit;/home/camillo/noetic_ws/devel_isolated/roslaunch;/home/camillo/noetic_ws/devel_isolated/python_qt_binding;/home/camillo/noetic_ws/devel_isolated/roscpp_serialization;/home/camillo/noetic_ws/devel_isolated/rostime;/home/camillo/noetic_ws/devel_isolated/rosservice;/home/camillo/noetic_ws/devel_isolated/rospy;/home/camillo/noetic_ws/devel_isolated/rosparam;/home/camillo/noetic_ws/devel_isolated/roslib;/home/camillo/noetic_ws/devel_isolated/rospack;/home/camillo/noetic_ws/devel_isolated/rosmaster;/home/camillo/noetic_ws/devel_isolated/rosmake;/home/camillo/noetic_ws/devel_isolated/roslisp;/home/camillo/noetic_ws/devel_isolated/roslint;/home/camillo/noetic_ws/devel_isolated/roslang;/home/camillo/noetic_ws/devel_isolated/rosgraph;/home/camillo/noetic_ws/devel_isolated/roscreate;/home/camillo/noetic_ws/devel_isolated/roscpp_traits;/home/camillo/noetic_ws/devel_isolated/roscpp_core;/home/camillo/noetic_ws/devel_isolated/rosclean;/home/camillo/noetic_ws/devel_isolated/rosbuild;/home/camillo/noetic_ws/devel_isolated/rosboost_cfg;/home/camillo/noetic_ws/devel_isolated/rosbash;/home/camillo/noetic_ws/devel_isolated/rosbag_migration_rule;/home/camillo/noetic_ws/devel_isolated/ros_tutorials;/home/camillo/noetic_ws/devel_isolated/ros_environment;/home/camillo/noetic_ws/devel_isolated/ros_core;/home/camillo/noetic_ws/devel_isolated/ros_comm;/home/camillo/noetic_ws/devel_isolated/ros_base;/home/camillo/noetic_ws/devel_isolated/ros;/home/camillo/noetic_ws/devel_isolated/robot;/home/camillo/noetic_ws/devel_isolated/qwt_dependency;/home/camillo/noetic_ws/devel_isolated/qt_gui_py_common;/home/camillo/noetic_ws/devel_isolated/qt_gui;/home/camillo/noetic_ws/devel_isolated/qt_dotgraph;/home/camillo/noetic_ws/devel_isolated/nodelet_core;/home/camillo/noetic_ws/devel_isolated/mk;/home/camillo/noetic_ws/devel_isolated/message_runtime;/home/camillo/noetic_ws/devel_isolated/message_generation;/home/camillo/noetic_ws/devel_isolated/media_export;/home/camillo/noetic_ws/devel_isolated/joint_state_publisher_gui;/home/camillo/noetic_ws/devel_isolated/gl_dependency;/home/camillo/noetic_ws/devel_isolated/geometry_tutorials;/home/camillo/noetic_ws/devel_isolated/geometry;/home/camillo/noetic_ws/devel_isolated/executive_smach;/home/camillo/noetic_ws/devel_isolated/diagnostics;/home/camillo/noetic_ws/devel_isolated/desktop;/home/camillo/noetic_ws/devel_isolated/cpp_common;/home/camillo/noetic_ws/devel_isolated/common_tutorials;/home/camillo/noetic_ws/devel_isolated/common_msgs;/home/camillo/noetic_ws/devel_isolated/class_loader;/home/camillo/noetic_ws/devel_isolated/cmake_modules;/home/camillo/noetic_ws/devel_isolated/bond_core;/home/camillo/noetic_ws/devel_isolated/genpy;/home/camillo/noetic_ws/devel_isolated/gennodejs;/home/camillo/noetic_ws/devel_isolated/genlisp;/home/camillo/noetic_ws/devel_isolated/geneus;/home/camillo/noetic_ws/devel_isolated/gencpp;/home/camillo/noetic_ws/devel_isolated/genmsg;/home/camillo/noetic_ws/devel_isolated/catkin'.split(';'):
        python_path = os.path.join(workspace, 'lib/python3/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/camillo/workspace/Learning/build/ur_msgs/devel/env.sh')

output_filename = '/home/camillo/workspace/Learning/build/ur_msgs/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
