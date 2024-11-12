# Rosbag Panel for the ROS 3D Robot Visualizer (RViz)

Control native `rosbag play` tool directly from RViz. This plugin allows you to play, pause, set speed, set start/end position, select topics, and generate filtered rosbags from the current selection.

![Preview Image](https://github.com/UniBwTAS/rosbag_panel/blob/master/assets/preview.png)

# Features

## Camera Preview for fast Navigation (while dragging the slider)

![Camera Preview](https://github.com/UniBwTAS/rosbag_panel/blob/master/assets/slider_preview.gif)
It is possible to configure the `sensor_msgs::Image` Topic for the preview. This is useful for fast navigation in the bag file in order to find the desired scenario. In this example, we check our multi-object tracker's performance for difficult scenarios (person in wheelchair crossing the street).

## Select Topics

![Select Topics](https://github.com/UniBwTAS/rosbag_panel/blob/master/assets/select_topics.gif)
You can easily select or deselect topics in a tree view. The tree view is very useful to quickly disable a whole group of topics by their namespace.

## Set Start and End Position

![Set Start and End Position](https://github.com/UniBwTAS/rosbag_panel/blob/master/assets/start_end_slider.gif)
You can set the start and end position of the playback with a slider. This is useful to focus on a specific scenario. Also useful with the 'loop' option for infinite playback of a specific scenario. While dragging the slider also a preview image is shown.

## Set Playback Speed

![Set Playback Speed](https://github.com/UniBwTAS/rosbag_panel/blob/master/assets/set_speed.gif)
You can set the playback speed with a slider. This is useful to slow down the playback for detailed analysis or speed up the playback for fast navigation.

## Open Rosbag File in GUI

![Open Rosbag File](https://github.com/UniBwTAS/rosbag_panel/blob/master/assets/open_bag.gif)
You can open a Rosbag file in the GUI. This is useful to quickly switch between different bags.

## Generate filtered Rosbag from Current Selection

![Generate filtered Rosbag](https://github.com/UniBwTAS/rosbag_panel/blob/master/assets/filter_bag.gif)
You can generate a filtered Rosbag from the current selection. This is useful to create a new bag with only the desired topics and a specific time range.

> [!NOTE]
> Unlike existing tools, such as `rosbag filter`, this tool takes care of latched topics (e.g. `/static_tf`), provides a GUI, and is faster since it is C++ based.

## Feature Summary:

- Open Rosbag File
- Play
- Pause
- Position & Start-End Slider
- Image Preview for fast Navigation
- Speed Control
- Select Topics
- Pause on Topic (Stepping)
  - Exact Pause for Topics included in Bag
  - Approximate Pause for (reprocessed) Topics not included in Bag (Pause delayed by Message Latency)
- Create new filtered Rosbag:
  - Uses start/end Position of slider
  - Uses selected Topics
  - Unlike `rosbag filter` CLI tool it takes care of latched topics (e.g. `/static_tf`), provides GUI, and is faster since it is C++ based
- Exposes all options of `rosbag play` as GUI Settings
  
## How to install?

1. This plugin requires a patched version of `ros_comm` in order to make the `rosbag play` tool remote controllable by ROS services. In the following we clone minimal version of `ros_comm`, which only contains the patched ROS packages and overlays the original ones. So there is no need to uninstall the original `ros_comm` packages:
```
cd ~/catkin_ws/src
git clone https://github.com/UniBwTAS/ros_comm.git
```
2. Clone this repository & build it:
```
git clone https://github.com/UniBwTAS/rosbag_panel.git
catkin build rosbag_panel
```
3. Source the workspace (or restart all Terminal Windows):
```
source ~/catkin_ws/devel/setup.bash
# alternatively source setup.zsh if you're using zsh instead of bash
```

## How to use?
1. Start `roscore` in a Terminal:
```
roscore
```
2. Start `rosbag play` in a Terminal with the `--server` option (only avalable in patched version) and assign this ROS node the name `rosbag_play` (important to find its ROS services):
```
rosbag play --server __name:=rosbag_play
```
3. Start RViz in a Terminal:
```
rviz
```
4. In RViz, click on `Panels` -> `Add New Panel` -> `rosbag_panel/RosbagPanel` -> `OK`
5. Open desired Bag (if everything is grayed out, then step 2 did not work)

## Troubleshooting:
- If the `--server` option is not available, then the patched version of `ros_comm` was not used. Sometimes it is required to clean your workspace, open a new terminal and rebuild the package such that the ROS workspace (typically in `/opt/ros/noetic`) is correctly overlay-ed:
```
cd ~/catkin_ws
catkin clean
catkin build rosbag_panel
source ~/catkin_ws/devel/setup.bash
# alternatively source setup.zsh if you're using zsh instead of bash
```
- Plugin is not available in RViz: Make sure the plugin was built and sourced correctly. If the plugin is still not available, then check the output of the `catkin build` command for any errors.