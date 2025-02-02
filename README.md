# hector_multi_robot_tools

Tools for ROS2 to make working in a multi-robot setup easier.

## multi_robot_tool_launch
Defines wrappers for common ros tools to start them in a robot namespace and remappings for relative `tf` topics.  The tool names are prefixed with "m".

Available tools: `mrviz` (`rviz2`), `mrqt` (`rqt`)

Examples:
```
mrviz athena
```
Start `rviz` in namespace `/athena`. There is auto-completion for available robot names.

If no parameter is provided, the command automatically detects available robots. If there is only one available, this namespace is used:

```
$ mrviz
Starting in namespace: /athena
```

If there are multiple available, the global namespace is used instead. Add the parameter `--global` to force the global namespace

```
mrviz --global
```
