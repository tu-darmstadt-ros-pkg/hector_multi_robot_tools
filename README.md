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

## prefix_tf_republisher
Republishes a `tf` topic from a namespaced topic to the global one with a robot-specific prefix attached to the frame id. The frame prefix is automatically determined from the node's namespace or can be set with the `frame_prefix` parameter. 

### Parameters

* `frame_prefix` (string, default: node namespace)

    Override the automatically-determined frame prefix.

* `global_frames` (string[], default: [])

    List of global frames that are relayed without a prefix.
  
### Subscribers & Publishers  
The node subscribes to `input_tf_topic` and publishes to `output_tf_topic` which need to be remapped accordingly.

### Example Usage

```yaml
launch:
- arg:
    name: namespace
    default: ""
# republish tf
- node:
    pkg: "hector_multi_robot_tools"
    exec: "prefix_tf_republisher"
    name: "tf_republisher"
    namespace: "$(var namespace)"
    param:
        - name: "global_frames"
          value: ["world"]
    remap:
        - from: "input_tf_topic"
          to: "tf"
        - from: "output_tf_topic"
          to: "/tf"
# republish tf_static
- node:
    pkg: "hector_multi_robot_tools"
    exec: "prefix_tf_republisher"
    name: "tf_static_republisher"
    namespace: "$(var namespace)"
    param:
        - name: "global_frames"
          value: ["world"]
    remap:
        - from: "input_tf_topic"
          to: "tf_static"
        - from: "output_tf_topic"
          to: "/tf_static"
```
