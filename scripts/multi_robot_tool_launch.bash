# multi_robot_tool_launch.bash - Defines wrappers for common ros tools to start them in a robot namespace

# Function to get valid robot names by searching for robot_description topic
get_robot_names() {
    ros2 topic list | grep robot_description | awk -F'/' '{print $2}'
}

# Function to display help message
show_help() {
    local func_name="$1"
    echo "Usage: $func_name [--global] [<robot_name>]"
    echo ""
    echo "Options:"
    echo "  --global        Start in the global namespace, ignoring available robot names"
    echo "  <robot_name>    Start with the specified robot namespace"
    echo "  -h, --help      Show this help message"
}

# Run ROS2 tool with appropriate parameters
run_ros_tool() {
    local pkg_name="$1"
    local exec_name="$2"
    local robot_name="$3"
    local cmd="ros2 launch hector_multi_robot_tools tool_namespace.launch.yaml pkg:=${pkg_name} exec:=${exec_name}"

    if [ -n "$robot_name" ]; then
        cmd+=" namespace:=/${robot_name}"
        echo "Starting in namespace: /${robot_name}"
    else
        echo "Starting in namespace: /"
    fi

    eval "$cmd"
}

ros_tool() {
    local func_name="$1"
    local pkg_name="$2"
    local exec_name="$3"
    shift
    shift
    shift

    # Process command line options
    if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
        show_help "$func_name"
        return
    fi

    if [ "$1" == "--global" ]; then
        run_ros_tool "${pkg_name}" "${exec_name}"  # Force global namespace
        return
    fi

    if [ -n "$1" ]; then
        # If a robot name is provided as an argument, use it directly
        run_ros_tool "${pkg_name}" "${exec_name}" "$1"
    else
        # No robot name provided, check available robots
        local robot_names=($(get_robot_names))
        local robot_count=${#robot_names[@]}

        if [ "$robot_count" -eq 1 ]; then
            # If exactly one robot is available, use it automatically
            run_ros_tool "${pkg_name}" "${exec_name}" "${robot_names[0]}"
        else
            # If multiple robots exist, run in global namespace
            run_ros_tool "${pkg_name}" "${exec_name}"
        fi
    fi
}

# Bash completion function for robot names and options
_ros_tool_completion() {
    local cur="${COMP_WORDS[COMP_CWORD]}"
    local options="--global --help -h"

    # Provide completion options only for first argument
    if [ ${#COMP_WORDS[@]} -eq 2 ]; then
        local robot_names=$(get_robot_names)
        COMPREPLY=($(compgen -W "${robot_names} ${options}" -- "${cur}"))
    fi
}

# Wrappers

# rqt function wrapper
mrqt() {
    ros_tool "$FUNCNAME" rqt_gui rqt_gui "$@"
}
complete -F _ros_tool_completion mrqt

# rviz function wrapper
mrviz() {
    ros_tool "$FUNCNAME" rviz2 rviz2 "$@"
}
complete -F _ros_tool_completion mrviz
