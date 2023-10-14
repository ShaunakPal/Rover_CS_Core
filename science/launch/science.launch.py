from launch import LaunchDescription
from launch_ros.actions import Node
import os


def _get_js_fd_path(joy_name):

    dir_path = "/dev/input/by-id"                                   # Searchs for joystick name in by-id
    
    if os.path.exists(dir_path) and os.path.isdir(dir_path):
        dir_contents = os.listdir(dir_path)

        for filename in dir_contents:
            if "event" not in filename and joy_name in filename:
                filepath = os.path.join(dir_path, filename)

                if os.path.islink(filepath):
                    symlink_target = os.path.realpath(filepath)     # Get realpath of the joystick symlink

                    return os.path.realpath(symlink_target)

def generate_launch_description():

    _joy_dev_path_buff = _get_js_fd_path("Dual_Action")
    if _joy_dev_path_buff:
        joy_dev_path = _joy_dev_path_buff
    else:
        joy_dev_path = "/dev/input/js0"
    
    ld = LaunchDescription()

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node_science',
        remappings = [
            ("joy", "Science/joy")
            #("/dev/input/js0", joy_dev_path)
        ]
        ,
        parameters = [
            #{"dev": joy_dev_path}
        ]
    )

    science_node = Node(
        package='science',
        executable='science_node',
        name='science_node',
        remappings = [
            ("joy", "Science/joy")
        ]
    )

    ld.add_action(science_node)
    ld.add_action(joy_node)

    return ld