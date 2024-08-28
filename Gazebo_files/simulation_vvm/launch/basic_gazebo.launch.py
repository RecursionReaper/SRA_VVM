import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python import get_package_prefix

def generate_launch_description():
    
    pkg_gazebo = get_package_share_directory("gazebo_ros")
    pkg_vvm_bot = get_package_share_directory('simulation_vvm')

    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo , "launch" , "gazebo.launch.py"),
    #     )
    # )

    world = "vvm_world_1.world"
    world_file = os.path.join(pkg_vvm_bot, "worlds", world)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, "launch", "gazebo.launch.py")
        ),
        launch_arguments={'world': world_file}.items(),  # Correctly pass the arguments as a dictionary
    )

    description_package_name = "simulation_vvm"
    install_dir = get_package_prefix(description_package_name)
    # Set the path to the WORLD model files. Is to find the models inside the models folder in my_box_bot_gazebo package

    gazebo_models_path = os.path.join(pkg_vvm_bot, 'models')
    
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))
       
    # vvm simulation launch
    vvm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_vvm_bot, 'launch', 'simulation.launch.py'),
        )
    )
    return LaunchDescription([
        gazebo,
        vvm
    ])