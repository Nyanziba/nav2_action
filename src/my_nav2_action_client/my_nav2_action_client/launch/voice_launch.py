from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # launchの構成を示すLaunchDescription型の変数の定義
    ld = LaunchDescription()

    # publisher nodeを、"talker_renamed1"という名前で定義
    pub_node1 = Node(
        package='voicevox_ros2',
        executable='voicevox_ros2_core',
    )

    # publisher nodeを、"talker_renamed2"という名前で定義
    pub_node2 = Node(
        package='my_nav2_action_client',
        executable='nav2_action_client',

    )

    # LaunchDescriptionに、起動したいノードを追加する
    ld.add_action(pub_node1)
    ld.add_action(pub_node2)

    # launch構成を返すようにする
    return ld
