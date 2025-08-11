import os
import sys
import launch

# 确保可以找到 hybrid_planning_common.py
sys.path.append(os.path.dirname(__file__))

from hybrid_planning_common import generate_common_hybrid_launch_description

def generate_launch_description():
    # 先生成 global/local/manager 的容器
    common_nodes = generate_common_hybrid_launch_description()

    return launch.LaunchDescription(common_nodes)

# https://www.youtube.com/watch?v=jY5_l84q21U&ab_channel=MarcWittwer
# https://github.com/moveit/moveit2/issues/1536