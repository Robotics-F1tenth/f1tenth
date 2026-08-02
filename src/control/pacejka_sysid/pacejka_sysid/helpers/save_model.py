# Adapted from ForzaETH On-Track-SysID (github.com/ForzaETH/On-Track-SysID), arXiv:2411.17508
# Original: src/helpers/save_model.py @ 7b2b712 (ROS1 / catkin)
# Changes: 경로 해석 rospkg -> paths.model_file(), rospy.loginfo -> ros_log.loginfo.
#          덮어쓰기 판정/직렬화 로직은 원본 그대로.

import yaml
import os
from .. import paths
from .. import ros_log


def save(model, overwrite_existing=True, verbose=False):

    file_path = paths.model_file(model['model_name'],
                                 model['model_name'] + "_" + model['tire_model'])
    if os.path.isfile(file_path):
        if (verbose): print("Model already exists")
        if overwrite_existing:
            if (verbose): print("Overwriting...")
        else:
            if (verbose): print("Not overwriting.")
            return 0

    try:
        model = model.to_dict()
    except:
        model = model

    # Write data to the file
    with open(file_path, "w") as f:
        ros_log.loginfo(f"MODEL IS SAVED TO: {file_path}")
        yaml.dump(model, f, default_flow_style=False)
