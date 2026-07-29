# Adapted from ForzaETH On-Track-SysID (github.com/ForzaETH/On-Track-SysID), arXiv:2411.17508
# Original: src/helpers/load_model.py @ 7b2b712 (ROS1 / catkin)
# Changes: 경로 해석만 rospkg -> paths.model_file() 로 교체. 파싱/반환 로직은 원본 그대로.

import yaml
import os
from .dotdict import DotDict
from .. import paths


def get_dict(model_name):
    model, tire = model_name.split("_")
    file_path = paths.model_file(model, model_name)
    with open(file_path, 'rb') as f:
        params = yaml.load(f, Loader=yaml.Loader)

    return params


def get_dotdict(model_name):
    dict = get_dict(model_name)
    params = DotDict(dict)
    return params
