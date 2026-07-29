# Adapted from ForzaETH On-Track-SysID (github.com/ForzaETH/On-Track-SysID), arXiv:2411.17508
# Original: src/helpers/dotdict.py @ 7b2b712 (ROS1 / catkin)
# Changes: 없음 (알고리즘·수식 원본 그대로)

class DotDict(dict):
  """dot.notation access to dictionary attributes"""
  __getattr__ = dict.get
  __setattr__ = dict.__setitem__
  __delattr__ = dict.__delitem__

  # convert back to normal dict 
  def to_dict(self):
    dict = {}
    for key, value in self.items():
        dict[key] = value
    return dict