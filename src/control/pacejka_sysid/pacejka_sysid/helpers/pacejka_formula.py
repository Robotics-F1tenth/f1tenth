# Adapted from ForzaETH On-Track-SysID (github.com/ForzaETH/On-Track-SysID), arXiv:2411.17508
# Original: src/helpers/pacejka_formula.py @ 7b2b712 (ROS1 / catkin)
# Changes: 없음 (알고리즘·수식 원본 그대로)

import numpy as np
def pacejka_formula(params, alpha, F_z):
  B, C, D, E = params[0], params[1], params[2], params[3]
  y = F_z * D * np.sin(C*np.arctan(B*alpha - E * (B*alpha -np.arctan(B * alpha))))
  return y