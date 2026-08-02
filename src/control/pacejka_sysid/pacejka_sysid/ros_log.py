# Adapted from ForzaETH On-Track-SysID (github.com/ForzaETH/On-Track-SysID), arXiv:2411.17508
#
# 신규 파일. 원본 헬퍼들은 `rospy.loginfo` / `rospy.logwarn` 을 직접 호출한다.
# ROS 2 에는 전역 로거가 없고 rclpy 로거는 노드에 매여 있으므로,
# 노드가 자기 로거를 여기에 주입하고 헬퍼는 이 얇은 shim 만 호출하게 한다.
# 알고리즘 코드의 나머지 부분은 손대지 않기 위한 장치다.

_logger = None


def set_logger(logger):
    """노드 생성 시 `ros_log.set_logger(self.get_logger())` 로 주입한다."""
    global _logger
    _logger = logger


def loginfo(msg):
    if _logger is not None:
        _logger.info(str(msg))
    else:
        print(f"[INFO] {msg}")


def logwarn(msg):
    if _logger is not None:
        _logger.warn(str(msg))
    else:
        print(f"\033[33m[WARN] {msg}\033[0m")


def logerror(msg):
    if _logger is not None:
        _logger.error(str(msg))
    else:
        print(f"\033[31m[ERROR] {msg}\033[0m")
