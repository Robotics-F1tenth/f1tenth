# Adapted from ForzaETH On-Track-SysID (github.com/ForzaETH/On-Track-SysID), arXiv:2411.17508
#
# 신규 파일. 원본은 `rospkg.RosPack().get_path('on_track_sys_id')` 로 catkin **소스 트리**를
# 직접 찾아 거기서 읽고 거기에 그대로 덮어썼다. ament 에서는 설치 공간(share/)이
# 산출물 저장소로 부적합하므로(심링크 설치 시 소스 오염, 일반 설치 시 재빌드로 소실)
# 읽기 경로와 쓰기 경로를 분리한다.
#
#   읽기(템플릿/기본 파라미터) : share/pacejka_sysid/...   (get_package_share_directory)
#   쓰기(식별 결과/LUT/CSV)    : output_root                (기본 ~/.ros/pacejka_sysid)
#
# 모델 txt 파일은 원본에서 "입력 템플릿"이자 "출력 파일"로 동시에 쓰인다.
# 그래서 최초 접근 시 share 의 템플릿을 output_root 로 한 번 복사(seed)해 두고,
# 이후에는 output_root 쪽만 읽고 쓴다. 이렇게 하면 원본의 읽기/쓰기 로직을
# 그대로 두면서도 결과가 재빌드에 살아남는다.

import os
import shutil

from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'pacejka_sysid'

_output_root = None


def set_output_root(path):
    """노드 시작 시 `output_dir` 파라미터 값을 주입한다."""
    global _output_root
    _output_root = os.path.expanduser(path)
    os.makedirs(_output_root, exist_ok=True)


def get_output_root():
    global _output_root
    if _output_root is None:
        # 노드를 거치지 않고 헬퍼를 직접 import 한 경우(스크립트 사용)의 기본값
        set_output_root(os.path.join('~', '.ros', PACKAGE_NAME))
    return _output_root


def share_dir():
    return get_package_share_directory(PACKAGE_NAME)


# colcon 의 --symlink-install 은 ament_python 패키지의 data_files 를 설치하지 않는다
# (setup.py develop 경로를 타기 때문). 그러면 params/ models/ launch/ 가 share 에
# 통째로 비게 되는데, 에러 메시지만 보면 원인을 알기 어려워 힌트를 붙인다.
_SYMLINK_HINT = (
    "\n  힌트: 이 패키지는 --symlink-install 로 빌드하면 params/models/launch 가 "
    "share 에 설치되지 않습니다.\n"
    "        colcon build --packages-select pacejka_sysid   (--symlink-install 없이) 로 "
    "다시 빌드하세요.\n"
    "        (~/.bashrc 의 'cb' 별칭은 --symlink-install 을 씁니다)")


def params_file(filename):
    """`params/` 아래 yaml 은 항상 share 에서 읽는다 (읽기 전용)."""
    path = os.path.join(share_dir(), 'params', filename)
    if not os.path.isfile(path):
        raise FileNotFoundError(f"파라미터 파일이 없습니다: {path}{_SYMLINK_HINT}")
    return path


def model_dir(racecar_version):
    """`models/<racecar_version>/` 쓰기 가능 경로. share 템플릿이 있으면 최초 1회 복사."""
    out = os.path.join(get_output_root(), 'models', racecar_version)
    os.makedirs(out, exist_ok=True)

    template_dir = os.path.join(share_dir(), 'models', racecar_version)
    if os.path.isdir(template_dir):
        for name in os.listdir(template_dir):
            src = os.path.join(template_dir, name)
            dst = os.path.join(out, name)
            if os.path.isfile(src) and not os.path.exists(dst):
                shutil.copy2(src, dst)
    return out


def model_file(racecar_version, model_name):
    """`models/<racecar_version>/<model_name>.txt` 의 쓰기 가능 절대 경로."""
    return os.path.join(model_dir(racecar_version), model_name + '.txt')


def data_dir(racecar_version):
    """수집 데이터 CSV 를 떨굴 디렉토리."""
    out = os.path.join(get_output_root(), 'data', racecar_version)
    os.makedirs(out, exist_ok=True)
    return out


def available_models():
    """share 에 템플릿이 들어 있는 racecar_version 목록 (오류 메시지용)."""
    template_root = os.path.join(share_dir(), 'models')
    if not os.path.isdir(template_root):
        return []
    return sorted(
        d for d in os.listdir(template_root)
        if os.path.isdir(os.path.join(template_root, d))
    )
