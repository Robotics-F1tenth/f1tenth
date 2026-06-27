#!/usr/bin/env python3
import os, time, rclpy
from rclpy.node import Node
from nav2_msgs.srv import SaveMap

class MapAutoSaver(Node):
    def __init__(self):
        super().__init__('map_autosaver')
        # 파라미터
        self.declare_parameter('save_dir', os.path.expanduser('/home/a/f1tenth_ws/src/localization/f1_localization/als_ros/maps/slam_map/'))
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('period_sec', 5.0)

        self.save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.period = self.get_parameter('period_sec').get_parameter_value().double_value

        os.makedirs(self.save_dir, exist_ok=True)

        # 서비스 클라이언트 (/map_saver/save_map)
        self.cli = self.create_client(SaveMap, '/map_saver/save_map')
        self.get_logger().info(f'Waiting for /map_saver/save_map...')
        self.cli.wait_for_service()

        # 5초마다 저장
        self.timer = self.create_timer(self.period, self._tick)
        self.get_logger().info(f'Auto saving every {self.period}s to {self.save_dir}')

    def _tick(self):
        stamp = time.strftime('%Y%m%d-%H%M%S')
        path = os.path.join(self.save_dir, f'map_{stamp}')
        req = SaveMap.Request()
        req.map_url = path            # 확장자 자동: .yaml/.pgm
        req.map_topic = self.map_topic
        req.image_format = 'pgm'      # 기본 포맷
        future = self.cli.call_async(req)
        future.add_done_callback(lambda f: self._done_cb(f, path))

    def _done_cb(self, future, path):
        try:
            resp = future.result()
            code = getattr(resp, 'result', None)
            msg  = getattr(resp, 'message', getattr(resp, 'result_message', ''))
            if code == 0:  # SUCCESS
                self.get_logger().info(f'Saved map -> {path}.yaml/pgm')
            else:
                self.get_logger().warn(f'Failed to save map (code={code}) {msg} :: {path}')
        except Exception as e:
            self.get_logger().error(f'Exception while saving: {e}')

def main():
    rclpy.init()
    node = MapAutoSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
