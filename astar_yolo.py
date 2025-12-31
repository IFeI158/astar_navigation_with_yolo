import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from math import atan2, sqrt, sin, pi
import heapq
import numpy as np
import cv2
import os

# 메시지 타입 및 변환 도구
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

# YOLO
from ultralytics import YOLO

class NodeAStar:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other): return self.position == other.position
    def __lt__(self, other): return self.f < other.f

class IntegratedNavigation(Node):
    def __init__(self):
        super().__init__('integrated_navigation')

        # 제어 파라미터
        self.lookahead_dist = 0.5  
        self.linear_vel = 0.08      
        self.stop_tolerance = 0.15 
        self.safety_margin_m = 0.23
        self.preferred_margin_m = 0.45 
        
        # 상태 변수
        self.map_data = None
        self.map_resolution = 0.05
        self.map_origin = [0.0, 0.0]
        self.map_width = 0
        self.map_height = 0
        self.current_pose = None
        self.current_yaw = 0.0
        self.global_path = [] 
        self.path_index = 0
        self.stop_flag = False

        # --- [수정] YOLO 모델 로드 (절대 경로) ---
        model_path = "/home/dev/ros-cv_ws/img_pkg/best.pt"
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file not found at: {model_path}")
        self.model = YOLO(model_path)
        
        # --- [추가] 이미지 변환 및 디버그용 퍼블리셔 ---
        self.bridge = CvBridge()
        self.pub_debug_img = self.create_publisher(Image, '/yolo_debug', 10)

        # --- QoS 설정 ---
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # 퍼블리셔 & 서브스크라이버
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)

        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile=map_qos)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, qos_profile=best_effort_qos)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        self.sub_image = self.create_subscription(
            CompressedImage,            
            '/image_raw/compressed',      
            self.image_callback, 
            qos_profile=best_effort_qos
        ) 

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Navigation Node with YOLO & Debug Visualization Started")

    def image_callback(self, msg):
        try:
            # 1. CompressedImage -> OpenCV
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) 

            if cv_image is None:
                return

            # 2. YOLO 추론
            results = self.model(cv_image, verbose=False) 

            # 3. [추가] 시각화 및 rqt용 이미지 발행
            # 박스가 그려진 이미지를 생성합니다.
            annotated_frame = results[0].plot() 
            # ROS Image 메시지로 변환하여 /yolo_debug 토픽으로 보냅니다.
            debug_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.pub_debug_img.publish(debug_msg)

            # 4. 객체 감지 여부에 따른 정지 플래그 설정
            object_detected = False
            for r in results:
                if len(r.boxes) > 0:
                    object_detected = True 
                    break
            
            if object_detected and not self.stop_flag:
                self.stop_flag = True
                self.get_logger().info("🚨 OBJECT DETECTED! STOPPING.")
            elif not object_detected and self.stop_flag:
                self.stop_flag = False
                self.get_logger().info("✅ Path Clear. Resuming.")
            
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))

    def pose_callback(self, msg):
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        self.current_yaw = atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))

    def goal_callback(self, msg):
        if self.map_data is None or self.current_pose is None: return
        goal_pose = [msg.pose.position.x, msg.pose.position.y]
        start_grid = self.world_to_grid(self.current_pose)
        goal_grid = self.world_to_grid(goal_pose)
        
        path_grid = self.run_astar(start_grid, goal_grid)
        
        if path_grid:
            self.global_path = [self.grid_to_world(p) for p in path_grid]
            self.path_index = 0
            self.publish_path_viz()
            self.get_logger().info(f"Path planning successful. Length: {len(self.global_path)}")
        else:
            self.get_logger().warn("No path found.")

    def run_astar(self, start, end):
        if not (0 <= start[0] < self.map_height and 0 <= start[1] < self.map_width): return None
        if not (0 <= end[0] < self.map_height and 0 <= end[1] < self.map_width): return None

        safety_cells = int(self.safety_margin_m / self.map_resolution)
        pref_cells = int(self.preferred_margin_m / self.map_resolution)

        start_node = NodeAStar(None, start)
        end_node = NodeAStar(None, end)
        open_list = []
        heapq.heappush(open_list, start_node)
        visited = set()
        moves = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]

        while open_list:
            current_node = heapq.heappop(open_list)
            if current_node.position in visited: continue
            visited.add(current_node.position)

            if current_node.position == end_node.position:
                path = []
                current = current_node
                while current:
                    path.append(current.position)
                    current = current.parent
                return path[::-1]

            for move in moves:
                ny, nx = current_node.position[0] + move[0], current_node.position[1] + move[1]
                if not (0 <= ny < self.map_height and 0 <= nx < self.map_width): continue

                y_s, y_e = max(0, ny - safety_cells), min(self.map_height, ny + safety_cells + 1)
                x_s, x_e = max(0, nx - safety_cells), min(self.map_width, nx + safety_cells + 1)
                if np.any(self.map_data[y_s:y_e, x_s:x_e] != 0): continue

                y_p, y_pe = max(0, ny - pref_cells), min(self.map_height, ny + pref_cells + 1)
                x_p, x_pe = max(0, nx - pref_cells), min(self.map_width, nx + pref_cells + 1)
                check_area = self.map_data[y_p:y_pe, x_p:x_pe]
                wall_indices = np.where(check_area != 0)
                
                wall_penalty = 0.0
                if len(wall_indices[0]) > 0:
                    dy = wall_indices[0] - (ny - y_p)
                    dx = wall_indices[1] - (nx - x_p)
                    min_dist_px = np.sqrt(np.min(dy**2 + dx**2))
                    dist_m = min_dist_px * self.map_resolution
                    if dist_m < self.preferred_margin_m:
                        wall_penalty = (self.preferred_margin_m - dist_m) * 15.0

                new_node = NodeAStar(current_node, (ny, nx))
                step_cost = sqrt(move[0]**2 + move[1]**2)
                new_node.g = current_node.g + step_cost + wall_penalty
                new_node.h = sqrt((ny - end[0])**2 + (nx - end[1])**2)
                new_node.f = new_node.g + new_node.h
                heapq.heappush(open_list, new_node)
        return None

    def control_loop(self):
        if self.stop_flag:
            self.stop_robot()
            return

        if not self.global_path or self.current_pose is None: return

        final_goal = self.global_path[-1] 
        dist_to_final = sqrt((final_goal[0]-self.current_pose[0])**2 + (final_goal[1]-self.current_pose[1])**2)

        if dist_to_final < self.stop_tolerance:
            self.global_path = []
            self.stop_robot()
            self.get_logger().info("🎯 Goal Reached.")
            return

        target_x, target_y = final_goal
        for i in range(self.path_index, len(self.global_path)):
            px, py = self.global_path[i]
            dist = sqrt((px - self.current_pose[0])**2 + (py - self.current_pose[1])**2)
            if dist >= self.lookahead_dist:
                target_x, target_y = px, py 
                self.path_index = i
                break

        dx = target_x - self.current_pose[0]
        dy = target_y - self.current_pose[1]
        alpha = atan2(dy, dx) - self.current_yaw
        
        while alpha > pi: alpha -= 2*pi
        while alpha < -pi: alpha += 2*pi
        
        angular_velocity = self.linear_vel * (2.0 * sin(alpha)) / self.lookahead_dist

        cmd = Twist()
        cmd.linear.x = self.linear_vel
        cmd.angular.z = max(min(angular_velocity, 1.0), -1.0)
        self.pub_cmd.publish(cmd)

    def world_to_grid(self, world):
        return (int((world[1]-self.map_origin[1])/self.map_resolution), int((world[0]-self.map_origin[0])/self.map_resolution))

    def grid_to_world(self, grid):
        return [(grid[1]*self.map_resolution)+self.map_origin[0], (grid[0]*self.map_resolution)+self.map_origin[1]]

    def publish_path_viz(self):
        msg = Path()
        msg.header.frame_id = 'map'
        for p in self.global_path:
            ps = PoseStamped()
            ps.pose.position.x, ps.pose.position.y = p[0], p[1]
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def stop_robot(self):
        self.pub_cmd.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()