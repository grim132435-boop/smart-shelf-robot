#!/usr/bin/env python3
# 모션팀용 ROS2 노드 스켈레톤 — webcam_seg_node(비전) 토픽을 받아 cuRobo 플래닝 → 두산 실행
#
# 구독(비전 노드가 발행):
#   /dsr01/curobo/grasp_candidates  PoseArray    GraspGen 후보 EE pose (base·m)  ← plan_grasp goalset 입력
#   /dsr01/curobo/grasp_class       String       'can'|'bottle'|'snack_bag'      → 그리퍼 전류
#   /dsr01/curobo/obstacles         String       장애물(매대·이웃물체)            → cuRobo world
#   /dsr01/curobo/pick_pose         PoseStamped  단일 파지 타겟(트리거/폴백)
#   (두산) /dsr01/joint_states       JointState   현재 관절                        → 시작 상태
# 실행:
#   두산 관절궤적 전송 (dsr_msgs2 — 실드라이버 인터페이스로 채울 것)
#   그리퍼 닫기 /gripper_service/set_position (클래스별 전류)
#
# ⚠ 이 파일은 "배선" 스켈레톤. 알고리즘은 curobo_planner_core.py 가 담당.

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String

import curobo_planner_core as core


class CuroboPlannerNode(Node):
    def __init__(self):
        super().__init__("curobo_planner_node")
        from curobo.types.base import TensorDeviceType
        self.ta = TensorDeviceType()

        # cuRobo 빌드(관절한계 URDF 포함 — 리스크①). 초기 world는 빈 것, obstacles 콜백서 갱신.
        self.motion_gen, gripper_links = core.build_motion_gen(self.ta)
        self.arm_joint_names = [...]   # TODO: e0509_gripper.yml cspace.joint_names 6개
        self.deps = core.CoreDeps(self.motion_gen, gripper_links, self.ta, self.arm_joint_names)

        # 상태 스냅샷
        self.cur_joints = None
        self.candidates = None
        self.grasp_class = "can"
        self.world_obstacles = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(PoseArray, "/dsr01/curobo/grasp_candidates",
                                 self.cb_candidates, latched)
        self.create_subscription(String, "/dsr01/curobo/grasp_class", self.cb_class, 10)
        self.create_subscription(String, "/dsr01/curobo/obstacles", self.cb_obstacles, 10)
        self.create_subscription(PoseStamped, "/dsr01/curobo/pick_pose", self.cb_pick, 10)
        self.create_subscription(JointState, "/dsr01/joint_states", self.cb_joints, 10)
        self.get_logger().info("curobo_planner_node 준비 — grasp_candidates 대기")

    # ── 콜백: 스냅샷만 저장 ──
    def cb_joints(self, msg):
        self.cur_joints = np.array(msg.position[:6], dtype=np.float32)   # TODO: 이름 매핑 확인

    def cb_class(self, msg):
        self.grasp_class = core.to_label(msg.data) if hasattr(core, "to_label") else msg.data

    def cb_candidates(self, msg: PoseArray):
        # ROS Pose quaternion = xyzw → cuRobo wxyz 로 변환(★계약: 순서 주의)
        n = len(msg.poses)
        pos = np.zeros((n, 3), np.float32); quat = np.zeros((n, 4), np.float32)
        for i, p in enumerate(msg.poses):
            pos[i] = [p.position.x, p.position.y, p.position.z]
            quat[i] = [p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z]
        self.candidates = core.GraspCandidates(pos, quat, self.grasp_class)
        self.get_logger().info(f"[candidates] {n}개 수신")

    def cb_obstacles(self, msg):
        # TODO: String → cuRobo WorldConfig 파싱(비전 obstacles 포맷에 맞춰). 없으면 매대 CAD 고정 사용.
        self.world_obstacles = _parse_obstacles_to_world(msg.data, self.ta)

    # ── 트리거: pick_pose 수신 = "이제 집어라" ──
    def cb_pick(self, msg: PoseStamped):
        if self.candidates is None or self.cur_joints is None:
            self.get_logger().warn("후보/관절 스냅샷 없음 — 스킵"); return
        inp = core.MotionInput(self.candidates, self.cur_joints,
                               self.world_obstacles or _empty_world(self.ta))
        plan = core.plan_pick_motion(inp, self.deps)
        if not plan.ok:
            self.get_logger().error(f"plan 실패: {plan.reason}"); return
        self.get_logger().info(f"goalset #{plan.grasp_idx} 선택 → 실행")

        # 1) 접근 → 2) 파지 직선 (저속 우선 — 안전)
        self._send_joint_traj(plan.approach_traj, tag="approach")
        self._send_joint_traj(plan.grasp_traj, tag="grasp")
        # 3) 그리퍼 닫기(클래스별 전류)
        self._gripper_grasp(plan.gripper_current_mA)
        # 4) 리프트 → 운반 → 적치 → 홈 : core.plan_to_pose / plan_to_joints 로 이어서 수행
        #    (매대 적치 좌표는 place_targets.yaml / SHELF3_SLOTS, stage7 참조)
        self.get_logger().info("TODO: lift→carry→place→home (plan_to_pose/plan_to_joints)")

    # ── 두산 실행 (실드라이버 인터페이스로 채울 것) ──
    def _send_joint_traj(self, traj_js, tag=""):
        """cuRobo JointState 궤적 → 두산. ★Cartesian(movel) 아니라 '관절궤적'으로 보낼 것
        (그래야 관절한계·플립방지가 유지됨 — 리스크①)."""
        if traj_js is None:
            return
        # TODO: dsr_msgs2 (예: MoveJointTrajectory/servoj) 로 traj_js.position 시퀀스 전송.
        self.get_logger().info(f"[{tag}] 궤적 {traj_js.position.shape} 전송(TODO 두산 드라이버)")

    def _gripper_grasp(self, current_mA):
        # TODO: /gripper_service/set_position 또는 전류제어 서비스로 닫기.
        self.get_logger().info(f"[그리퍼] 닫기 {current_mA}mA (TODO)")


def _parse_obstacles_to_world(s, ta):
    return None   # TODO: 비전 obstacles String 포맷 → curobo WorldConfig

def _empty_world(ta):
    from curobo.geom.types import WorldConfig
    return WorldConfig()


def main():
    rclpy.init()
    node = CuroboPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
