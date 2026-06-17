# FEM 변형체 의사-소성(pseudo-plastic) — 응력 기반 항복 노드 freeze (hijimasa 기법 이식, torch 배치 계산).
"""
Isaac Sim 5.1엔 native plasticity가 없어, hijimasa/isaac-sim-plastic-deformation 방식을 이식한다.
FEM(beta) 볼륨 변형체의 시뮬 노드를 매 스텝 읽어 요소별 응력(Von Mises)을 계산하고,
항복(yield)한 노드를 FROZEN으로 표시해 위치를 고정(+속도 0)함으로써 탄성 복원을 억제 = 영구 변형.

상태머신(노드별): FREE → YIELDING(vm>yield) → FROZEN(vm가 peak-yield 아래로 떨어짐) → 재충격 시 YIELDING.

★beta DeformableBodyView엔 stress API가 없어, 노드 위치+tet 인덱스로 응력을 직접 계산한다.
  변형구배 F = P·Q⁻¹, Green 변형 E=(FᵀF−I)/2(회전 불변), 2nd-PK S=2μE+λtr(E)I, Von Mises(S).
  (회전 불변 Green 변형 사용 → 봉지가 들려 회전해도 헛항복 안 함. Warp 극분해 불요.)

사용:
  pd = PlasticDeformation("/World/snack_bag", youngs=3e5, poisson=0.45, yield_stress=2000.0)
  # my_world.play() 후:
  pd.initialize()
  my_world.add_physics_callback("snack_plastic", lambda dt: pd.step())   # 매 물리스텝
"""
import omni.physics.tensors as tensors


class PlasticDeformation:
    def __init__(self, root_prim_path, youngs=3.0e5, poisson=0.45,
                 yield_stress=2000.0, device="cuda:0"):
        self.root = root_prim_path
        self.E = float(youngs)
        self.nu = float(poisson)
        self.yield_stress = float(yield_stress)
        self.device = device
        self._ready = False
        self._torch = None

    def initialize(self):
        """★my_world.play() 후 호출(physics view 필요). 반환: 성공 여부."""
        import torch
        self._torch = torch
        sim_view = tensors.create_simulation_view("torch")
        self.view = sim_view.create_volume_deformable_body_view(self.root)
        if self.view is None or self.view.count == 0:
            print(f"[PlasticDeformation] view 생성 실패: {self.root}", flush=True)
            return False
        self.N = self.view.max_simulation_nodes_per_body

        pos = self.view.get_simulation_nodal_positions()          # (1, N, 3) torch cuda
        dev = pos.device
        self.device = str(dev)
        self.rest = pos[0].clone()                                # (N,3) rest 위치
        tet = self.view.get_simulation_element_indices()[0].long()  # (E,4)
        self.tet = tet
        self.tet_flat = tet.reshape(-1)                           # (4E,)
        self.Ecount = tet.shape[0]
        self._I = torch.eye(3, device=dev)

        # rest 요소 변형구배 역행렬 Q⁻¹ (E,3,3), 열=rest 엣지
        r = self.rest[tet]                                        # (E,4,3)
        Q = torch.stack([r[:, 1] - r[:, 0], r[:, 2] - r[:, 0], r[:, 3] - r[:, 0]], dim=-1)  # (E,3,3)
        Q = Q + 1e-9 * self._I                                    # 퇴화 정규화
        self.Qinv = torch.linalg.inv(Q)                          # (E,3,3)

        # Lamé
        self.mu = self.E / (2.0 * (1.0 + self.nu))
        self.lam = self.E * self.nu / ((1.0 + self.nu) * (1.0 - 2.0 * self.nu))

        # 노드 상태
        self.is_yield = torch.zeros(self.N, dtype=torch.bool, device=dev)
        self.frozen = torch.zeros(self.N, dtype=torch.bool, device=dev)
        self.peak = torch.zeros(self.N, device=dev)
        self.frozen_vm = torch.zeros(self.N, device=dev)
        self.frozen_pos = self.rest.clone()
        self.body_idx = torch.zeros(1, dtype=torch.int32, device=dev)   # body 0

        self._ready = True
        print(f"[PlasticDeformation] init OK — nodes={self.N} elems={self.Ecount} "
              f"mu={self.mu:.1f} lam={self.lam:.1f} yield={self.yield_stress}", flush=True)
        return True

    def _node_von_mises(self, pos):
        """현재 노드 위치(N,3) → 노드별 Von Mises 응력(N,). 요소응력을 노드 max로 집계."""
        torch = self._torch
        p = pos[self.tet]                                         # (E,4,3)
        P = torch.stack([p[:, 1] - p[:, 0], p[:, 2] - p[:, 0], p[:, 3] - p[:, 0]], dim=-1)  # (E,3,3)
        F = P @ self.Qinv                                        # (E,3,3)
        Eg = 0.5 * (F.transpose(-1, -2) @ F - self._I)          # Green 변형(회전불변)
        trE = Eg.diagonal(dim1=-2, dim2=-1).sum(-1)             # (E,)
        S = 2.0 * self.mu * Eg + self.lam * trE[:, None, None] * self._I   # 2nd-PK (E,3,3)
        s11, s22, s33 = S[:, 0, 0], S[:, 1, 1], S[:, 2, 2]
        s12, s23, s31 = S[:, 0, 1], S[:, 1, 2], S[:, 2, 0]
        vm = torch.sqrt(0.5 * ((s11 - s22) ** 2 + (s22 - s33) ** 2 + (s33 - s11) ** 2)
                        + 3.0 * (s12 ** 2 + s23 ** 2 + s31 ** 2) + 1e-12)   # (E,)
        node_vm = torch.zeros(self.N, device=pos.device)
        node_vm.scatter_reduce_(0, self.tet_flat, vm.repeat_interleave(4),
                                reduce="amax", include_self=True)
        return node_vm

    def post_physics_step(self):
        """응력 계산 + 상태 전이."""
        if not self._ready:
            return
        pos = self.view.get_simulation_nodal_positions()[0]      # (N,3)
        vm = self._node_von_mises(pos)
        y = self.yield_stress
        # FREE → YIELDING
        to_y = (vm > y) & (~self.is_yield) & (~self.frozen)
        self.is_yield |= to_y
        self.peak[to_y] = vm[to_y]
        # peak 갱신
        up = self.is_yield & (vm > self.peak)
        self.peak[up] = vm[up]
        # YIELDING → FROZEN (응력이 peak-yield 아래로 = 그립 해제)
        fr = self.is_yield & (vm < self.peak - y)
        self.is_yield[fr] = False
        self.frozen[fr] = True
        self.frozen_vm[fr] = vm[fr]
        self.frozen_pos[fr] = pos[fr]
        # FROZEN → YIELDING (재충격)
        re = self.frozen & (vm > self.frozen_vm + y)
        self.frozen[re] = False
        self.is_yield[re] = True
        self.peak[re] = vm[re]

    def pre_physics_step(self):
        """FROZEN 노드 위치 고정 + 속도 0 → 탄성 복원 억제."""
        if not self._ready:
            return
        hold = self.frozen & (~self.is_yield)
        if not bool(hold.any()):
            return
        pos = self.view.get_simulation_nodal_positions()         # (1,N,3)
        vel = self.view.get_simulation_nodal_velocities()
        pos[0][hold] = self.frozen_pos[hold]
        vel[0][hold] = 0.0
        self.view.set_simulation_nodal_positions(pos, self.body_idx)
        self.view.set_simulation_nodal_velocities(vel, self.body_idx)

    def step(self):
        """add_physics_callback용 — 매 물리스텝: 상태갱신(직전 스텝 결과) + freeze(다음 스텝)."""
        self.post_physics_step()
        self.pre_physics_step()

    def stats(self):
        """진단: (항복중, 고정됨, 평균변위mm, 최대변위mm)."""
        if not self._ready:
            return (0, 0, 0.0, 0.0)
        pos = self.view.get_simulation_nodal_positions()[0]
        d = (pos - self.rest).norm(dim=-1)
        return (int(self.is_yield.sum()), int(self.frozen.sum()),
                d.mean().item() * 1000.0, d.max().item() * 1000.0)
