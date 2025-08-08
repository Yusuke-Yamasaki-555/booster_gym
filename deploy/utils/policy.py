import numpy as np
import torch


class Policy:
    def __init__(self, cfg):
        try:
            self.cfg = cfg
            self.policy = torch.jit.load(self.cfg["policy"]["policy_path"])
            self.policy.eval()
        except Exception as e:
            print(f"Failed to load policy: {e}")
            raise
        self._init_inference_variables()

    def get_policy_interval(self):
        return self.policy_interval

    def _init_inference_variables(self):
        self.default_dof_pos = np.array(self.cfg["common"]["default_qpos"], dtype=np.float32)
        self.stiffness = np.array(self.cfg["common"]["stiffness"], dtype=np.float32)
        self.damping = np.array(self.cfg["common"]["damping"], dtype=np.float32)

        # 99次元に修正
        self.obs = np.zeros(99, dtype=np.float32)
        self.actions = np.zeros(21, dtype=np.float32)
        self.last_action = np.zeros(21, dtype=np.float32)
        self.policy_interval = self.cfg["common"]["dt"] * self.cfg["policy"]["control"]["decimation"]
        self.dof_targets = np.copy(self.default_dof_pos)

    def inference(
        self, time_now, dof_pos, dof_vel, imu_orientation, imu_ang_vel, target_pos_obs
    ):
        # 必要な正規化係数はcfgから取得（なければ1.0でOK）
        norm = self.cfg["policy"].get("normalization", {})
        get_norm = lambda k, default=1.0: norm.get(k, default)

        idx = 0
        # 0: target_pos_obs (4,)
        self.obs[idx:idx+4] = target_pos_obs * get_norm("target_pos_obs", 1.0)
        idx += 4
        # 1: joint_pos_rel (23,)
        self.obs[idx:idx+23] = (dof_pos - self.default_dof_pos)[:23] * get_norm("dof_pos", 1.0)
        idx += 23
        # 2: joint_vel_rel (23,)
        self.obs[idx:idx+23] = dof_vel[:23] * get_norm("dof_vel", 1.0)
        idx += 23
        # 3: imu_orientation (4,)
        self.obs[idx:idx+4] = imu_orientation * get_norm("imu_orientation", 1.0)
        idx += 4
        # 4: imu_angular_velocity (3,)
        self.obs[idx:idx+3] = imu_ang_vel * get_norm("ang_vel", 1.0)
        idx += 3
        # 5: actions (21,)
        self.obs[idx:idx+21] = self.actions
        idx += 21
        # 6: last_action (21,)
        self.obs[idx:idx+21] = self.last_action
        idx += 21

        # 推論
        input_tensor = torch.from_numpy(self.obs).unsqueeze(0).float()
        out_actions = self.policy(input_tensor).detach().numpy().squeeze()
        out_actions = np.clip(
            out_actions,
            -get_norm("clip_actions", 1.0),
            get_norm("clip_actions", 1.0),
        )
        self.last_action[:] = self.actions
        self.actions[:] = out_actions

        # dof_targetsのうち制御対象の21自由度だけ更新（必要に応じて調整）
        self.dof_targets[:21] = self.default_dof_pos[:21] + self.cfg["policy"]["control"]["action_scale"] * self.actions

        return self.dof_targets
