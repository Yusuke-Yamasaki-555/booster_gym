import numpy as np
import time
import yaml
import logging
import threading

from booster_robotics_sdk_python import (
    ChannelFactory,
    B1LocoClient,
    B1LowCmdPublisher,
    B1LowStateSubscriber,
    LowCmd,
    LowState,
    B1JointCnt,
    RobotMode,
)

from utils.command import create_prepare_cmd, create_first_frame_rl_cmd
from utils.remote_control_service import RemoteControlService
from utils.rotate import rotate_vector_inverse_rpy
from utils.timer import TimerConfig, Timer
from utils.policy_gym import Policy


cmd_q_log = []
obs_time_log = []
obs_dof_pos_leg_log = []
obs_dof_vel_leg_log = []
obs_base_ang_vel_log = []
obs_projected_gravity_log = []
obs_controller_cmd_log = []
obs_gait_freq_log = []
obs_action_log = []

class Controller:
    def __init__(self, cfg_file) -> None:
        # Setup logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        # Load config
        with open(cfg_file, "r", encoding="utf-8") as f:
            self.cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

        # Initialize components
        self.remoteControlService = RemoteControlService()
        self.policy = Policy(cfg=self.cfg)

        self._init_timer()
        self._init_low_state_values()
        self._init_communication()
        self.publish_runner = None
        self.running = True

        self.publish_lock = threading.Lock()

        self.count_step = 0
        # self.cmd_log_1 = []
        # self.cmd_log_2 = []
        # self.cmd_log_3 = []
        self.obs_log = []

    def _init_timer(self):
        self.timer = Timer(TimerConfig(time_step=self.cfg["common"]["dt"]))
        self.next_publish_time = self.timer.get_time()
        self.next_inference_time = self.timer.get_time()

    def _init_low_state_values(self):
        self.base_ang_vel = np.zeros(3, dtype=np.float32)
        self.projected_gravity = np.zeros(3, dtype=np.float32)
        self.dof_pos = np.zeros(B1JointCnt, dtype=np.float32)
        self.dof_vel = np.zeros(B1JointCnt, dtype=np.float32)

        self.dof_target = np.zeros(B1JointCnt, dtype=np.float32)
        self.filtered_dof_target = np.zeros(B1JointCnt, dtype=np.float32)
        self.dof_pos_latest = np.zeros(B1JointCnt, dtype=np.float32)

    def _init_communication(self) -> None:
        try:
            self.low_cmd = LowCmd()
            self.low_state_subscriber = B1LowStateSubscriber(self._low_state_handler)
            self.low_cmd_publisher = B1LowCmdPublisher()
            self.client = B1LocoClient()

            self.low_state_subscriber.InitChannel()
            self.low_cmd_publisher.InitChannel()
            self.client.Init()
        except Exception as e:
            self.logger.error(f"Failed to initialize communication: {e}")
            raise

    # 観測に使っているフィードバックデータを受け取る関数
    def _low_state_handler(self, low_state_msg: LowState):
        if abs(low_state_msg.imu_state.rpy[0]) > 1.0 or abs(low_state_msg.imu_state.rpy[1]) > 1.0:
            self.logger.warning("IMU base rpy values are too large: {}".format(low_state_msg.imu_state.rpy))
            self.running = False
        self.timer.tick_timer_if_sim()
        time_now = self.timer.get_time()
        for i, motor in enumerate(low_state_msg.motor_state_serial):
            self.dof_pos_latest[i] = motor.q
        if time_now >= self.next_inference_time:
            self.projected_gravity[:] = rotate_vector_inverse_rpy(
                low_state_msg.imu_state.rpy[0],
                low_state_msg.imu_state.rpy[1],
                low_state_msg.imu_state.rpy[2],
                np.array([0.0, 0.0, -1.0]),
            )
            self.base_ang_vel[:] = low_state_msg.imu_state.gyro
            for i, motor in enumerate(low_state_msg.motor_state_serial):
                self.dof_pos[i] = motor.q
                self.dof_vel[i] = motor.dq

    def _send_cmd(self, cmd: LowCmd):
        global cmd_q_log
        self.low_cmd_publisher.Write(cmd)
        input_data = cmd.motor_cmd[11:]  # Only take the last 12 motors
        list_cmd_q = [motor_cmd.q for motor_cmd in input_data]
        cmd_q_log.append(list_cmd_q)
        # input_data[0].q, input_data[0].kp, input_data[0].kd

    def cleanup(self) -> None:
        """Cleanup resources."""
        self.remoteControlService.close()
        if hasattr(self, "low_cmd_publisher"):
            self.low_cmd_publisher.CloseChannel()
        if hasattr(self, "low_state_subscriber"):
            self.low_state_subscriber.CloseChannel()
        if hasattr(self, "publish_runner") and getattr(self, "publish_runner") != None:
            self.publish_runner.join(timeout=1.0)

    def start_custom_mode_conditionally(self):
        print(f"{self.remoteControlService.get_custom_mode_operation_hint()}")
        while True:
            if self.remoteControlService.start_custom_mode():
                break
            time.sleep(0.1)
        start_time = time.perf_counter()
        create_prepare_cmd(self.low_cmd, self.cfg)
        for i in range(B1JointCnt):
            self.dof_target[i] = self.low_cmd.motor_cmd[i].q
            self.filtered_dof_target[i] = self.low_cmd.motor_cmd[i].q
        self._send_cmd(self.low_cmd)
        send_time = time.perf_counter()
        self.logger.debug(f"Send cmd took {(send_time - start_time)*1000:.4f} ms")
        self.client.ChangeMode(RobotMode.kCustom)
        end_time = time.perf_counter()
        self.logger.debug(f"Change mode took {(end_time - send_time)*1000:.4f} ms")

    def start_rl_gait_conditionally(self):
        print(f"{self.remoteControlService.get_rl_gait_operation_hint()}")
        while True:
            if self.remoteControlService.start_rl_gait():
                break
            time.sleep(0.1)
        create_first_frame_rl_cmd(self.low_cmd, self.cfg)
        self._send_cmd(self.low_cmd)
        self.next_inference_time = self.timer.get_time()
        self.next_publish_time = self.timer.get_time()
        self.publish_runner = threading.Thread(target=self._publish_cmd)
        self.publish_runner.daemon = True
        self.publish_runner.start()
        print(f"{self.remoteControlService.get_operation_hint()}")

    def run(self):
        global obs_dof_pos_leg_log, obs_dof_vel_leg_log, obs_base_ang_vel_log, obs_projected_gravity_log, obs_controller_cmd_log, obs_gait_freq_log

        time_now = self.timer.get_time()
        if time_now < self.next_inference_time:
            time.sleep(0.001)
            return
        self.logger.debug("-----------------------------------------------------")
        self.next_inference_time += self.policy.get_policy_interval()
        self.logger.debug(f"Next start time: {self.next_inference_time}")
        start_time = time.perf_counter()

        self.dof_target[:], self.obs_log[:] = self.policy.inference(
            time_now=time_now,
            dof_pos=self.dof_pos,
            dof_vel=self.dof_vel,
            base_ang_vel=self.base_ang_vel,
            projected_gravity=self.projected_gravity,
            vx=self.remoteControlService.get_vx_cmd(),
            vy=self.remoteControlService.get_vy_cmd(),
            vyaw=self.remoteControlService.get_vyaw_cmd(),
        )

        # obs_time_log.append(time_now)
        obs_projected_gravity_log.append(self.obs_log.copy()[0:3])
        obs_base_ang_vel_log.append(self.obs_log.copy()[3:6])
        obs_controller_cmd_log.append([self.obs_log.copy()[6],
                                       self.obs_log.copy()[7],
                                       self.obs_log.copy()[8]])
        obs_gait_freq_log.append([self.obs_log.copy()[9],
                              self.obs_log.copy()[10]])
        obs_dof_pos_leg_log.append(self.obs_log.copy()[11:23])
        obs_dof_vel_leg_log.append(self.obs_log.copy()[23:35])
        obs_action_log.append(self.obs_log.copy()[35:47])
        
        
        inference_time = time.perf_counter()
        self.logger.debug(f"Inference took {(inference_time - start_time)*1000:.4f} ms")
        time.sleep(0.001)

    def _publish_cmd(self):
        while self.running:
            time_now = self.timer.get_time()
            if time_now < self.next_publish_time:
                time.sleep(0.001)
                continue
            self.next_publish_time += self.cfg["common"]["dt"]
            self.logger.debug(f"Next publish time: {self.next_publish_time}")

            # 前の制御入力（各関節角度） * 0.8 + 今回の目標角度値 * 0.2で、今回の制御入力を計算している
                # 変化量を抑えるため？
            self.filtered_dof_target = self.filtered_dof_target * 0.8 + self.dof_target * 0.2

            for i in range(B1JointCnt):  # 足首以外は位置制御っぽい
                self.low_cmd.motor_cmd[i].q = self.filtered_dof_target[i]

            # Use series-parallel conversion for torque to avoid non-linearity
            # 足首関節だけに適用。足首だけトルク制御になっているっぽい
            for i in self.cfg["mech"]["parallel_mech_indexes"]:
                self.low_cmd.motor_cmd[i].q = self.dof_pos_latest[i]
                self.low_cmd.motor_cmd[i].tau = np.clip(
                    (self.filtered_dof_target[i] - self.dof_pos_latest[i]) * self.cfg["common"]["stiffness"][i],
                    -self.cfg["common"]["torque_limit"][i],
                    self.cfg["common"]["torque_limit"][i],
                )
                self.low_cmd.motor_cmd[i].kp = 0.0

            start_time = time.perf_counter()
            self._send_cmd(self.low_cmd)
            publish_time = time.perf_counter()
            self.logger.debug(f"Publish took {(publish_time - start_time)*1000:.4f} ms")
            time.sleep(0.001)

    def __enter__(self) -> "Controller":
        return self

    def __exit__(self, *args) -> None:
        self.cleanup()


if __name__ == "__main__":
    import argparse
    import signal
    import sys
    import os

    def logging():
        global cmd_q_log, obs_dof_pos_leg_log, obs_dof_vel_leg_log, obs_base_ang_vel_log, obs_projected_gravity_log, obs_controller_cmd_log, obs_gait_freq_log, obs_action_log
        print("\noutput logging...")
        with open("cmd_q_log.dat", "w") as f:
            for cmd_list in cmd_q_log:
                f.write("   ".join(map(str, cmd_list)) + "\n")
        # with open("obs_time_log.dat", "w") as f:
        #     for time_value in obs_time_log:
        #         f.write(f"{time_value}\n")
        with open("obs_dof_pos_leg_log.dat", "w") as f:
            for pos_list in obs_dof_pos_leg_log:
                f.write("   ".join(map(str, pos_list)) + "\n")
        with open("obs_dof_vel_leg_log.dat", "w") as f:
            for vel_list in obs_dof_vel_leg_log:
                f.write("   ".join(map(str, vel_list)) + "\n")
        with open("obs_base_ang_vel_log.dat", "w") as f:
            for ang_vel in obs_base_ang_vel_log:
                f.write("   ".join(map(str, ang_vel)) + "\n")
        with open("obs_projected_gravity_log.dat", "w") as f:
            for gravity in obs_projected_gravity_log:
                f.write("   ".join(map(str, gravity)) + "\n")
        with open("obs_controller_cmd_log.dat", "w") as f:
            for cmd in obs_controller_cmd_log:
                f.write("   ".join(map(str, cmd)) + "\n")
        with open("obs_gait_freq_log.dat", "w") as f:
            for gait_freq in obs_gait_freq_log:
                f.write("   ".join(map(str, gait_freq)) + "\n")
        with open("obs_action_log.dat", "w") as f:
            for action in obs_action_log:
                f.write("   ".join(map(str, action)) + "\n")
        print("Logging complete.")
        print("Exiting gracefully...")

    def signal_handler(sig, frame):

        logging()

        print("\nShutting down...")

        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, type=str, help="Name of the configuration file.")
    parser.add_argument("--net", type=str, default="127.0.0.1", help="Network interface for SDK communication.")
    args = parser.parse_args()
    cfg_file = os.path.join("configs", args.config)

    print(f"Starting custom controller, connecting to {args.net} ...")
    ChannelFactory.Instance().Init(0, args.net)

    with Controller(cfg_file) as controller:
        time.sleep(2)  # Wait for channels to initialize
        print("Initialization complete.")
        controller.start_custom_mode_conditionally()
        controller.start_rl_gait_conditionally()

        try:
            while controller.running:
                controller.run()
            controller.client.ChangeMode(RobotMode.kDamping)
            logging()
            print("\n Controller stopped. Switching to damping mode. Shutdown...")
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received. Cleaning up...")
            controller.cleanup()
