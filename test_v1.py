#!/usr/bin/env python
import os
import time
import numpy as np
import pandas as pd
import pinocchio as pin
import sys
import termios
import select
import tty
from pinocchio.visualize import MeshcatVisualizer
from scipy.spatial.transform import Rotation as R
import transformations 
class PinocchioCSVPlayer:
    def __init__(self):
        # 获取当前脚本路径
        current_path = os.path.dirname(os.path.abspath(__file__))

        # 设置URDF路径和Mesh资源目录
        urdf_path = os.path.join(current_path, 'src/g1_description/urdf', 'g1.urdf')
        package_dirs = [os.path.join(current_path, 'src')]

        # 验证URDF存在
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF文件不存在: {urdf_path}")

        # 使用自由浮动根关节（6自由度）
        self.robot = pin.RobotWrapper.BuildFromURDF(
            urdf_path,
            root_joint=pin.JointModelFreeFlyer(),
            package_dirs=package_dirs
        )
        print(f"[INFO] URDF加载成功，共 {self.robot.model.nq} 个自由度")

        # 初始化MeshCat可视化器
        self.viz = MeshcatVisualizer(
            model=self.robot.model,
            collision_model=self.robot.collision_model,
            visual_model=self.robot.visual_model
        )
        self.viz.initViewer(open=True)
        self.viz.loadViewerModel()
        print("[INFO] MeshCat 可视化器就绪")
        # time.sleep(10)  # 等待可视化器加载
        # 加载CSV运动数据
        self.load_csv_data(os.path.join(current_path, "x_pre_finall_data/337_bixin.seq"))

    def load_csv_data(self, csv_path):
        if not os.path.exists(csv_path):
            raise FileNotFoundError(f"CSV文件不存在: {csv_path}")
        self.motion_data = pd.read_csv(csv_path)
        expected_dof = self.robot.model.nq
        if self.motion_data.shape[1]+1 != expected_dof:
            raise ValueError(f"CSV列数 {self.motion_data.shape[1]} 不匹配机器人自由度 {expected_dof}")
        print(f"[INFO] 加载CSV成功，共 {len(self.motion_data)} 帧")

    def run(self, fps=30):
        print(f"[INFO] 开始播放动画（{fps} FPS）...")
        frame_interval = 1.0 / fps
        def wait_for_space():
            print("按空格键继续播放，Ctrl+C 可随时退出...", end='', flush=True)
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                while True:
                    # select 超时0.1秒，及时响应 Ctrl+C
                    rlist, _, _ = select.select([fd], [], [], 0.1)
                    if rlist:
                        ch = sys.stdin.read(1)
                        if ch == ' ':
                            print()
                            break
                        if ch.lower() == 'q':
                            print("\n[INFO] 用户按Q键，退出播放。")
                            sys.exit(0)
            except KeyboardInterrupt:
                print("\n[INFO] 用户中断，退出播放。")
                sys.exit(0)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        for i in range(len(self.motion_data)):
            row = self.motion_data.iloc[i]
            # 构造配置向量 q
            base_pos = row.iloc[0:3].values               # tx, ty, tz
            base_pos[2]-=0.05
            base_rpy = row.iloc[3:6].values
            base_quat = R.from_euler('xyz', base_rpy).as_quat()  # 转四元数: x, y, z, w
            q = np.zeros(self.robot.model.nq)
            q[0:3] = base_pos
            q[3:7] = [base_quat[0], base_quat[1], base_quat[2],base_quat[3]]  # w, x, y, z
            q[7:] = row.iloc[6:].values.astype(float)  # 关节角度
            if not i :
                time.sleep(5)
            self.viz.display(q)
            print(f"[INFO] 显示帧 {i+1}/{len(self.motion_data)}")
            # wait_for_space()
            time.sleep(frame_interval)

        print("[INFO] 播放完毕。")

if __name__ == '__main__':
    player = PinocchioCSVPlayer()
    player.run(fps=50)
