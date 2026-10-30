"""智能巡检机器人核心调度程序，Python 3.10+。

每20ms调用tick；IO由实际主控/上位机适配实现。
硬件急停必须独立于本程序。所有阈值须由实机标定。
导航避障规划、仪表识别、关节闭链约束与充电握手由对应接口实现。
"""
from dataclasses import dataclass
from enum import Enum, auto
from math import isfinite
from typing import Protocol


@dataclass(frozen=True)
class Feedback:
    stamp: float                 # 单调时钟，秒
    battery: float               # 0~1，由BMS提供
    clearance: float             # 所有运动方向有效的最小净空，米
    left_speed: float            # 编码器换算为m/s
    right_speed: float
    estop: bool
    fault: bool                  # 包括过流、过温、碰撞、关节过载等
    arm_stowed: bool             # 实际位置反馈，不是命令已发送
    arm_ready: bool              # 已达到当前观察目标并稳定
    charging: bool               # BMS确认已进入充电


class IO(Protocol):
    def feedback(self) -> Feedback: ...
    def wheels(self, left: float, right: float) -> None:
        """归一化有符号输出[-1,1]；适配层转换为方向与PWM。"""
        ...
    def stop(self) -> None:
        """立即停车并清除运动命令；使用驱动器实际支持的安全停车方式。"""
        ...
    def alarm(self, message: str) -> None: ...
    def stow_arm(self) -> None:
        """发送已标定收拢轨迹，遵守关节限位、闭链约束和碰撞限制。"""
        ...
    def observe(self, point: str) -> None:
        """发送该巡检点已标定的五关节观察轨迹，夹爪保持相机安装状态。"""
        ...
    def navigate(self, target: str) -> tuple[float, float, bool]:
        """返回v(m/s)、角速度(rad/s)、是否到达；须融合定位与路径规划。"""
        ...
    def inspect(self, point: str) -> bool | None:
        """非阻塞视觉任务：None处理中，True有效，False失败。保存读数与告警。"""
        ...
    def dock(self) -> bool:
        """非阻塞停靠握手；由独立低速对接控制器执行，返回是否对接完成。"""
        ...


class State(Enum):
    IDLE = auto()
    STOW = auto()
    TRAVEL = auto()
    ARM = auto()
    READ = auto()
    DOCK = auto()
    CHARGE = auto()
    FAULT = auto()


class PI:
    def __init__(self, kp=1.0, ki=0.3):
        self.kp, self.ki, self.integral = kp, ki, 0.0

    def update(self, target, actual, dt):
        if abs(target) < 1e-5:
            self.integral = 0.0
            return 0.0
        error = target - actual
        candidate = max(-1.0, min(1.0, self.integral + error * dt))
        raw = self.kp * error + self.ki * candidate
        if abs(raw) <= 1.0 or raw * error < 0:
            self.integral = candidate
        return max(-1.0, min(1.0, raw))


class Robot:
    # 以下为调试起点，非已验证的85kg设备运行参数。
    TRACK_SPACING = 0.50         # 履带中心距m，须测量替换
    MAX_TRACK_SPEED = 0.20
    STOP_CLEARANCE = 0.50        # 须按制动距离、盲区及延迟重新标定
    LOW_BATTERY = 0.20
    CRITICAL_BATTERY = 0.10

    def __init__(self, io: IO, points: list[str]):
        if not points:
            raise ValueError('至少配置一个巡检点')
        self.io, self.points = io, tuple(points)
        self.state = State.IDLE
        self.index = 0
        self.returning = False
        self.last_time = None
        self.entered = 0.0
        self.left, self.right = PI(), PI()

    def transition(self, state, now):
        self.state, self.entered = state, now

    def halt(self):
        self.io.stop()
        self.left.integral = self.right.integral = 0.0

    def fail(self, message, now):
        self.transition(State.FAULT, now)
        self.halt()
        self.io.alarm(message)

    def start(self, now):
        """故障后禁止自动重启；需检修并重新创建控制器。"""
        if self.state != State.IDLE:
            return
        self.index, self.returning = 0, False
        self.halt()
        self.io.stow_arm()
        self.transition(State.STOW, now)

    def drive(self, v, omega, f, dt):
        if not all(isfinite(x) for x in (v, omega)):
            raise ValueError('导航命令无效')
        l = v - omega * self.TRACK_SPACING / 2
        r = v + omega * self.TRACK_SPACING / 2
        scale = max(1.0, abs(l)/self.MAX_TRACK_SPEED, abs(r)/self.MAX_TRACK_SPEED)
        self.io.wheels(self.left.update(l/scale, f.left_speed, dt),
                       self.right.update(r/scale, f.right_speed, dt))

    def tick(self, now):
        try:
            self._tick(now)
        except Exception as exc:
            self.fail(f'控制异常：{exc}', now)

    def _tick(self, now):
        dt = 0.02 if self.last_time is None else now - self.last_time
        self.last_time = now
        if not isfinite(now) or not 0 < dt <= 0.10:
            self.fail('控制周期超时或时钟无效', now)
            return
        f = self.io.feedback()
        if (not all(isfinite(x) for x in (f.stamp, f.battery, f.clearance,
                                         f.left_speed, f.right_speed))
                or not 0 <= now - f.stamp <= 0.20
                or not 0 <= f.battery <= 1 or f.clearance < 0):
            self.fail('反馈无效或通信超时', now)
            return
        if f.estop or f.fault or f.battery <= self.CRITICAL_BATTERY:
            self.fail('急停、设备故障或电量过低', now)
            return
        if self.state in (State.IDLE, State.FAULT, State.CHARGE):
            self.halt()
            return
        if f.battery <= self.LOW_BATTERY and not self.returning:
            self.returning = True
            self.halt()
            self.io.stow_arm()
            self.transition(State.STOW, now)
        timeout = {State.STOW: 20, State.TRAVEL: 180, State.ARM: 20,
                   State.READ: 10, State.DOCK: 60}[self.state]
        if now - self.entered > timeout:
            self.fail('当前任务超时，需人工处理', now)
            return
        if self.state == State.STOW:
            self.halt()
            if f.arm_stowed:
                self.transition(State.TRAVEL, now)
        elif self.state == State.TRAVEL:
            if not f.arm_stowed:
                self.fail('行驶时机械臂未收拢', now)
                return
            if f.clearance <= self.STOP_CLEARANCE:
                self.halt()     # 停车等待；不盲目后退或绕行
                return
            target = 'charging_station' if self.returning else self.points[self.index]
            v, omega, arrived = self.io.navigate(target)
            if not arrived:
                self.drive(v, omega, f, dt)
                return
            self.halt()
            if self.returning:
                self.transition(State.DOCK, now)
            elif max(abs(f.left_speed), abs(f.right_speed)) < 0.01:
                self.io.observe(target)
                self.transition(State.ARM, now)
        elif self.state == State.ARM:
            self.halt()
            if f.arm_ready:
                self.transition(State.READ, now)
        elif self.state == State.READ:
            self.halt()
            result = self.io.inspect(self.points[self.index])
            if result is None:
                return
            if not result:
                self.fail('视觉读数失败，需人工复核', now)
                return
            self.index += 1
            if self.index == len(self.points):
                self.returning = True
            self.io.stow_arm()
            self.transition(State.STOW, now)
        elif self.state == State.DOCK:
            if not f.arm_stowed:
                self.fail('对接时机械臂未收拢', now)
                return
            # 对接控制器必须独立限速、检测接触及充电电气状态。
            if self.io.dock() and f.charging:
                self.halt()
                self.transition(State.CHARGE, now)
