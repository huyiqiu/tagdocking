"""Dual-only signed odometry watchdog; shared single-tag executor unchanged."""
import math


class ActionWatch:
    def __init__(self, plan, now, pose, target, speed, p):
        self.plan, self.start, self.pose, self.target, self.p = plan, now, pose, target, p
        self.yaw = bool(plan.turn_angle)
        self.lateral = bool(plan.lateral_distance)
        amount = plan.turn_angle if self.yaw else (plan.lateral_distance or plan.jog_distance)
        self.sign = math.copysign(1., amount)
        self.noise = p('odom_noise_rad' if self.yaw else 'odom_noise_m')
        # Executor's target is already divided by odom_scale. Use actual issued
        # speed (half-speed small turns), but cap every action in wall clock.
        self.deadline = min(p('action_timeout_sec'), max(p('response_timeout_sec'),
                            3*abs(amount)/max(abs(speed), 1e-6)+1.))
        self.signed = 0.

    def check(self, now, pose, odom_stamp):
        if (odom_stamp <= 0 or not 0 <= now-odom_stamp <= self.p('odom_fresh_sec')*1e9
                or not all(math.isfinite(v) for v in pose)):
            return 'dual odometry stale / invalid'
        dx, dy = pose[0]-self.pose[0], pose[1]-self.pose[1]
        if self.yaw:
            d = pose[2]-self.pose[2]
            self.signed = math.atan2(math.sin(d),math.cos(d))
        elif self.lateral:
            self.signed = -math.sin(self.pose[2])*dx+math.cos(self.pose[2])*dy
        else:
            self.signed = math.cos(self.pose[2])*dx+math.sin(self.pose[2])*dy
        progress = self.sign*self.signed
        elapsed = (now-self.start)*1e-9
        # 反向判据只在"底盘已有机会真正起步"之后成立。四足起步有姿态瞬态:
        # 抬腿换步期间机体先朝反向晃一下再跟上, 加上 VIO 高频里程计本身的
        # 抖动, 起步 100ms 内出现零点几度的反向位移是常态而非故障。现场
        # 3° 转向在起步 108ms 就被判 "opposite commanded direction", 正是
        # 拿瞬态当了故障。
        #
        # 门槛同时从 min() 改为 max(): 用 min 时门槛被噪声地板 3×odom_noise
        # 锁死在 0.34° —— 那是静止底盘的量化噪声, 不是匍匐四足的机体晃动。
        # 改用 max(噪声地板, 目标量的一半) 既高于瞬态, 又保留原意图: 真正
        # "整步走反了" (位移达 -target) 依然远超半程门槛, 必被抓到。
        threshold = max(3*self.noise, self.target*.5)
        if elapsed > self.p('action_startup_sec') and progress < -threshold:
            return (f'dual signed odometry opposite commanded direction '
                    f'(signed={self.signed:+.5f} 指令符号={self.sign:+.0f} '
                    f'门槛={threshold:.5f} 已历时={elapsed:.2f}s)')
        if elapsed > self.deadline:
            return 'dual action timeout'
        if elapsed > self.p('response_timeout_sec') and progress < min(self.noise,self.target*.2):
            return 'dual action no odometry response'
        # Straight executor uses unsigned distance. Veto sideways drift completing it.
        if not self.yaw and not self.lateral and math.hypot(dx,dy) >= self.target and progress < self.target*.8:
            return 'dual odometry displacement inconsistent with straight command'
        return ''
