#!/usr/bin/env python3
"""
debug_coords.py - 调试坐标系的脚本

用法: 在 user_entry.py 的 game() 函数中调用此脚本

输出信息:
- 机器人全局位置 (MOS坐标系: X-Forward, Y-Left)
- 机器人朝向 (度)
- 球相对机器人的位置 (机器人坐标系: X-Forward, Y-Left)
- 球的全局位置
"""

import math
import time

class CoordDebugger:
    def __init__(self, agent):
        self.agent = agent
        self.logger = agent.get_logger().get_child("CoordDebug")
        self.last_print_time = 0
        self.print_interval = 0.5  # 每0.5秒打印一次
        
    def run(self):
        """调用此方法打印坐标信息"""
        current_time = time.time()
        if current_time - self.last_print_time < self.print_interval:
            return
        self.last_print_time = current_time
        
        # 获取机器人全局位置
        my_pos = self.agent.get_self_pos()
        my_yaw = self.agent.get_self_yaw()
        
        # 获取球的相对位置
        ball_visible = self.agent.get_if_ball()
        ball_rel = self.agent.get_ball_pos() if ball_visible else None
        ball_dist = self.agent.get_ball_distance() if ball_visible else None
        
        # 获取球的全局位置 (如果可用)
        try:
            ball_global = self.agent.vision.get_ball_pos_in_map() if ball_visible else None
        except:
            ball_global = None
        
        # 打印分隔线
        print("\n" + "="*70)
        print(f"[COORD DEBUG] Time: {current_time:.2f}")
        print("="*70)
        
        # 机器人位置
        print(f"\n【机器人全局位置】(MOS坐标系: X-Forward, Y-Left)")
        if my_pos is not None:
            print(f"  位置: X={my_pos[0]:.3f}m (前+), Y={my_pos[1]:.3f}m (左+)")
            print(f"  朝向: Yaw={my_yaw:.1f}° (0°=前/X+, 90°=左/Y+, -90°=右)")
        else:
            print("  位置: 无数据")
        
         # 球的位置
        print(f"\n【球相对位置】(机器人坐标系: X-Forward, Y-Left)")
        if ball_visible and ball_rel is not None:
            # ball_rel 输出格式是 [Forward, Left]
            print(f"  相对位置: 前={ball_rel[0]:.3f}m, 左={ball_rel[1]:.3f}m")
            print(f"  距离: {ball_dist:.3f}m")
            
            # 计算球的角度: atan2(Left, Forward), 正值=左侧, 负值=右侧
            ball_angle = math.degrees(math.atan2(ball_rel[1], ball_rel[0]))
            print(f"  角度: {ball_angle:.1f}° (0°=正前方, 正值=左侧, 负值=右侧)")
        else:
            print("  球不可见")
            
        # 球的全局位置
        print(f"\n【球全局位置】")
        if ball_global is not None:
            print(f"  位置: X={ball_global[0]:.3f}m, Y={ball_global[1]:.3f}m")
        else:
            print("  球不可见或无数据")
        
        print("="*70 + "\n")


# 全局实例，用于方便调用
_debugger = None

def init_debugger(agent):
    global _debugger
    _debugger = CoordDebugger(agent)
    
def debug_coords(agent):
    """在 game() 函数中调用此函数来打印坐标"""
    global _debugger
    if _debugger is None:
        _debugger = CoordDebugger(agent)
    _debugger.run()


# 使用方法:
# 在 user_entry.py 的 game() 函数最前面添加:
#   from debug_coords import debug_coords
#   debug_coords(agent)
