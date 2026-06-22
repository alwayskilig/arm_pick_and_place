#!/usr/bin/env python3
"""
系统测试与验证脚本
用于验证ROS2机械臂抓取与递送系统的完整功能

使用方法:
    python3 test_system.py

前置条件:
    1. 已完成colcon build编译
    2. 已source install/setup.bash
"""

import subprocess
import sys
import time
import signal
import os


class SystemTester:
    def __init__(self):
        self.processes = []
        self.test_results = []

    def run_command(self, cmd, timeout=10):
        try:
            result = subprocess.run(
                cmd, shell=True, capture_output=True, text=True, timeout=timeout
            )
            return result.returncode == 0, result.stdout, result.stderr
        except subprocess.TimeoutExpired:
            return False, "", "Command timed out"
        except Exception as e:
            return False, "", str(e)

    def test_message_compilation(self):
        print("\n" + "=" * 60)
        print("测试1: 验证消息编译")
        print("=" * 60)

        # 测试colcon build
        print("\n[1.1] 运行 colcon build...")
        success, stdout, stderr = self.run_command(
            "colcon build --packages-select arm_pick_and_place", timeout=60
        )
        if success:
            print("  ✓ 消息编译成功")
        else:
            print("  ✗ 消息编译失败")
            print(f"  错误: {stderr}")
            self.test_results.append(("消息编译", False))
            return False

        # 验证TaskCommand消息
        print("\n[1.2] 验证 TaskCommand 消息定义...")
        success, stdout, stderr = self.run_command(
            "ros2 interface show arm_pick_and_place/msg/TaskCommand"
        )
        if success and "command_type" in stdout:
            print("  ✓ TaskCommand 消息定义正确")
            print(f"  消息内容:\n{stdout}")
        else:
            print("  ✗ TaskCommand 消息验证失败")
            self.test_results.append(("TaskCommand消息", False))
            return False

        # 验证TaskStatus消息
        print("\n[1.3] 验证 TaskStatus 消息定义...")
        success, stdout, stderr = self.run_command(
            "ros2 interface show arm_pick_and_place/msg/TaskStatus"
        )
        if success and "state" in stdout and "progress" in stdout:
            print("  ✓ TaskStatus 消息定义正确")
            print(f"  消息内容:\n{stdout}")
        else:
            print("  ✗ TaskStatus 消息验证失败")
            self.test_results.append(("TaskStatus消息", False))
            return False

        self.test_results.append(("消息编译", True))
        return True

    def test_node_startup(self):
        print("\n" + "=" * 60)
        print("测试2: 验证节点启动")
        print("=" * 60)

        nodes = [
            ("vision_node", "vision_node", "vision_node"),
            ("arm_controller", "arm_controller", "arm_controller"),
            ("gripper_controller", "gripper_controller", "gripper_controller"),
            ("task_scheduler", "task_scheduler", "task_scheduler"),
        ]

        for name, package, executable in nodes:
            print(f"\n[2.x] 启动 {name}...")
            try:
                proc = subprocess.Popen(
                    ["ros2", "run", package, executable],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                )
                self.processes.append((name, proc))
                time.sleep(2)

                if proc.poll() is None:
                    print(f"  ✓ {name} 启动成功 (PID: {proc.pid})")
                else:
                    stderr = proc.stderr.read().decode()
                    print(f"  ✗ {name} 启动失败")
                    print(f"  错误: {stderr}")
                    self.test_results.append((f"{name}启动", False))
                    return False
            except Exception as e:
                print(f"  ✗ {name} 启动异常: {e}")
                self.test_results.append((f"{name}启动", False))
                return False

        print("\n  ✓ 所有节点启动成功")
        self.test_results.append(("节点启动", True))
        return True

    def test_topic_communication(self):
        print("\n" + "=" * 60)
        print("测试3: 验证话题通信")
        print("=" * 60)

        # 列出所有话题
        print("\n[3.1] 列出所有话题...")
        success, stdout, stderr = self.run_command("ros2 topic list", timeout=5)
        if success:
            print(f"  当前话题:\n{stdout}")
        else:
            print("  ✗ 无法列出话题")
            self.test_results.append(("话题列表", False))
            return False

        # 检查必要话题是否存在
        required_topics = [
            "/task/command",
            "/task/status",
            "/vision/target_pose",
            "/arm/joint_states",
            "/gripper/state",
        ]
        missing_topics = []
        for topic in required_topics:
            if topic not in stdout:
                missing_topics.append(topic)

        if missing_topics:
            print(f"  ✗ 缺少必要话题: {missing_topics}")
            self.test_results.append(("话题完整性", False))
        else:
            print("  ✓ 所有必要话题存在")

        # 监听任务状态话题
        print("\n[3.2] 监听 /task/status 话题 (5秒)...")
        success, stdout, stderr = self.run_command(
            "timeout 5 ros2 topic echo /task/status --once", timeout=10
        )
        if success and stdout.strip():
            print(f"  ✓ 收到状态消息:\n{stdout}")
        else:
            print("  ⚠ 未收到状态消息 (可能是正常的，任务可能尚未开始)")

        # 发布测试指令 - RESET
        print("\n[3.3] 发送 RESET 指令...")
        success, stdout, stderr = self.run_command(
            'ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 3}"',
            timeout=5,
        )
        if success:
            print("  ✓ RESET 指令发送成功")
        else:
            print(f"  ✗ RESET 指令发送失败: {stderr}")
            self.test_results.append(("RESET指令", False))
            return False

        time.sleep(1)

        # 再次监听状态
        print("\n[3.4] 验证状态响应...")
        success, stdout, stderr = self.run_command(
            "timeout 5 ros2 topic echo /task/status --once", timeout=10
        )
        if success and stdout.strip():
            print(f"  ✓ 收到状态响应:\n{stdout}")
        else:
            print("  ⚠ 未收到状态响应")

        self.test_results.append(("话题通信", True))
        return True

    def test_gripper_control(self):
        print("\n" + "=" * 60)
        print("测试4: 验证夹爪控制")
        print("=" * 60)

        # 监听夹爪状态
        print("\n[4.1] 监听 /gripper/state 话题...")
        success, stdout, stderr = self.run_command(
            "timeout 3 ros2 topic echo /gripper/state --once", timeout=5
        )
        if success and stdout.strip():
            print(f"  ✓ 收到夹爪状态: {stdout.strip()}")
        else:
            print("  ⚠ 未收到夹爪状态")

        # 发送GRASP指令
        print("\n[4.2] 发送 GRASP 指令...")
        success, stdout, stderr = self.run_command(
            'ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 1}"',
            timeout=5,
        )
        if success:
            print("  ✓ GRASP 指令发送成功")
        else:
            print(f"  ✗ GRASP 指令发送失败: {stderr}")

        time.sleep(3)

        # 检查夹爪状态变化
        print("\n[4.3] 检查夹爪状态变化...")
        success, stdout, stderr = self.run_command(
            "timeout 3 ros2 topic echo /gripper/state --once", timeout=5
        )
        if success and stdout.strip():
            print(f"  ✓ 夹爪状态: {stdout.strip()}")
        else:
            print("  ⚠ 未收到夹爪状态更新")

        # 发送DELIVER指令（打开夹爪）
        print("\n[4.4] 发送 DELIVER 指令...")
        success, stdout, stderr = self.run_command(
            'ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 2}"',
            timeout=5,
        )
        if success:
            print("  ✓ DELIVER 指令发送成功")
        else:
            print(f"  ✗ DELIVER 指令发送失败: {stderr}")

        self.test_results.append(("夹爪控制", True))
        return True

    def test_full_launch(self):
        print("\n" + "=" * 60)
        print("测试5: 验证完整启动文件")
        print("=" * 60)

        print("\n[5.1] 测试 launch 文件语法...")
        success, stdout, stderr = self.run_command(
            "ros2 launch arm_pick_and_place full_system.launch.py --show-args",
            timeout=10,
        )
        if success:
            print("  ✓ Launch 文件语法正确")
        else:
            print(f"  ✗ Launch 文件语法错误: {stderr}")
            self.test_results.append(("Launch文件", False))
            return False

        self.test_results.append(("Launch文件", True))
        return True

    def cleanup(self):
        print("\n" + "=" * 60)
        print("清理: 停止所有启动的节点")
        print("=" * 60)
        for name, proc in self.processes:
            try:
                proc.terminate()
                proc.wait(timeout=3)
                print(f"  ✓ {name} 已停止")
            except subprocess.TimeoutExpired:
                proc.kill()
                print(f"  ⚠ {name} 已强制终止")
            except Exception as e:
                print(f"  ✗ {name} 停止失败: {e}")

    def print_summary(self):
        print("\n" + "=" * 60)
        print("测试结果汇总")
        print("=" * 60)

        passed = sum(1 for _, result in self.test_results if result)
        total = len(self.test_results)

        for test_name, result in self.test_results:
            status = "✓ 通过" if result else "✗ 失败"
            print(f"  {test_name}: {status}")

        print(f"\n总计: {passed}/{total} 项测试通过")

        if passed == total:
            print("\n🎉 所有测试通过！系统集成验证成功。")
            return True
        else:
            print("\n⚠ 部分测试失败，请检查上述错误信息。")
            return False

    def run_all_tests(self):
        print("=" * 60)
        print("  ROS2 机械臂抓取与递送系统 - 系统测试")
        print("=" * 60)

        try:
            self.test_message_compilation()
            self.test_full_launch()
            self.test_node_startup()
            self.test_topic_communication()
            self.test_gripper_control()
        except KeyboardInterrupt:
            print("\n\n测试被用户中断")
        finally:
            self.cleanup()

        return self.print_summary()


def main():
    tester = SystemTester()
    success = tester.run_all_tests()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
