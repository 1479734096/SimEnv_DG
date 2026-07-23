#!/usr/bin/env python3
"""根据当前文档结构生成可重复使用的中文示意图。"""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / "assets"


def setup() -> None:
    ASSETS.mkdir(parents=True, exist_ok=True)
    plt.rcParams["font.sans-serif"] = ["Noto Sans CJK JP"]
    plt.rcParams["axes.unicode_minus"] = False


def box(ax, xy, width, height, text, color="#E8F0F7", edge="#36566F", size=10):
    patch = FancyBboxPatch(
        xy,
        width,
        height,
        boxstyle="round,pad=0.02,rounding_size=0.03",
        linewidth=1.2,
        edgecolor=edge,
        facecolor=color,
    )
    ax.add_patch(patch)
    ax.text(xy[0] + width / 2, xy[1] + height / 2, text, ha="center", va="center", fontsize=size)
    return patch


def arrow(ax, start, end, color="#52616B", style="-|>"):
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle=style, mutation_scale=13, linewidth=1.2, color=color))


def finish(fig, ax, name):
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 7)
    ax.axis("off")
    fig.tight_layout()
    fig.savefig(ASSETS / name, dpi=180, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def architecture():
    fig, ax = plt.subplots(figsize=(12, 7.4))
    ax.set_title("四足机器人危险源自主搜索系统总体架构", fontsize=16, pad=18)
    box(ax, (0.3, 5.4), 1.7, 0.8, "Livox 点云\n机身 IMU", "#DDEBF7")
    box(ax, (0.3, 3.9), 1.7, 0.8, "RealSense\nRGB + 深度", "#DDEBF7")
    box(ax, (0.3, 2.4), 1.7, 0.8, "公开场景信息\n门/电梯服务", "#DDEBF7")
    box(ax, (2.6, 5.4), 1.8, 0.8, "合规定位\nPoseProvider", "#E2F0D9")
    box(ax, (2.6, 3.9), 1.8, 0.8, "传感器缓存\n时间匹配", "#E2F0D9")
    box(ax, (2.6, 2.4), 1.8, 0.8, "建筑特征\n权限校验", "#E2F0D9")
    box(ax, (5.0, 5.4), 1.8, 0.8, "分层 2.5D 建图\n前沿提取", "#FFF2CC")
    box(ax, (5.0, 3.9), 1.8, 0.8, "红球检测\n多帧跟踪", "#FCE4D6")
    box(ax, (5.0, 2.4), 1.8, 0.8, "门洞/电梯感知\n非阻塞任务", "#FCE4D6")
    box(ax, (7.4, 5.4), 1.8, 0.8, "前沿评分 + A*\n路径跟踪", "#FFF2CC")
    box(ax, (7.4, 3.9), 1.8, 0.8, "任务状态机\n跨层与返航", "#EADCF8")
    box(ax, (7.4, 2.4), 1.8, 0.8, "地面支撑门\n统一速度出口", "#F4CCCC")
    box(ax, (7.4, 0.7), 1.8, 0.7, "结果原子写出\ndetected_danger.json", "#E2F0D9")
    for start, end in [
        ((2.0, 5.8), (2.6, 5.8)), ((2.0, 4.3), (2.6, 4.3)), ((2.0, 2.8), (2.6, 2.8)),
        ((4.4, 5.8), (5.0, 5.8)), ((4.4, 4.3), (5.0, 4.3)), ((4.4, 2.8), (5.0, 2.8)),
        ((6.8, 5.8), (7.4, 5.8)), ((6.8, 4.3), (7.4, 4.3)), ((6.8, 2.8), (7.4, 2.8)),
        ((8.3, 5.4), (8.3, 4.7)), ((8.3, 3.9), (8.3, 3.2)), ((8.3, 2.4), (8.3, 1.4)),
    ]:
        arrow(ax, start, end)
    ax.text(0.3, 0.4, "正式算法只使用公开输入；真值与布局元数据仅供任务结束后的离线评估。", fontsize=10, color="#8B1A1A")
    finish(fig, ax, "01-系统总体架构.png")


def state_machine():
    fig, ax = plt.subplots(figsize=(12, 7.4))
    ax.set_title("顶层任务状态机与失败收口", fontsize=16, pad=18)
    states = {
        "初始化": (0.4, 5.3), "等待定位与底盘": (2.5, 5.3), "单层自主探索": (5.2, 5.3),
        "换层任务": (5.2, 3.2), "自主返航": (7.7, 5.3), "写出结果": (7.7, 2.0),
        "结束": (7.7, 0.6), "安全停车/失败": (2.5, 2.0),
    }
    colors = {"安全停车/失败": "#F4CCCC", "结束": "#E2F0D9", "写出结果": "#E2F0D9"}
    sizes = {}
    for name, xy in states.items():
        sizes[name] = (1.7 if name != "等待定位与底盘" else 2.0, 0.75)
        box(ax, xy, *sizes[name], name, colors.get(name, "#E8F0F7"), size=10)
    def center(name):
        x, y = states[name]; w, h = sizes[name]; return x + w / 2, y + h / 2
    for a, b in [("初始化", "等待定位与底盘"), ("等待定位与底盘", "单层自主探索"),
                 ("单层自主探索", "自主返航"), ("自主返航", "写出结果"),
                 ("写出结果", "结束")]:
        arrow(ax, center(a), center(b))
    arrow(ax, (6.05, 5.3), (6.05, 3.95))
    arrow(ax, (6.9, 3.58), (7.7, 5.45))
    arrow(ax, (5.2, 3.58), (4.1, 5.35))
    ax.text(6.25, 4.55, "需换层", fontsize=9)
    ax.text(6.85, 4.25, "返程换层完成", fontsize=9)
    ax.text(4.1, 4.15, "目标层确认", fontsize=9)
    for name in ["等待定位与底盘", "单层自主探索", "换层任务", "自主返航"]:
        arrow(ax, center(name), center("安全停车/失败"), color="#B85450")
    arrow(ax, center("安全停车/失败"), center("写出结果"), color="#B85450")
    ax.text(0.5, 0.65, "触发条件：定位失效、输入陈旧、姿态异常、支撑不足、服务失败、预算耗尽或不可恢复卡滞。", fontsize=9.5)
    finish(fig, ax, "02-任务状态机.png")


def perception():
    fig, ax = plt.subplots(figsize=(12, 7.4))
    ax.set_title("危险源 RGB-D 识别与三维定位流程", fontsize=16, pad=18)
    labels = [
        "同步 RGB、深度、内参\n及时间匹配位姿", "HSV 双红色区间\n连通域候选", "面积、圆度与\n长宽比筛选",
        "深度反投影\n生成三维点集", "RANSAC/最小二乘\n球面拟合", "半径、RMSE、深度\n与刚体变换校验",
        "世界坐标候选\n置信度计算", "多帧关联与质量确认\n空间 NMS", "输出三维危险源位置",
    ]
    positions = [(0.3, 5.2), (3.5, 5.2), (6.7, 5.2), (6.7, 3.3), (3.5, 3.3), (0.3, 3.3), (0.3, 1.3), (3.5, 1.3), (6.7, 1.3)]
    for i, (label, xy) in enumerate(zip(labels, positions)):
        color = "#FCE4D6" if i in {1, 2, 3, 4, 5} else "#E2F0D9"
        box(ax, xy, 2.4, 0.85, label, color, size=9.5)
    for a, b in zip(positions[:-1], positions[1:]):
        start = (a[0] + 1.2, a[1] + 0.42)
        end = (b[0] + 1.2, b[1] + 0.42)
        arrow(ax, start, end)
    ax.text(0.35, 0.45, "拒绝策略：任一输入缺失、非有限、不同步或几何不成立时返回空候选，不读取真值回退。", fontsize=10, color="#8B1A1A")
    finish(fig, ax, "03-危险源识别流程.png")


def test_layers():
    fig, ax = plt.subplots(figsize=(12, 7.4))
    ax.set_title("分层测试与证据闭环", fontsize=16, pad=18)
    rows = [
        ("静态合规", "接口白名单、生产/离线依赖、官方文件哈希", "边界审计与差异台账"),
        ("单元测试", "定位契约、建图、规划、感知、任务、安全、结果", "462 项自动化测试"),
        ("ROS 集成", "话题频率、时间戳、坐标系、就绪门、零速失效", "节点级闭环证据"),
        ("Gazebo 专项", "主入口、门、电梯、防坠、定位漂移、返航", "固定 seed 可复现实验"),
        ("正式回归", "多随机场景、全预算覆盖、识别率、虚警率", "待补最终评分数据"),
    ]
    y = 5.7
    for i, (level, content, evidence) in enumerate(rows):
        box(ax, (0.5, y), 1.55, 0.7, level, "#DDEBF7" if i < 4 else "#FFF2CC", size=10)
        box(ax, (2.45, y), 4.25, 0.7, content, "#F7F7F7", size=9.5)
        box(ax, (7.1, y), 2.35, 0.7, evidence, "#E2F0D9" if i < 4 else "#FCE4D6", size=9.5)
        if i < len(rows) - 1:
            arrow(ax, (5.0, y), (5.0, y - 0.65))
        y -= 1.15
    finish(fig, ax, "04-测试分层.png")


def main() -> None:
    setup()
    architecture()
    state_machine()
    perception()
    test_layers()
    print(f"配图已生成到：{ASSETS}")


if __name__ == "__main__":
    main()
