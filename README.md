# 🧲 磁驱动连续体机器人 (MCR) 高级物理仿真系统

基于 **SOFA (Simulation Open Framework Architecture)** 开发的磁控导丝机器人仿真平台。核心目标是**模拟并验证多磁铁阵列（3-Magnets 分布式磁场响应结构）相对于传统 1-Magnet 设计的卓越导航与异形弯曲能力**，为学术论文提供定量实验数据支撑。

---

## 🔬 研究验证框架

本项目实现了一套**三级递进式验证体系**，用于定量对比 1-Magnet 与 3-Magnets 导丝设计：

### 指标一：能量分析 (Energy-Based Analysis)
基于 Cosserat Rod 理论的弹性能与磁能分布对比，参考 [Lee et al., IEEE RA-L 2026] 的能量最小化框架。

### 指标二：工作空间体积对比 (Workspace Volume) ✅ 已完成
基于蒙特卡洛采样的 3D 可达体积（Convex Hull Volume）与尖端姿态覆盖率（Orientation Dexterity）对比。
- `run_workspace_MC.py` — 自动化蒙特卡洛采样脚本
- `plot_workspace.py` — 3D 凸包工作空间可视化
- `plot_dexterity.py` — 单位球面姿态覆盖可视化

**运行方式：**
```bash
# 1-Magnet 工作空间采样 (100次)
MODE=1 SAMPLES=100 runSofa run_workspace_MC.py

# 3-Magnets 交变极性 (+1,-1,+1) 工作空间采样
MODE=3 SAMPLES=100 runSofa run_workspace_MC.py

# 3-Magnets 同向极性 (+1,+1,+1) 工作空间采样
MODE=4 SAMPLES=100 runSofa run_workspace_MC.py

# 生成对比图
python plot_workspace.py
python plot_dexterity.py
```

### 指标三：血管穿越安全性与通过率 (Navigation Safety) 🚧 开发中
在真实主动脉弓模型与合成 S 型急弯管中，对比两种设计的穿越成功率、血管壁接触力、任务完成时间和路径跟踪误差。

---

## ✨ 核心技术特性

### 真实永磁体空间衰减磁场模型
基于磁偶极子 $1/r^3$ 空间距离衰减与非线性梯度方向变化：
$$B(r) = \frac{\mu_0}{4\pi} \cdot \frac{3(m_{em} \cdot \hat{r})\hat{r} - m_{em}}{||r||^3}$$

### 横向手术台基准架构
利用 `scipy.spatial.transform` 将整个医疗场景旋转 90°，符合卧床导管手术条件。

### SOFA v25.12+ 兼容性
全面升级至现代碰撞管线（BroadPhase、BVHNarrowPhase、DefaultContactManager），消除了旧版段错误。

---

## 📂 项目结构

```
guidewire_simulation/
├── run_prb_mcr_sofa.py          # 3-Magnets 交互式仿真入口 (带血管)
├── run_prb_mcr_sofa_1mag.py     # 1-Magnet 交互式仿真入口 (带血管)
├── run_workspace_MC.py          # 蒙特卡洛工作空间自动采样
├── plot_workspace.py            # 3D 凸包工作空间对比图
├── plot_dexterity.py            # 姿态覆盖率对比图
├── validation_scheme_design.md  # 验证方案设计文档
├── mCR_simulator-master/        # 底层仿真引擎 (子模块)
│   └── python/mcr_sim_prb/      # PRB 专用模块
│       ├── mcr_instrument.py    # 导丝机械结构定义
│       ├── mcr_external_magnet.py # EPM 偶极子模型
│       ├── mcr_mag_controller.py  # 磁力矩实时计算
│       └── mcr_controller_sofa.py # 键盘 6-DOF 控制器
└── workspace_results_*.csv      # 蒙特卡洛采样数据
```

---

## 🎮 交互式操控 (6-DOF)

| 功能 | 按键 | 说明 |
|------|------|------|
| EPM Y 轴平移 | `W` / `S` | 控制外部磁铁高度 |
| EPM X 轴平移 | `A` / `D` | 水平追踪导丝前端 |
| EPM Z 轴平移 | `Q` / `E` | 深度方向探伤 |
| EPM 偏航旋转 | `J` / `L` | Yaw 自转 |
| EPM 俯仰旋转 | `I` / `K` | Pitch 自转 |
| 导丝推进/后撤 | `↑` / `↓` | 沿管腔轴心送丝 |

---

## 🚀 快速启动

```bash
# 1. 激活 SOFA 环境
conda activate sofanew

# 2. 启动 3-Magnets 交互式仿真
runSofa run_prb_mcr_sofa.py

# 3. 或运行自动化工作空间采样
MODE=3 SAMPLES=2000 runSofa run_workspace_MC.py
```

---

## 📖 参考文献
- Lee et al., "Energy-Based Kinematic Analysis on AMSCR", IEEE RA-L, 2026
- Luo et al., "RL-MG: RL Framework for Magnetic Guidewire Path Following", RCAR, 2025
- Kim et al., "Ferromagnetic Soft Continuum Robots", Science Robotics, 2019
- Dreyfus et al., "A Simulation Framework for Magnetic Continuum Robots", IEEE RA-L, 2022
