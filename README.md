# 🧲 磁驱动连续体机器人 (MCR) 高级物理仿真系统

基于 **SOFA (Simulation Open Framework Architecture)** 开发的磁控导丝机器人仿真平台。核心目标是**模拟并验证多磁铁阵列（3-Magnets 分布式磁场响应结构）相对于传统 1-Magnet 设计的卓越导航与异形弯曲能力**，为学术论文提供定量实验数据支撑。

---

## 🔬 研究验证框架

本项目实现了一套**三级递进式验证体系**，用于定量对比 1-Magnet 与 3-Magnets 导丝设计：

### 1. 变刚度模型验证 (Metric 1: Model Validation) ✅ 已校准
基于 Python 编写的 2D 变刚度求解器，通过真实相机照片的 RMSE 最小化完成了杨氏模量校准。
- 目录：`python_solver/`
- 关键脚本：`mcr_python_solver.py`, `visualize_shape.py`

### 2. 工作空间体积对比 (Metric 2: Workspace Volume) ✅ 已完成
基于蒙特卡洛采样的 3D 可达体积（Convex Hull Volume）与尖端姿态覆盖率（Orientation Dexterity）对比。
- 目录：`sofa_standard/`
- 关键脚本：`run_workspace_MC.py`, `plot_workspace.py`

### 3. 血管穿越安全性与通过率 (Metric 3: Navigation Safety) ✅ 开发中
在真实主动脉弓模型与合成 S 型急弯管中，对比穿越成功率、接触力、任务完成时间和跟踪误差。
- 目录：`mcr_prb/`
- 关键脚本：`run_navigation_test.py`, `run_prb_mcr_sofa.py`

---

## 📂 项目结构

```
guidewire_simulation/
├── python_solver/               # [核心] 变刚度求解器与精度验证
│   ├── mcr_python_solver.py     # 2D 变刚度数学模型
│   └── visualize_shape.py       # 真机照片对比校准工具
├── mcr_prb/                     # [核心] PRB SOFA 导航仿真
│   ├── run_prb_mcr_sofa.py      # 3-Magnets 交互式仿真
│   └── run_navigation_test.py   # 指标三自动化导航测试
├── sofa_standard/               # [通用] 蒙特卡洛采样与分析
│   ├── run_workspace_MC.py      # 工作空间自动化采样
│   └── plot_workspace.py        # 凸包可视化分析
├── logs_and_data/               # [归档] 实验结果与档案库
│   ├── csv_results/             # 历史仿真输出 CSV
│   ├── plots/                   # 自动生成的图表
│   └── calibration/             # 原始实验照片与中心线坐标
└── mCR_simulator-master/        # 底层仿真引擎支持库
```

---

## ✨ 核心技术特性

*   **真实永磁体空间衰减磁场模型**：基于磁偶极子 $1/r^3$ 空间距离衰减公式。
*   **横向手术台基准架构**：旋转 90° 场景以符合卧床手术条件并抵消重力干扰。
*   **SOFA v25.12+ 现代碰撞管线**：全面使用现代 Collision 插件，杜绝段错误。
*   **自适应变刚度接口**：支持从 Python 验证的 $E(s)$ 公式直接映射到 SOFA 梁单元。

---

## 🚀 快速启动

### 方式 A：运行 Python 求解器验证 (无需 SOFA)
```bash
cd python_solver
python run_python_workspace.py
```

### 方式 B：执行 SOFA 交互式导航实验
```bash
cd mcr_prb
runSofa run_prb_mcr_sofa.py
```

### 方式 C：运行蒙特卡洛大规模采样
```bash
cd sofa_standard
MODE=3 SAMPLES=2000 runSofa run_workspace_MC.py
```

---

## 📖 参考文献
- Lee et al., "Energy-Based Kinematic Analysis on AMSCR", IEEE RA-L, 2026
- Luo et al., "RL-MG: RL Framework for Magnetic Guidewire Path Following", RCAR, 2025
- Kim et al., "Ferromagnetic Soft Continuum Robots", Science Robotics, 2019
- Dreyfus et al., "A Simulation Framework for Magnetic Continuum Robots", IEEE RA-L, 2022
