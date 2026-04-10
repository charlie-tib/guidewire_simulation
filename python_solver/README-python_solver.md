# 🧲 Python 离散变刚度 (DVS) 磁性导丝求解器

## 1. 核心设计理念

该 Python 求解器是一个基于**能量最小化 (Energy Minimization)** 原理的稳态物理引擎，专门设计用于验证多磁铁磁控导丝的非线性变形和 S 型折叠能力。

### 物理建模
- **离散变刚度 (DVS)**: 将 110mm 的导丝离散为 **110 个 1mm 的微小段**。每一段都可以独立定义杨氏模量 $E$。
- **静态平衡 (Static Equilibrium)**: 假设系统总是停留在总位能最低的状态：
  $$E_{total} = E_{elastic} + E_{magnetic} \to \min$$
- **弹性能 ($E_{el}$)**: 模拟导丝的弯曲抗力。
- **磁能 ($E_{mag}$)**: 模拟内外部磁场之间的偶极子相互作用 $-\vec{m} \cdot \vec{B}$。

---

## 2. 目前已实现的功能

### A. 核心求解引擎 (`mcr_python_solver.py`)
- 使用 `scipy.optimize.minimize` (L-BFGS-B 算法) 高效寻找平衡态角度。
- 支持 **3D 空间变形**（俯仰 $\theta$ 与 方位 $\phi$ 全解耦）。
- 支持任意数量、任意极性的离散内部磁铁。

### B. 自动化采样系统 (`run_python_workspace.py`)
- **蒙特卡洛采样**：随机生成 2000 个外部永磁体 (EPM) 位姿，覆盖 100-150mm 的全球面分布。
- **多模式对比**：一次性运行 `1-Mag`, `3-Mag(+-+)`, `3-Mag(+++)` 多个实验组。
- **S 型特征提取**：自动检测曲率翻转点 (Inflection Points)，量化 S 型变形概率。

### D. 曲率特征扫描 (`analyze_curvature.py`) [NEW]
- **功能**：自动寻找能够诱导导丝产生最大曲率波动的 EPM 角度。
- **核心指标**：**拐点数 (Inflection Points)**。通过计算代数曲率 $\kappa$ 的零点，量化导丝的 S 型与多波段 M 型变形能力。
- **验证结果**：
  - **3-Magnets (+-+)**：在特定磁场下表现出明显的 **S 型和高阶 M 型** 弯曲（实测可达 2-11 个拐点）。
  - **1-Magnet**：在相同磁场下仅表现为简单的 **C 型** 弯曲（0 拐点）。

![Curvature Comparison](metric1_curvature_comparison.png)

---

## 3. 关键物理参数定义

| 变量 | 代码中的位置 | 物理含义 |
|------|------------|----------|
| `E` | `segments` 列表 | 每一段的硬度（软铰链 vs 刚性管） |
| `epm_m` | `solver.solve()` | 外部磁铁磁矩（取决于 N52 属性） |
| `m_internal`| `run_sampling()` | 内部磁铁磁矩（决定力矩大小） |
| `dl` | `1.0e-3` | 空间分辨率 (1mm) |

---

## 4. 快速使用指南

### 1. 运行工作空间采样
```bash
python run_python_workspace.py
```
*输出：`python_workspace_results_*.csv`*

### 2. 可视化当前导丝形态
```bash
python visualize_shape.py
```

### 3. 生成数据对比报告
```bash
python plot_python_results.py
```

---
> **注**：该 Python 模拟器已通过物理学一致性验证。相比 SOFA，它在处理多磁铁变刚度段时的数值稳定性更高，且更容易提取导丝内部的应变能分布数据，是撰写论文中“机理分析”部分的核心工具。
