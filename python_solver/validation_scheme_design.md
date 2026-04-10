# 磁驱动导丝机器人 (mCR) 虚实双重验证方案设计

此文档详细记录了为对比 3-Magnets 与 1-Magnet 导丝性能而设计的基于顶刊文献的验证方案。该方案结合了宏观真实实验和微观仿真，从力学本质、运动学灵活性及临床安全性等维度证实多段磁化结构的优越性。

## 第一阶段：宏观运动学验证 (实体实验提取)

在真实的实体水槽实验中进行性能评估，为论文的工程可行性提供事实依据。
- **验证指标**：轨迹跟踪误差 (RMSE) 和 目标到达成功率。
- **目的**：打牢基础，证实物理样机实际可用且运动学表现符合预期。

## 第二阶段：微观力学与极限环境建模 (利用 SOFA 仿真提取专属指标)

通过 SOFA 仿真域，提取三个被顶会/顶刊验证过的高阶参数，深入验证 3-Magnets (交替极性) 在深水区/受限环境下碾压 1-Magnet 的底层科学依据。

### 指标一：沿躯干分布的曲率波峰与内部弹性能 (Internal Strain Energy & Local Curvature)
- **底层原理**：最小能量原理 (Energy Minimum Principle)。3-Magnets 在外场下形成局部能量陷阱，主动产生异构弯曲 (Active Shaping)。
- **测量手段**：基于 Cosserat Rod 原理 (SOFA Beam 模型)，输出并绘制导丝全段每一节点的弯曲/扭转应变能。
- **对比表现**：
  - **1-Magnet**：由于是被管壁“硬掰弯”，全剧弹性能会呈现庞大的积聚压力。
  - **3-Magnets**：主动折叠使得弹性能图谱中精准出现3个极低的能量波峰，其余部分保持松弛状态。
- **对标文献出处**："Energy-Based Kinematic Analysis on Magnetic Soft Continuum Robot With Asymmetric Magnetization" (*IEEE RA-L*)

### 指标二：受限三维工作空间盲区计算 (Expansion of 3D Reachable Workspace)
- **底层原理**：高维工作空间和灵活性 (Dexterity)。相反极性能极大拓展 3D 空间包络。
- **测量手段**：利用 SOFA 在 3D 巨型空腔（如大动脉瘤模型）中进行蒙特卡洛随机采样。
  - **环境约束**：采用**悬臂梁定长测试 (Fixed-base Cantilever)**，导丝伸出固定长度 **110mm**（确保 90mm 的 3-Magnets 活性段加上 20mm 过渡段完全置于空腔内），锁死基座推移自由度。
  - **磁场降维采样**：外部永磁体 (EPM) 约束在距靶点安全距离 $R$ 的**半球面包络面**上运动，剥离自旋无效自由度，使用准蒙特卡洛序列覆盖 $Pitch$ 与 $Yaw$，实现约 2000-5000 次高效位姿遍历。
- **专家补充记录（姿态与散点流提取）**：每次稳态后，获取导丝 Tip 的刚体 `[x,y,z, qx,qy,qz,qw]`。不仅记录 3D 散点云计算凸包体积 (Convex Hull Volume)，同时将四元数转化为圆锥角，展示尖端在该点位的纯磁控多角度抵近能力。
- **对比表现**：
  - **1-Magnet**：受限单一控制，仅能画出一条窄曲面，存在大量死角和盲区。
  - **3-Magnets**：解耦的高自由度能画出巨大饱满的球状体积分布。
- **对标文献出处**："Workspace Expansion of Magnetic Soft Continuum Robot Using Movable Opposite Magnet" (*IEEE RA-L*)

### 指标三：时空组织碰撞应力场 / 热力图 (Spatiotemporal Tissue Contact Force / Stress Field)
- **底层原理**：与组织的接触力分布直接决定手术安全性。
- **测量手段**：记录导丝过各种急弯时网格节点的 SOFA 碰撞惩罚力 (Collision Penalty Forces)，渲染成时空组织接触力的热力图 (Contact Force Heatmap)。
- **对比表现**：
  - **1-Magnet**：缺乏顺应性，管壁大面积呈现表示危险应力的“红色块”。
  - **3-Magnets**：主动顺应中心线，管壁多为“蓝色安全低压色块”，证明能显著降低血管损伤风险。
- **对标文献出处**："Model-aided 3D shape and force estimation of continuum robots based on Cosserat rod theory and using a magnetic localization system" (*Intell. Serv. Robotics*); "A Simulation Framework for Magnetic Continuum Robots" (*IEEE RA-L*)

---

## 论文撰写的方法论排版总结方案

按照上述思路，论文的实验板块大纲可如下规划：
- **Experiment 1 (Physical)**: Trajectory and success rate tracking (证明这东西造出来真的能动，真实可用)。
- **Experiment 2 (Simulation in SOFA)**:
  - **2a**: Energy-based Kinematic extraction (引用 IEEE RA-L 弹性能论文：揭示形变机理与能量优势)。
  - **2b**: 3D Workspace Volume & Orientation (引用 IEEE RA-L Opposite Magnet 论文：证明灵活度与能力上限)。
  - **2c**: Tissue Contact Stress field (引用 A Simulation Framework for MCR / Intell. Serv. Robotics 论文：证明安全性与临床价值)。
