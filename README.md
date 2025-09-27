# FluxhWeave (R³ 架构 - 动态 URDF 生成工具 / Dynamic URDF Generator for R³ Architecture)

---

## 项目初衷 | Motivation
在机器人开发和仿真过程中，常常需要快速生成 **URDF** 模型。  
然而，SolidWorks 自带的 **SolidWorks to URDF Exporter** 工具在实际使用中存在较多不便：  
- 操作复杂  
- 灵活性不足  
- 难以适应模块化拼装需求  

因此，我们结合 **R³ 架构**的项目需求，开发了本工具，旨在实现：  
- 根据模块拼插过程 **动态生成 URDF 文件**  
- 简化机器人硬件配置的建模过程  
- 为后续仿真与部署提供自动化支持  

In robotics development and simulation, generating **URDF** models quickly is a frequent requirement.  
However, the official **SolidWorks to URDF Exporter** is often inconvenient to use:  
- Complicated workflow  
- Limited flexibility  
- Not suitable for modular robot assembly  

To address this, we developed this tool within the **R³ Architecture**, with the goals of:  
- **Dynamically generating URDF files** from modular assembly  
- Simplifying robot hardware modeling  
- Supporting automated workflows for simulation and deployment  

---

## 特别感谢 | Special Thanks
本项目深受 [URDF_kitchen](https://github.com/Ninagawa123/URDF_kitchen) 项目的启发 🙏。  
在此基础上，我们进行了如下扩展与完善：  

- **STL 模型处理能力增强**：引入坐标系旋转、原点变换操作  
- **模型构建过程优化**：增加 `marker point` 的创建与校验机制  
- **惯量与质量属性输入改进**：提升模型导入过程的易用性  

This project is heavily inspired by [URDF_kitchen](https://github.com/Ninagawa123/URDF_kitchen) 🙏.  
Based on it, we made the following improvements:  

- **Enhanced STL handling**: Added coordinate rotation and origin adjustment  
- **Improved model building process**: Added `marker point` creation and validation  
- **Better inertia input**: Simplified the way users specify mass and inertia properties  

---

## 使用流程 | Usage Guide
本工具的使用流程与 `URDF_kitchen` 工具链类似，并在此基础上扩展：  

The workflow is similar to the `URDF_kitchen` toolchain, with added features:  

### 1. STL 模型预处理 / STL Preprocessing  
- 对应工具：`urdf_kitchen_StlSourcer_V2`  
- 在本项目中：  
  - 导入 STL 文件  
  - 进行 **坐标轴调整**、**原点校正**  
  - 确保零件在统一的坐标系下对齐  

- Equivalent tool: `urdf_kitchen_StlSourcer_V2`  
- In this project:  
  - Import STL files  
  - Perform **axis rotation** and **origin correction**  
  - Align all parts to a unified coordinate system  

---

### 2. 模块属性定义 / Module Property Definition  
- 对应工具：`urdf_kitchen_PartsEditor_V2`  
- 在本项目中：  
  - 定义 **连接点 (marker point)**  
  - 设置 **惯量、质量、摩擦系数** 等动力学属性  
  - 提供更直观的输入方式，简化惯量参数配置  

- Equivalent tool: `urdf_kitchen_PartsEditor_V2`  
- In this project:  
  - Define **connection points (marker points)**  
  - Assign **inertia, mass, friction** and other dynamics  
  - Provide more intuitive UI for inertia input  

---

### 3. 模块拼装与 URDF 生成 / Assembly & URDF Generation  
- 对应工具：`urdf_kitchen_Assembler_V2`  
- 在本项目中：  
  - 按照硬件拼插顺序 **组装零件**  
  - 自动生成完整的 **URDF 模型**  
  - 支持导入至 ROS / Gazebo / IsaacSim 等仿真环境  

- Equivalent tool: `urdf_kitchen_Assembler_V2`  
- In this project:  
  - Assemble parts based on physical connections  
  - Automatically generate a complete **URDF model**  
  - Compatible with ROS, Gazebo, IsaacSim and other platforms  

---

## 未来计划 / Future plans:

- 增加直接输出 USD 模型 / Add direct USD export

- 引入更多预定义的几何模型和作动器 / Add more predefined geometries & actuators

- 增加力学模型的同步载入 / Load physical models alongside URDF

- 增加高质量视觉素材的同步导入 / Import high-quality visual assets

- 增加一键生成 IsaacLab 模型 / One-click IsaacLab model generation
---

## 开源协议 | License
本项目基于 Apache 2.0 License 开源。
欢迎自由使用、修改与贡献。

This project is released under the Apache 2.0 License.
Feel free to use, modify and contribute.

---

## 欢迎提交 Issue 与 PR / Contributions via Issues and PRs are welcome!



