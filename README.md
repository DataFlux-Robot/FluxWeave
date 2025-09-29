# FluxhWeave (动态 URDF 生成工具 / Dynamic URDF Generator)

---

## 项目初衷 | Motivation
在机器人开发和仿真过程中，常常需要快速生成 **URDF** 模型。  
然而，SolidWorks 自带的 **SolidWorks to URDF Exporter** 工具在实际使用中存在较多不便：  
- 操作复杂  
- 灵活性不足  
- 难以适应模块化拼装需求  

因此，我们结合项目需求，开发了本工具，旨在实现：  
- 根据模块拼插过程 **动态生成 URDF 文件**  
- 简化机器人硬件配置的建模过程  
- 为后续仿真与部署提供自动化支持  

In robotics development and simulation, generating **URDF** models quickly is a frequent requirement.  
However, the official **SolidWorks to URDF Exporter** is often inconvenient to use:  
- Complicated workflow  
- Limited flexibility  
- Not suitable for modular robot assembly  

To address this, we developed this tool, with the goals of:  
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

FluxhWeave_Workbench.py 集成 STL 预处理、部件元数据管理与 URDF 装配，全流程围绕 SolidWorks 导出的 STL 实现快速、模块化的机器人模型构建。
FluxhWeave_Workbench.py unifies STL preprocessing, part metadata editing, and URDF assembly, delivering a fast modular robot modeling pipeline for SolidWorks exports.
### Step 1 · STL 预处理 / STL Preprocessing
#### 1.导入 STL & 批量管理 /Import STL & batch-ready: 
选择单个或多个 SolidWorks 导出的 STL；工具自动记录最近使用路径，便于批量整理。
Load one or more SolidWorks STL files; recent directories are remembered for streamlined batch work.

#### 2.坐标调整与归一化 / Coordinate alignment: 
通过轴向旋转、镜像、缩放、原点平移等操作，将模型统一到规范坐标系；实时 VTK 预览辅助确认。
Rotate, mirror, scale, and translate the origin to normalize geometry; a live VTK preview validates each tweak.

#### 3.基础清理与导出 /Cleanup & export: 
自动执行 STL 清理、法线修正，并可将处理结果导出供后续步骤直接使用。
Automated STL cleanup and normal fixes ensure clean meshes that can be exported for the next stage.

### Step 2 · 部件元数据 / Part Metadata
加载预处理 STL：一步导入 Step 1 的结果，界面自动读取模型尺寸并准备属性面板。
Load preprocessed STL: Bring in the refined meshes; dimensions are parsed and panels are primed for metadata entry.
定义连接点 (Marker Points)：为每个连接点定位、命名、设置轴向；支持三维坐标微调、方向矢量可视化，以及与 SolidWorks 基准对齐。
Define marker points: Position, name, and orient every docking point; fine-tune coordinates, visualize joint axes, and align with original CAD datums.
物理属性录入：输入质量、惯量矩阵、摩擦等参数，提供模板与单位提示；自动校验对角元和正定性，防止常见错误。
Physical properties: Specify mass, inertia tensor, friction, etc., with guided templates and unit hints; built-in validation ensures positive-definite tensors.
元数据嵌入 & 导出：将配置写入 STL 中继或导出独立 XML，供装配阶段自动解析；支持版本号与作者信息追踪。
Embed & export metadata: Save metadata back into STL headers or external XML files, enabling automatic discovery in assembly; versioning and authorship tags are supported

### Step 3 · URDF 装配 / URDF Assembly
导入部件：读取嵌入元数据的 STL，自动生成节点图；缺失数据会提示回到 Step 2 补充。
Import parts: Load metadata-augmented STL files to auto-create graph nodes; missing data triggers guided reminders to revisit Step 2.
建立连接：拖拽连接器节点，绑定父子连接点；面板即时显示链路、关节类型与坐标校验信息。
Connect modules: Drag connector nodes between parts, binding parent/child markers; the panel shows link names, joint types, and alignment diagnostics.
关节参数调节：在 “关节控制” 中实时修改角度、上下限、力矩、速度等；连续/滑动关节单位自动切换，支持延迟刷新减轻渲染卡顿。
Tune joint parameters: Adjust angle, limits, effort, and velocity live; units adapt to joint type and debounced updates smooth the viewport.
预览与导出 URDF：一键生成 URDF 文本，支持复制、保存及批量导出 STL；同时保留项目 JSON，方便二次编辑。
Preview & export: Generate URDF with one click, copy/save the file, and export STL assets; project JSON snapshots enable later revisions.
Tips

推荐按顺序完成三个阶段，以便自动传递路径、元数据与项目状态。
Follow the three-stage order to keep paths, metadata, and state flowing automatically.
URDF 生成后可直接导入 ROS / Gazebo / Isaac Sim；未来版本将拓展 USD、IsaacLab 等输出选项。
Generated URDFs drop into ROS, Gazebo, or Isaac Sim; future releases aim for USD/IsaacLab exporters.

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



