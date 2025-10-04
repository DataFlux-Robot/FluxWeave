# FluxhWeave Workbench ｜ 动态 URDF 生成工具

FluxhWeave Workbench 将 STL 预处理、部件元数据编辑、URDF 装配与 USD 模板生成串联在一个窗口中，为模块化机器人项目提供端到端的建模体验。

FluxhWeave Workbench unifies STL preprocessing, part metadata authoring, URDF assembly, and USD templating in a single window, enabling an end-to-end workflow for modular robot projects.

## 项目初衷 | Motivation

在机器人开发与仿真中，快速生成 **URDF** 模型是一项常见需求。但官方 **SolidWorks to URDF Exporter** 工具在实际项目里经常显得笨重且缺乏灵活性：操作步骤繁琐、难以服务模块化拼装场景。FluxhWeave 借鉴并扩展了 URDF_kitchen 的理念，让开发者可以按照真实的部件拼插过程动态生成 URDF，同时保持对 STL 预处理、惯量录入与节点装配的细粒度控制。

Rapid URDF authoring is a recurring bottleneck across robotics projects. The official SolidWorks to URDF Exporter often proves cumbersome and inflexible for modular builds. FluxhWeave extends the URDF_kitchen approach so that engineers can mirror the physical assembly process, cleanly preprocess meshes, author inertial metadata, and stitch everything into a URDF with minimal friction.

---

在机器人开发的世界里，我们习惯了传统的路径：在 SolidWorks 中精心设计，通过一个插件导出 URDF，然后在 ROS/Gazebo 的世界里进行配置、调试和仿真。这条路我们走了很久，但它是否是最高效、最灵活的路径？

今天，我们将 **FluxWeave** 与传统的 **SolidWorks URDF Exporter** 进行对比，不是为了评判一个工具的好坏，而是为了探讨两种截然不同的开发哲学。我们相信，FluxWeave 代表着未来。



## 核心差异：一个“导出工具” vs 一个“集成平台”

| 特性维度 | SolidWorks URDF Exporter | FluxWeave |
| :--- | :--- | :--- |
| **核心理念** | **一个导出器**。它的唯一使命是将 SolidWorks 的装配体信息转换为 URDF 文件。它是一个工作流中的单一环节。 | **一个一体化平台**。它集建模、仿真、可视化、控制于一体，是一个完整的开发生态。 |
| **工作流程** | **线性且割裂**：SolidWorks → 导出URDF → 手动编辑 → ROS/Gazebo → 代码开发。每一步都需要切换工具和环境。 | **集成且循环**：在单一平台内完成模型构建、参数调整、仿真测试和代码部署。快速迭代，无缝衔接。 |
| **起点与灵活性** | **强依赖 SolidWorks**。你的整个机器人模型必须在一个昂贵的、重量级的 CAD 软件中完成。 | **始于通用格式 (STL/STEP)**。你可以使用任何 CAD 软件（FreeCAD, Fusion 360, SolidWorks 等）建模，甚至直接导入现有模型。**不绑定任何工具**。 |
| **模型维护** | **困难且易出错**。机械设计的任何微小改动，都需要重新导出 URDF，并手动处理可能出现的坐标、碰撞或视觉模型错乱问题。 | **动态且直观**。在 FluxWeave 中直接调整关节参数、连杆属性，所有改动实时反映在仿真和可视化中。所见即所得。 |
| **仿真与验证** | **无内置仿真**。导出的 URDF 需要手动配置 Gazebo 或 MoveIt，这是一个复杂且耗时过程，充满了配置文件的调试。 | **开箱即用的仿真**。内置物理引擎，一键启动仿真环境。直接在 3D 视图中测试运动学、动力学和路径规划。 |
| **可视化与调试** | **依赖外部工具**。需要使用 RViz、Gazebo 等独立工具进行可视化，配置繁琐，数据流不直观。 | **原生集成可视化**。强大的 3D 可视化界面与平台深度融合。你可以直接在模型上查看 TF 树、传感器数据、关节状态，调试从未如此直观。 |
| **技术栈与扩展性** | **传统桌面应用**。基于 C++ 和 Qt 插件，扩展和二次开发门槛高，与现代 Web 技术栈脱节。 | **现代 Web 技术栈**。基于 Web 技术，天然具备跨平台、易分享、易扩展的特性。可以轻松集成 Web UI、AI 模型和云服务。 |
| **学习曲线** | **陡峭**。需要同时精通 SolidWorks、URDF 语法、ROS/Gazebo 配置等多个领域的知识。 | **平缓**。通过图形化界面和引导式工作流，大大降低了入门门槛，让你专注于机器人逻辑本身。 |

---

## 深入剖析：FluxWeave 的颠覆性优势

### 1. 从“文件”到“项目”的思维转变

SolidWorks URDF Exporter 给你的是一个静态的 `robot.urdf` **文件**。你得到的是终点，过程却充满了痛苦。

FluxWeave 给你的是一个活的、可交互的机器人**项目**。你得到的是一个起点，一个可以不断演进、测试和优化的动态系统。

### 2. 打破工具链的枷锁

传统工作流是“工具链的囚徒”。你被锁定在 SolidWorks → ROS 的特定路径上，任何一个环节出错，整个流程都会中断。

FluxWeave 是一个“自由的平台”。你用任何顺手的 CAD 软件生成 STL/STEP 文件，然后剩下的所有事情，FluxWeave 为你一站式搞定。这种敏捷性是传统工作流无法比拟的。

### 3. 可视化驱动的开发

想象一下：在 RViz 中发现一个关节的 TF 坐标不对，你需要回头去猜测是 SolidWorks 中的装配问题，还是 URDF 中的 `<origin>` 标签写错了。

在 FluxWeave 中，你直接在 3D 视图中拖拽、旋转、调整关节，系统会实时更新参数。**调试不再是阅读代码，而是与模型直接对话**。


---

## 特别感谢 | Special Thanks

本项目深受 [URDF_kitchen](https://github.com/Ninagawa123/URDF_kitchen) 启发 🙏，在其基础上扩展了以下能力：
- **STL 处理增强**：支持坐标旋转、镜像、缩放、原点平移与批量清理。
- **装配标记更智能**：提供 marker point 创建校验、方向矢量可视化以及 STL 内嵌元数据。
- **惯量输入更易用**：质量、惯量矩阵与摩擦参数带有单位提醒与正定性校验。

This work builds on [URDF_kitchen](https://github.com/Ninagawa123/URDF_kitchen) 🙏 with the following upgrades:
- **Richer STL tooling** with rotations, mirrors, scaling, origin alignment, and cleanup.
- **Smarter assembly markers** for authoring and embedding marker points directly inside STL assets.
- **Simplified inertia authoring** with unit hints, validation, and friction presets.

## 核心特性 | Highlights

- **一站式流程 / Unified pipeline**：五个分页对应 STL 预处理 → 部件元数据 → URDF 装配 → USD 导出 → 仿真配置。
- **VTK 可视化 / VTK visualization**：实时查看坐标调整效果，支持相机与线框快捷操作。
- **元数据嵌入 / Embedded metadata**：将连接点、惯性属性写入 STL 或外部 XML，装配时自动解析。
- **URDF 生成 / URDF authoring**：拖拽连接器构建机械链，实时调整关节类型与约束。
- **USD 模板导出 / USD templating**：一键调用 IsaacLab `convert_urdf.py`，从 URDF 自动产出 USD 模型骨架。
- **仿真配置 / Simulation presets**：集中管理关节默认角度、作动器参数与 Isaac Sim 运行所需 JSON。

## 快速开始 | Quick Start

1. **安装依赖 / Install dependencies**：推荐使用 Conda 或 mamba，确保安装 `PySide6`, `vtk`, `numpy` 及 STL 处理所需库。
2. **启动工作台 / Launch the workbench**：在项目根目录执行 `python FluxhWeave_Workbench.py`，应用会自动载入深色主题。
3. **遵循分页顺序 / Follow the tabs**：建议按照 ① → ⑤ 顺序完成工作，以便路径、元数据与配置自动传递。

## 使用流程 | Usage Guide

### Step 1 · STL 预处理 / STL preprocessing

1. **批量导入 STL**：读取 SolidWorks 导出的单个或多个 STL，自动记录最近使用目录。
2. **坐标规范化**：通过旋转、镜像、缩放、原点平移等操作统一坐标系，VTK 视图实时预览。
3. **清理与输出**：执行网格修复、法线校正，可直接导出处理后的 STL 进入下一阶段。

Load one or more SolidWorks STL files, normalize axes via rotation/mirroring/scaling/origin shifts, preview changes with VTK, then export cleaned meshes for downstream steps.

### Step 2 · 部件元数据 / Part metadata

1. **载入预处理结果**：读取 Step 1 输出，自动解析尺寸信息。
2. **标记连接点**：定位、命名并设置各 marker point 的坐标与方向，支持与原始基准面对齐。
3. **录入物理属性**：输入质量、惯量矩阵、摩擦参数等，并通过内建校验避免常见错误。
4. **嵌入与导出**：可将元数据回写至 STL 头或生成 XML，中英双语注释便于协作。

The part editor imports the refined STL, helps author marker points, inertia tensors, and metadata, and embeds the result into the STL header or exports an XML companion file.

### Step 3 · URDF 装配 / URDF assembly

1. **导入部件**：自动解析嵌入元数据的 STL，为每个部件生成节点图。
2. **关联连接点**：拖拽连接器建立父子关系，面板即时展示关节类型、坐标对齐与命名建议。
3. **调节关节参数**：实时修改角度、上下限、力矩、速度，连续/滑动关节会自动切换单位。
4. **生成 URDF**：一键输出 URDF 文本，可复制、保存并批量导出 STL 附件。

Import metadata-enriched parts, connect marker points via drag-and-drop, review live alignment diagnostics, tune joint limits/effort, then export the URDF alongside referenced meshes.

### Step 4 · USD 模板导出 / USD template export

FluxhWeave Workbench 在拓展功能分页中集成了 IsaacLab 的 `convert_urdf.py`。只需配置环境与脚本路径即可把 URDF 转换为 Isaac Sim 友好的 USD 模板：

1. **定位 IsaacLab**：选择安装了 IsaacLab 的 Conda 环境名称，并指定 `convert_urdf.py` 所在路径；程序会向上查找 `isaaclab.sh` 以确认根目录。
2. **选择输入输出**：指定待转换的 URDF 与目标 USD 文件路径，缺失目录会提示立即创建。
3. **启动转换**：点击“执行导出”后，Workbench 会自动执行 `isaaclab.sh -p convert_urdf.py <urdf> <usd>`，在控制台区域流式显示所有日志。
4. **监控状态**：状态栏会实时更新“任务开始 / 正在取消 / 完成”等信息，若需要可随时点击“取消”。

This tab wraps IsaacLab’s converter: activate the proper Conda environment, validate the script under the `isaaclab.sh` root, choose URDF and USD paths, then launch the conversion. All output is streamed into the Workbench, and successful runs produce a USD template ready for Isaac Sim/IsaacLab pipelines.

### Step 5 · 仿真配置 / Simulation presets

1. **关节默认角度**：自动加载或手动导入 `joint_defaults.json`，并与装配阶段保持同步。
2. **作动器参数库**：支持 ImplicitPD、IdealPD、DCMotor 等多种 Isaac Sim 作动器类型，提供刚度、阻尼、摩擦、延迟等字段的上限校验。
3. **场景 JSON 导出**：集中管理仿真所需的额外配置字段，可一键导出供 Isaac Sim 或自定义框架直接读取。

The simulation tab curates joint defaults and actuator profiles, letting teams export ready-to-use JSON presets that pair with the generated URDF/USD assets.

## USD 模板自动化流程 | USD automation flow

- **环境准备 / Environment setup**：确保 IsaacLab 已安装并可通过 `isaaclab.sh` 启动；将 `convert_urdf.py` 与 Workbench 放在同一工作站即可。
- **命令拼装 / Command construction**：Workbench 自动生成 `eval "$(conda shell.bash hook)" && conda activate <env> && cd <isaaclab_root> && ./isaaclab.sh -p convert_urdf.py <urdf> <usd>`。
- **错误防护 / Error handling**：若脚本不在 IsaacLab 根目录下、文件缺失或进程崩溃，界面会弹出对话框并记录日志，避免静默失败。
- **模板产出 / Template delivery**：输出的 USD 保留 URDF 的关节层级与惯量结构，可作为 Isaac Sim/IsaacLab 项目骨架继续填充材质、控制器与任务逻辑。

## 项目路线图 | Future roadmap

- 增加直接输出 USD 模型 / Add direct USD export without IsaacLab dependency.
- 引入更多预定义几何与作动器 / Ship richer built-in part and actuator libraries.
- 同步加载力学模型 / Load analytic or data-driven dynamics alongside URDF.
- 导入高质量视觉素材 / Integrate material and render-ready assets.
- 一键生成 IsaacLab 任务模板 / Provide full IsaacLab scene templates out of the box.

## 开源协议 | License

项目基于 **Apache 2.0 License** 开源，欢迎自由使用、修改与分发。

Distributed under the **Apache 2.0 License** — feel free to use, modify, and share.

## 贡献指南 | Contributions

欢迎通过 Issue 或 PR 提出建议、报告问题、提交功能改进，也欢迎分享你基于 FluxhWeave 构建的机器人案例。

Issues and PRs are warmly welcomed—share feature ideas, bug reports, or robot builds powered by FluxhWeave!
