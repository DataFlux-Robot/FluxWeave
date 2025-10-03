# FluxhWeave Workbench ｜ 动态 URDF 生成工具

FluxhWeave Workbench 将 STL 预处理、部件元数据编辑、URDF 装配与 USD 模板生成串联在一个窗口中，为模块化机器人项目提供端到端的建模体验。

FluxhWeave Workbench unifies STL preprocessing, part metadata authoring, URDF assembly, and USD templating in a single window, enabling an end-to-end workflow for modular robot projects.

## 项目初衷 | Motivation

在机器人开发与仿真中，快速生成 **URDF** 模型是一项常见需求。但官方 **SolidWorks to URDF Exporter** 工具在实际项目里经常显得笨重且缺乏灵活性：操作步骤繁琐、难以服务模块化拼装场景。FluxhWeave 借鉴并扩展了 URDF_kitchen 的理念，让开发者可以按照真实的部件拼插过程动态生成 URDF，同时保持对 STL 预处理、惯量录入与节点装配的细粒度控制。

Rapid URDF authoring is a recurring bottleneck across robotics projects. The official SolidWorks to URDF Exporter often proves cumbersome and inflexible for modular builds. FluxhWeave extends the URDF_kitchen approach so that engineers can mirror the physical assembly process, cleanly preprocess meshes, author inertial metadata, and stitch everything into a URDF with minimal friction.

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
