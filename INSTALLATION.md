# FluxWeave 安装指南

## 使用 Pixi 管理环境（推荐）

### 1. 安装 Pixi

如果还没有安装 pixi，请先安装：

```bash
# Linux/macOS
curl -fsSL https://pixi.sh/install.sh | bash

# 或使用 conda
conda install -c conda-forge pixi
```

### 2. 安装项目依赖

```bash
cd /home/publictest/zmai/robot_hardware/FluxWeave

# 安装核心依赖（默认环境）
pixi install

# 或安装开发环境（包含开发工具）
pixi install -e dev

# 或安装完整环境（包含 Isaac Lab 支持）
pixi install -e full
```

### 3. 运行应用

```bash
# 启动主应用（FluxWeave Workbench）
pixi run start

# 或启动单个组件
pixi run stl-sourcer    # STL 预处理器
pixi run parts-editor   # 零件编辑器
pixi run assembler      # URDF 装配器
pixi run sim-config     # 仿真配置器
```

### 4. 开发任务

```bash
# 代码格式化
pixi run format

# 代码检查
pixi run lint

# 运行测试（如果有）
pixi run test
```

### 5. 进入 Pixi Shell

如果需要在环境中运行其他命令：

```bash
pixi shell

# 现在你可以直接运行 Python 或其他命令
python FluxhWeave_Workbench.py
```

---

## 使用传统方式（pip/conda）

### 使用 pip

```bash
# 创建虚拟环境
python -m venv venv
source venv/bin/activate  # Linux/macOS
# 或 Windows: venv\Scripts\activate

# 安装依赖
pip install -r requirements.txt

# 运行应用
python FluxhWeave_Workbench.py
```

### 使用 conda

```bash
# 创建 conda 环境
conda create -n fluxweave python=3.10
conda activate fluxweave

# 安装依赖
conda install -c conda-forge pyside6 vtk numpy
pip install NodeGraphQt Qt.py

# 运行应用
python FluxhWeave_Workbench.py
```

---

## 依赖包说明

### 核心依赖（必需）

| 包 | 版本 | 用途 |
|---|---|---|
| **PySide6** | >=6.5.0 | Qt6 Python 绑定，GUI 框架 |
| **Qt.py** | >=2.4.0 | Qt 抽象层，兼容不同 Qt 版本 |
| **vtk** | >=9.2.0 | 3D 可视化工具包，STL 渲染 |
| **numpy** | >=1.24.0 | 数值计算，矩阵运算 |
| **NodeGraphQt** | >=0.6.0 | 节点图/可视化编程框架 |

### 可选依赖

| 包 | 用途 |
|---|---|
| **torch** | PyTorch，用于 Isaac Lab 集成 |
| **isaac-lab** | NVIDIA Isaac Lab，USD 导出和仿真 |

### 标准库（无需安装）

- sys, os, signal, math, traceback
- pathlib, tempfile, atexit
- xml.etree.ElementTree
- json, base64, datetime, io, zipfile
- typing, functools, dataclasses

---

## 环境选择指南

### `default` 环境（推荐初学者）
- 包含所有核心功能
- STL 预处理、元数据编辑、URDF 装配
- **不包含** Isaac Lab 集成
- 适合：快速上手、URDF 建模

```bash
pixi install
pixi run start
```

### `dev` 环境（推荐开发者）
- 包含核心功能 + 开发工具
- 代码格式化、linting、测试工具
- 适合：贡献代码、定制开发

```bash
pixi install -e dev
pixi run format
pixi run lint
```

### `full` 环境（推荐高级用户）
- 包含所有功能
- Isaac Lab 集成、USD 导出、仿真配置
- 需要较大磁盘空间（PyTorch + CUDA）
- 适合：完整工作流、Isaac Sim 集成

```bash
pixi install -e full
pixi run start
```

---

## 常见问题

### 1. VTK 导入错误

如果遇到 `ImportError: cannot import name 'QVTKRenderWindowInteractor'`：

```bash
# 确保 vtk 版本正确
pixi list | grep vtk

# 如果版本不匹配，更新
pixi update vtk
```

### 2. NodeGraphQt 找不到

```bash
# NodeGraphQt 可能需要通过 pip 安装
pixi run pip install NodeGraphQt
```

### 3. Isaac Lab 集成问题

Isaac Lab 需要单独安装，请参考官方文档：
https://docs.omniverse.nvidia.com/isaacsim/latest/index.html

```bash
# 激活 full 环境后
pixi shell -e full

# 然后按照 Isaac Lab 官方指南安装
```

### 4. 检查环境是否正确

```bash
# 列出已安装的包
pixi list

# 列出所有环境
pixi info

# 检查 Python 版本
pixi run python --version
```

---

## 性能优化建议

### 1. 使用 mamba solver（更快）

Pixi 默认使用 conda solver，可以切换到更快的 mamba solver：

```bash
# 在 pixi.toml 中添加：
# [tool.pixi]
# solver = "mamba"
```

### 2. 使用本地镜像（中国用户）

在 `~/.condarc` 或 `pixi.toml` 中配置镜像：

```toml
[project]
channels = ["https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge/"]
```

### 3. 缓存清理

```bash
# 清理 pixi 缓存
pixi clean cache

# 清理未使用的包
pixi clean
```

---

## 卸载

```bash
# 删除 pixi 环境
pixi clean

# 删除 .pixi 目录
rm -rf .pixi

# 完全卸载 pixi
# Linux/macOS:
rm -rf ~/.pixi
```

---

## 更多信息

- **Pixi 文档**: https://pixi.sh/
- **FluxWeave GitHub**: https://github.com/DataFlux-Robot/FluxWeave
- **问题反馈**: 请在 GitHub Issues 中报告
