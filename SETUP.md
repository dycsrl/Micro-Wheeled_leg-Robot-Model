# 安装与复现

## 1. 直接运行已有模型

在项目根目录执行 README 中的安装命令。运行只需 `requirements.txt`，无需安装 CAD 工具或下载原始 STEP。所有资源以脚本所在项目目录定位，不依赖固定盘符。

Windows 快捷入口：`check-model.cmd`、`check-ground.cmd`、`view-model.cmd`。入口只使用本项目 `.venv`，缺少时会提示安装。

Linux 可用 `python3 -m venv .venv` 与 `.venv/bin/python`；macOS 交互查看通常需 `mjpython scripts/eval_sim.py --view`。本次未做 Linux/macOS 运行验收，不保证图形驱动兼容性。

输出目录 `outputs/` 可重新生成，未纳入 Git。`docs/evidence/` 保存本次验证摘要，不会被脚本重跑覆盖。

## 2. CAD 重建（可选）

```powershell
.venv\Scripts\python.exe -m pip install -r requirements-build.txt
git clone https://github.com/MuShibo/Micro-Wheeled_leg-Robot.git upstream-robot
.venv\Scripts\python.exe scripts/build_wl_model.py --step upstream-robot/1.RobotModel/OriginalRobotModel.stp
```

原始 STEP 不随本项目重新分发。下载与使用遵守上游权利说明。建议上游克隆放在本项目之外；命令中的路径按实际位置填写。重建会更新 `assets/` 和 `docs/cad-manifest.json`，会重新网格化但使用已附带的分解碰撞网格；若改变源几何，必须重新做壳体凸分解及全套验证。

需要重新生成凸分解时，先完成一次重建，再执行：

```powershell
.venv\Scripts\python.exe scripts/decompose_shell.py
.venv\Scripts\python.exe scripts/build_wl_model.py --step upstream-robot/1.RobotModel/OriginalRobotModel.stp
```

CoACD 碰撞网格是近似，不是精确制造边界；新分解结果可能与随附网格不同。不得仅凭重建成功宣称几何回归通过。

## 3. 几何验证

```powershell
.venv\Scripts\python.exe scripts/eval_sim.py
.venv\Scripts\python.exe scripts/check_ground.py --label local
.venv\Scripts\python.exe scripts/check_interference.py --step upstream-robot/1.RobotModel/OriginalRobotModel.stp
```

先运行 `eval_sim.py` 产生新运动轨迹，再做实体求交。求交脚本校验 STEP SHA256 与来源清单一致，检查初始位形及 30 个运动帧、每帧 10 组指定零件对，阈值 0.1 mm³；不能替代全零件/连续全行程检查。

运动检查含 3 s 固定基座/零重力测试，以及 2 s 自由基座重力跌落；地面检查独立覆盖 3 s 固定测试和 5 s 跌落，采样间隔 1 ms，允许视觉几何最低点不低于地面 -1 mm。固定基座仅用于机构检查，正式 XML 仍为自由基座。

## 4. 依赖与范围

- MuJoCo 固定为 3.2.5；曾在原开发机器上遇到其他版本 DLL 初始化异常，但未确认系统级根因。
- 本次已新建项目独立 `.venv` 并联网安装运行依赖，运动/地面/测试及快捷入口均通过；实体求交复用已有 CAD 环境，未在干净系统安装 CAD 依赖。
- `requirements-build.txt` 的 CAD 依赖用于可选重建，直接看模型不需要。
- 不含固件、串口、网络控制和实机测试，不会操作机器人。
