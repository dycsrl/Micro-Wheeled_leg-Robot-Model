# 新目录验证记录

验证时间：2026-11-08T13:28:44.288711+08:00。

## 本次真实执行

在新项目内创建独立 `.venv`，从 `requirements.txt` 安装运行依赖，`pip check` 无依赖冲突。执行 `check-model.cmd`、`check-ground.cmd` 及 `python -m unittest discover -s tests -v` 全部通过。实体求交使用既有 CAD 环境，因为独立运行环境按设计不安装 CAD 工具。

| 检查 | 结果 |
|---|---|
| 离线测试 | 4 项通过 |
| MuJoCo | 3.2.5 |
| 模型 | 11 bodies，nq=15，nv=14，4 actuators，2 equalities |
| 网格 | 156 个可视零件 + 96 个凸分解碰撞片 |
| 总质量 | 0.336000000000 kg |
| 3 s 固定机身运动最大闭环误差 | 0.034946 mm |
| 2 s 自由基座跌落最大闭环误差 | 0.060431 mm |
| 5 s 跌落视觉网格最低离地余量 | 0.064533 mm |
| 求解器警告 | 0 |
| STEP 实体抽样求交 | 31 帧、310 组指定零件对，最大交集 0 mm³ |
| 离屏渲染 | 1280 × 960 PNG、30 帧运动 GIF 已生成 |

## 未验证

未打开交互 Viewer 界面；未在干净系统安装可选 CAD 依赖；未重跑 CAD 网格化与 CoACD 分解；未验收 Linux/macOS。没有验证站立、跳跃、全行程、全零件连续自干涉或实机迁移。源模型资产逐字节复制，未调改物理参数。

## 可核验依据

- `evidence/validation-summary.json`：本次摘要与模型 SHA256。
- `evidence/interference-probe.json`：310 组求交原始结果。
- `evidence/runtime-freeze.txt`：本次运行环境实装版本。
- `../outputs/`：本机完整运动/地面轨迹（未进入 Git）。

固定机身与零重力仅为测试夹具，正式 XML 仍为自由基座。图片不是自主平衡证明。
