# 第三方来源声明

## 原机械工程

- 项目：Micro-Wheeled_leg-Robot
- 仓库：https://github.com/MuShibo/Micro-Wheeled_leg-Robot
- 原工程 README 列出的贡献者：Mu Shibo（穆世博）、Li Yufeng（李育锋）。
- 来源：`1.RobotModel/OriginalRobotModel.stp`，本项目来源 hash 见 `docs/cad-manifest.json`。
- 派生资产：`assets/meshes/part_*.obj`、`contact_*.obj` 及 MJCF 中由 CAD 提取的几何、组装和惯量信息。
- 处理：坐标变换、米制网格化、刚体分组、四杆闭合、碰撞近似，移除原电池实体并以本项目实物电池尺寸建模。

本项目基于 [MuShibo/Micro-Wheeled_leg-Robot](https://github.com/MuShibo/Micro-Wheeled_leg-Robot) 整理，原始设计及相关资料的权利归原作者所有，相关权利及使用条件以原项目声明为准，本项目不另行变更其授权条件。感谢原作者的分享与贡献。

如本项目内容涉及侵权，请联系维护者 dycsrl，我们将及时核实并修改或删除相关内容。

## 工具依赖

MuJoCo、CadQuery、CoACD、trimesh、NumPy、Pillow 通过依赖安装，不捆绑其源代码或二进制。各自适用其发行版许可证；本说明不改变它们的许可。
