# 40×40 b9 Dense-channel Agent×Task 扩展矩阵 V1

日期：2026-09-14

## 目的

在不修改冻结 quick 77 与 full 518 的前提下，新增一个独立、可审计的
40×40 dense-channel block-edge 压力 benchmark。场景沿用
`generate_dense_channel_suite.py::build_dense_case()` 和
`INTERIOR_TO_BLOCK_EDGE_SET`，不复制地图、目标或 witness 生成算法。

## 冻结场景矩阵

所有实例固定：

- 地图：40×40；
- storage block：9×9；
- aisle width：1；
- 每 block 货架：76/81（93.827%）；
- instance seed：1；
- target 起点：所在 block 的内部 storage，`cell_depth >= 1`；
- 每个 target 的 eligible goals：其起始 block 的全部 32 个 edge storage；
- witness：生成器反向轨迹，必须由 `ddbench.validator.validate_plan()` 完整重放；
- witness 只作可解性证书，不写入 solver YAML，也不作为 solver 输入。

矩阵为：

```text
robots = 4, 8, 12, 16, 24, 32, 48, 64
targets = 24, 48, 96
```

两轴做笛卡尔积，共 24 个 testcase。创建后实例、配置、seed、目标语义和
数量均视为 protected，不根据求解率或质量删改。

## 与现有 benchmark 的映射

历史 `benchmark/release_benchmark.json`（quick 77）和
`benchmark/full_benchmark.json`（full 518）保持逐字节不变。新增 suite 使用：

- 生成目录：
  `benchmark/viz_web/dense_channel_b9_agent_task_suite_v1_20260914/`
- suite config：
  `benchmark/dense_channel_b9_agent_task_benchmark_v1.json`
- family：`dense_channel_b9_agent_task_v1`
- method：`carrier`
- solver seed：0
- timeout：每例严格 10 秒
- objective weights：`[1,1,1,1]`
- following：allowed
- jobs：14（16 个物理核保留 2 个）

开发阶段先运行相关 Python tests 和冻结 quick 77。新增 24 例只在代码、生成器、
validator 语义和 quick 结果通过独立 GPT-5.6 Sol/high review 后运行。

## 实现落点

1. 新增轻量 suite 组合器，调用现有 `build_dense_case()`、`_manifest_row()`、
   `_plan_text()`、`save_instance()` 和 `write_html()`；不得复制核心生成逻辑。
2. 新增独立 suite config；不改 quick/full manifest 和 runner 的 tier 定义。
3. `generate_dense_channel_report.py` 的 block-edge 说明改为从 manifest 中读取
   robots/targets，并增加 agent×task 结果矩阵；旧三档报告语义保持兼容。
4. 网页只为 `rows.csv` 中 `success=1` 且 plan SHA 匹配的结果生成动画，动画生成
   继续调用现有 `generate_web_viz.py`，由其权威 validator 重放。

## 验收

- manifest 恰好 24 行，名称唯一；
- robots/targets 恰好覆盖冻结笛卡尔积；
- 每例 96 个及以下 target 全部起于 block 内部；
- 每个 target 恰有 32 个 eligible goals，且全部是同一初始 block 的 edge；
- 全部 witness 权威重放合法并到达 goal；
- quick 77 不变且通过；
- 24 例按固定 10 秒、14 workers 跑完并产出完整 rows/timing；
- 网页统计与 rows/timing/manifest 一致，成功例动画数与成功数一致；
- 独立 reviewer 对代码/测试/quick 和最终网页分别明确 `APPROVE`。

## Steering update：与 carrier_brd 同协议对比

用户要求最终网页复用既有 carrier vs carrier_brd 左右动画界面，而不是只展示
carrier 单方法矩阵。因此同一批 24 个 YAML 还要使用与 carrier 完全相同、且已
通过冻结 quick 77 的 `build-release/dd_benchmark` 运行 `carrier_brd`；冻结
binary SHA-256 为
`d7d76608ba49716f6d07e1b841b8aa729ea73666e6611c68885ee5b949dbe49f`。专用启动
入口必须在创建结果目录或派发 testcase 之前校验该 SHA，并把校验后的只读快照
交给原 runner。timeout、jobs、solver seed、weights 和 following 必须与 carrier
完全相同。两种方法各自保留独立 rows/timing/work 目录；对比页复用既有
`generate_carrier_vs_brd_first_solution_page.py` 的三张散点图、testcase 联动和
左右同步播放器，不复制播放器实现。carrier 时间继续标为首个全局 goal，
baseline 时间明确标为 final/deliverable，避免伪装成同阶段首解比较。
