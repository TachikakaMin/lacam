# Carrier-BRD 完整规划证书与 Drop 前缀重规划修复计划

**日期：2026-09-09**

**约束：** 本计划严格遵守根目录 `rules.md`。实现必须增量修改现有
LaCAM-TAPF execution path，不新增平行 planner、第二套搜索循环、fallback
或 testcase 特判。

## 1. 问题与正确语义

当前 `carrier_brd` 把 upper 生成的 `StorageTransfer.route` 当作 lower
移动合法性约束，并把 contract-mode LaCAM 的 goal 定义为“任意一个 active
task 新发生 Drop”。因此 lower 可以返回一个只保证当前 Drop、但其余 carrying
task 已进入不可恢复死锁的短前缀；controller 随后永久提交该前缀。

正确语义是：

```text
当前物理状态
  → carrying agents 保留 shelf/task/endpoint 绑定，但丢弃旧路径
  → 对所有 free agents 与当前滚动 frontier 的 PENDING tasks 重新匹配
  → LaCAM 从当前完整物理状态做一次全局联合规划
  → goal = 本 epoch 全部 active transfers 均已完成 Drop
  → 只有完整计划存在时，才执行到最早发生 Drop 的 joint step
  → 同拍 Drop 全部提交；其余未 Lift assignment 释放
  → carrying bindings 保留，所有 agent 路径全部作废并重新规划
  → 重复直到 wave 完成
```

这里的“全部 active transfers”包括：

- epoch 开始时已经 carrying 的 locked tasks；
- 本 epoch 给 free agents 新分配的 provisional tasks。

当前 wave 中尚未匹配到 robot 的其他 PENDING tasks 不属于本 epoch 的
LaCAM goal；它们会在后续 Drop 后重新参加匹配。修复 wave barrier 后，
候选集合不再只读取当前 wave：令 `w` 为最早未完成 wave，先纳入 `w` 中的
ready task；只额外纳入 `w+1` 中“同一 shelf 在 `w` 已完成前序、source
grounded”的 continuation。这样短任务完成后
可不等无关 carrying task 而继续下一段，同时不会把远期 wave 的任务一次性
塞进 lower solve。若下一 wave continuation 的 Drop endpoint 恰好是仍 carrying
transfer 的 source，则将它延后至下一次 Drop event，避免把 source handoff 与
当前完整 certificate 强耦合。除此之外 endpoint 可以是另一 active task 的
source；先 Lift 清空、
再 Drop 的合法时序由 LaCAM 和 `apply_ops()` 决定，而不是 dispatch 静态拒绝。
`w+1` continuation 不施加额外容量上界，但只有当前 wave 没有 source-grounded、
尚未 Lift 的 PENDING task 时才进入同一次 certificate；当前 wave 的未取货任务
优先重新匹配。这不限制任何 robot 的 MOVE/WAIT，也不会改变已 Lift pair 的 lock。

## 2. 必须保持的边界

### 2.1 `MOVE` 没有 BRD 特有合法性限制

contract mode 下所有机器人仍通过现有 LaCAM-TAPF operator tree 和
`apply_ops()` 生成、验证 joint transition。`MOVE` 只受原始物理规则约束：

- 相邻可通行格；
- robot vertex/swap collision；
- carried shelf 与其他 shelf 的 upper-deck collision；
- 现有 following 语义。

允许 carrying robot 前进、后退、侧移、绕路和等待。upper 的 canonical
route 最多用于候选排序，不得用于拒绝合法 `MOVE`。

### 2.2 task contract 只约束操作语义

- provisional robot 只能 Lift 分配给自己的 shelf；
- carrying robot 保持 shelf/task/endpoint 绑定；
- task 只能在指定 endpoint Drop；
- endpoint 必须是合法 storage cell；
- completed task 的 shelf 固定在 endpoint；对应 robot 在完整计划剩余部分
  可以 `MOVE/WAIT`，但不能再次 `LIFT/DROP`；
- unassigned robot 可以 `MOVE/WAIT`，但不能 `LIFT/DROP`。

### 2.3 完整计划与执行前缀分离

一次 `TAPFPlanner::solve()` 必须返回全部 active transfers 完成的完整计划。
controller 对完整计划做权威逐步重放，只截取并提交到最早 Drop step 的前缀。
完整计划是“当前前缀存在可执行后缀”的证书；被截掉的后缀不复用。

若完整计划不存在或 timeout，controller 不得提交任何未认证前缀。

## 3. 对现有 LaCAM-TAPF execution path 的修改映射

| 新机制 | 现有修改位置 | 约束 |
|---|---|---|
| carrying phase 不依赖 route | `lacam/src/tapf_event_contract.cpp::carrier_event_phase_of` | 根据 `kappa`、shelf identity、endpoint 判断 |
| MOVE 不受 route 限制 | `validate_carrier_event_transition` | `MOVE/WAIT` 只要求 task phase/ownership 保持，物理由 `apply_ops()` 裁决 |
| endpoint-only Drop | `validate_carrier_event_transition` | 仅 endpoint 可 Drop |
| contract 候选完整 | `lacam/src/tapf_planner_pibt.cpp::funcPIBT` | carrying 暴露全部邻接 MOVE + WAIT；endpoint 增加 DROP |
| 完整 active-set goal | `TAPFPlanner::is_goal_config` | 所有 active transfers 都必须 `COMPLETED` |
| completed robot 可让路 | contract candidate/validator | 完成后只允许 MOVE/WAIT，保持 shelf 在 endpoint |
| 完整计划后截前缀 | `lacam/src/dd_planner_brd.cpp` | solve 完整计划，重放到 earliest Drop，提交该前缀 |
| Drop 后重匹配与全局重规划 | 现有 controller while-loop | 全 frozen plan 的 causally-ready PENDING frontier；carrying task 保留，未 Lift assignment 释放；新 planner/CLOSED |
| route 降级为 hint | `FixedRobotTransfer` 现有字段 | 不新增第二套 route planner；只用于 source/endpoint 与候选排序 |

普通 production `carrier`、无 event contract 的 Carrier-LaCAM、shelf-free
TAPF/MAPF 路径不得改变。

## 4. TDD 与 protected-test 处理

严格执行：

```text
新增 test → 确认 RED → implementation → GREEN
```

新增 `tests/test_carrier_brd_complete_replan.cpp`，固定以下 representative
cases；文件创建后即为 protected：

1. carrying robot 可以离开 canonical route、后退并再次接近 endpoint；
2. carrying MOVE 合法，但非 endpoint Drop 非法；
3. contract-mode LaCAM 不在第一个 Drop 停止，返回时全部 active tasks 完成；
4. completed robot 可在完整计划后缀中移动让路，但不能再次 Lift/Drop；
5. 两条 canonical routes 反向重叠、地图存在旁路时，完整联合 LaCAM 成功；
6. controller 对同一场景只执行 earliest-Drop prefix，下一 epoch 重匹配并
   全局重规划，最终完成 frozen wave；
9. two-wave 计划中，已完成短 transfer 的下一段在另一条无关长 transfer
   Drop 前重新进入匹配；不得等待整个前 wave 清空；
7. 完整 lower solve 失败时不提交 partial raw plan；
8. null contract、production carrier 和 shelf-free TAPF 行为不变。

现有 protected tests 中锁定“first-completion goal”或“fixed-route MOVE”的
断言与本计划冲突。修改它们之前必须取得独立 GPT-5.6 Sol/high reviewer 对
测试迁移的明确 `APPROVE`；不得先改测试来迁就实现。

## 5. 实施阶段

### P0：计划、设计与基线

- 写入本计划；
- 在 `design_final.md` 追加本修复的规范性设计边界；
- 记录工作树和相关现有测试基线；
- 不运行 full benchmark。

### P1：新增 RED regression

- 新增 test target 与上述 8 类行为；
- 运行新测试，确认失败原因分别来自：
  - off-route MOVE 被拒；
  - goal 在首个 Drop 提前成立；
  - controller 消费的是短 segment 而非完整计划证书。
- 保存 RED 输出。

### P2：迁移 event contract 到 endpoint contract

- carrying phase 不再要求当前位置属于 frozen route；
- provisional Lift、carrying ownership、completed endpoint 仍严格校验；
- carrying/completed/unassigned 的 MOVE/WAIT 使用原始物理 successor；
- Drop 只允许指定 endpoint；
- canonical route 仅影响确定性排序。

### P3：完整 active-set LaCAM goal

- contract goal 改为所有 active transfers `COMPLETED`；
- completed robot 在后缀中可 MOVE/WAIT；
- 保持 `MAKESPAN_THEN_WORK + FIRST_FEASIBLE + macro=false`；
- 复用现有 `TAPFPlanner::solve()`、OPEN/CLOSED、constraint tree、
  `funcPIBT()` 与 `apply_ops()`，不新增搜索循环。

### P4：controller 完整计划证书与 Drop 前缀执行

- 每个 epoch 仍先锁定 carrying task ownership，再重匹配 free robots；
- 调用 LaCAM 得到完整 active-set plan；
- 权威重放完整 plan，确认所有 active tasks 完成；
- 找到 earliest Drop step，只提交到该 step；
- 同拍完成项全部更新 ledger；
- 其他 carrying tasks 保留 task/endpoint，不保留 path；
- 未 Lift provisional tasks 回到 PENDING；
- 下一 epoch 新建 planner 并全局重规划。

### P5：protected test migration

- 独立 reviewer 明确 `APPROVE` 后，才把旧的 fixed-route/first-drop 测试迁移
  为 endpoint-contract/full-plan 语义；
- reviewer 若 `REJECT`，保持旧测试不变并重新审视实现与本计划。

### P6：相关回归与固定开发 benchmark

先运行所有相关 C++ tests，再运行全量代码 tests。开发期 benchmark 只使用
冻结 quick 77；不得运行 factorial/full case。

固定 focused quick cases：

```text
benchmark/instances_brap_pool/g4x10/brap_h4w10_a5_e1_R1_seed0.yaml
benchmark/instances_brap_pool/g6x10/brap_h6w10_a6_e1_R1_seed0.yaml
benchmark/viz_web/warehouse_block_suite/instances/warehouse_blocks_h20w20_b3_a1_d50_r8_t12_seed0.yaml
benchmark/viz_web/warehouse_block_suite/instances/warehouse_blocks_h20w20_b9_a1_d75_r8_t12_seed0.yaml
```

统一使用 `carrier_brd`、seed 0、unit weights、单例 10 秒。

### P7：quick、独立 review、full 与最终报告

1. 全部 C++/Python tests 通过；
2. 固定 quick 77 完成；
3. 清理并审计最终 diff；
4. 独立 GPT-5.6 Sol/high reviewer 明确 `APPROVE`；
5. approval 绑定当前 suite/corpus/binary 后才运行 protected full 518；
6. 生成中文静态汇报网页；
7. 独立 subagent 审查网页并明确 `APPROVE`。

## 6. 验收条件

以下条件必须全部有当前工作区证据：

- 新 endpoint contract 不限制合法 MOVE；
- LaCAM 返回全部 active transfers 完成的完整计划；
- controller 只执行 earliest-Drop prefix，并在 Drop 后重新匹配 free agents、
  为所有 agents 全局重规划；
- carrying shelf/task/endpoint 保持，旧 path 不保持；
- root-cause 反向重叠路线的最小 regression 成功；
- lower 失败不泄漏或提交 partial prefix；
- 普通 MAPF、shelf-free TAPF、production Carrier-LaCAM regression 全绿；
- quick 77 使用固定 10 秒协议完成；
- 独立 review 后 full 518 完成；
- 最终报告网页与数据一致并通过独立审查。

## 7. 明确不做

- 不以“全部串行执行”作为正式修复；
- 不给 route conflict 写 testcase-specific 特判；
- 不复制或替换 LaCAM search loop；
- 不让 upper route 成为 MOVE 合法性约束；
- 不在完整计划失败时执行 deepest/best-effort prefix；
- 不提前使用 protected full 结果调参。
