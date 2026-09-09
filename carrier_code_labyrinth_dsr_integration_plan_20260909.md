# Carrier-LaCAM 接入 Code-Labyrinth DSR：代码调研与实施计划

日期：2026-09-09
Carrier-LaCAM 基线：`dd-lacam@2d4563c`
Code-Labyrinth 基线：`share/bofu/DSRS-mppf-classic-dig@1b4ca13`
参考旧版：`share/aormiga/DSRS-official@9bf9d99`

## 1. 结论

这次接入应该以较新的 MPPF 分支为底座，但不是把 `DsrMppfPlanner` 里的 Br-LaCAM 换成 Carrier-LaCAM 就结束。现在的 MPPF 只决定“哪个 pod 搬到哪里”，随后 `DsrScheduler` 把联合结果拆成独立 activity，`DigoutDriveBindingLogic` 再用最近距离重新分配机器人，`MultiAgentSystem` 又重新规划机器人路径。这样会丢掉 Carrier-LaCAM 已经算出的 tau、rho、显式 WAIT、LIFT/DROP 时刻和联合无碰撞时序。

正确边界是：

```text
Code-Labyrinth：
  产生 Pick 和 DsrDigout
  定义 storage block 与边界格
  保存真实 Drive / Pod / Storage 状态
  DIG 完成后继续原来的送站流程

Carrier-LaCAM：
  选择目标 pod 对应的边界格 tau
  决定需要挪动哪些普通 pod
  分配机器人 rho
  生成并执行完整的 WAIT / MOVE / LIFT / DROP 联合计划
```

因此，Carrier 模式下要绕过：

```text
DsrMppfPlanner
  -> ActionDependencyGraph
  -> DigoutActivity
  -> DsrScheduler
  -> DigoutDriveBindingLogic
  -> MultiAgentSystem 再规划路径
```

但要保留：

```text
PickAllocator
  -> DsrDigout pending token
  -> DsrBlockManager
  -> Carrier 规划与执行
  -> dsrDigoutComplete()
  -> 原 PickDriveBindingLogic 送站
```

第一版只接管“把目标 pod 从 storage block 内搬到该 block 的合法边界格”，不把后续送站也并入 Carrier。这样既是完整的 Carrier-LaCAM DIG，又能继续复用 Code-Labyrinth 已有的 Pick、station 和业务统计。

## 2. 两边代码现在分别做了什么

### 2.1 Code-Labyrinth 当前 DSR 主链

`PickAllocator.allocate()` 先创建 Pick，并将目标 pod 标记为 `PICK`、原位置标为 `ALLOCATED_FOR_LIFT`。`PickDriveBindingLogic` 发现这是 DSR Pick 后，调用 `DSRAllocator.dsrDigoutRequired()` 创建 `DsrDigout`。这个对象不是具体搬运任务，而是“原 Pick 正在等待 DIG 完成”的 pending dependency。

`DSRAllocator.allocate()` 周期性完成以下工作：

1. 检查旧 `DigoutActivity` 是否完成。
2. 根据已提交 activity 投影未来 pod 和空位状态。
3. 判断哪些 `DsrDigout` 已完成。
4. 对未完成目标重新规划。
5. 可选地产生 dig-in。
6. 通过 `DsrScheduler` 提交一部分 activity。
7. 对完成目标调用 `dsrDigoutComplete()`。

MPPF 分支在第 4 步调用 `DsrMppfPlanner`。它按 storage block 求解，每个 requested pod 是一个 planner agent，每个目标拥有该 block 内全部可用边界格。这里的 agent 是 pod，不是机器人。

随后发生三次相互独立的决定：

```text
DsrMppfPlanner：pod 搬到哪里
DigoutDriveBindingLogic：哪个机器人搬
MultiAgentSystem：机器人实际怎么走
```

`DsrScheduler` 和 `DigoutDriveBindingLogic` 还会提前修改 `StorageManager`、`PodManager` 和 reservation。它们适合旧的独立 `SLIDE/DIG` activity，但不适合执行 Carrier 的同步联合计划。

### 2.2 Carrier-LaCAM 当前主链

Carrier 的正式路径是：

```text
solve_carrier_lacam_result()
  -> run_search_attempt()
  -> TAPFPlanner::solve()
  -> attach_carrier_guidance()
  -> tau + Task-BR + rho + timed transport
  -> Carrier-PIBT / funcPIBT()
  -> apply_ops()
  -> 新 PhysConfig
```

静态输入是 `DDInstance`：

- `grid`：规则矩形四邻接网格。
- `robots`：机器人起点，数组顺序就是机器人身份。
- `shelves`：所有目标和普通货架。
- `shelf_storage`：允许 DROP 的格子。
- `target_starts`：目标货架身份和初始位置。
- `target_goal_sets`：每个目标允许到达的多个终点。

运行时状态是 `PhysConfig`：

- `robots[i]`：机器人位置。
- `target_pos[b]`：目标货架位置。
- `anon_occ`：落地普通货架的位置。
- `kappa[i]`：机器人空闲、携带普通货架或携带哪个目标货架。

输出是 `DDPlan[timestep][robot]`，动作只能是 `WAIT`、`MOVE`、`LIFT`、`DROP`。一个 timestep 必须整体执行，不能拆成每台机器人的独立路径。

Carrier 内部已经能在相邻真实状态之间增量继承：

- tau 的 PairCost dependency；
- incremental Hungarian 状态；
- carried shelf custody；
- RootGoalCommitment；
- rho continuity；
- timed transport guidance。

但这些状态目前只活在一次 `TAPFPlanner` 调用内部。公开入口只能从 `DDInstance` 初态启动；`TAPFSearchConfig.initial_physical` 虽然存在，却被限制为必须与 BRD 的 `CarrierEventContract` 一起使用。

`CarrierEventContract` 不能拿来做本次接入，因为它会提前固定机器人、货架、endpoint 和完整 route，并绕过正常 tau、Task-BR 和 rho。我们需要的是“从当前 PhysConfig 启动正常 Carrier 主链”，不是 BRD frozen-plan segment。

### 2.3 现有 JNA 接口能复用什么

MPPF 已经有可工作的 native 构建和手写 JNA 接口：

```text
Java opaque Pointer
  -> create instance
  -> 填图、agent、goal
  -> native plan
  -> Java 查询结果
  -> destroy
```

这套“opaque handle + 手写 JNA + shared library 随 Brazil 包发布”的骨架可以复用。但是现有 `BrLacamInterface` 和 `BrLacamSolution` 只能返回 pod 的顶点轨迹，WAIT 被删除，也没有机器人身份、LIFT 或 DROP，所以必须为 Carrier 建立独立接口，不能继续给旧接口加含义不同的字段。

## 3. 目标执行路径

首版目标路径如下：

```text
PickAllocator 创建 Pick
  -> DSRAllocator 创建 DsrDigout
  -> CarrierDsrCoordinator 按 source block 收集未完成请求
  -> 获取该 block 及相邻通道交接点的 execution lease
  -> 收集一组尚未绑定 pod 的 idle drives
  -> 必要时先由现有交通系统把这组 drives 停到不同交接点
  -> 等待参与 drives 的旧路径完全结束并移交控制权
  -> CarrierProblemAdapter 构造 DDInstance + 当前 PhysConfig
  -> libcarrier_lacam.so 调正常 Carrier-LaCAM
  -> 返回完整 DDPlan
  -> CarrierJointExecutor 按 timestep 原样执行
  -> 每一步后用真实 Drive/Pod/Storage 状态核对 PhysConfig
  -> 目标 pod 已 DROP 在合法边界
  -> DSRAllocator.dsrDigoutComplete()
  -> 原 PickDriveBindingLogic 接着把 pod 送到工作站
```

这里 DSR 只提供任务语义和合法集合：

- 哪些 pod 是 requested target；
- 每个 target 属于哪个 source block；
- 该 block 有哪些边界格；
- 哪些业务 reservation 当前不可使用。

DSR 不再为 Carrier 任务指定具体边界点，也不再重新分配机器人。

这里的 drive 收集和预定位只是执行资源交接，不是 DIG task assignment：Java 可以决定“哪些空闲机器人进入本次候选池”，但不能决定候选池中的哪台机器人搬哪个 pod，也不能决定 pod 去哪个 border cell。这两个决定仍分别由 Carrier 的 rho 和 tau 完成。

## 4. 第一版的明确边界

为了先证明语义正确，第一版采用保守但可验证的运行边界。

### 4.1 按 storage block 求解

一次 Carrier session 的任务范围只处理一个 storage block 内的未完成 `DsrDigout`，但运动范围不能只包含 block。`DsrBlockManager` 的 block 只由 `STORAGE` vertex 组成，而 drive 可能位于外部 `TRAVEL` vertex；因此首版 session grid 定义为：

- storage region：该 block 的全部 storage vertices；
- handoff portals：每个 border storage vertex 紧邻、且有双向边的第一层 travel vertices；
- targets：该 block 中所有未完成 requested pods；
- goal set：该 target 所属 block 的合法 border storage cells；
- anonymous shelves：该 block 中其余可由 Carrier 搬动的 pods；
- robots：已经位于 session grid 内，或由现有交通系统预定位到不同 handoff portal 的 idle drives；
- shelf storage mask：只有 block 的 storage vertices 为真，travel portals 可以行驶但不能 DROP。

这不是让 DSR 重新做 assignment。block 只定义 pod 问题边界，portal 只定义机器人控制权的交接边界；target 到 border 的 tau、候选机器人之间的 rho、普通 pod 清障和进入 block 后的完整路线都由 Carrier 决定。预定位阶段只能把一组未绑定机器人送到一组空 portal，不能预先把某台机器人绑定给某个 pod。

先按 block 做有三个好处：

1. 与当前 `DsrMppfPlanner` 的问题粒度一致，容易做 A/B 对比。
2. DIG 的终点就在 storage border，不需要把 station/travel 全图先纳入 Carrier。
3. 可以先验证规则 storage block 加一层 handoff portal 到 `DDGrid` 的映射，不必一开始就把任意 KMAP 有向图改造成 Carrier 显式图。

候选池大小取当前 eligible idle drives、空 portals 和 `CarrierMaxDrivesPerSession` 三者的最小值；target 数可以大于 robot 数，由 Carrier 顺序完成任务。如果当前连一组“一个空 portal + 一个 idle drive”都无法形成，coordinator 保持 `DsrDigout` pending，等待下一次 allocator tick；不能通过减少 targets、预先绑定 pod 或切回旧 MPPF 来掩盖资源不足。

### 4.2 只接受能无损映射为 DDGrid 的 session grid

Code-Labyrinth 的 `Graph` 是任意有向图，Carrier 当前是矩形四邻接网格。另一个实际细节是 `GraphReader` 会把 KMAP 的小数毫米坐标四舍五入成整数，因此名义上相同的 1054.1 mm 间距可能交替读成 1054 和 1055；不能用整数整除要求坐标绝对等距。

Java adapter 应先根据水平、垂直边的众数长度推导 nominal lattice spacing，再把接近同一 x/y 的坐标聚成 coordinate bands，按 band 排序得到 row/column。之后对“block storage vertices + handoff portals”做严格拓扑校验：

- session grid 内 vertex 坐标唯一；
- storage 网格的横纵 nominal lattice spacing 在配置容差内一致；
- 纳入 session grid 的 edge 都近似水平或垂直，并映射为一个 row/column step；
- edge 实际长度与 nominal spacing 的误差在配置容差内；
- 每条纳入的 edge 都有反向 edge；
- 两个坐标上相邻且都被纳入 session grid 的 vertices 之间必须存在双向 edge；
- 每个 handoff portal 都是某个 border storage vertex 的第一层 `TRAVEL` 邻居；
- 不允许 portal 重合、对角边、长跳边、单向边或 arc edge；
- 归一化后的矩形尺寸和 cell 数量在配置上限内。

通过后，用 coordinate-band rank 建立 cell：

```text
row = rank(clusteredY, y)
col = rank(clusteredX, x)
cell = row * width + col
```

建立 `Graph.Vertex.id <-> DDGrid cell` 双向表，缺失坐标作为 wall。容差只能吸收 KMAP 解析产生的小量坐标抖动；不能把真实长跳边、对角边或不规则拓扑压成相邻网格。

portal 连向全局 travel graph 的其他边不进入 session grid。这不是静默删边：execution lease 明确把 portal 定义为控制权边界，Carrier 执行期间任何 drive 都不能从 portal 继续驶向外部，也不能从外部驶入 portal。Carrier 计划完成后，控制权交还给 `MultiAgentSystem`，外部边才重新开放。

如果 block 到 portal 的局部图本身无法通过校验，Carrier 模式必须 fail closed，留下清楚的 incompatibility 日志；不能近似 edge，也不能 fallback 到旧 MPPF。等真实布局证明需要后，再在现有 Carrier 主路径上把 `DDGrid` 扩展成显式 adjacency，而不是在 Java 侧伪造网格。

### 4.3 首版使用独占执行 epoch

Carrier 计划没有建模外部正在移动的 drive 或正在执行的 Pick/Stow。首版启动一个 block 的 Carrier epoch 前必须满足：

- execution lease 已覆盖 block storage vertices 和全部 handoff portals；
- 参与 drives 全部 idle、未绑定 pod，并已停在 session grid 内互不相同的 cells；
- 如果参与 drive 原来在 session grid 外，预定位 MorePath 已经完整执行结束；
- `DeterministicExecution` 已确认这些 drives 没有未执行 path、grant 或 snapshot commitment；
- session grid 内没有其他 drive，也没有外部 drive 已获准进入；
- block 内没有非本 session 的 carried pod；
- 没有非本批 DSR target 的在途 Pick/Stow/Digout；
- 没有无法由 Carrier 改动的外部 storage reservation；
- `MultiAgentSystem` 中没有尚未执行完的 MorePath 或 WorkAssigned 进入该 lease。

预定位期间仍由现有 `MultiAgentSystem` 负责全局交通安全，但这些 drives 只是去 portal，不携带或绑定本批 pod。最后一台 drive 到位后，先清空其普通执行状态，再启动 Carrier epoch。epoch 执行期间暂停会改变该 lease 状态的新 Pick/Stow/Drive binding。第一版可以使用全局 allocator pause，确认正确后再缩小为 block lease。

这样做是为了建立明确的权威状态：Carrier session 开始后，lease 内只有 CarrierJointExecutor 能改变 drive 和 pod。后续“并行正常业务 + Carrier”需要显式建模固定 pod、外部机器人和时空 reservation，不能靠乐观假设。

## 5. Java 与 C++ 的输入输出合同

### 5.1 Java 构造的输入

每个 session 固定以下身份映射：

| Carrier 字段 | Code-Labyrinth 来源 |
|---|---|
| robot index | 按 `IDrive.getId()` 排序后的参与 drives |
| robot cell | `drive.getNearestVertex()` 经 session grid map 转换 |
| target index | 按稳定 pod id 排序后的 `DsrDigout` targets |
| target position | target pod 当前 location，或 carrying drive 的 cell |
| anonymous shelf | block 内所有非 target pods |
| `kappa` | `drive.getCarriable()` 与 target/anonymous pod 映射 |
| traversable grid | block storage vertices + handoff portals |
| storage mask | 仅 block 中的 storage vertices |
| target goal sets | target 的 source block 中合法 border cells |

`DsrDigout` 创建时要保存稳定的 `sourceBlockId`。不能在 target 已经被搬到 border 后，再根据当前 location 猜它原来属于哪个 block。

goal set 过滤规则：

- 必须属于 target 的 source block；
- 必须是 `DsrBlockManager.verticesAtBorder` 中的 storage cell；
- 外部硬 reservation 不可作为 goal；
- 被本次 Carrier 可移动 anonymous pod 占据的 border 仍然可以作为 goal，Carrier 可以先清空；
- 所有 target 的 goal sets 最终必须通过 Carrier 的 injective matching 校验。

### 5.2 Native C ABI

新库使用独立名字 `libcarrier_lacam.so`，接口前缀统一为 `carrier_lacam_`。建议从一开始按 session 设计，第一版内部可以先冷启动，后续增加缓存时不改 Java API。

核心接口形态：

```c
void* carrier_lacam_create(int seed);
void  carrier_lacam_destroy(void* handle);

int carrier_lacam_set_grid(
    void* handle,
    int height,
    int width,
    const uint8_t* wall_mask,
    const uint8_t* storage_mask);

int carrier_lacam_set_entities(
    void* handle,
    int robot_count,
    const int* robot_cells,
    int shelf_count,
    const int* shelf_cells,
    int target_count,
    const int* target_shelf_indices,
    const int* goal_offsets,
    const int* goal_cells);

int carrier_lacam_set_state(
    void* handle,
    const int* robot_cells,
    const int* target_cells,
    int anonymous_count,
    const int* anonymous_cells,
    const int* kappa);

int carrier_lacam_solve(void* handle, int timeout_ms);

int carrier_lacam_get_status(void* handle);
int carrier_lacam_get_timestep_count(void* handle);
int carrier_lacam_get_robot_count(void* handle);
int carrier_lacam_get_action_kind(
    void* handle, int timestep, int robot);
int carrier_lacam_get_action_destination(
    void* handle, int timestep, int robot);

int carrier_lacam_commit_prefix(
    void* handle,
    int executed_steps,
    const int* observed_robot_cells,
    const int* observed_target_cells,
    int observed_anonymous_count,
    const int* observed_anonymous_cells,
    const int* observed_kappa);

double carrier_lacam_get_first_solution_ms(void* handle);
double carrier_lacam_get_deliverable_ms(void* handle);
long carrier_lacam_get_makespan(void* handle);
long long carrier_lacam_get_work_scaled(void* handle);
const char* carrier_lacam_last_error(void* handle);
```

输入在 setter 返回前复制到 native handle；结果由 handle 持有；Java 只读取，不持有 C++ 内部数组裸指针。`destroy` 必须在 `solve` 返回后调用，同一个 handle 第一版不允许并发调用。

所有 `extern "C"` 函数都必须捕获 C++ 异常。错误至少区分：

```text
OK
INVALID_ARGUMENT
INVALID_STATE
EXHAUSTED
TIMEOUT
CANCELLED
INTERNAL_ERROR
```

`carrier_lacam_last_error()` 返回 handle 内部字符串，Java 立即复制。不能让异常穿过 JNA 边界。

### 5.3 输出语义

Java 必须读取完整矩阵：

```text
plan[t][robot] = kind + destination
```

不能只读取 moved robots，也不能删除 WAIT。`LIFT` 和 `DROP` 不改变机器人位置，但必须保留。

报告和日志优先展示 `first_solution_ms`，然后才是 `deliverable_ms`、makespan 和 work。`first_solution_ms` 表示搜索第一次找到合法解的时间；`deliverable_ms` 还包括 cleanup、repair 和最终 replay，两者不能混写成一个 Runtime。

## 6. Carrier-LaCAM 侧修改

### 6.1 正常 arbitrary-root API

新增公开入口：

```cpp
DDSolveResult solve_carrier_lacam_from_state_result(
    const DDInstance& ins,
    const PhysConfig& current,
    double time_limit_sec,
    int seed,
    DDStats* stats = nullptr,
    DDPlan* best_effort = nullptr);
```

这个入口仍然调用同一个 `TAPFPlanner::solve()`，不能增加新的搜索循环。

具体修改：

1. `TAPFSearchConfig.initial_physical` 允许在没有 `CarrierEventContract` 时使用。
2. `TAPFPlanner` 构造时调用 `validate_phys_config_root()`。
3. `TAPFPlanner::solve()` 只要存在 `initial_physical`，就用它构造 root config 和 shelf state。
4. `event_contract` 只决定 BRD fixed-transfer 语义，不再决定是否可以使用 arbitrary root。
5. normal arbitrary-root 模式继续走正常 `attach_carrier_guidance()`，保留 tau、Task-BR、rho 和 Carrier-PIBT。

### 6.2 所有 finalization helper 都必须认识 root

目前多个 helper 硬编码从 `initial_phys_config(ins)` 重放。只改搜索 root 会导致正确计划在 final replay 中被误判。以下函数要增加 root 参数，并保留旧 wrapper：

- `normalize_goal_prefix`
- `plan_work_scaled`
- `plan_cost_checked` / `plan_cost`
- `replay_raw_prefix`
- `build_reference_plan`
- `fixed_goal_instance_from_plan`
- `repair_carrier_plan`
- `repair_carrier_plan_from_replay`

`run_search_attempt()` 也要接收 optional root，并把它传入 `TAPFSearchConfig`、normalization、repair、cost 和 reference plan。

现有 `solve_carrier_lacam_result()` 改为用 `initial_phys_config(ins)` 调新的共用实现，以保证旧调用路径自然保持原行为。

### 6.3 Native shared-library target

在当前 CMake 中新增 `carrier_lacam_jna` shared target：

- 链接现有 `lacam` 算法代码；
- 算法源码设置 `POSITION_INDEPENDENT_CODE`；
- shared target 不使用 `-march=native/-mtune=native`；
- C ABI 放在独立文件，例如 `lacam/interface/carrier_lacam_jna.cpp`；
- 不把接口塞进旧 `libbrlacam.so`。

当前 `lacam` 使用源码 glob，并将 YAML loader 与核心算法编在一起。Brazil native 构建不应依赖构建机恰好安装的 `yaml-cpp`。建议把 YAML-only 代码拆成：

```text
lacam_core       内存中的 graph/instance/planner/carrier
lacam_yaml_io    load_dd_instance / YAML TAPF loader
```

JNA 只链接 `lacam_core`。命令行工具和现有测试继续链接两者。这个拆分只改变构建边界，不复制算法实现。

## 7. Code-Labyrinth 侧修改

### 7.1 新增 Carrier Java wrapper

建议新增：

```text
.../workgeneration/ar/allocators/carrier/jna/CarrierLacamInterface.java
.../workgeneration/ar/allocators/carrier/jna/CarrierLacamNative.java
.../workgeneration/ar/allocators/carrier/CarrierAction.java
.../workgeneration/ar/allocators/carrier/CarrierPlan.java
```

`CarrierLacamNative` 使用明确状态机：

```text
NEW -> READY -> SOLVING -> SOLVED/FAILED -> DESTROYED
```

不要重复现有 `BrLacam` 的双 handle 模式。一个 Java session 对应一个 native handle，由 `AutoCloseable.close()` 唯一释放。

native library 加载必须使用稳定路径。优先让 Brazil/Apollo 把 `${ENVROOT}/lib` 加入 `jna.library.path`，或从 classpath 解压到受控临时目录后按绝对路径加载。不能依赖当前工作目录中碰巧存在 `build/lib`。

### 7.2 新增 session-grid adapter

建议新增：

```text
CarrierSessionGrid.java
CarrierProblemAdapter.java
CarrierStateSnapshot.java
CarrierProblemValidation.java
```

职责分别是：

- 从 storage block 和第一层 travel portals 构造 session grid，并校验是否可无损映射为规则网格；
- 维护 vertex/cell、drive/robot、pod/shelf/target 双向映射；
- 从真实对象构造 `DDInstance` 和 `PhysConfig` 数组；
- 检查 reservation、活动任务和目标 goal sets；
- 将 native MOVE destination 映射回真实 `Graph.Vertex`。

adapter 只能读 Code-Labyrinth 的权威状态，不能提前修改 Pod 或 Storage。只有 executor 真正完成动作后才写状态。

### 7.3 新增 Carrier DSR coordinator

建议新增：

```text
CarrierDsrCoordinator.java
CarrierExecutionLease.java
CarrierDriveStager.java
CarrierPlanningSession.java
```

coordinator 的职责顺序固定为：选取未绑定的 idle drive 候选池、为候选池申请互不相同的 portal、让 `CarrierDriveStager` 使用现有交通系统完成预定位、确认普通路径状态已经清空，然后才创建 native planning session。drive-to-portal 可以按全局交通距离做确定性的最小代价匹配，并用 drive/vertex id 打破平局；这只是交接点分配，不能在 Java 侧把 drive 绑定给 target pod。

`DSRAllocator.allocate()` 的步骤 1–3 和步骤 7 保持共用。中间执行部分变成：

```text
legacy mode：
  原 planner -> scheduler -> Digout

carrier mode：
  CarrierDsrCoordinator.planOrAdvance(incompleteDigouts)
```

不要让 `CarrierDsrCoordinator` 实现 `IDsrPlanner`，因为 `IDsrPlanner` 的返回值是会丢语义的 `DigoutActivity`。Carrier 是完整 execution backend，不是另一种 activity planner。

`FC.java` 增加 `DsrPlanningStrategy=carrier`。选择该模式后：

- 不构造 `DsrMppfPlanner`；
- 不调用 `DsrScheduler.scheduleActivities()`；
- 不生成 `DigoutAllocator.Digout`；
- `DigoutDriveBindingLogic` 看不到 Carrier 管辖的任务；
- `DsrStoragePolicy` 不为 Carrier pod 选择 endpoint。

首版 Carrier 模式禁止 `EnableDsrDigins=true`，配置冲突时直接报错。普通 pod 的必要 relocation 已由 Carrier Task-BR 处理，不应再并行启动旧 dig-in。

### 7.4 修正 DSR 完成判定

`Pod.getLocation()` 在 pod 被举起时会返回 drive 位置。当前 `checkIfDsrDigoutCompleted()` 只看 pod 是否来到 border，Carrier 模式下可能在机器人到达 border、尚未 DROP 时提前完成。

完成条件必须增加：

```text
pod.getDriveUnderneath() == null
```

并确认：

- `StorageManager` 在该 border cell 上绑定的是该 pod；
- cell 状态已经是对应的落地状态；
- 没有 Carrier session commitment 计划继续移动该 target。

只有这时才能调用 `dsrDigoutComplete()`。

## 8. 联合执行器

### 8.1 为什么不能使用 MorePath

`MorePath` 只有顶点列表；`DeterministicExecution` 会删除隐式 WAIT；`MAPointDrive` 把每个顶点解释成 MOVE。它无法表达 LIFT、DROP 和联合 timestep，因此不能作为 Carrier 执行协议。

### 8.2 新执行状态机

建议新增：

```text
CarrierJointExecutor
CarrierStepCommand
CarrierStepCompleted
PodTransferStateUpdater
```

每个 timestep 的状态机：

```text
VERIFY_START
  -> PRE_ROTATE_MOVERS
  -> START_ALL_ACTIONS
  -> WAIT_FOR_ALL_COMPLETIONS
  -> COMMIT_LOGICAL_STATE
  -> VERIFY_SUCCESSOR
  -> NEXT_STEP / REPLAN / COMPLETE
```

规则：

- 所有 MOVE 在预旋转完成后同一模拟时刻开始。
- WAIT 是真实 barrier participant，不能省略。
- LIFT/DROP 使用 drive 的 lift/lower duration。
- 下一 timestep 必须等当前 timestep 所有动作完成。
- Java 不重排 native 动作，也不自行改变 robot assignment。

`PodTransferStateUpdater` 从 `MissionLifecycleWorker` 中抽取当前 lift/lower 的状态更新语义，供普通 mission 和 Carrier executor 共用，避免复制两套 Pod/Storage 更新：

```text
LIFT：
  drive.carriable = pod
  pod.driveUnderneath = drive
  pod.location = null
  StorageManager.unbindPodWithVertex(...)

DROP：
  drive.carriable = null
  pod.driveUnderneath = null
  pod.location = drive vertex
  StorageManager.bindPodWithVertex(...)
```

target pod 在 Carrier DIG 中保持原 Pick 的业务身份；anonymous blocker 在 Carrier custody 期间标记为 DIGOUT/TO_STORAGE，DROP 后恢复 AVAILABLE/NONE。

### 8.3 与 MultiAgentSystem 的关系

`MorePath` 只允许用于 Carrier epoch 之前的 drive 预定位。Carrier epoch 开始前必须确认所有参与 drives 没有未完成 MorePath，并把这些 drives 从普通 planner 的 active snapshot/grant 中安全移交出来。执行期间暂停 portfolio planner 对这些 drives 的新 WorkAssigned。

Carrier 直接执行一条经过验证的联合计划，不再让 portfolio planner 重算路径。epoch 结束后，`DeterministicExecution` 必须通过一个明确的 `resetToActualPositions()` 或等价接口，与真实 drive 位置重新同步，再恢复普通任务。

不能只移动 drive 对象而不更新 `DeterministicExecution.ordering/granted/snapshot`，否则下一次普通规划会从旧位置开始。

### 8.4 动力学约束

Carrier 是离散同步模型，`MADynamicDrive` 有旋转和连续运动时间。首版要验证：

- session grid 内所有 Carrier MOVE edge 的实际长度一致；
- 所有参与 drive 使用同一 dynamics；
- 同一步 MOVE 的实际 edge travel duration 一致或满足安全同步条件；
- 旋转在 MOVE barrier 之前完成。

如果真实 dynamics 不能保证 following move 的安全同步，需要在现有 Carrier 主路径中增加 conservative no-following execution policy，并让 search、PIBT 和 `apply_ops` 使用同一配置。不能只在 Java 侧删除或延迟某个 MOVE，因为那会破坏原联合计划。

### 8.5 执行偏差

每一步完成后重新从真实 Drive/Pod/Storage 构造 `PhysConfig`，与 native 对计划前缀的预测比较。

如果不一致：

- 立即停止剩余 plan；
- 不报告任何尚未 grounded 的 DSR target 完成；
- 保持 Carrier lease；
- 从真实 `PhysConfig` 重新规划；
- 若状态本身非法，则 fail closed 并输出对象级差异。

不能继续执行已经失去前提的 plan tail。

## 9. 增量续算

完整接入分两步实现，Java API 从一开始保持 session 形态。

### 9.1 第一阶段：arbitrary root，跨调用冷启动

第一阶段每次从当前 `PhysConfig` 调 `solve_carrier_lacam_from_state_result()`。这已经能正确处理中途 carried pod，并保留一次 solve 内部所有 incremental tau/custody/rho 逻辑。

执行策略先支持：

- 整个计划一次执行到底；或
- 执行到第一个 target DROP / 固定最大 prefix 后重新调用。

这一步先验证输入、输出、执行和完成语义，不宣称跨 solve 已经复用缓存。

### 9.2 第二阶段：持久化 CarrierPlanningSession

随后让 native handle 持有可跨 solve 复用的 session 状态：

- `CarrierEngine` 或其可持久化 cache；
- `UpperEpochCache`；
- PairCost dependency context；
- incremental Hungarian state；
- 上一真实 `PhysConfig`；
- 上一 root `CarrierGuidance`；
- 已实际执行的 ops；
- RootGoalCommitment 和 active custody；
- rho continuity fingerprints。

`carrier_lacam_commit_prefix()` 必须验证：

```text
apply_ops(previous_state, returned_plan_prefix) == observed_state
```

验证成功后，下一次 root attach 使用现有：

```text
previous_X + previous_guidance + executed_ops
```

继续走同一个 `attach_carrier_guidance()`。不能新建 simulator 专用 tau/rho 实现。

如果外部状态跳变：

- transition-only custody/route continuity 失效；
- 按 dependency 检查仍安全的 upper PairCost cache 可以保留；
- target 集合、goal sets 或 block topology 改变时整体重建 session。

### 9.3 rho 增量化单独实施

当前 tau 已经有真正的 incremental Hungarian；rho 只有 continuity 和 changed-row telemetry，仍会 full solve。rho 的目标是 bottleneck + secondary + canonical tie，不是普通 additive Hungarian，不能直接复制 tau 的单行 repair。

rho 增量化放在 session 正确性之后：

1. 固定 column identity、mode/conflict 和 objective version。
2. 持久化 matching、dual potentials 和 row fingerprints。
3. 只有列模型完全相同且仅少量 robot rows 变化时 repair。
4. 列或 tie 语义变化时 full solve。
5. 增量结果必须与当前 full solver bit-for-bit 相同。

## 10. 分阶段编码计划

所有阶段遵循 `test -> RED -> implementation -> GREEN -> quick benchmark -> regression`。新建测试后即视为 protected。

### Phase 0：冻结设计和基线

修改：

- 在 `design_final.md` 增加 Code-Labyrinth integration contract。
- 记录两边 commit、配置、seed 和第一批固定 end-to-end cases。
- 为 dd-lacam 和 Code-Labyrinth 分别建立实现分支。

验收：

- 每个新增机制都能对应现有 `TAPFPlanner::solve()` 主路径中的修改点。
- 明确不使用 BRD event contract、旧 MPPF fallback 或第二套搜索循环。

### Phase 1：Carrier arbitrary-root 支持

先写 RED tests：

- grounded shelf 已经变化的 current root 能继续求解；
- root 中已有机器人携带 target，能继续到 DROP；
- root 中已有机器人携带 anonymous shelf，能继续清障；
- invalid `PhysConfig` 返回 INVALID；
- normalization、cost、repair、reference plan 都从 current root 重放；
- 旧 `solve_carrier_lacam_result()` 与修改前行为一致。

修改文件：

```text
lacam/include/tapf_types.hpp
lacam/include/dd_planner.hpp
lacam/src/tapf_planner.cpp
lacam/src/dd_planner.cpp
lacam/src/dd_planner_internal.hpp
lacam/src/dd_plan_repair.cpp
```

验收：

- normal arbitrary root 仍经过 tau、Task-BR、rho 和 Carrier-PIBT。
- 旧 tests 全绿。
- 无 rack 的 TAPF case 自然保持原行为。

### Phase 2：C ABI 与 shared library

先写 RED tests：

- bulk grid/entity/goal-set 输入；
- 四种 action 完整 round-trip；
- zero-tick success 与 failure 可区分；
- invalid input 不抛出到 C 边界；
- timeout、重复 solve、reset、destroy；
- `first_solution_ms` 和 `deliverable_ms` 单独可读。

修改/新增：

```text
lacam/interface/carrier_lacam_jna.h
lacam/interface/carrier_lacam_jna.cpp
lacam/CMakeLists.txt
根 CMakeLists.txt
YAML/core 构建拆分文件
```

验收：

- `libcarrier_lacam.so` 可独立构建。
- shared library 不依赖主机特定 CPU 指令。
- 无 native handle 泄漏。

### Phase 3：Java graph/state adapter 与 JNA wrapper

先写 RED tests：

- 规则 storage block 加第一层 travel portals 映射正确；
- 1054/1055 mm 这类取整抖动仍映射到同一 nominal lattice；
- portal 的外部 travel edge 不进入 session grid；
- portal 与 border 之间不是一个 lattice step 时被拒绝；
- diagonal、单向、长跳、缺反向 edge 被拒绝；
- vertex/cell 双向映射稳定；
- target goal sets 按 source block 构造；
- robot/pod 顺序不受 HashMap iteration 影响；
- current carrying state 正确生成 `kappa`；
- WAIT/LIFT/DROP 不丢失；
- native error 被转换成带上下文的 Java exception/result。

修改/新增：

```text
.../allocators/carrier/jna/*
.../allocators/carrier/CarrierSessionGrid.java
.../allocators/carrier/CarrierProblemAdapter.java
.../allocators/carrier/CarrierStateSnapshot.java
```

验收：

- 相同 simulator snapshot 总是生成相同 native arrays。
- adapter 本身不修改真实状态。

### Phase 4：联合执行器

先写 RED tests：

- 两机器人同 timestep MOVE；
- WAIT 不被压缩；
- LIFT 后 Pod/Drive/Storage 三方一致；
- DROP 后 Pod/Drive/Storage 三方一致；
- carried target 到 border 但未 DROP 时不算完成；
- timestep barrier 不会提前启动下一步；
- 实际状态偏差会截断 plan；
- epoch 后 `DeterministicExecution` 从真实位置恢复。

修改/新增：

```text
.../allocators/carrier/CarrierJointExecutor.java
.../allocators/carrier/CarrierStepCommand.java
.../allocators/carrier/CarrierStepCompleted.java
.../allocators/drive/PodTransferStateUpdater.java
MissionLifecycleWorker.java
MultiAgentSystem.java
DeterministicExecution.java
MADynamicDrive.java / MAPointDrive.java
```

验收：

- Java 执行后的每一步状态与 C++ `apply_ops` 预测一致。
- executor 不调用 portfolio path replanning。

### Phase 5：接入 DSR 生命周期

先写 RED integration tests：

- idle drives 在 block 外时先到不同 portals，再移交给 Carrier；
- 预定位阶段没有提前建立 drive-to-pod binding；
- target 数大于候选 robot 数时仍完整交给 Carrier 求解；
- 没有任何可用 drive-portal pair 时 DsrDigout 保持 pending；
- 外部 drive 已获准进入 lease 时不能启动 Carrier epoch；
- 单 target 无 blocker；
- 单 target 有 anonymous blocker；
- 两个 target 竞争同一圈 border，tau 给出 injective assignment；
- 多机器人 rho 身份在执行中不被重分配；
- timeout 后 DsrDigout 保持 pending；
- target DROP 后才调用 `dsrDigoutComplete()`；
- 完成后原 Pick 能继续正常送站；
- legacy `mppf/adaptive/baseline` 行为不变。

修改：

```text
FC.java
DSRAllocator.java
DsrBlockManager.java
DriveAllocator.java
PickAllocator.java / StowAllocator.java（只接 execution lease）
HumanReadableManifest.json
```

新增：

```text
CarrierDsrCoordinator.java
CarrierExecutionLease.java
CarrierDriveStager.java
CarrierPlanningSession.java
```

验收：

- Carrier mode 没有产生 `DigoutActivity` 或 `DigoutAllocator.Digout`。
- `DigoutDriveBindingLogic` 没有接触 Carrier 任务。
- 完整 Pick -> DIG -> border DROP -> Pick-to-station 链路通过。

### Phase 6：跨 prefix 增量 session

先写 RED tests：

- 执行一拍后续算，tau cache hit；
- 只移动少数 pod 时只重算相关 PairCost edges；
- carried target 保持原 goal commitment；
- observed state 与 prefix 不一致时 continuity 被拒绝；
- target/goal schema 改变时 session 重建；
- warm solve 与 cold solve 结果语义一致。

修改：

```text
CarrierEngine/session 所有权位置
TAPFSearchConfig 的 root continuation 输入
tapf_planner_carrier.cpp root attach
C ABI commit/replan 方法
Java CarrierPlanningSession
```

验收：

- 不复用搜索树裸指针。
- 不增加第二条 search loop。
- warm/cold 都调用同一个 `TAPFPlanner::solve()`。

### Phase 7：rho incremental Hungarian

先固定当前 full rho solver 的 canonical output tests，再实现 changed-row repair。只有 column identity、value、mode/conflict 和 tie contract 全部一致时才允许增量修复。

验收：

- 每个测试中 incremental rho 与 full rho 完全相同。
- benchmark 中单步少量 drive 变化时 rho 时间明显下降。

### Phase 8：从独占 epoch 扩展到一般并发

这一步在首版正确后再做，不能偷偷放宽：

- block-level lease 代替全局 pause；
- 固定、不可搬动 pod 的 upper-deck obstacle 语义；
- 外部机器人和未来路径的时空 commitment；
- normal Pick/Stow 与 Carrier drive eligibility；
- 不规则或有向 KMAP 的显式 adjacency 支持。

这些扩展仍必须落在现有 Carrier state、operator 和 `TAPFPlanner::solve()` 上，不能在 Java 侧再加一套避障 planner。

## 11. Brazil 构建和发布

Code-Labyrinth 当前使用 HappyTrails、Java 17 和 JNA，`build.xml` 已在 `ht-pre-compile` 中构建 native Br-LaCAM。建议保留这个挂载点，增加独立 target：

```text
build-native-carrier-lacam
copy-native-carrier-lacam
copy-carrier-native-for-tests
```

`Config` 中要补齐 native 构建所需的 CMake、编译器和其他明确依赖。Brazil chroot 禁网，不能依赖构建机环境。

当前 dd-lacam remote 是 GitHub。正式 Code-Labyrinth/Brazil 构建不能依赖外网拉取；实施前要把 dd-lacam 固定 commit 发布到可由 Code.amazon/Brazil 获取的 internal package，然后在 MPPF 包中作为 `external/carrier-lacam` submodule 或 Brazil native dependency 使用。不能复制一份算法源码进 LMS 包长期漂移。

构建验收：

```text
dd-lacam:
  cmake build
  test_all
  shared-library C ABI tests

Code-Labyrinth:
  brazil-build release
  brazil-build test
  native library load test
  Java unit tests
  end-to-end simulator tests
```

## 12. 配置与可观测性

新增配置建议保持少而明确：

```text
DsrPlanningStrategy = carrier
CarrierPlanTimeLimitMs
CarrierExecutionPrefix = full | first_target_drop | max_steps
CarrierMaxPrefixSteps
CarrierRequireQuiescentBlock = true
CarrierMaxDrivesPerSession
CarrierMaxGridCells
CarrierCoordinateToleranceMm
CarrierEdgeLengthToleranceMm
CarrierReplanOnStateMismatch = true
```

seed 默认从 simulator 全局 seed 派生，不在代码中写死 42。

每次 plan 至少记录：

```text
block_id
target_count
robot_count
shelf_count
status
first_solution_ms
deliverable_ms
makespan
work_scaled
adapter_ms
native_ms
staging_ms
executed_steps
replan_count
cache_hits / changed_pair_edges
failure_reason
```

最终报告把“首次找到解 Runtime”放在最前面，`deliverable_ms` 作为第二时间指标。

## 13. 验证与 benchmark

开发期遵守仓库 `rules.md`：

1. 每个重要行为先写 test 并确认 RED。
2. 相关 tests GREEN 后运行固定 quick 77。
3. 发现 bug 先写 regression test，再改 implementation。
4. 不根据结果更换 testcase、seed 或 timeout。
5. 单 case timeout 保持 10 秒。
6. full 518 只能在全部测试和 quick 通过、最终 diff 清理、独立 GPT-5.6 Sol/high reviewer 明确 APPROVE 后运行。

Carrier 本体至少验证：

- 旧 TAPF/Carrier 全部测试；
- arbitrary-root tests；
- C ABI tests；
- quick 77 与获批后的 full 518；
- 无 rack case 的 backward compatibility。

Code-Labyrinth 至少固定以下 end-to-end cases：

- 一个 target、一个空位；
- 一个 target、一个 blocker；
- 两个 target、两个 border goals；
- target 需要多次 anonymous relocation；
- robot 数少于任务数；
- plan timeout；
- 执行一半后 replan；
- target 到 border 但尚未 DROP；
- legacy MPPF 对照。

对比 MPPF 和 Carrier 时使用相同 layout、seed、drive count、业务请求和模拟时长。主要指标是：

- DSR request 到 border DROP 的完成率和时延；
- `first_solution_ms`；
- DIG 期间 makespan/work；
- station 端到端吞吐；
- replan 次数和状态偏差；
- native/adapter/executor 时间分布。

## 14. 主要风险与应对

**图模型不一致。** 首版严格校验 storage block 加第一层 handoff portals；不兼容就失败并记录具体 edge/vertex。不要静默近似。

**执行层破坏同步时序。** 不使用 MorePath，不删除 WAIT；每一步做 barrier 和真实状态核对。

**target 在 border 上仍被举着却提前完成。** 完成判定必须要求 grounded，并核对 StorageManager binding。

**外部任务改变 block 或 portal。** 首版使用 execution lease；一般并发放到独立阶段建模。

**预定位被误解成第二次 task assignment。** stager 只能把未绑定的候选 drive 放到 portal，不能选择具体 target、goal 或搬运路线；这些都必须由 Carrier 产生。

**arbitrary root 搜索正确，但 final replay 从错误初态开始。** 所有 normalize/cost/repair/reference helper 一起 root-aware，不能只改 `TAPFPlanner::solve()`。

**用 EventContract 省事却丢掉 tau/rho。** normal Carrier root 不创建 `FixedRobotTransfer`；BRD contract 保持原用途。

**跨调用看似 incremental，实际重新建 PairCost。** 第一阶段明确标注 cold；只有 session telemetry 证明 cache/matching 被复用后，才报告 warm incremental。

**native 包在开发机可加载、Apollo 中失败。** 使用 `${ENVROOT}/lib` 或受控解压路径，并加入 Brazil runtime load test。

## 15. 完成标准

这项接入只有同时满足以下条件才算完成：

1. Code-Labyrinth 的 DSR target、block、handoff portals 和 border goal set 能无损转换成 Carrier 输入。
2. normal Carrier arbitrary-root API 保留完整 tau、Task-BR、rho 和 Carrier-PIBT。
3. Java 收到并原样执行完整 WAIT/MOVE/LIFT/DROP 联合计划。
4. pod 搬运方案和 Carrier DDPlan 执行没有经过 `DigoutActivity`、最近机器人 binding 或第二次 MAPF；session 前只允许用普通交通系统完成不绑定 pod 的 portal 预定位。
5. 每一步真实 simulator 状态与 C++ 预测一致。
6. target 只有在合法 border 上 DROP 后才解除原 Pick dependency。
7. 原 Pick 能继续完成送站。
8. cold 和 warm session 的解语义一致，warm session 有可观测的 tau/PairCost cache reuse。
9. 原有非 Carrier 模式和无 rack TAPF 行为保持不变。
10. 相关 tests、quick 77、获批 full 518、Brazil build/test 和 end-to-end simulator cases 全部通过。

## 16. 建议提交顺序

为便于 review，建议按以下独立 commit 推进：

1. `design: define Code-Labyrinth Carrier DSR contract`
2. `carrier: support normal planning from arbitrary physical root`
3. `carrier: add root-aware replay repair and reference handling`
4. `native: expose Carrier-LaCAM session C ABI`
5. `build: add portable carrier_lacam shared library`
6. `lms: add Carrier JNA wrapper and block adapter`
7. `lms: add exact joint Carrier executor`
8. `lms: route DSR lifecycle through Carrier coordinator`
9. `carrier: persist incremental planning continuation`
10. `carrier: add exact incremental rho repair`
11. `validation: add end-to-end benchmarks and final report`

每个 commit 都应能说明它修改了现有哪一条 execution path，并保持测试可运行；不要把 native API、Java adapter、executor 和 DSR wiring 混成一个无法定位问题的大提交。
