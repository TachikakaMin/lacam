# Carrier-LaCAM Task-BR-PIBT 实现审查与调试记录

状态：2026-09-03，Task-BR-PIBT production 迁移、回归修复、全量验证和同机
配对 benchmark 已完成，post-review 修订已通过最终独立复核；§2–§10 保留为
迁移前审计记录，§11 给出最终闭环。

## 0. 结论

production guidance 已从旧 Objective-PIBT 切换为：

```text
physical X
  -> UpperSignature U
  -> single-root Task-BR-PIBT PairCost
  -> exact injective tau_guide
  -> all-root joint dependency graph D
  -> ReadyTasks(D, X)
  -> exact-TaskId rho / transition-anchored custody
  -> Carrier-PIBT preferred joint action
  -> unchanged apply_ops
  -> physical X'
```

LaCAM-TAPF 的 physical search、OPEN/CLOSED、operator constraint tree、
`funcPIBT()`、`apply_ops()`、两遍求解、repair、strict deadline 与 final
replay 没有被另起炉灶替换。旧 ObjectiveOption、execution-price、
target parking、taboo/reguide、futile-Lift cooldown、one-empty 专用 compiler
和 runtime fallback 已退出 production path。

最终没有已知 correctness blocker。保留的风险是启发式质量并非逐例单调：
paired common-success 中有 2 个 makespan 回退、3 个 weighted-SOC 回退；
但候选成功集合扩展 2 例，且所有成功例满足 10s release gate，详见 §11。

## 1. 最终审查证据

检查范围包括 `new.md`、`design_final.md`、所有 carrier/TAPF production
改动、protected tests、benchmark runner/diagnostics、最终二进制 provenance
与逐行 benchmark 结果。

```text
./build/test_all --gtest_color=no
  222 / 222 PASS
  168.952 s

PYTHONPATH=benchmark python3 -m unittest discover \
  -s benchmark/tests -p 'test_*.py' -v
  80 / 80 PASS
  42.876 s
```

最终同机配对协议为 68 cases、jobs=14、10s/case、seed=0、unit weights、
following allowed：

```text
baseline sha256  1c32ba3e21136fe902c7d8ef0dbfdf13360b2d512e2668d409778f324830b097
baseline         36 / 68 solved, wall 33.8s
baseline small   36 / 36 solved, wall 11.3s

candidate sha256 f7127998c198aa0cbb698e92fec0bea9e5a673314c701e3113164994efc4fc17
candidate        38 / 68 solved, wall 29.2s
candidate small  36 / 36 solved, wall 7.1s
```

baseline 的 36 个成功例全部保留，candidate 另解出两个
`h20w20_a40_e100_R1` seed。common-set makespan 几何比 `0.4616`，
weighted SOC 几何比 `0.4820`。candidate 成功实例最大
deliverable/solver runtime 分别为 `6834.72/6834.75 ms`。数据位于
`benchmark/results_task_br_release_*_20260903/`。相对
`results_v3_strict_return_final`，candidate 解出 `38 vs 33`，common-33
makespan/SOC 几何比为 `0.5206/0.5343`。

## 2. 迁移前审计：应保留的正确基础

本节至 §10 记录的是编码前发现的问题和实施路线。其中“当前”“必须新增”
等措辞均指迁移前快照，不代表 2026-09-03 最终代码状态。

以下部分与最终设计一致，不应在迁移中重写：

1. `ShelfState`、`Config` 和 `SearchKey{Config, ShelfState}` 只包含物理
   状态。`tapf_planner.cpp:47-85` 没有把 tau、task、priority、rho 或 cache
   放进 CLOSED key。
2. `PhysConfig` 的 target label、canonical anonymous occupancy 和
   `kappa` 表达正确。`dd_carrier.cpp:245-256` 的初态构造可保留。
3. `is_dd_goal()` 与 `TAPFPlanner::is_goal_config()` 都按 eligible goal set
   判断 target，并要求 target grounded；它们不读取 tau。
4. `apply_ops()` 是完整 joint primitive action 的最终物理裁判，且支持
   following、Lift/Drop、robot/shelf vertex conflict。它应继续保持唯一
   authoritative transition semantics。
5. operator constraint tree 会保留并最终枚举 primitive
   `Wait/Move/Lift/Drop`；guidance 只改变未约束部分的首选补全顺序。
6. `DDDistCache`、`LowerDist`、wall distance 和共享 Hungarian
   infrastructure 可以直接复用。
7. `funcPIBT()` 中 lower-deck robot priority inheritance 和失败后尝试其他
   carrier candidate 的基本框架可以保留。
8. 两遍求解、plan repair、strict return deadline、C++ replay、Python
   validator 与 guidance 正交，应继续作为 release gate。
9. zero-shelf 的原 LaCAM-TAPF execution path 目前是结构性退化，不是 runtime
   legacy fallback；这一性质必须保住。

本次同时复核了与算法正交的新增 diff：

* `repair_carrier_plan_from_replay()` 用已经物化且稳定存储的 state vector
  避免重复 replay，pointer-key map 的 hash/equality 仍按状态内容比较；repair
  末尾和 solver return 前仍有独立合法性重放，可以保留。
* `solve_carrier_lacam()` 把大型 CLOSED tree 的析构计入 strict return
  deadline，并给 final replay 留 reserve；对应 deadline/finalization tests
  已通过，应在 guidance 重写期间保持。
* benchmark runner 现在同时机器检查 `deliverable_ms`、
  `solver_runtime_ms`，并把 carrier subprocess wall timeout 收紧为同一个
  10s protocol；这一部分与最终 release gate 一致。
* 这些 support changes 当前输出的是旧 Objective diagnostics；算法迁移时
  只替换字段，不要回退 strict deadline、repair 或 replay 语义。

## 3. 迁移前 production path 的关键不一致

下面条目按阻塞程度排序。所有 P0 项都必须解决后，才能声称最终算法已经
实现。

### P0-1：没有 UpperSignature 和 upper epoch

当前 `CarrierGuidance` 没有 `UpperSignature`，也没有把 upper-epoch
immutable data 与每节点 data 分开。`attach_carrier_guidance()` 对每个普通
child 重新进入完整旧流程，而 `carrier_rollout()` 又会把整份 guidance
直接冻结复用 8 步。

直接后果：

* 无法证明 free robot `Move/Wait` 后 PairCost、tau、priority、D 逐位不变；
* 无法在 Lift/Drop 后只修 custody/ready/rho；
* 无法在 loaded Move 后可靠地强制重建完整 upper guidance；
* 无法用同一个 upper layout 跨不同 robot configurations 复用 PairCost。

必须新增：

```cpp
struct UpperSignature {
  std::vector<int> target_pos;
  std::vector<int> anon_pos;  // grounded + carried anonymous, sorted
};
```

`make_upper_signature(X)` 必须：

* target 使用 `X.target_pos`；
* grounded anonymous 使用 `X.anon_occ`；
* 每个 `X.kappa[r] == KAPPA_ANON` 再加入 `X.robots[r]`；
* 不记录 free robot、carrier id、grounded/carried bit、rho 或 ancestry。

建议把以下四项放进一个 immutable upper-epoch snapshot，由同 U 的节点共享：

```text
PairCost table
tau_guide
target_priority
ShelfTaskGraph D
```

每节点单独保存：

```text
custody
ready_tasks
rho_task_id
rho_ready_index
preferred robot order
```

### P0-2：`solve_tau()` 混合了 guidance matching 与 admissible h

`carrier_guidance.hpp:804-1019` 的 `solve_tau()` 同时承担：

* admissible LB matrix；
* parent tau hysteresis；
* carried target lock；
* settled target lock；
* tau taboo；
* execution price；
* guidance tau；
* `h_out`。

这违反最终设计的两个独立 API：

```text
solve_tau_guide(pair_cost_matrix)
solve_tau_lb(X)
```

当前 guidance tau 还会读取不属于 U 的信息：

* `carried` 和 `settled`：`carrier_guidance.hpp:822-827`；
* parent assignment：`carrier_guidance.hpp:872-885`；
* taboo：`carrier_guidance.hpp:888-895`；
* carried/settled locks：`carrier_guidance.hpp:904-933`；
* execution price：`carrier_guidance.hpp:935-950`。

必须修改：

1. 保留当前 wall-distance/Lift-Drop lower bound 的合理部分，迁入
   `solve_tau_lb(X)`，只用于 node creation 的 `h_shelf_LB`。
2. 新建 `solve_tau_guide(PairCostTable)`，primary objective 只读
   `PairCost(U,b,g)`。
3. `tau_guide` 的 exact tie order 为：

   ```text
   total PairCost
   moved-away-eligible-count
   assignment-vector by target id
   ```

4. 删除 parent hysteresis、carried/settled lock、taboo 和 robot execution
   price 对 `tau_guide` 的影响。
5. `h_shelf_LB` 只在新 node 初始化时加一次；guidance refresh、rewire 和
   rollout reattach 绝不能再次修改 h。

### P0-3：single-root Task-BR-PIBT PairCost 完全缺失

代码库中没有以下任何最终设计结构或 API：

```text
PairPlan
PairCostTable
compile_single_root_task_br_pibt()
pair_cost()
PairCacheKey(UpperSignature,b,g,version)
```

当前 tau primary cost 只是 wall distance + operation LB；blocker
displacement、anonymous movement、episode manipulation 和 bounded rollout
都没有进入 guidance matching。

必须实现：

```text
PairCost(U,b,g):
  U_hat = U
  建立 rollout-local AbstractShelfToken
  open_episode = none

  while b != g and budget remains:
    D = CompileTaskBRPIBT(U_hat, roots={b->g}, single_root=true)
    ready = ReadyAbstractTasks(D,U_hat)
    m = deterministic highest-ranked ready task
    if no m:
      stalled = true
      break
    按 shelf token 计算 Lift/Drop episode
    abstractly apply adjacent m.from -> m.to
    累加 alpha / gamma / delta

  未到达时加 finite residual + finite stall/truncation penalty
```

关键实现约束：

* 不能读取 robot positions、robot 数量、lower distance、rho、custody、
  execution price 或 parent history；
* single-root 模式下，其他 target blockers 没有自己的 terminal objective，
  candidate ordering 不得偷偷读取 production `tau_guide`；
* anonymous token 只在一次 rollout 内存在，初始编号由 sorted anonymous
  cells 决定；
* compiler failure、zero-empty cycle 和 budget exhaustion 都返回有限 cost；
* 只有 eligibility 或 wall component 能令 pair 为 INF；
* cache 必须有容量上限/LRU，并把 cost version/weights 纳入 key 或 engine
  version，不能由 cache warm-up history 改变结果。

### P0-4：joint Task-BR-PIBT compiler 完全缺失

当前 `build_guidance()` 通过 least-blocking path 扫描、`ObjectiveOption`
套餐、claims ledger 和 resolver 生成任务。相关 production 代码位于：

* `carrier_guidance.hpp:180-541`；
* `carrier_guidance.hpp:1288-1657`。

这不是 shelf-level recursive displacement。它没有：

```text
reserved_shelf_effect
reserved_destination
recursion_stack
exact effect index
dependency graph
root-level transactional DFS
paused_roots
rotation candidates
reverse-topological demand propagation
```

必须用一个 joint compiler 替换整条 ObjectiveOption 管道：

```text
CompileJoint(U, tau_guide, target_priority):
  roots = all targets with position != tau_guide[target]
  stable-sort roots by priority desc, target id
  root-level bounded DFS
  ResolveShelf recursively displaces blockers
  return best complete/partial graph + paused roots + rotations
```

`ResolveShelf()` 每个 candidate 必须 transactionally snapshot/restore graph
与 reservations。blocker 失败后 requester 必须继续试下一个 candidate。
root-level `forced_first_to` 只能约束 root 自己的第一步，不能传给 blocker。

完成 graph 后必须做 reverse-topological root-demand propagation：

```text
requesting task.roots
  -> every predecessor.roots
priority(task) = max priority(root in task.roots)
```

否则高优 root 只合并到内部 shared node，而真正 ready 的最深 blocker leaf
仍保持低优，这是容易出现的隐蔽错误。

### P0-5：当前 task 不是相邻 exact shelf effect

`ManipulationTask` 当前允许：

* serve task 的 `to = terminal tau`：
  `carrier_guidance.hpp:1393-1405`；
* clear task 的 `to = -1`：
  `carrier_guidance.hpp:1412-1487`；
* `to_committed`/advisory 两种语义；
* TaskId 只含 `(shelf,from)`：
  `carrier_guidance.hpp:102-152`。

这与最终定义直接冲突。必须改为：

```cpp
struct ShelfSelector {
  enum class Kind { TARGET, ANON_AT_EPOCH_CELL };
  Kind kind;
  int value;
};

struct TaskId {
  ShelfSelector shelf;
  int from;
  int to;
  // exact equality; hash 只用于 unordered container
};

struct ShelfTask {
  TaskId id;
  std::vector<RootDemand> roots;
  int priority;
};
```

强制 invariant：

```text
from != to
from/to traversable
from 与 to 相邻
to 永不为 -1
TaskId == exact (shelf,from,to)
```

相同 exact effect 才能合并。以下必须作为 conflict 触发 backtracking：

```text
same shelf/from, different to
different shelves, same destination
same shelf reserved for two effects
```

当前 `uint64_t id` 既不是 exact tuple，又存在理论 hash collision；不能继续
作为跨 snapshot identity。

### P0-6：没有 dependency graph，也没有 ReadyTasks 过滤

当前 `g.tasks` 全部投影成 `g.requests`：

```cpp
for (const auto& t : g.tasks)
  g.requests.push_back(CarrierRequest{t.from, t.priority});
```

位置：`carrier_guidance.hpp:1647-1650`。

随后所有 requests 都可能进入 rho。代码没有 predecessors/successors，
也无法表达“只有最深 blocker leaf 当前可执行”。这会把 robot 派往仍被
占用的内部 dependency node。

必须新增 `ShelfTaskGraph`：

```cpp
struct ShelfTaskGraph {
  std::vector<ShelfTask> tasks;
  std::vector<std::vector<int>> predecessors;
  std::vector<std::vector<int>> successors;
  std::vector<int> paused_roots;
  std::vector<RotationCandidate> rotations;
};
```

`ReadyTasks(D,X,custody)` 必须同时检查：

```text
zero physical predecessor obligation
from 仍由指定 shelf 占据
to 当前 upper cell 为空
shelf 未被 unrelated carrier 持有
task 未被其他 custody 占用
```

只有 grounded ready tasks 可以进入 free-robot rho；carried exact
continuation 直接绑定当前 carrier。

### P0-7：`ACTIVE_TARGET_CAP` 静默丢弃 roots

`carrier_guidance.hpp:1328-1365` 在 unfinished targets 超过阈值时只保留
最多 64 个 active roots。

最终设计要求 joint compiler 接收所有 unfinished roots。预算不足可以返回
partial graph，并把未编译 roots 显式放进 `paused_roots`，但不能在 compiler
入口前静默截断。

必须删除 production `ACTIVE_TARGET_LIMIT/ACTIVE_TARGET_CAP` 语义。runtime
guard 应放在 recursion/backtrack budget 内，并保留可观测的
`joint_paused_roots` stats。

### P0-8：one-empty 仍有专用 vacancy BFS

`carrier_guidance.hpp:1367-1375` 先计算 `n_vacancies`；
`carrier_guidance.hpp:1420-1445` 在 `n_vacancies == 1` 时运行专用 BFS，
再把 vacancy-adjacent shelf 编译成 committed task。

最终设计明确禁止 one-empty 特判。相同 generic recursion 应自然生成：

```text
C:2->3 -> B:1->2 -> A:0->1
```

必须删除 `n_vacancies == 1` production branch。测试仍应覆盖 one-empty，
但 assertion 应证明它来自通用 recursion，而不是检查特殊分支。

### P0-9：rho 匹配的是旧 task pool，且先截断再 Hungarian

当前 rho：

* 按 legacy `priority` 和 `depth` 排序：
  `carrier_guidance.hpp:1747-1755`；
* 当 tasks 多于 free robots 时，只把一个最高 `task_priority` task 旋转进
  最后一个 slot：`carrier_guidance.hpp:1764-1776`；
* 然后只对前 `free_left` 个 rows 做 Hungarian：
  `carrier_guidance.hpp:1800-1834`；
* parent continuity 依赖旧 task pool index 和 `(shelf,from)` hash。

最终 rho 必须：

1. 只接收 grounded ready tasks；
2. 跨 snapshot identity 使用 exact `TaskId`；
3. `rho_ready_index` 每次由当前 graph 重新解析；
4. 若 ready tasks 多于 free robots，严格按 priority cutoff：

   ```text
   高于 cutoff 的 task 不能丢
   cutoff 同级 task 先用 approach distance 决定选谁
   低于 cutoff 本节点不服务
   ```

5. lexicographic objective：

   ```text
   task priority
   selected-task approach quality
   beta * robot-to-from distance
   exact TaskId switch penalty
   stable robot-id/TaskId tie
   ```

   这里的层级是强契约：TaskId switch penalty 只能在 priority 和 approach
   distance 之后生效，不能借“粘滞”推翻优先级截断或 cutoff 同级的距离
   选择。旧
   `dd_objective_priority_integration.farther_root_owns_the_frontline_slot`
   所保护的“高优 mission 获得稀缺 assignment row”行为必须迁移到这个
   ready-only cutoff 契约；删除 reserved-slot 实现不等于删除该行为。

6. IDLE 使用 `nullopt`，不能与 index、hash 或 sentinel TaskId 混用。

### P0-10：custody 不是由真实 transition 恢复

当前 `recover_inflight_custody()`：

* 只要新状态 robot loaded，就可能从任意 `source->rho_task` 推断 custody：
  `carrier_guidance.hpp:661-685`；
* 不验证实际执行的是 Lift、Wait、Move 还是 Drop；
* committed destination 失效时直接改成扫描到的第一个 vacancy：
  `carrier_guidance.hpp:686-702`；
* custody 用 `id == 0` sentinel；
* 旧 TaskId 会跨多次 loaded Move 一直保持到 Drop。

这会违反以下最终语义：

```text
custody 只能从紧邻真实 transition anchor 恢复
assigned-ready Lift 把上一拍 rho 的 exact TaskId 转入 custody
loaded Wait 只保持仍有效的同一 exact TaskId
loaded Move 完成或使旧 TaskId 失效，Recover 阶段绝不绑定 continuation
ReadyTasks 仍暴露刚完成 predecessor 的 held shelf continuation candidate
只有 BindReadyContinuations 阶段可为开启新 upper epoch 的 loaded Move
carrier 建立新 exact continuation TaskId
Drop 清 custody
forced/unassigned Lift 产生 loaded-but-unbound nullopt
```

必须将 attach 接口改为接收：

```cpp
struct TransitionContext {
  PhysicalState previous_X;
  const CarrierGuidance* previous_guidance;
  std::vector<Op> executed_joint_ops;
};
```

并使用：

```cpp
std::vector<std::optional<Custody>> custody_by_robot;
```

禁止：

* 从没有真实 edge 的旧 guide 手工注入 custody；
* 因 task vector index 相同就延续 custody；
* 在同一旧 TaskId 内重写 `to`；
* forced non-ready Lift 后伪造一个 assignment。

attach 必须明确分成三个有唯一职责的阶段，不能把它们重新揉进一个启发式
恢复函数：

```text
phase 1 RecoverCustodyBindingsFromActualTransition:
  preserve valid loaded-Wait custody
  complete/invalidate loaded-Move, Drop and deviation custody
  transfer previous rho binding on assigned-ready Lift
  NEVER bind a loaded-Move continuation

phase 2 ReadyTasks:
  evaluate the physical ready predicate
  keep the held shelf that just completed its predecessor visible as a
  continuation candidate

phase 3 BindReadyContinuationsToCurrentCarriers:
  the ONLY continuation-binding site
  eligible only when that carrier's last action was the loaded Move that
  opened the current upper epoch
```

### P0-11：loaded carrier 执行绕过 one-step task

`funcPIBT()` 当前 loaded target 的主要行为是：

* 朝 terminal `tau` 或 cached path head 持续移动；
* 到 tau 后 Drop；
* 或按 `target_park/parking_cell` 移动；
* committed old task 也可以是多格 destination。

位置：`tapf_planner.cpp:1464-1559`。

最终行为必须改为：

```text
loaded + exact custody:
  首选一步 Move 到 custody.to
  Wait/Drop/other Move 仍保留为 completeness fallback

loaded + new exact continuation:
  不 Drop，直接执行新相邻 task

loaded + unbound:
  首选原地 Drop
```

不能让 loaded target 绕过 graph 直接向 terminal tau 连续行走。每个 loaded
Move 都完成一个 one-step task、改变 U，并触发新 upper epoch。

### P0-12：upper invalidation 条件与最终语义相反

`attach_carrier_guidance()` 目前：

```cpp
target_drop_boundary = parent target loaded && child free;
preserve_tau = has_parent_tau && !reguide && !target_drop_boundary;
```

位置：`tapf_planner.cpp:172-186`。

这意味着 loaded Move 改变 shelf coordinate 后仍倾向保留 parent tau；只有
target Drop 才释放。最终设计恰好相反：

```text
free Move/Wait  -> U 不变，复用 PairCost/tau/priority/D
Lift            -> U 不变，复用 upper epoch
Drop            -> U 不变，复用 upper epoch
loaded Move     -> U 改变，重取 PairCost/tau/priority/D
```

此外，当前 robot-only node 仍会因以下历史/robot 信息改变 shelf guidance：

* execution price；
* no-progress aging；
* tau taboo/reguide；
* option hysteresis；
* rho taboo；
* path-cache inertia；
* carried/settled lock。

这些都必须从 `U -> PairCost -> tau -> D` 链中移除。

### P0-13：macro rollout 冻结整份 guidance 8 步

`tapf_planner.cpp:94` 定义 `GUIDANCE_REFRESH_STEPS = 8`；
`tapf_planner.cpp:1953-1962` 在非 refresh step 直接移动上一节点
`guide`，没有重新计算 custody、ready 或 rho。

这会跨 Lift、Drop 和 loaded Move 使用过期 task/custody，违反 upper-epoch
规则。必须改为每个 primitive rollout step 都执行 lightweight attach：

```text
same U:
  复用 PairCost/tau/priority/D
  恢复 custody
  重算 ready/rho/order

changed U:
  新 PairCost/tau/priority/D
  再恢复 continuation、ready/rho/order
```

`carrier_rollout()` 返回值必须增加：

```text
terminal_guidance
vector<TransitionStep> transition_trace
```

macro child 应直接安装 terminal guidance，不能再拿 macro 起点作为 distant
parent 调一次普通 attach。

### P0-14：现有 macro adjacency 无法支持正确 rewire

当前结构：

```cpp
std::set<TAPFNode*> neighbor;
std::map<std::pair<const TAPFNode*, const TAPFNode*>, MacroEdge> macro_edges;
```

问题：

1. 同一 `(from,to)` 只能保存一个 macro trace；
2. ordinary edge 与 macro edge 无法作为不同 candidate 区分；
3. duplicate macro endpoint 在 `CLOSED` 已存在时不会登记 edge：
   `tapf_planner.cpp:844-874`；
4. `rewrite()` 只遍历 neighbor node：
   `tapf_planner.cpp:1126-1158`；
5. `get_edge_cost(from,to)` 只要 map 中有 macro entry 就采用它，无法知道
   当前 relaxation 实际选择哪条 edge；
6. plan extraction 同样按 `(parent,node)` 查 map：
   `tapf_planner.cpp:1027-1049`；
7. guidance stale rebuild 只从当前 parent 直接 attach，不会沿选中的 macro
   primitive trace 重放；
8. child 先从 OPEN 弹出时，不会先递归刷新 stale parent chain。

必须引入 immutable edge record：

```cpp
struct TransitionStep {
  PhysicalState previous_X;
  std::vector<Op> ops;
  PhysicalState next_X;
};

struct SearchEdge {
  TAPFNode* to;
  double physical_cost;
  std::vector<TransitionStep> transition_trace;
};

using SearchEdgeHandle = std::shared_ptr<const SearchEdge>;
```

节点保存：

```text
outgoing_edges
parent
incoming_edge
```

所有 normal/macro successor 必须在 CLOSED duplicate 判断前构造并登记
candidate edge。rewrite 必须遍历 edge records，并原子更新：

```text
parent
incoming_edge
g
f = g + h
guidance_stale
OPEN status
```

`EnsureGuidanceFresh(node)` 必须先递归刷新 parent，再从
`incoming_edge.transition_trace` 逐拍调用 transition-aware attach。refresh
不得修改 node 的 physical key、g/h/f 或 frozen `constraint_order`。

从 `std::set<TAPFNode*>` 迁移到 edge records 时还必须显式保留确定性顺序。
对 zero-shelf 实例，normal edge 的注册顺序和 rewrite/neighbor traversal
顺序必须逐位复现旧 set-based LaCAM-TAPF；不能仅保证 successor 集合相同。
建议给 `SearchEdge` 定义稳定的 registration sequence 或与旧 node ordering
等价的 comparator，并让测试 #27 同时审计 guidance 数值、计划和 edge
遍历顺序。

### P0-15：旧 history-dependent guidance 仍在 production

以下机制最终设计明确要求删除，但当前都在真实 execution path：

* `compute_execution_prices()`：
  `carrier_guidance.hpp:1177-1268`；
* parent tau hysteresis / carried/settled locks / tau taboo；
* `ObjectiveOption`、claims、resolver、yield；
* `target_park` / `parking_cell`：
  `carrier_guidance.hpp:1675-1737` 和 `1851-1887`；
* previous-path inertia：
  `carrier_guidance.hpp:706-756`、`1077-1082`；
* wait-for cycles 和 rho taboo：
  `carrier_guidance.hpp:1098-1175`、
  `tapf_planner.cpp:210-261`；
* revisit/no-progress `reguide`：
  `tapf_planner.cpp:935-945`；
* global futile-Lift memory/cooldown：
  `tapf_planner.cpp:1326-1369`、`1563-1576`；
* old Objective diagnostics 和 compile-time ablations。

这些机制不只是 dead code；它们实际改变 tau、task pool、rho、robot order
或 loaded-carrier action。迁移完成后必须从 production 数据结构、调用点、
stats、probes、tests 和 CMake flags 一起删除，不能保留 runtime legacy
fallback。

### P0-16：当前 path/task 构造会读取 robot 距离

`compile_option()` 在 option score 中加入 nearest free robot distance：

`carrier_guidance.hpp:1503-1511`。

随后 Objective resolver 和 execution-price repair 可以据此改变 selected
package 甚至 tau。即使删除 `compute_execution_prices()`，只要这个
robot-distance option score 仍参与 upper guidance，仍不满足
“shelf-goal assignment 与 shelf dependency 只读取 U”。

Task-BR-PIBT candidate ordering、PairCost、tau 和 D 中不能出现
`LowerDist`。`LowerDist` 只能在 rho 和 lower-deck Carrier-PIBT 使用。

### P1-1：priority 更新时机和来源不对

当前 `CarrierEngine::base_priority` 只在 pass 第一次 attach 时按 wall
distance 冻结：`tapf_planner.cpp:278-280`。之后
`objective_no_progress` 会沿每个 search node 增长，包括 robot-only、
Lift 和 Drop：`tapf_planner.cpp:282-378`。

最终第一版应采用：

```text
priority rank = PairCost(b,tau[b]) descending, target id tie
```

并只在 upper epoch 改变时更新。建议第一轮先不实现 starvation age；如果
以后加入：

* 只能在 loaded Move 开启新 epoch 时更新；
* progress/tau change 时清零；
* joint graph cache key 必须包含 priority vector；
* robot-only、Lift、Drop 不得更新。

### P1-2：path cache 不是 U 的纯函数

`PathCache::get()` 使用 previous cached path 作为 tie bias：
`carrier_guidance.hpp:1077-1082`；默认 lazy invalidation 甚至有测试明确
记录相同当前 X 因 cache history 得到不同 park 结果。

最终 shelf candidate order 必须只由：

```text
U
root/goal
priority input
stable cell id
compiler version/budget
```

决定。若保留 `least_blocking_path()` 作为候选估计 helper：

* production API 删除 `prev_path`；
* 邻居最终 tie 必须按 cell id，而不是 cache/history/RNG；
* helper 不得直接发射 task；
* helper 结果不能替代 recursive displacement。

### P1-3：stats 与 benchmark diagnostics 仍描述旧算法

当前 stats 仍输出：

```text
tau_price_repairs
obj_default_resolutions
obj_reselect_requests
obj_inherit_depth_max
obj_backtracks
obj_yields
tasks_merged  // old (shelf,from)
futile_lift_demotions
path_cache_hits/recomputes
```

应替换为：

```text
upper_epoch_builds
pair_cache_hits
pair_cache_misses
pair_rollout_steps
pair_rollout_truncations
pair_rollout_stalls
tau_guide_changes_on_upper_move
joint_task_nodes
joint_task_edges
joint_shared_effects
joint_effect_conflicts
joint_candidate_backtracks
joint_paused_roots
ready_task_count
rho_repairs
custody_continuations
zero_empty_no_ready
```

`deliverable_ms`、solver runtime、repair/replay 和 deadline diagnostics
继续保留。

## 4. 目标数据结构

建议在 `tapf_planner.hpp` 中完成以下替换。名字可微调，但语义不能弱化。

```cpp
struct UpperSignature {
  std::vector<int> target_pos;
  std::vector<int> anon_pos;
};

struct ShelfSelector {
  enum class Kind { TARGET, ANON_AT_EPOCH_CELL };
  Kind kind;
  int value;
};

struct RootDemand {
  int target;
  int goal;
};

struct TaskId {
  ShelfSelector shelf;
  int from;
  int to;
};

struct ShelfTask {
  TaskId id;
  std::vector<RootDemand> roots;
  int priority;
};

struct RotationCandidate {
  std::vector<TaskId> cycle;
};

struct ShelfTaskGraph {
  std::vector<ShelfTask> tasks;
  std::vector<std::vector<int>> predecessors;
  std::vector<std::vector<int>> successors;
  std::vector<int> paused_roots;
  std::vector<RotationCandidate> rotations;
};

struct PairPlan {
  double estimated_cost;
  int rollout_steps;
  bool reached_goal;
  bool truncated;
  bool stalled;
};

struct Custody {
  TaskId task_id;
  std::optional<int> current_task_index;
  std::vector<RootDemand> roots;
  int priority;
};

struct CarrierGuidance {
  UpperSignature upper_signature;
  PairCostTable pair_cost;
  std::vector<int> tau_guide;
  std::vector<int> target_priority;
  ShelfTaskGraph task_graph;
  std::vector<int> ready_tasks;
  std::vector<std::optional<TaskId>> rho_task_id;
  std::vector<int> rho_ready_index;
  std::vector<std::optional<Custody>> custody_by_robot;
};
```

为了避免同一 U 在大量 robot nodes 上复制大表和 graph，可以把 upper-epoch
四项放入 `shared_ptr<const UpperEpochGuidance>`；但它仍必须是
`CarrierGuidance` 的真实 production 数据，不得形成第二套 pipeline。

旧字段在迁移完成后删除：

```text
CarrierRequest
ManipulationTask 的 advisory/committed 双语义
ObjectiveOption
requests
tasks 旧 pool
rho / rho_task local-index identity
free_goal
parking_cell
target_next
target_park
selected_option
selected_packages
objective_* old ancestry state
plan_bound 在 production guidance 中的旧耦合
```

B1 baseline 若仍需固定 plan，可以保留独立 baseline adapter 数据，但不得
把它重新接回 production Task-BR-PIBT path。

## 5. 逐文件修改位置

### 5.1 `lacam/include/tapf_planner.hpp`

保留：

* `ShelfState`；
* `TAPFNode` physical fields；
* `constraint_order`；
* generic search metrics；
* `TAPFPlanner::solve()` 所需接口。

修改：

1. 用 §4 的新结构替换 `DemandKey/ManipulationTask/ObjectiveOption`。
2. `TaskId` 使用 exact tuple，不再使用 `uint64_t` 作为 identity。
3. custody 改为 `vector<optional<Custody>>`。
4. rho 持久化 exact `rho_task_id`，index 只作当前 snapshot derived view。
5. `TAPFNode::neighbor` 改成 immutable `outgoing_edges`。
6. 节点增加 `incoming_edge`。
7. 删除 authoritative `macro_edges` map。
8. `CarrierRollout` 增加 transition trace 和 terminal guidance。
9. 删除 `futile_clock/lift_futile/lift_on_cooldown`。
10. `attach_carrier_guidance()` 改为 root attach 或紧邻
    `TransitionContext`，删除 `reguide`/distant parent-guide 参数。
11. 增加 `EnsureGuidanceFresh()`、`RegisterOutgoingEdge()` 和统一
    `InstallGuidance()`。

### 5.2 `lacam/src/carrier_guidance.hpp`

可保留：

* `load_solver_weights()`；
* `LowerDist`；
* `DDDistCache` 使用方式；
* Hungarian wrapper；
* occupancy scratch 中可证明与 U/X 一致的部分；
* wall-aware distance helper。

新增：

```text
make_upper_signature()
make_abstract_upper_state()
compile_single_root_task_br_pibt()
pair_cost()
PairCostCache
solve_tau_guide()
solve_tau_lb()
compile_joint_task_br_pibt()
propagate_root_demands()
ready_tasks()
recover_custody_from_transition()
bind_ready_continuations()
match_ready_tasks_by_task_id()
resolve_rho_ready_indices()
```

删除 production 代码：

```text
task_hard_claims()
option_claim_conflict()
merge_objective_tasks() old semantics
resolve_objective_options()
update_objective_progress() old node-step semantics
solve_tau() mixed API
PathCache history bias
waitfor_cycles()
compute_execution_prices()
build_guidance() ObjectiveOption implementation
n_vacancies branch
park/yield/parking placement
legacy request priority/depth truncation
```

实现 compiler 时要特别注意：

* `DDGrid::neighbors()` 的返回顺序不是最终 stable tie contract；必须显式
  sort candidate，最后按 cell id；
* transaction rollback 要恢复 task vector、edges、effect index、
  reservations、paused/rotation 暂存；
* exact shared node 合并后要重新做 predecessor closure propagation；
* zero-empty recursion cycle 只记录 rotation candidate，不返回 infeasible；
* PairCost compiler 与 joint compiler 应共享同一 recursion core，但
  single-root mode 不得读取其他 target 的 tau。

### 5.3 `lacam/src/tapf_planner.cpp`

`attach_carrier_guidance()` 重写为：

```text
1. 验证 transition.previous_X + ops == current X
2. 构造 U
3. U 变化：
     PairCost table
     tau_guide
     target priority
     joint graph
   U 不变：
     复用 parent upper snapshot
4. 从真实 transition 恢复 exact custody
5. 计算 ready
6. loaded Move 后允许 exact continuation direct bind
7. free robots 与 grounded ready tasks 做 rho
8. 解析 current rho indices
9. 刷新 mutable preferred robot order
10. 仅首次 node creation 初始化 h 和 frozen constraint_order
```

`funcPIBT()` 修改：

* free assigned：approach exact task.from，到位且仍 ready 才 Lift；
* loaded bound：首选 exact adjacent task.to；
* loaded unbound：首选原地 Drop；
* idle：只避让当前 ready/custody effect footprint；
* lower-deck recursion 与 completeness fallback 保留；
* 删除 terminal tau drive、park、old path、taboo 和 cooldown。

`solve()` 修改：

* normal successor 在 duplicate lookup 前创建一拍 `SearchEdge`；
* macro successor 同样先登记完整 trace；
* duplicate 也保留 candidate edge；
* new node 安装该 edge 和正确 terminal guidance；
* expansion 前统一 `EnsureGuidanceFresh(S)`；
* plan extraction 沿每个 node 的 `incoming_edge` trace；
* best-effort extraction 使用同一 trace；
* cleanup 统一释放 edge handles。

`rewrite()` 修改：

* 遍历 outgoing edge records，而不是 neighbor nodes；
* 用 edge 自带 physical cost；
* 原子替换 parent + incoming edge + g/f/stale；
* 按现有 OPEN policy reinsert/reprioritize；
* 传播 cost relaxation 和 guidance stale；
* 不允许从旧 incoming macro map 猜 trace。

`carrier_rollout()` 修改：

* 每步 attach；
* 每步保存 `{previous_X,ops,next_X}`；
* loaded Move 立即新 epoch；
* 返回 terminal guidance；
* 删除 `GUIDANCE_REFRESH_STEPS` 的语义。

### 5.4 `lacam/include/dd_planner.hpp` 与 `lacam/src/dd_planner.cpp`

删除或替换旧 probes：

```text
dd_compute_park
dd_least_blocking_path(prev_path)
dd_waitfor_cycle_robots
dd_resolve_objective_options_probe
dd_objective_progress_probe
dd_solve_tau_with_pressure_probe
dd_enumerate_node_successors_reguided
old dd_build_tasks / rho_task index probe
old custody-through-Drop trace contract
```

新增：

```text
dd_upper_signature_probe
dd_pair_cost_probe
dd_tau_guide_probe
dd_tau_lb_probe
dd_compile_joint_graph_probe
dd_ready_tasks_probe
dd_rho_ready_probe
dd_custody_continuation_probe
dd_transition_replay_probe
```

`dd_root_admissible_h()` 应只走 `solve_tau_lb()`。

B0 继续共享 production rollout core；B1 继续只是离线 baseline，不得成为
production fallback。两遍求解、repair、deadline 和 replay adapter 不改
算法语义。

### 5.5 `CMakeLists.txt`

迁移完成后删除：

```text
DD_OBJECTIVE_FORCE_DEFAULT
DD_OBJECTIVE_NO_INHERIT
DD_OBJECTIVE_DROP_SECOND_ROOT
对应三个 ablation executables
```

如需新研究消融，只允许：

```text
A: wall-only PairCost vs rollout PairCost
B: independent roots vs joint compiler
C: ready-only rho vs legacy all-task rho
```

消融不能成为 runtime fallback，也不能改变 production default。

建议同时修复 CTest registration，确保 `ctest` 不再空跑。

### 5.6 benchmark 与工具

`tools/dd_benchmark.cpp`、`benchmark/run_benchmark.py` 和 diagnostics schema
应切换到新 stats。旧 `results_v4_1_final7` 保留为 immutable baseline，
不要覆盖目录。

所有 success 仍必须满足：

```text
deliverable_ms <= 10000
solver_runtime_ms <= 10000
plan_sha256 present
C++ replay PASS
Python validator PASS
```

## 6. protected tests 的迁移

`rules.md` 规定新增 tests 一经创建即 protected。以下测试与最终设计直接
冲突，主实现 agent 不能直接改；必须先由独立 GPT-5.6 Sol/high reviewer
阅读两份设计、代码、旧 test 和 proposed change，并明确 `APPROVE`。

### 6.1 必须替换的旧契约

| 当前测试/组 | 当前保护的旧行为 | 新 expectation |
|---|---|---|
| `test_dd_tasks::robot_placement_flips_tau_guide_goal` | robot 距离可翻转 tau | 同 U 的 tau 逐位相同 |
| `test_dd_tau::hysteresis_is_tie_break_only` | parent tau 参与 guide tie | tau_guide 不读 parent |
| carried/settled/tau-taboo tests | kappa/settled/taboo 锁 guide goal | 只由 U + PairCost 决定 |
| serve/clear task tests | non-adjacent serve、`to=-1` clear | 所有 task 为 adjacent exact effect |
| old TaskId tests | identity 不含 `to` | identity 必含 `(shelf,from,to)` |
| custody-through-Drop tests | 同一 id 跨 Lift/Move/Drop | id 在一步 loaded Move 完成；Drop 清除 |
| one-empty compiler tests | 专用 vacancy branch | generic recursion 生成 chain |
| depth/truncated rho tests | legacy priority/depth 先截断 | ready-only lexicographic cutoff |
| `farther_root_owns_the_frontline_slot` | reserved slot 保证高优 mission 获得稀缺 assignment row | 删除 reserved-slot 机制，但把行为迁移为 ready-only priority-cutoff 测试；cutoff tie 先看 approach distance，switch penalty 不得推翻前两层 |
| Objective-PIBT tests | option/claims/yield/default budget | joint recursive displacement/backtracking |
| Objective ablation tests | 三个旧结构消融 | 删除或换新消融 |
| park purity/sticky park tests | target_park 与 history cache | 无 production park；U-only determinism |
| path inertia tests | ancestry bias | stable cell-id tie |
| wait-for/reguide tests | taboo/reguide API | 相同合法 successor set，不再有该输入 |
| futile-Lift test | global cooldown | ready Lift 与历史无关 |
| frozen rollout test | 8-step guide reuse | 每步 lightweight attach |
| old rewire tests | node-pair macro map 足够 | exact candidate edge trace replacement |
| old objective diagnostics tests | obj/tau-price counters | Pair/joint/ready/rho counters |

### 6.2 应继续保护的测试

以下 tests 的行为目标与最终设计一致，应保留或只做接口适配：

* physical state/hash/canonical anonymous semantics；
* eligible-goal loader、Hall covering 和 terminal semantics；
* settled-pool tau tests 所保护的 eligible/injective global feasibility，以及
  matching 必要时重开 settled target blocker 的行为；只替换其对旧
  settled/carried lock 的机制性断言；
* `apply_ops()` rule table；
* G1 brute-force successor completeness；
* zero-shelf LaCAM-TAPF compatibility；
* admissible h oracle；
* plan repair legality/SOC non-increase；
* two-pass result selection；
* C++ replay/Python validator；
* strict return deadline/finalization classifier；
* output metrics 和 weight parser；
* zero-empty physical rotation legality。

## 7. 必须先写成 RED 的测试

下面 39 项对应最终设计的完整 correctness surface。不能只选容易实现的
子集。

1. 同一 U、不同 free robot positions：PairCost matrix 与 tau 逐位相同。
2. Lift/Drop 不改变 UpperSignature、PairCost、tau、priority、D。
3. loaded Move 改变 UpperSignature，并重新计算 PairCost/tau/D。
4. tau_guide 使用 PairCost；tau_LB 与 PairCost 改动完全隔离。
5. 所有 ShelfTask 相邻，TaskId 精确包含 shelf/from/to。
6. single-root blocker candidate 失败后 requester 尝试下一 candidate。
7. one-empty 不检查 empty 数量也生成完整 dependency chain。
8. 只有 empty-adjacent zero-indegree leaf ready。
9. 两个 roots 请求相同 exact effect 时合并 roots 和 max priority。
10. shared non-ready node 后加入高优 root 时，高优 demand 传播到最深 leaf。
11. 同 shelf/from、不同 to 是 conflict，不能 merge。
12. 不同 shelves 抢同一 destination 触发 joint backtracking。
13. target blocker 固定 tau 下优先尝试替代 displacement，并参考自己的 tau。
14. 高优 root 首选阻碍低优 root 时，最终 exact root effect 确实切到备选。
15. joint compiler 接收全部 unfinished roots，不受 active cap 截断。
16. non-ready internal task 永不进入 rho。
17. carried ready continuation 直接绑定当前 carrier，不参加 Hungarian。
18. 同一 carrier 连续执行多个 one-step tasks，不重复 Lift/Drop。
19. preferred path 中 in-flight task 不被 rho/priority 改写；forced Drop/偏离
    Move 仍在 successor set 且正确清 custody。
20. 同一 anonymous shelf 连续移动两步只计一次 Lift/Drop episode；匿名输入
    排列不改变 PairCost。
21. zero-empty 无 ready 返回有限 PairCost/empty guidance，不判无解。
22. `|G_b|=1` 自然退化为 fixed-goal Carrier-LaCAM。
23. tau_guide 改变不改变 admissible h；mixed TAPF/carrier h 不重复计数。
24. macro rollout 跨至少两次 loaded Move，每步更新 guidance，terminal
    anchor 与逐拍 fresh attach 相同。
25. parent/child 同时 stale 且 child 先出 OPEN：先更新 g/f/OPEN，再递归刷新
    parent，沿 candidate trace 重锚 child；h 和 constraint_order 不变。
26. guidance compiler failure 前后，operator tree 的合法 physical successor
    集合相同。
27. zero-shelf 原 LaCAM-TAPF 逐位一致，包括 edge 注册/rewrite 遍历的旧
    确定性顺序；mixed h 两分量正确相加。
28. 所有返回计划通过 apply_ops、C++ replay 和 Python validator。
29. strict deadline 覆盖 search、cleanup、repair、SOC 和 final replay。
30. production 无 target_park/parking_cell；unbound loaded 首选 Drop；idle
    只读当前 D/ready/custody footprint。
31. 相同 U/priority、不同 ancestry/cache warm-up：candidate order、
    PairCost、tau、D 完全相同。
32. 相同 X/D/ready/previous TaskId，改变旧 revisit/wait-for counters 不改变
    rho，且 production 不再接收这些 counters。
33. ready 且合法的 Lift 在不同历史下首选次序相同，无 global cooldown。
34. custody 严格执行三阶段：Recover 只能保留有效 loaded Wait、完成/失效
    loaded Move/Drop/deviation，并在 assigned-ready Lift 上转入上一拍 rho，
    绝不能绑定 loaded-Move continuation；ReadyTasks 继续暴露 held shelf
    continuation；只有 BindReadyContinuations 可在开启新 upper epoch 的
    loaded Move 后建立新 exact custody。重复 attach/手工旧 guide 不能注入。
35. task vector 重排但 TaskId 不变时不计 switch；index 相同但 TaskId 改变
    时必须计 switch。
36. macro-created node 被 ordinary/另一 macro edge reparent 时，incoming
    trace 完全替换并可逐拍 replay 到 node.X。
37. Lift 后 task vector 重排使原 index 指向其他 effect；loaded Wait 后
    custody exact TaskId 不变，derived index 重解或 nullopt。
38. constraint tree 强制 Lift 未 assigned/non-ready shelf：successor 仍
    存在，robot loaded 且 custody=nullopt，随后走 unbound fallback。
39. `|ready| > |free|` 时，高于 priority cutoff 的 ready task 必获
    assignment row；cutoff 同级先按 approach distance 选择；TaskId switch
    penalty 只在更后层生效，不能推翻 priority/distance。

## 8. 推荐实施顺序

必须遵守 `test -> RED -> implementation -> GREEN -> benchmark ->
regression test -> debug`。

### Phase 0：冻结旧基线并审批 test migration

1. 保留当前 source SHA、旧 binary provenance 和
   `results_v4_1_final7` artifact，不覆盖旧目录。
2. 记录本次 194 C++、80 Python 和 36/68 artifact。
3. 对 §6.1 的 protected test changes 先取得独立 reviewer `APPROVE`。
4. 不修改 benchmark case、timeout、seed、metric 或 success semantics。
5. 正式 release run 必须在同一会话、同机、同 jobs 并行度下重跑候选与
   `results_v4_1_final7` 对应的旧 binary；旧 artifact 只提供历史参照，
   不能替代配对基线。

### Phase 1：UpperSignature 与 tau 分层

先 RED：

* robot-only invariance；
* Lift/Drop invariance；
* loaded Move invalidation；
* robot placement cannot flip tau；
* tau_guide/tau_LB isolation。

再实现：

* UpperSignature；
* `solve_tau_lb()`；
* 临时 shelf-only cost 接通纯 `solve_tau_guide()`；
* 删除 execution price、locks、hysteresis、taboo 对 guide matching 的影响；
* node h 只初始化一次。

### Phase 2：single-root PairCost

先 RED：

* recursive displacement；
* candidate backtracking；
* finite stall/truncation；
* anonymous token episode；
* cache purity/capacity。

再实现：

* exact adjacent task core；
* single-root compiler；
* abstract rollout；
* residual/stall penalty；
* bounded PairCost cache。

### Phase 3：joint compiler 和 dependency graph

先 RED：

* exact shared effect；
* same-shelf/different-to conflict；
* destination conflict；
* requester/backtracking；
* root-level backtracking；
* all roots；
* predecessor root propagation；
* one-empty generic chain；
* zero-empty finite result。

再用 joint graph 替换 ObjectiveOption/claims。

### Phase 4：ready-only rho 与 one-step custody

先 RED：

* ready leaf only；
* TaskId-index separation；
* forced unassigned Lift；
* one-step completion；
* continuation；
* unbound Drop；
* rho exact switch semantics。

再修改：

* transition-aware attach；
* ReadyTasks；
* rho；
* custody；
* `funcPIBT()`。

### Phase 5：SearchEdge、macro 和 rewire

先 RED：

* multi-loaded-Move macro fresh attach；
* duplicate macro edge registration；
* alternate edge reparent；
* stale parent-chain refresh；
* exact trace replay；
* `f == g + h`、constraint_order frozen。

再实现 immutable edge records，删除 macro map。

### Phase 6：删除旧语义

确认新 production path GREEN 后一次性清除：

```text
ObjectiveOption/claims/resolver
execution price
one-empty branch
old TaskId
active root truncation
target park/parking
path inertia
wait-for/taboo/reguide
futile-Lift memory
old diagnostics
old probes
old ablation flags
```

最终 `git grep` 不得同时看到两套 carrier guidance pipeline。

### Phase 7：完整验证和性能调优

correctness gate 全绿后，才调整：

* Pair rollout step/backtrack budget；
* cache capacity/LRU；
* joint candidate/backtrack cap；
* optional starvation age；
* optional zero-empty rotation bundle。

性能调优不能改变 U-purity、TaskId、ready-only rho、successor completeness
或 h admissibility。

验证不能只看成功数。必须在 common-success set 上同时报告相对
`results_v3_strict_return_final` 与 `results_v4_1_final7` 的 makespan/SOC
几何比和逐例改善/持平/恶化；priority-first 在 `|ready|>|free|` 时仍有
重现 farthest-first/LPT 式 SOC 税的风险，若出现系统性恶化，应先检查 base
priority 方向和 upper-epoch starvation age，而不是恢复 node-level 抢占
或 reserved-slot 机制。

## 9. 迁移期完成判据（现已满足）

只有以下全部成立，才能认为迁移完成：

1. production source 中存在并实际调用：

   ```text
   UpperSignature
   PairPlan/PairCost
   solve_tau_guide
   solve_tau_lb
   joint Task-BR-PIBT
   ShelfTaskGraph
   ReadyTasks
   exact TaskId rho
   transition-aware custody
   SearchEdge trace
   EnsureGuidanceFresh
   ```

2. production source 中不再存在：

   ```text
   ObjectiveOption
   compute_execution_prices
   preserve_tau
   n_vacancies == 1 compiler branch
   ACTIVE_TARGET_CAP root truncation
   target_park / parking_cell
   prev_path inertia
   wait-for taboo / reguide
   lift_futile / cooldown
   GUIDANCE_REFRESH_STEPS freeze
   authoritative macro_edges map
   ```

3. §7 的 39 项 tests 全绿，且 protected test 变更有独立审批记录。
4. 原 LaCAM-TAPF tests 在 zero-shelf 情况逐位兼容。
5. 所有 success plans 通过 C++ 与 Python 独立 replay。
6. 完整固定 68-case benchmark 按“配对不回退 + 绝对下限”这一条规则验收：

   ```text
   candidate 与旧 results_v4_1_final7 binary 在同一会话、同机、同 jobs
   并行度下配对重跑
   candidate full success >= paired baseline full success
   candidate small success >= paired baseline small success
   absolute floor: full >= 34/68
   absolute floor: small >= 34/36
   paired baseline 若达到 full 36/68，则 candidate full >= 36/68
   paired baseline 若达到 small 36/36，则 candidate small >= 36/36
   every successful deliverable_ms <= 10000
   every successful solver_runtime_ms <= 10000
   ```

   历史 nominal 36/68 和 36/36 只是期望值，不是脱离配对基线的第二条
   独立门槛。
7. 对两个历史基线都报告 common-success makespan/SOC 几何比和逐例
   改善/持平/恶化；不能用固定成功数掩盖系统性 SOC/makespan 回退。
8. 报告新 Pair/joint/ready/rho/custody diagnostics。
9. 最终 diff 没有 parallel planner、runtime legacy fallback、
   benchmark-specific hack 或死代码。
10. 按 `rules.md` 生成中文最终汇报网页，并由独立 GPT-5.6 Sol subagent
    review 内容、数据、链接和可读性。

## 10. 历史实施切入点

第一轮实际编码不应从调参数或修 benchmark 开始，而应按以下顺序切：

1. 在 `tapf_planner.hpp` 定义 exact 新数据结构和 transition context。
2. 在 `carrier_guidance.hpp` 抽出纯 `make_upper_signature()` 与
   `solve_tau_lb()`。
3. 写 robot-only/Lift/Drop/loaded-Move RED tests。
4. 把 `attach_carrier_guidance()` 改成按 U 复用/失效，但先用简单 shelf-only
   guide cost 保持编译。
5. 写并接入 single-root PairCost。
6. 写并接入 joint compiler/ReadyTasks。
7. 最后切 rho、custody、funcPIBT、macro/rewire。
8. 新路径全绿后删除旧 Objective 代码；不要长期保留双路实现。

如果先继续优化当前 `ObjectiveOption`、execution price、park 或 cooldown，
即使旧 benchmark 上升，也不会让代码更接近 `new.md` 的最终算法。

## 11. 实现闭环与最终调试结论

### 11.1 关键实现

* `UpperSignature` 只含 target 位置和 canonical anonymous occupancy；
  robot-only transition 复用 PairCost、tau 与 graph。
* PairCost 使用 shelf-only bounded rollout、dense arrays、可复用 scratch、
  256-entry upper-epoch LRU 和 exact lexicographic matching；四邻接候选用
  等价的固定 insertion sort 消除热路径排序开销。PairCost 与 joint
  compiler 共享同一个 templated recursion core；Pair context 仅省略不被
  PairPlan 消费的 graph materialization，并以 direct index、
  generation-stamped reservations 和复用 scratch 加速。
* lazy PairCost matrix 使用可验证 branch-and-bound：prefix cost 是完整
  PairCost 的 lower bound；当前 Hungarian edges 先全部求精；每条未求精
  edge 再计算 forced injective assignment lower bound，`<=` 当前精确最优值
  时必须求精。终止后所有 primary-optimal edges 均 exact，因此 secondary/
  tertiary tie 也精确。`PairPlan.exact=false` 不得进入 priority 或完整
  rollout diagnostics，selected edge 有运行时 exact 断言。
* joint compiler 使用 undo-log transaction、exact
  `(shelf, from, to)` effect、shared roots、反向 demand propagation 和
  root-level backtracking。每个 root option 有 256 次局部递归窗口，
  epoch 有 512 次 branch cap；vacancy 大于 2 时另有 768 次累计递归 cap。
  success vector 相同时，先比较 aggregate remaining distance；随后 0/1
  vacancy 只按 priority 比较 root completion，至少 2 vacancy 才比较完整
  逐根 residual，最后比较 work。这样同时避免单空位局部追逐与多空位反向
  复位。
* ready-only rho 按 exact TaskId 匹配。custody 只能由真实 Lift、loaded
  Wait 或 loaded Move 后的新 epoch anchor 恢复；loaded-but-unbound 首选
  Drop。
* roomy layout 不自动绑定 immediate reverse，dense layout 可绑定；反向
  Move 始终保留在 physical successor set。
* priority commitment 先按新 tau 过滤：singleton fixed-goal self move 和
  roomy multi-goal root 被排除；其余 group 在严格多数、`active_roots >
  vacancies && active_roots >= 2 * vacancies`，或与上一轮 collective
  commitment 相交时整组续约，否则通常只续约最高 root。
* macro rewire 使用完整 incoming edge trace 原子更新；guidance refresh
  不修改 `g/h/f`。strict deadline 覆盖 cleanup、repair、SOC 与 final replay。

### 11.2 调试中固化的回归

主要问题均先由 regression test 复现，再修 implementation：

1. exact reverse 被 continuation 自动绑定后产生两格震荡；
2. 过度宽松的跨 epoch priority 续约造成 roomy multi-goal 偏置；
3. joint recursion cap 若只全局累计，会截断 one/two-vacancy 长链；若每个
   option 无条件重置，又会在 roomier e3/e8 case 上放大搜索；
4. PairCost 每步对最多四个邻居调用 `stable_sort`，在 full-suite 热状态下
   把贴线实例推近 10s；
5. Pair/joint 候选语义统一后，联合候选在 aggregate residual 平局时偏好
   少一个 shared task，把已搬开的 blocker 立即反向复位；新增 2×2 RED
   regression 后发现，完整逐根 residual 若无条件使用又会让单空位长链
   追逐局部距离。最终顺序是 aggregate first，再做 density-aware tie：
   0/1 空位只比较 root completion，至少 2 空位比较完整逐根 residual。
   独立 aggregate test 与 two-vacancy 10s regression 同时保护两侧边界；
6. 128-step Pair rollout 在最后一步到 goal 时曾被误标 truncated；现在每次
   abstract shift 后立即检查 goal；
7. zero-empty cycle 曾只检测而不写 production graph；现在 canonical
   rotation 通过 undo transaction 记录，失败 branch 不泄漏；
8. duplicate/reparent、loaded-unbound、tau 改变、shared-effect roots 与
   deadline cleanup 的边界均已有独立测试。

最终策略是“局部窗口 + density-aware 累计预算 + density-aware candidate
tie”。它把关键 `e3 B seed1` 从约 7.2s 降至 release 配对运行的 5.616s，
同时保持 222 项 C++ 测试全绿。

### 11.3 Benchmark 与剩余差异

paired baseline 的 36 个成功例全部保留，candidate 另外解出两个
`h20w20_a40_e100_R1` seed。common-success 36 例中，makespan 为 33
改善、1 持平、2 恶化；weighted SOC 为 32 改善、3 恶化、1 个 `0/0`
平凡实例持平且不进几何均值。
代表性改善：

```text
h6w10_a6_e15_R1_seed0   makespan 791 -> 109, SOC 1562 -> 236
h10w10_a12_e3_B_seed0   makespan 749 -> 123, SOC 1111 -> 277
h4w10_a5_e10_R1_seed0   makespan 1060 -> 174, SOC 2059 -> 344
```

已知回退：

```text
h10w10_a1_e1_B_seed0_pool    makespan 104 -> 130, SOC 174 -> 182
h6w10_a6_e1_B_seed1_pool     makespan 240 -> 258
h10w10_a12_e3_B_seed1_pool   SOC 2203 -> 2209
h8w10_a10_e20_B_seed1_pool   SOC 482 -> 493
```

这些是后续质量优化目标，不是 correctness 或 release blocker。任何优化仍
须保持 shelf-only PairCost、admissible `tau_LB`、exact TaskId、
successor completeness 和相同 benchmark protocol。

相对 `results_v3_strict_return_final`，candidate 多解出 5 例；common-33
makespan 为 30 改善、1 持平、2 恶化，几何比 `0.5206`；weighted SOC 为
29 改善、3 恶化、1 个 `0/0` 持平，几何比 `0.5343`（`0/0` 不进入
几何均值）。因此 §9 要求的两个历史基线对照均已覆盖。
