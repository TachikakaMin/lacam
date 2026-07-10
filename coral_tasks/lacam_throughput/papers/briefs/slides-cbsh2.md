# Improved Heuristics for Multi-Agent Path Finding with Conflict-Based Search

简短介绍：CBSH2 在 CBS 中用智能体两两依赖构造 DG/WDG 启发式，把最小顶点覆盖（含边权）作为可采纳下界来排序搜索节点。幻灯片报告 h_WDG ≥ h_DG ≥ h_CG，额外开销较小，整体性能表现为 WDG 优于 DG、DG 优于 CG。它为 LaCAM 吞吐优化提供了把冲突依赖转成轻量下界与优先级信号的依据，可用于拥堵/瓶颈场景下的候选选择和冲突处理。

- 原论文：[arcs-group/Slides_cbsh2.pdf](../arcs-group/Slides_cbsh2.pdf)
- 来源：arcs-group
- 类型：pdf
- 外部链接：[https://jiaoyang-li.github.io/files/slides/cbsh2.pdf](https://jiaoyang-li.github.io/files/slides/cbsh2.pdf)

## 检索规则

先从 [../README.md](../README.md) 定位到本简介；只有当本简介显示相关且需要细节时，才继续打开原论文确认章节、算法、设计模式或实验观察。
