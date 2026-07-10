# Nested ECBS for Bounded-Suboptimal Multi-Agent Path Finding

简短介绍：NECBS在ECBS外层搜索中按重复冲突合并代理为meta-agent，并用内层ECBS而非联合状态空间搜索解决组内冲突，保持完备与有界次优。实验在5分钟时限下显示NECBS(MR)成功率高于ECBS(RR)等变体，整体最高为71.8%。这为LaCAM吞吐量优化提供了“冲突热点成组、组内快速次优修复、必要时重启”的机制依据。

- 原论文：[arcs-group/Preprint_ChanWoMAPF20.pdf](../arcs-group/Preprint_ChanWoMAPF20.pdf)
- 来源：arcs-group
- 类型：pdf
- 外部链接：[https://jiaoyang-li.github.io/files/ChanWoMAPF20.pdf](https://jiaoyang-li.github.io/files/ChanWoMAPF20.pdf)

## 检索规则

先从 [../README.md](../README.md) 定位到本简介；只有当本简介显示相关且需要细节时，才继续打开原论文确认章节、算法、设计模式或实验观察。
