# A Hierarchical Approach to Multi-Agent Path Finding

简短介绍：HMAPP 将地图划分为区域，用高层区域序列引导、区域内用 ECBS 局部重规划，并通过有向边界对协调跨区移动。实验显示它能扩展到贪心方法可处理的大规模实例，同时路径质量更好，部分大规模场景 makespan 比 Ros-dmapf 少约一半。对 LaCAM 吞吐优化而言，可借鉴其区域分解、边界容量/方向控制和拥塞感知高层路由，把全局冲突压力转化为局部可并行协调。

- 原论文：[arcs-group/Preprint_2021-HPLAN.pdf](../arcs-group/Preprint_2021-HPLAN.pdf)
- 来源：arcs-group
- 类型：pdf
- 外部链接：[https://jiaoyang-li.github.io/files/2021-HPLAN.pdf](https://jiaoyang-li.github.io/files/2021-HPLAN.pdf)

## 检索规则

先从 [../README.md](../README.md) 定位到本简介；只有当本简介显示相关且需要细节时，才继续打开原论文确认章节、算法、设计模式或实验观察。
