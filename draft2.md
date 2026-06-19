你先阅读项目代码。

我现在想加入一个agent能拿n个货物，但流程还是按照现在的来，agent分成loaded，unloaded和full

1. 对于full的agent，它的goal set就是现在拥有的所有task的goal set叠加
2. 对于loaded的agent，它的goal set是现在拥有的所有task的goal set叠加，并且再加上其他可选task
3. 对于unloaded的agent，它的goal set是现在所有其他可选task

但是我们现在重新设计cost matrix，因为我们现在的最终目标是throughput，所以我们应该用throughput来作为cost matrix的cost

首先考虑每个agent只能拿一个货物的情况，此时对于loaded的agent，它的cost应该是1除以（现在到goal set中每个goal的距离）

对于unloaded agent，它的cost应该是1除以（现在到task start loc的距离+这个task到最近的它goal的距离）

那么对于agent能拿多个货，它的cost应该是：

1. 如果这个目标点是一个新task的start loc，它的cost应该是(c1+c2+c_circle_new)/(n+1),
n是目前已经拿到的货物数量
c1是从当前点到这个task的start loc的距离
c2是从这个task的start loc到这个task goal set中最近点的距离
c_circle_new是从当前点到每个task的goal set中选一个最近点，然后这些最近点按照x轴从左到右或者从右到左组成一个环的总cost（选最小cost方向），如果只有一个task，这个环的cost是0,两个以上则是环的cost

2. 如果目标点是个已拿到货的goal set中的goal loc，它的cost应该是 (c3 + circle) / n， 此时c3是当前点到这个点的距离

所有实验优先跑smoke test测试，smoke test不应该超过10s。