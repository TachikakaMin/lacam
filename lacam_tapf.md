下面是一个 **fast LaCAM-style TAPF** 的伪代码版本。这里 assignment 只是 **guidance**，不作为永久约束。

```text
Algorithm Fast-TAPF-LaCAM
Input:
    graph G=(V,E)
    agents A = {1,...,n}
    start configuration S
    task/goal set T = {t1,...,tn}

Output:
    TAPF solution or NO SOLUTION

Initialize:
    root constraint node Cinit = empty
    Ninit = {
        config: S,
        parent: null,
        tree: queue(Cinit),
        order: InitAgentOrder(S, T),
        prefixCost: 0,
        assignment: Hungarian(CostMatrix(S, T))
    }

    OPEN = stack()
    EXPLORED = map from configuration to node

    OPEN.push(Ninit)
    EXPLORED[S] = Ninit

while OPEN is not empty:

    N = OPEN.top()
    X = N.config

    if IsTAPFGoal(X, T):
        return Backtrack(N)

    if N.tree is empty:
        OPEN.pop()
        continue

    C = N.tree.pop()

    # Expand low-level local constraints
    if depth(C) < n:
        i = N.order[depth(C)]
        v = X[i]

        for u in neigh(v) ∪ {v}:
            Cnew = C plus local constraint (i must be at u next step)
            N.tree.push(Cnew)

    # Compute assignment only as guidance
    M = CostMatrix(X, T, N.assignment)
    tau = Hungarian(M)

    # Generate next configuration using PIBT-like one-step planner
    Xnew = GetNewConfig(
        currentConfig = X,
        localConstraints = ConstraintsOnPath(C),
        assignment = tau
    )

    if Xnew == FAIL:
        continue

    newCost = N.prefixCost + StepCost(X, Xnew)

    if Xnew not in EXPLORED:
        Nnew = {
            config: Xnew,
            parent: N,
            tree: queue(Cinit),
            order: GetAgentOrder(Xnew, T, tau),
            prefixCost: newCost,
            assignment: tau
        }

        EXPLORED[Xnew] = Nnew
        OPEN.push(Nnew)

    else:
        E = EXPLORED[Xnew]

        # Optional cost relaxation
        if newCost < E.prefixCost:
            E.prefixCost = newCost
            E.parent = N
            E.assignment = tau

        # LaCAM-style reinsert
        OPEN.push(E)

return NO SOLUTION
```

其中 cost matrix 可以写成：

[
M[i,j] = \operatorname{dist}(X[i], T[j])
]


这里的重点是：**assignment 不进入判重 key，也不作为 pruning 条件**。判重仍然主要用 configuration (X)。这样它更像原始 LaCAM：high-level 搜 configuration，low-level lazy 加 one-step positive constraints，PIBT 负责生成 promising successor。
