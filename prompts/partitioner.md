实现 `partitioner.h` 和 `partitioner.cc`。本模块接收 `parser::parse(in_path)` 得到的 `const common::Problem& problem`，构建用于 CTS topology 的自适应层次化 partition tree。不要实现 routing、buffer insertion、writer、evaluator 或 main 逻辑。

## Namespace 和 API

- `partitioner.h` `partitioner.cc`
- 所有 partitioner 相关函数放在 `namespace partitioner`。
- 复用 `common.h` 中已有输入数据结构；只在必要时向 `common.h` 的 `namespace common` 中新增 partition 结果结构。
- 对外暴露一个主 API：

```cpp
common::PartitionTree partitioner::build(const common::Problem& problem);
```

## 输出数据要求

partition 结果必须保留层次结构，不能只输出 flat cluster list。

每个 partition node 至少包含：
- 唯一 node id；
- child partition node ids；
- 当前 subtree 包含的 sink indices；
- bounding box；
- centroid point；
- 是否为 leaf cluster；
- 是否为 single-sink outlier。

root partition 必须包含 `problem.sinks` 中的全部 sinks。

## Adaptive Multi-Gap Partition 规则

从全部 sink indices 开始递归构建 partition tree。

对当前 sink set：
1. 如果 `set size <= min_cluster_size`，停止并生成 leaf cluster。
2. 分别尝试 x-axis 和 y-axis。
3. 对某个 axis，按该坐标排序，计算所有相邻 gap。
4. 用 positive gaps 的 median 作为 `small_gap`；如果没有 positive gap，则该 axis 不可 split。
5. 所有满足 `gap >= min_abs_gap` 且 `gap >= gap_ratio * small_gap` 的 gap 都是 big gap。
6. 用该 axis 的所有 big gaps 一次性把当前 set 切成多个连续 groups。
7. multi-way split 后，每个 normal group 至少包含 2 个 sinks。
8. 如果某个 group 只有 1 个 sink，只有当它相邻的 gap 满足 `outlier_gap_ratio` 和 `outlier_min_abs_gap` 时，才保留为 single-sink outlier；否则取消相关 split 或并入相邻 group。
9. 分别得到 x-axis 和 y-axis 的 multi-way split candidate，只选择更合理的一个 candidate 作为当前 node 的 split；不要同时按两个 axis split。
10. 对 split 后的 child groups 递归 partition。
11. 如果两个 axis 都没有合法 candidate，则当前 node 是 leaf cluster。

实现中使用以下常量：
- `min_cluster_size = 4`
- `gap_ratio = 2.0`
- `min_abs_gap = 10`
- `outlier_gap_ratio = 3.5`
- `outlier_min_abs_gap = 20`

选择 x-axis 或 y-axis candidate 时，优先选择：
- big gaps 更显著的 candidate；
- 沿当前 bounding box major axis 切分的 candidate；
- 不产生 tiny non-outlier fragment 的 candidate。

不允许预设 cluster 数量。

## Leaf Cluster 规则

最终 leaf cluster 表示局部密集 sink group。后续模块会把 leaf cluster 转成 local topology。这里不要构建 sink-to-sink topology。

## Debug 和 Part 输出要求

- 像其他模块一样提供 debug 开关：

```cpp
void partitioner::debug_enable(bool enable);
void partitioner::debug_output(const common::PartitionTree& tree);
```

- debug 关闭时，不打印任何 partition log。
- debug 开启时，记录 partition 过程中的关键 log：当前 node 的 sink 数量、bounding box、尝试的 axis、`small_gap`、big gaps、选择的 split、生成的 child groups、leaf/outlier 判定。
- `debug_output()` 用缩进打印最终 partition tree，每个 node 至少显示 node id、sink 数量、children、bounding box、centroid、leaf/outlier 标记。
- 无论 debug 是否开启，都将最终 partition 类别输出到 `part/sample<k>.txt`。
- `sample<k>` 从输入文件名推导，和其他模块保持一致。
- `part/sample<k>.txt` 至少输出每个 leaf cluster / outlier 的类别结果：cluster id、是否 outlier、包含的 sink ids。
- 如果 `part/` 目录不存在，需要创建目录。

## Main 接入要求

- 修改 `main.cc`，把 partitioner 接入到 `parse` 之后、`treer` 之前。
- 当前阶段 main 只运行到 partitioner：
  1. parse input；
  2. 如果 `problem.valid == false`，直接返回错误；
  3. 调用 `partitioner::build(problem)`；
  4. 输出 `part/sample<k>.txt`；
  5. 结束程序。
- 将后续 `treer`、`bu`、`td`、`writer` 的调用全部从 main 流程中注释掉，保留代码但暂不执行。
- main 中 debug 开关风格与已有模块一致；需要能打开/关闭 partitioner debug。

## Robustness 要求

- 安全处理 zero sinks、one sink、重复坐标、相等 gap。
- 使用适合 Manhattan/grid 的整数几何。
- 对所有派生的 bounding box / centroid 进行 die boundary 校验或 clamp。
- sink indices 必须与 `problem.sinks` 顺序一致。
- 实现必须 deterministic。
- 提供可选 debug output，用缩进打印 partition tree。

## Non-Goals

不要在本模块优化 skew。不要 place buffers。不要生成 rectilinear routes。不要写 solution 文件。本模块只生成用于后续 topology construction 的自适应空间 partition hierarchy。