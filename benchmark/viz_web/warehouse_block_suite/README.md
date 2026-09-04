# 20×20 warehouse-block testcase suite

- Map size: 20×20
- Aisle width: 1
- Block sizes: 3, 4, 9
- Requested density levels: 25%, 50%, 75%
- Robots / relocation targets: 8 / 12
- Seed: 0

Every testcase has one block size and exactly the same shelf count in every
block. Initial shelves, target starts, and target goals are storage cells;
robots start in aisle cells. The YAML ``storage_map`` is authoritative:
carried shelves may traverse aisles, but DROP is legal only in storage cells.

All nine YAML files are also members of the protected formal release
benchmark through `benchmark/release_benchmark.json`; they are no longer
visualization-only smoke cases.

| block | target density | actual density | blocks | shelves/block | total shelves | YAML |
|---:|---:|---:|---:|---:|---:|---|
| 3×3 | 25% | 22.2% | 25 | 2 | 50 | `instances/warehouse_blocks_h20w20_b3_a1_d25_r8_t12_seed0.yaml` |
| 3×3 | 50% | 55.6% | 25 | 5 | 125 | `instances/warehouse_blocks_h20w20_b3_a1_d50_r8_t12_seed0.yaml` |
| 3×3 | 75% | 77.8% | 25 | 7 | 175 | `instances/warehouse_blocks_h20w20_b3_a1_d75_r8_t12_seed0.yaml` |
| 4×4 | 25% | 25.0% | 16 | 4 | 64 | `instances/warehouse_blocks_h20w20_b4_a1_d25_r8_t12_seed0.yaml` |
| 4×4 | 50% | 50.0% | 16 | 8 | 128 | `instances/warehouse_blocks_h20w20_b4_a1_d50_r8_t12_seed0.yaml` |
| 4×4 | 75% | 75.0% | 16 | 12 | 192 | `instances/warehouse_blocks_h20w20_b4_a1_d75_r8_t12_seed0.yaml` |
| 9×9 | 25% | 24.7% | 4 | 20 | 80 | `instances/warehouse_blocks_h20w20_b9_a1_d25_r8_t12_seed0.yaml` |
| 9×9 | 50% | 50.6% | 4 | 41 | 164 | `instances/warehouse_blocks_h20w20_b9_a1_d50_r8_t12_seed0.yaml` |
| 9×9 | 75% | 75.3% | 4 | 61 | 244 | `instances/warehouse_blocks_h20w20_b9_a1_d75_r8_t12_seed0.yaml` |
