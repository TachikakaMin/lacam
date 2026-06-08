# arXiv 2307.00663 comparison figures

Generated from local result files.

## Inputs
- LaCAM/IR rows: `build/results/full_three_method_hindrance_exp1_exp2_ir/rows.csv`
- Extra row files: `build/results/full_2307_exp1_exp2_ir_anytime_10s_timeout30/rows.csv, build/results/maze_exp2_40_60_lacam_ir_10s_timeout30/rows.csv, build/results/symbotic_exp1_ir_tapf_10s_adaptive/rows.csv, build/results/symbotic_exp1_lacam_tapf_10s_adaptive/rows.csv, build/results/symbotic_exp2_ir_lacam_tapf_10s_adaptive/rows.csv`
- ITA-CBS rows: `build/results/full_10s_solver_parallel_with_maze_40_60_plus_symbotic_itacbs.csv`
- LaCAM method plotted as LaCAM-TAPF: `lacam_focal_h`
- Timeout cap used in plots: `10s`

## Output figures
- `figure2a_exp1_success_rates.{png,pdf}`
- `figure2b_exp2_success_rates.{png,pdf}`
- `figure3_runtime_scatter.{png,pdf}`
- `figure3b_first_solution_scatter.{png,pdf}`
- `figure3c_solution_quality_scatter.{png,pdf}`
- `figure4_ta_runtime_nodes.{png,pdf}`
- `figure5_runtime_breakdown.{png,pdf}`

## Data caveats
- The paper uses 30s optimal-solution limits; these local LaCAM/IR/ITA-CBS rows use a 10s rerun.
- If multiple LaCAM/IR row files contain the same `(case_key, method)`, later inputs override earlier rows. This is used to replace stale IR rows with the latest rerun without double-counting cases.
- Figure 3b compares LaCAM-TAPF `first_solution_time_ms` against IR-TAPF `initial_solution_time_ms`; missing or unsolved cases are plotted at the timeout cap.
- Figure 3c plots `IR-TAPF SOC / LaCAM-TAPF SOC` against IR-TAPF SOC only on cases solved by both methods; out-of-axis ratios are shown as boundary triangles using 1%-99% ratio limits.
- Figure 4 includes only methods with comparable assignment/profile instrumentation. IR and IR+Opt rows are omitted there because the normalized exp1/exp2 rows do not expose comparable TA/node fields.
- Figure 5 maps non-CBS methods to the closest available instrumentation. For LaCAM, target-assignment time is measured directly; the rest is search/refinement. For IR and IR+Opt, only total solver time is used in these exp1/exp2 rows.
- exp1/exp2 `opt_*` IR rows present in the plotted inputs: `False`.

## Coverage

| method        |   rows |   solved | maps                                                                                                   | scenarios         |   min_agents |   max_agents |   median_runtime_s |   median_cost |
|:--------------|-------:|---------:|:-------------------------------------------------------------------------------------------------------|:------------------|-------------:|-------------:|-------------------:|--------------:|
| ir            |   9760 |     9039 | Boston_0_256,den312d,maze-32-32-2,orz900d,random-32-32-10,room-64-64-8,symbotic,warehouse-10-20-10-2-1 | G,000,030,060,100 |            5 |          200 |             10     |          1120 |
| itacbs        |   9760 |     4763 | Boston_0_256,den312d,maze-32-32-2,orz900d,random-32-32-10,room-64-64-8,symbotic,warehouse-10-20-10-2-1 | G,000,030,060,100 |            5 |          200 |             10     |           536 |
| lacam_focal_h |   9760 |     9727 | Boston_0_256,den312d,maze-32-32-2,orz900d,random-32-32-10,room-64-64-8,symbotic,warehouse-10-20-10-2-1 | G,000,030,060,100 |            5 |          200 |              9.059 |          1170 |
