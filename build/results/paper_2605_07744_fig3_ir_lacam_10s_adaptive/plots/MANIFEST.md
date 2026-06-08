# Paper Figure Manifest

Input rows:
- build/results/paper_2605_07744_fig3_ir_lacam_10s_adaptive/rows.csv

Generated figures:
- `figure3_components_grid_with_lacam.png` / `figure3_components_grid_with_lacam.pdf`: Fig.3-style grid with LaCAM-TAPF baselines. Panels with large LaCAM outliers switch to log y for readability. Bottom panels use final improvement vs agent count because rows do not contain per-refinement cost histories.

Notes:
- Current experiment rows do not contain per-refinement cost histories, so figures that correspond to paper improvement-over-time plots use recorded final improvement instead of synthetic time curves.
- All plotted LaCAM-TAPF rows come from the same-instance paper-suite reruns.
