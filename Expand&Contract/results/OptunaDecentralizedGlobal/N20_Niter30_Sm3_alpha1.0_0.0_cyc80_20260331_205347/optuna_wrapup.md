# Optuna Decentralized Optimization Run

## Run Information
- **Run ID**: `N20_Niter30_Sm3_alpha1.0_0.0_cyc80_20260331_205347`
- **Run Tag**: `N20_Niter30_Sm3_alpha1.0_0.0_cyc80`
- **Timestamp**: 20260331_205347
- **Save Folder**: `results/OptunaDecentralizedGlobal/N20_Niter30_Sm3_alpha1.0_0.0_cyc80_20260331_205347`

## Scenario Configuration
- **Fleet Size**: 20 agents
- **Total Cycles**: 80 (20 exp + 40 con + 20 exp)
- **Expansion α**: 1.0
- **Contraction α**: 0.0
- **Search Margin**: 3
- **Optuna Iterations**: 30 per agent per cycle
- **Communication Range**: 3
- **Domain**: [0, 10] × [0, 10]

## Results Summary
- **Total Iterations**: 1600
- **Solutions Found**: 1570 / 1600 (98.1%)
- **Solutions Failed**: 30

### Metrics Statistics
| Metric | Mean | Std Dev |
|--------|------|---------|
| Connectivity (M_C) | 0.2537 | 0.2129 |
| Coverage (M_E) | 0.6665 | 0.2136 |
| Composite Cost (J^γ) | 0.6580 | 0.2240 |

## Artifacts Generated
1. **optuna_metrics.npz** — Compressed array data (iteration-by-iteration time series)
2. **optuna_config.json** — Full configuration and recap statistics
3. **optuna_summary.yaml** — Human-readable summary
4. **temporal_metrics.png** — Top: M_C & M_E moving averages; Bottom: J^γ composite cost
5. **physical_snapshots.png** — Network configurations at 0%, 30%, 70%, 100% of runtime
6. **pareto_traversal.png** — Empirical Pareto front in (connectivity, coverage) space

## Notes
- Each cell in the optimization loop performs a decentralized, local Optuna search constrained to agent neighborhoods
- Metrics are normalized to [0,1] per scenario baseline
- Feasibility constraints ensure graph connectivity is maintained at all times
