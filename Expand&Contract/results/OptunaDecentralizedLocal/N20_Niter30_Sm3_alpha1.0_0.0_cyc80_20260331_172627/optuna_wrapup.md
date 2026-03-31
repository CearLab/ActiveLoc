# Optuna Decentralized Local-Objective Run

## Run Information
- **Run ID**: `N20_Niter30_Sm3_alpha1.0_0.0_cyc80_20260331_172627`
- **Run Tag**: `N20_Niter30_Sm3_alpha1.0_0.0_cyc80`
- **Timestamp**: 20260331_172627
- **Save Folder**: `results/OptunaDecentralizedLocal/N20_Niter30_Sm3_alpha1.0_0.0_cyc80_20260331_172627`

## Scenario Configuration
- **Fleet Size**: 20 agents
- **Total Cycles**: 80 (20 exp + 40 con + 20 exp)
- **Expansion alpha**: 1.0
- **Contraction alpha**: 0.0
- **Search Margin**: 3
- **Optuna Iterations**: 30 per agent per cycle
- **Communication Range**: 3
- **Domain**: [0, 10] x [0, 10]
- **Local Goal Agents**: [0, 5]

## Results Summary
- **Total Iterations**: 1600
- **Solutions Found**: 1550 / 1600 (96.9%)
- **Solutions Failed**: 50

### Metrics Statistics
| Metric | Mean | Std Dev |
|--------|------|---------|
| Connectivity (M_C) | 0.1214 | 0.1129 |
| Coverage (M_E) | 0.7430 | 0.1366 |
| Composite Cost (J^gamma) | 0.5273 | 0.3512 |

### Connectivity Robustness (Single-Run Monte Carlo-like)
| Metric | Value |
|--------|-------|
| Connected steps (%) | 100.00 |
| Disconnected steps (%) | 0.00 |
| Disconnected cycles (%) | 0.00 |
| Expansion M_C (connected-only) | 0.0545 |
| Expansion M_E (connected-only) | 0.8664 |
| Contraction M_C (connected-only) | 0.1882 |
| Contraction M_E (connected-only) | 0.6196 |

## Artifacts Generated
1. **optuna_metrics.npz** — Compressed array data (iteration-by-iteration time series)
2. **optuna_config.json** — Full configuration and recap statistics
3. **optuna_summary.yaml** — Human-readable summary
4. **temporal_metrics.png** — Moving-average temporal metrics
5. **physical_snapshots_local_goals.png** — Local-objective snapshots and local costs
6. **pareto_traversal.png** — Empirical Pareto traversal in (M_C, M_E)
