# Optuna Decentralized Optimization Run

## Run Information
- **Run ID**: `N20_Niter30_Sm3_alpha0.8_0.2_cyc80_20260401_112254`
- **Run Tag**: `N20_Niter30_Sm3_alpha0.8_0.2_cyc80`
- **Timestamp**: 20260401_112254
- **Save Folder**: `results/OptunaDecentralizedGlobal/N20_Niter30_Sm3_alpha0.8_0.2_cyc80_20260401_112254`

## Scenario Configuration
- **Fleet Size**: 20 agents
- **Total Cycles**: 80 (20 exp + 40 con + 20 exp)
- **Expansion α**: 0.8
- **Contraction α**: 0.2
- **Search Margin**: 3
- **Optuna Iterations**: 30 per agent per cycle
- **Communication Range**: 3
- **Domain**: [0, 10] × [0, 10]

## Results Summary
- **Total Iterations**: 1600
- **Solutions Found**: 1562 / 1600 (97.6%)
- **Solutions Failed**: 38

### Metrics Statistics
| Metric | Mean | Std Dev |
|--------|------|---------|
| Connectivity (M_C) | 0.1954 | 0.1366 |
| Coverage (M_E) | 0.7355 | 0.1307 |
| Composite Cost (J^γ) | 0.5384 | 0.1633 |

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
