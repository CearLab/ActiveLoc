# Noise Monte Carlo Run

## Run Information
- **Run ID**: `Nexp50_N20_Niter30_Sm3_sigmap0p05_sigmameas0p2_alpha0.8_0.2_cyc80_20260331_181900`
- **Run Tag**: `Nexp50_N20_Niter30_Sm3_sigmap0p05_sigmameas0p2_alpha0.8_0.2_cyc80`
- **Timestamp**: 20260331_181900
- **Save Folder**: `results/NoiseMonteCarlo/Nexp50_N20_Niter30_Sm3_sigmap0p05_sigmameas0p2_alpha0.8_0.2_cyc80_20260331_181900`

## Scenario Configuration
- **Experiments**: 50
- **Fleet Size**: 20 agents
- **Total Cycles**: 80
- **Expansion alpha**: 0.8
- **Contraction alpha**: 0.2
- **Search Margin**: 3
- **Optuna Iterations**: 30 per agent per cycle
- **Process Noise (sigmap)**: 0.050 m
- **Measurement Noise (sigmam)**: 0.200 m

## Results Summary (Across Experiments)
- **Connected Steps %**: 71.07 ± 13.69
- **Disconnected Cycles %**: 41.00 ± 13.64
- **Expansion Connectivity (connected-only)**: 0.0538 ± 0.0020
- **Expansion Coverage (connected-only)**: 0.8881 ± 0.0019
- **Contraction Connectivity (connected-only)**: 0.1252 ± 0.0287
- **Contraction Coverage (connected-only)**: 0.8030 ± 0.0257

## Artifacts Generated
1. **noise_mc_metrics.npz** — Core Monte Carlo arrays
2. **noise_mc_config.json** — Full run configuration and recap
3. **noise_mc_summary.yaml** — Human-readable aggregate summary
4. **noise_mc_results.yaml** — Recap + per-experiment records
5. **noise_mc_results.json** — Recap + per-experiment records (JSON)
