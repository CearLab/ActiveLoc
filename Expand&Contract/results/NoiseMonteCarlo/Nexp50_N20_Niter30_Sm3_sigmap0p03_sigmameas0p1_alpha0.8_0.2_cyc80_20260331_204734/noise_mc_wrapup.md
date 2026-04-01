# Noise Monte Carlo Run

## Run Information
- **Run ID**: `Nexp50_N20_Niter30_Sm3_sigmap0p03_sigmameas0p1_alpha0.8_0.2_cyc80_20260331_204734`
- **Run Tag**: `Nexp50_N20_Niter30_Sm3_sigmap0p03_sigmameas0p1_alpha0.8_0.2_cyc80`
- **Timestamp**: 20260331_204734
- **Save Folder**: `results/NoiseMonteCarlo/Nexp50_N20_Niter30_Sm3_sigmap0p03_sigmameas0p1_alpha0.8_0.2_cyc80_20260331_204734`

## Scenario Configuration
- **Experiments**: 50
- **Fleet Size**: 20 agents
- **Total Cycles**: 80
- **Expansion alpha**: 0.8
- **Contraction alpha**: 0.2
- **Search Margin**: 3
- **Optuna Iterations**: 30 per agent per cycle
- **Process Noise (sigmap)**: 0.030 m
- **Measurement Noise (sigmam)**: 0.100 m

## Results Summary (Across Experiments)
- **Connected Steps %**: 74.76 ± 14.51
- **Disconnected Cycles %**: 33.42 ± 14.50
- **Expansion Connectivity (connected-only)**: 0.0545 ± 0.0018
- **Expansion Coverage (connected-only)**: 0.8896 ± 0.0015
- **Contraction Connectivity (connected-only)**: 0.1279 ± 0.0301
- **Contraction Coverage (connected-only)**: 0.8037 ± 0.0314

## Artifacts Generated
1. **noise_mc_metrics.npz** — Core Monte Carlo arrays
2. **noise_mc_config.json** — Full run configuration and recap
3. **noise_mc_summary.yaml** — Human-readable aggregate summary
4. **noise_mc_results.yaml** — Recap + per-experiment records
5. **noise_mc_results.json** — Recap + per-experiment records (JSON)
