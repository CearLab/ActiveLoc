# Noise Monte Carlo Run

## Run Information
- **Run ID**: `Nexp50_N20_Niter30_Sm3_sigmap0_sigmameas0_alpha0.8_0.2_cyc80_20260401_122741`
- **Run Tag**: `Nexp50_N20_Niter30_Sm3_sigmap0_sigmameas0_alpha0.8_0.2_cyc80`
- **Timestamp**: 20260401_122741
- **Save Folder**: `results/NoiseMonteCarlo/Nexp50_N20_Niter30_Sm3_sigmap0_sigmameas0_alpha0.8_0.2_cyc80_20260401_122741`

## Scenario Configuration
- **Experiments**: 50
- **Fleet Size**: 20 agents
- **Total Cycles**: 80
- **Expansion alpha**: 0.8
- **Contraction alpha**: 0.2
- **Search Margin**: 3
- **Optuna Iterations**: 30 per agent per cycle
- **Process Noise (sigmap)**: 0.000 m
- **Measurement Noise (sigmam)**: 0.000 m

## Results Summary (Across Experiments)
- **Connected Steps %**: 100.00 ± 0.00
- **Disconnected Cycles %**: 0.00 ± 0.00
- **Expansion Connectivity (connected-only)**: 0.0577 ± 0.0020
- **Expansion Coverage (connected-only)**: 0.8894 ± 0.0020
- **Contraction Connectivity (connected-only)**: 0.1585 ± 0.0233
- **Contraction Coverage (connected-only)**: 0.7682 ± 0.0173

## Artifacts Generated
1. **noise_mc_metrics.npz** — Core Monte Carlo arrays
2. **noise_mc_config.json** — Full run configuration and recap
3. **noise_mc_summary.yaml** — Human-readable aggregate summary
4. **noise_mc_results.yaml** — Recap + per-experiment records
5. **noise_mc_results.json** — Recap + per-experiment records (JSON)
