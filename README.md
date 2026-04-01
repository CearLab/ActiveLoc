# ActiveLoc - Expand and Contract Experiments

This repository contains notebooks and support modules for decentralized multi-agent optimization experiments based on an expand-contract behavior.

The project focuses on how a fleet of agents balances:

1. Connectivity preservation.
2. Spatial coverage.
3. Optional local task objectives for selected agents.

## Problem Statement

A team of mobile agents must coordinate in a bounded workspace while preserving communication links and maintaining useful spatial distribution.

At each decentralized update, an agent solves a local optimization problem that contributes to a global trade-off between connectivity and coverage. This trade-off is controlled by a scalar weight $\alpha$ in an expansion-contraction schedule:

- Expansion phases ($\alpha > 0.5$) favor dispersion/coverage.
- Contraction phases ($\alpha < 0.5$) favor tighter and more robust connectivity.

The core research question is how well this decentralized policy:

1. Traces the Pareto structure of connectivity vs coverage.
2. Scales to full-fleet global-only optimization.
3. Supports local mission objectives for selected agents.
4. Remains robust under sensing and actuation noise.

For this reason, the project and manuscript follow the sequence:

1. Pareto analysis.
2. Global decentralized optimization.
3. Local-goal decentralized optimization.
4. Noise robustness analysis.

## Optimization Problem and Metrics

At each decentralized step, one moving agent optimizes its local decision while neighbors are fixed.

### Composite objective

The implemented objective combines a global soft term and an optional local task term:

$$
J = w_l J_l + w_g J_s
$$

where:

$$
J_s = \alpha M_E + (1-\alpha) M_C
$$

- $M_E$: normalized coverage metric.
- $M_C$: normalized connectivity metric.
- $\alpha$: expansion-contraction weight.
- $w_l, w_g$: local/global weights (for example global-only: $[0,1]$, local+global: $[10,1]$).

For agents with assigned local goals, the local term is proportional to negative target distance:

$$
J_l = -\lVert p_i - p_{\mathrm{tgt},i} \rVert
$$

and for agents without local tasks, $J_l = 0$.

### Connectivity metric

Connectivity is based on algebraic connectivity of the communication graph (normalized Laplacian):

$$
M_C = \tfrac{1}{2}\lambda_2
$$

where $\lambda_2$ is the Fiedler eigenvalue. Higher values indicate stronger graph connectivity.

### Coverage metric

Coverage is modeled from pairwise overlap penalties between agent disks of radius $r$ (implemented with $r=\text{max\_dist}$ in the notebooks):

$$
R = \sum_{i<j,\ d_{ij}<2r}(2r-d_{ij})^2
$$

and normalized as:

$$
M_E = 1 - \frac{R}{R_{\max}}
$$

Higher $M_E$ means better spatial spread (less overlap).

### Feasibility constraint in optimization

Candidate states are filtered by a hard connectivity condition: connected graphs are feasible and disconnected graphs are infeasible. Optuna's constrained sampler uses this feasibility signal during search.

## Repository Structure

Main work lives in [Expand&Contract](Expand&Contract):

- [Expand&Contract/ParetoMonteCarlo.ipynb](Expand&Contract/ParetoMonteCarlo.ipynb): Monte Carlo exploration of connectivity-coverage trade-offs across fleet sizes.
- [Expand&Contract/OptunaDecentralizedGlobal.ipynb](Expand&Contract/OptunaDecentralizedGlobal.ipynb): Decentralized optimization with global objective only.
- [Expand&Contract/OptunaDecentralizedLocal.ipynb](Expand&Contract/OptunaDecentralizedLocal.ipynb): Decentralized optimization with additional local goals for selected agents.
- [Expand&Contract/NoiseMonteCarlo.ipynb](Expand&Contract/NoiseMonteCarlo.ipynb): Robustness analysis under process and measurement noise.
- [Expand&Contract/modules/FrameworkLib.py](Expand&Contract/modules/FrameworkLib.py): Graph generation and metric utilities.
- [Expand&Contract/modules/Optimization_optuna.py](Expand&Contract/modules/Optimization_optuna.py): Objective definition and Optuna integration.
- [Expand&Contract/results](Expand&Contract/results): Stored outputs from previous experiment runs.

## Quick Start

From the repository root:

```bash
cd Expand\&Contract
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

Then open the notebook folder in VS Code or Jupyter and select the environment created above.

## Recommended Reading and Execution Order

If you are new to the project, follow this order:

1. [Expand&Contract/ParetoMonteCarlo.ipynb](Expand&Contract/ParetoMonteCarlo.ipynb)
2. [Expand&Contract/OptunaDecentralizedGlobal.ipynb](Expand&Contract/OptunaDecentralizedGlobal.ipynb)
3. [Expand&Contract/OptunaDecentralizedLocal.ipynb](Expand&Contract/OptunaDecentralizedLocal.ipynb)
4. [Expand&Contract/NoiseMonteCarlo.ipynb](Expand&Contract/NoiseMonteCarlo.ipynb)

This progression matches the paper flow: Pareto, Global, Local, Noise.

## Notebook Guide

### 1) ParetoMonteCarlo.ipynb

Purpose:

- Generates connected random fleets over a 10x10 domain.
- Computes normalized connectivity and coverage metrics.
- Builds Pareto fronts for different fleet sizes.

Use it when:

- You want to understand feasible trade-off regions before running decentralized optimization.

Typical outputs:

- Pareto plots and Monte Carlo metric datasets under [Expand&Contract/results/ParetoMonteCarlo](Expand&Contract/results/ParetoMonteCarlo).

### 2) OptunaDecentralizedGlobal.ipynb

Purpose:

- Runs decentralized, sequential agent updates using Optuna (NSGA-II sampling).
- Uses only global fleet objectives (coverage/connectivity blend).
- Evaluates expansion-contraction schedules controlled by `alpha`.

Use it when:

- You want a baseline behavior without individual agent task constraints.

Key parameters to inspect:

- `N_agents`, `max_dist`, `box_margin`.
- `Niter` (Optuna trials per update).
- `n_cycles` and `alpha` schedule.
- `cost_weights = [0, 1]` (global objective emphasis).

Typical outputs:

- Per-run summaries and metrics under [Expand&Contract/results/OptunaDecentralizedGlobal](Expand&Contract/results/OptunaDecentralizedGlobal).

### 3) OptunaDecentralizedLocal.ipynb

Purpose:

- Extends decentralized optimization with local-goal terms for selected agents.
- Combines local task tracking with global connectivity/coverage objectives.

Use it when:

- You want to evaluate mission-like behavior where specific agents are assigned target waypoints.

Key parameters to inspect:

- `local_goal_agent_ids` and `local_goal_positions`.
- `cost_weights = [1e1-1e3, 1]` (play with different local-term influence).
- Same cycle and alpha scheduling controls as global notebook.

Typical outputs:

- Per-run summaries and metrics under [Expand&Contract/results/OptunaDecentralizedLocal](Expand&Contract/results/OptunaDecentralizedLocal).

### 4) NoiseMonteCarlo.ipynb

Purpose:

- Repeats decentralized local-goal optimization over many experiments.
- Injects process noise and measurement noise.
- Aggregates robustness statistics (connectivity, coverage, connected-time behavior).

Use it when:

- You want sensitivity/robustness evidence for realistic sensing and actuation uncertainty.

Key parameters to inspect:

- `N_experiments`.
- `process_noise_std` and `measurement_noise_std`.
- `Niter`, `n_cycles`, and local-goal settings.

Typical outputs per run:

- `noise_mc_metrics.npz`
- `noise_mc_config.json`
- `noise_mc_summary.yaml`
- `noise_mc_results.yaml`
- `noise_mc_results.json`
- `noise_mc_wrapup.md`

Saved under timestamped folders in [Expand&Contract/results/NoiseMonteCarlo](Expand&Contract/results/NoiseMonteCarlo).

## Reproducing Existing Results

The repository already contains example outputs under [Expand&Contract/results](Expand&Contract/results). You can:

1. Open a run folder.
2. Check configuration in `*_config.json`.
3. Read summary in `*_summary.yaml` or `*_wrapup.md`.
4. Load `*.npz` files for plotting/analysis scripts.

## Practical Tips

- Run notebooks from top to bottom on a fresh kernel.
- Keep random seeds fixed when comparing parameter changes.
- Change one variable at a time (`alpha`, noise level, local weights, or fleet size).
- Preserve output folders with timestamps to keep experiments traceable.

## Troubleshooting

- Import errors: verify the selected Python environment and install missing packages.
- Very slow optimization: reduce `Niter`, `N_agents`, or `n_cycles` for quick checks.
- Inconsistent comparisons: ensure seeds and scenario constants are identical across runs.

## Citation and Context

Notebook text references a CLAWAR 2026 context and manuscript-style reporting. Figures and related assets are available in [Expand&Contract/CLAWAR2026_figures](Expand&Contract/CLAWAR2026_figures).
