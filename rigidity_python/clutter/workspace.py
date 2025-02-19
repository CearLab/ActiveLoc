if 0:
# max edge_relation
# s.t. EGVL_RIG > threshold & nodes in box
    N_agents = 4
    max_dist = 4
    threshold = 0.5
    box_margin = 10

    sampler = optuna.samplers.NSGAIISampler(constraints_func=OP.Objective.constraints, seed=0)
    study = optuna.create_study(
        directions=["minimize","minimize"],
        sampler=sampler,
        study_name="edge_relation",
        storage="sqlite:///EGVL_RIG.db",
        load_if_exists=True,
    )
    study.optimize(OP.Objective(N_agents,max_dist,threshold,box_margin), n_trials=1000)