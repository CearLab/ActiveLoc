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
    
fig1, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))
trans = 0.2

# Draw the box_margin box in the first subplot
box = plt.Rectangle((0, 0), box_margin, box_margin, fill=True, edgecolor='red', linestyle='-', facecolor='white', linewidth=2)
ax1.add_patch(box)
nx.draw_networkx(G0, pos=nx.get_node_attributes(G0, 'pos'), with_labels=False, node_color=['red'] + ['orange'] * (N_agents - 1), edge_color='red', ax=ax1, hide_ticks=False)
ax1.grid(True)

box = plt.Rectangle((0, 0), box_margin, box_margin, fill=True, edgecolor='green', linestyle='-', facecolor='white', linewidth=2)
ax2.add_patch(box)
nx.draw_networkx(G, pos=nx.get_node_attributes(G, 'pos'), with_labels=False, node_color=['green'] + ['lightgreen'] * (N_agents - 1), edge_color='green', ax=ax2, hide_ticks=False)
ax2.grid(True)    

ax3.scatter(FL.get_edge_relation(G0), FL.get_coverage(G0), color='red', s=400, edgecolors='black', label='G0', alpha=trans)
ax3.scatter(FL.get_edge_relation(G), FL.get_coverage(G), color='lightgreen', s=400, edgecolors='black', label='G', alpha=trans)
ax3.legend(loc='lower left')
ax3.grid(True)
ax3.set_xlabel('Edge Relation')
ax3.set_ylabel('Coverage')

plt.tight_layout()
plt.show()

# 
# Check if there is a solution saved as True and constraints_saved[0] > 0
solution_exists = any(sol and con[0] > 0 for sol, con in zip(solution_found, constraints))
print(solution_exists)

c = 10
a = 2
item = N_agents*(c-1) + a
print('neighbors_saved:', neighbors_replay[item])
print('solution found_saved:', solution_found[item])
print('constraints saved:', constraints[item])

G_neighbors = FL.generate_graph(neighbors_replay[item], max_dist)    
isrigid_neighbor, egvl_rig_neighbor = FL.is_rigid(G_neighbors, rigidity_threshold)
print('isrigid_saved:', isrigid_neighbor)
print('eigenvalues_saved:', egvl_rig_neighbor)

pos_opt_saved = pos_opt_replay[item]
print('pos_opt_saved:', pos_opt_saved)

N_agents_neighbor = len(G_neighbors)
pos_fix = neighbors_replay[item][:N_agents_neighbor-1,:]
print('pos_fix:', pos_fix)
op.pos_fix = pos_fix.flatten()
op.N_agents = N_agents_neighbor
constraints_tmp = np.array(op.constraint_function(pos_opt_saved))
print('constraints computed:', constraints_tmp)