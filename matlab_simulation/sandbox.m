a1 = agent(1,[1 1], 1);
a2 = agent(2,[1 2], 1);
%%
manager = AgentManager.getInstance();
manager.createAgent([-5 4],1,'team_leader');
manager.createAgent([-5 4],1,'team_mate');
%% .
map = Map.getInstance()
map.add_polygon([1 1 2 2; 1 2 2 1])
map.add_polygon([1 1 2 2; 1 2 2 1]+3)
map.draw_all_obs()
%%
manager = AgentManager.getInstance();
manager.createAgent([-2 4],1,'team_leader');
manager.createAgent([-5 4],1,'team_mate');
manager.createAgent([-5 2],1,'team_mate');
%%
manager.createAgent([2 -4],2,'team_leader');
manager.createAgent([5 -4],2,'team_mate');
manager.createAgent([5 -2],2,'team_mate');
manager.TeamList{1}.plot_team()