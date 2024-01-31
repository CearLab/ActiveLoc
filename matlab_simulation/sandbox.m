%% Sandbox
% file: sanbox.m
% author: Ido Sherf 
% date: 22/01/2024
% description: test map/teams creation

%% workspace section

%%% define agents (Agent class)
% a1 = agent(1,[1 1], 1);
% a2 = agent(2,[1 2], 1);

%%% define manager
% manager = AgentManager.getInstance();
% manager.createAgent([-5 4],1,'team_leader');
% manager.createAgent([-5 4],1,'team_mate');

%%% draw single team
% manager.TeamList{1}.plot_team()
% manager.TeamList{2}.plot_team()

%% test section

%%% define map  .
map = Map.getInstance();
map.add_polygon([1 1 2 2; 1 2 2 1])
map.add_polygon([1 1 2 2; 1 2 2 1]+3)
map.draw_all_obs()

%%% create agent manager
manager = AgentManager.getInstance();

% create agents of team 1
manager.createAgent([-2 4],1,'team_leader');
manager.createAgent([-5 4],1,'team_mate');
manager.createAgent([-5 2],1,'team_mate');

% create agents of team 2
manager.createAgent([2 -4],2,'team_leader');
manager.createAgent([5 -4],2,'team_mate');
manager.createAgent([5 -2],2,'team_mate');

% get a list of all agents
agents = manager.get_all_agent();

% plot all teams
map.draw_all_teams();

% draw LOS on map
map.draw_los_map()