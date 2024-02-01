%% Sandbox
% file: sanbox.m
% author: Ido Sherf 
% date: 22/01/2024
% description: test map/teams creation

%% workspace section

%%% define agents (Agent class)
% a1 = Agent(1,[1 1], 1);
% a2 = Agent(2,[1 2], 1);

%%% define manager
% manager = AgentManager.getInstance();
% manager.createAgent([-5 4],1,'team_leader');
% manager.createAgent([-5 4],2,'team_mate');

%%% draw single team
manager.team_list{1}.plotTeam()
manager.team_list{2}.plotTeam()

%% test section
clc;clear all;close all;
%%% define map  .
map = Map.getInstance();
map.addPolygon([1 1 2 2; 1 2 2 1])
map.addPolygon([1 1 2 2; 1 2 2 1]+3)
map.drawAllObs()

%%% create agent manager
manager = AgentManager.getInstance();

% create agents of team 1
manager.createAgent([-2 3],1,'team_leader');
manager.createAgent([-5 4],1,'team_mate');
manager.createAgent([-6 2],1,'team_mate');

% create agents of team 2
manager.createAgent([2 -4],2,'team_leader');
manager.createAgent([5 -4],2,'team_mate');
manager.createAgent([5 -2],2,'team_mate');
%%
% get a list of all agents
agents = manager.getAllAgent();

% plot all teams
map.drawAllTeams();

[los_table,agents_list] = calcLosMap(agents);
% draw LOS on map
drawLosMap(los_table)