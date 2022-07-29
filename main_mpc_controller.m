%% setup path
clear; close all; clc;
restoredefaultpath; matlabrc;

%% init
stance_duration = [0.3, 0.3];
duty_factor = [0.5, 0.5];
full_cycle_period = 0.6;
cycle_num = 0;
loop_num = 0;
init_leg_phase = [0, 1]; % [left, right]; 
init_leg_state = [0, 1]; % 0 swing, 1 stance
next_leg_state = [1, 0];
last_leg_state = [1, 0];
leg_state = [0, 1];
normalized_phase = [0, 0];
phase_threshold = 0.5;
switch_foot_position = [0, 0];
foot_position = cell(2,1);
foot_position_all = cell(2,1);
last_foot_position_world = cell(2,1);
last_foot_position_world{1} = [0,0];
last_foot_position_world{2} = [0,0];
t0 = tic;
Ts = 0.03;
max_t = 5;
max_cycle_num = 50;
kp_v = 0.05;
desired_height = 0.5;
desired_vx = 0.5;
desired_vy = 0;
max_clearance = 0.1;
phase_rate = 1.06;

% log data
state_all = [];
t_all = [];
left_leg_state = [];
right_leg_state = [];
left_leg_phase = [];
right_leg_phase = [];
t_augment_all = [];
switch_foot_position_all = [];
stance_point_all = [];
left_leg_state_all = [];
swing_position_all = [];

p_h = 5; % prediction horizion
c_h = 2; %  control horizion
p.m = 3;  % (kg) robot mass
p.g = 9.81;  % (m/s^2) gravity
p.I = 0.5;   % (kg*m^2)inertia

u_low = [-10, 0, p.g];
u_upp = [10, p.m*p.g*10, p.g];
x_low = [-10, 0.4, -pi/4, -100, -100, -100];
x_upp = [ 10, 0.6,  pi/4,  100,  100,  100];
mv(length(u_low)) = struct('Min',0,'Max',0);
ov(length(x_low)) = struct('Min',0,'Max',0);
for i = 1:1:length(u_low)
    mv(i).Min = u_low(i);
    mv(i).Max = u_upp(i);
end
for i = 1:1:length(x_low)
    ov(i).Min = x_low(i);
    ov(i).Max = x_upp(i);
end
Q = [10, 500, 1000, 1000, 100, 10];
R = [0.01, 0.01, 0.01];
weight = struct('OutputVariables', Q, 'ManipulatedVariables', R);

% t_augment = toc(t0) + init_leg_phase * full_cycle_period;
% leg_phase = mod(t_augment, full_cycle_period); 
state_actual = [0, 0.4, 0, 0, 0, 0]';
%while toc(t0) < max_t
while cycle_num < max_cycle_num
    t_augment = loop_num*Ts + init_leg_phase*full_cycle_period;
    %t_augment = toc(t0) + init_leg_phase*full_cycle_period;
    leg_phase = min(mod(t_augment, full_cycle_period) / full_cycle_period * phase_rate, 1);
    t_augment_all = [t_augment_all, t_augment(1)];
    left_leg_phase = [left_leg_phase, leg_phase(1)];
    right_leg_phase = [right_leg_phase, leg_phase(2)];
    for i=1:length(init_leg_phase)
        if leg_phase(i) <= phase_threshold
            leg_state(i) = init_leg_state(i);
            normalized_phase(i) = min(leg_phase(i)/phase_threshold * phase_rate,1);
        else
            leg_state(i) = next_leg_state(i);  
            normalized_phase(i) = min((leg_phase(i)-phase_threshold)/(1-phase_threshold)* phase_rate, 1);
        end
    end
    pcx = state_actual(1);
    pcy = state_actual(2);
    vx = state_actual(4);
    vy = state_actual(5);
    % get switch foot position
    new_leg_state = leg_state;
    for i=1:length(new_leg_state)
        if new_leg_state(i) == 0 && last_leg_state(i) == 1
            switch_foot_position = last_foot_position_world{i} - [pcx, pcy];
            switch_foot_position_all = [switch_foot_position_all; last_foot_position_world{i}];
            temp = last_foot_position_world{i};
            cycle_num = cycle_num + 1;           
        end
    end
    last_leg_state = new_leg_state;
    
    left_leg_state = [left_leg_state, new_leg_state(1)];
    right_leg_state = [right_leg_state, new_leg_state(2)];
        
    for i=1:length(leg_state)
        if leg_state(i) == 0
            % swing phase
            % suppose foot can directly follow the traj
            target_foot_position = [0.5 * vx * stance_duration(i) - kp_v * (desired_vx - vx), -pcy];
            swing_position =  gen_parabola(normalized_phase(i), switch_foot_position, target_foot_position, max_clearance);
            foot_position{i} = swing_position;  
            swing_position_world = swing_position + [pcx, pcy];
            last_foot_position_world{i} = foot_position{i} + [pcx, pcy];
            foot_position_all{i} = [foot_position_all{i}; last_foot_position_world{i}];
        else 
            % stance phase
            stand_point_world = last_foot_position_world{i};
            foot_position_all{i} = [foot_position_all{i}; stand_point_world];
            pex = stand_point_world(1);
            pey = stand_point_world(2);
            A = [0, 0, 0, 1, 0, 0;
                 0, 0, 0, 0, 1, 0;
                 0, 0, 0, 0, 0, 1;
                 0, 0, 0, 0, 0, 0;
                 0, 0, 0, 0, 0, 0;
                 0, 0, 0, 0, 0, 0];
            B = [0,                0,      0;  %   B = [0,        0,      0;
                 0,                0,      0;  %        0,        0,      0;
                 0,                0,      0;  %        0,        0,      0;
                 1/p.m,            0,      0;  %        1/p.m,    0,      0;
                 0,            1/p.m,     -1;  %        0,    1/p.m,     -1;
                 -(pey-pcy), pex-pcx,      0]; %        -ry,     rx,      0]
            C = diag(ones(1,6));
            sys = ss(A,B,C,[]);
            sys_n = c2d(sys,Ts);
            state_ref = gen_ref(pcx, pcy, vx, vy, desired_height, desired_vx, desired_vy, p_h, Ts);
            mpcobj = mpc(sys_n, Ts, p_h, c_h, weight, mv, ov);
            setEstimator(mpcobj,'custom');
            state = mpcstate(mpcobj);
            state.Plant = state_actual;
            u = mpcmove(mpcobj, state, state_actual, state_ref, []);
            state_pre = state_actual;
            state_actual = sys_n.A * state_pre + sys_n.B * u;
        end
    end
    
    state_all = [state_all, state_actual];
    left_leg_state_all = left_leg_state;
    stance_point_all = [stance_point_all; stand_point_world];
    swing_position_all = [swing_position_all; swing_position_world];
    t_all = [t_all, toc(t0)];
    loop_num = loop_num + 1;
end
%%
draw_Run(state_all, stance_point_all, swing_position_all, left_leg_state_all)
%%
plot(t_augment_all, state_all(3,:));
%%
plot(t_augment_all, left_leg_state);
%%
hold on
plot(t_augment_all, right_leg_state);
%%
figure()
plot(t_augment_all, left_leg_phase);
%%
hold on
plot(t_all, right_leg_phase);
%%
plot(t_augment_all);
%%
plot_leg(state_all, foot_position)
%%
for i = 1:1:length(switch_foot_position_all)
    pos = switch_foot_position_all(i,:);
    plot(pos(1), pos(2),'o',...
              'MarkerSize',5)
    axis equal
    xlim([-1,1]);
    ylim([-1,2]);  
    hold on
    pause(0.3)
end
%% 
x_all = [];
y_all = [];
for i=0:0.05:1
    pos = gen_parabola(i, [1,-0.5], [1.2, -0.5], 0.1);
    x_all = [x_all, pos(1)];
    y_all = [y_all, pos(2)];
end
plot(x_all, y_all)
%%
left = foot_position_all{2};
for i = 1:length(left)
    pos = left(i,:);
    plot(pos(1), pos(2),'o',...
              'MarkerSize',5)
    axis equal
    xlim([-1,1]);
    ylim([-1,1]);  
    hold on
    pause(0.03)
end