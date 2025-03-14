clc; clear; close all;

%% Simulation parameters
dt = 0.01; % Simulation step (fine-grained)
control_update_dt = 0.1; % Control update interval
control_update_steps = control_update_dt / dt; % Control update every 10 steps
T = 40; % Simulation duration
t = 0:dt:T;
N = length(t);

%% System update
M = 37.758;
I = 18.35; 
Xu = 8.9149;
Xuu = 11.2101;
Nr = 16.9542;
Nrrr = 12.8966;
Yv = 15;
Yvv = 3;
Yr = 6;
Nv = 6;

%% Initialize states for x and y
x = zeros(1, N); u = zeros(1, N);
y = zeros(1, N); v = zeros(1, N);
psi = zeros(1, N); r = zeros(1, N);
psi(1) = -150*3.141592/180;
y(1) = 15;
x(1) = 0;

tau_u = zeros(1, N);
tau_r = zeros(1, N);
tau_v = zeros(1, N);

disturbance_u = zeros(1, N);
disturbance_r = zeros(1, N);
disturbance_v = zeros(1, N);
estim_disturbance = zeros(3, N);
estim_state_history = zeros(3, N);
z_history = zeros(3, N);

tu = 10;
tr = 20;
tv = 15;

Fv = 20;
lf = 3.5;
lb = -3;
bow = 1;

hp = 4;

alpha1 = 0.2;
alpha2 = 0.5;

alphac= 10;

Tu_con = 0;
Tv_con = 0;
Tr_con = 0;

eta_xd = 0;
eta_yd = 0;

xd = 0;
yd = 0;

a = 4;
b = 4;

acceptance_rad = 0.2;

% a11 = 200;
a12 = 0;
a13 = 0;
a21 = 0;
% a22 = 200;
a23 = 0;
a31 = 0;
a32 = 0;
% a33 = 200;


a11 = 1;
a22 = 1;
a33 = 1;

g_matrix = [1/M, 0, 0; 0, 1/M, 0; 0, 0, 1/I]; 
estim_d = [0;0;0];
filtered_d = [0;0;0];
before_filtered_d = [0;0;0];
estim_state = [0;0;0];
state_error = [0;0;0];
z = [0;0;0];
estim_x_dot = [0;0;0];
estim_x_plus = [0;0;0];

%% Figure Setup
fig = figure('Position', [100, 100, 1200, 600]); % Adjust figure size

% Left: Ship Motion Animation
subplot(3, 4, [1 2 5 6 9 10]); % 왼쪽 큰 플롯 (애니메이션)
hold on;
axis equal;
grid on;
xlabel('X Position');
ylabel('Y Position');
title('Ship Motion Simulation');
xlim([-20, 20]); ylim([-5, 30]);

% ** 정박지(Anchorage Area) 추가 **
fill([-50 50 50 -50], [-50 -50 0 0], [0.7 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
fill([2.5 5 5 2.5], [0 0 5 5], [0.7 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
fill([-2.5 -5 -5 -2.5], [0 0 5 5], [0.7 0.7 0.7], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

% Initial ship shape (Pentagon)
ship_x = [3.5 2.5 -3 -3 2.5]; % Shape of the ship (pentagon)
ship_y = [0 1.2 1.2 -1.2 -1.2];

% Plot initial ship
ship_plot = fill(ship_x, ship_y, 'b'); % Ship shape

% ** Acceptence Radius 업데이트 (선두와 선미 위치에 원 그리기) **
theta = psi(1); % 초기 선박의 회전각
radii = acceptance_rad * ones(1, 2);  % 두 원의 반지름 (선두, 선미)
center_x = [x(1) + lf * cos(theta), x(1) + lb * cos(theta)]; % 선두, 선미의 x 위치
center_y = [y(1) + lf * sin(theta), y(1) + lb * sin(theta)]; % 선두, 선미의 y 위치

% 선두와 선미에 빨간 원 추가
circle_hf = rectangle('Position', [center_x(1)-acceptance_rad, center_y(1)-acceptance_rad, 2*acceptance_rad, 2*acceptance_rad], 'Curvature', [1, 1], 'EdgeColor', 'r', 'LineWidth', 2);
circle_hb = rectangle('Position', [center_x(2)-acceptance_rad, center_y(2)-acceptance_rad, 2*acceptance_rad, 2*acceptance_rad], 'Curvature', [1, 1], 'EdgeColor', 'r', 'LineWidth', 2);

% Add y = ln(x^2 + 1) plot in red
x_vals = linspace(-20, 20, 1000);  % Define a range for x values
y1_vals = a*log(x_vals.^2 + 1); 
y2_vals = b*log(x_vals.^2 + 1); 

% Plot the function in red
% plot( x_vals, y1_vals, 'r', 'LineWidth', 2);
% plot( x_vals, y2_vals, 'm', 'LineWidth', 2);

% Add simulation time display (Top-left corner)
time_text = text(-45, 45, 'Time: 0.00 s', 'FontSize', 12, 'Color', 'r');

% Right: 3x2 Subplots
ax(1) = subplot(3,4,3); hold on; title('X Position');
ax(2) = subplot(3,4,4); hold on; title('Y Position');
ax(3) = subplot(3,4,7); hold on; title('Velocity (u, v)');
ax(4) = subplot(3,4,8); hold on; title('Angular Velocity (r)');
ax(5) = subplot(3,4,11); hold on; title('u disturbance');
ax(6) = subplot(3,4,12); hold on; title('Heading Angle (\psi)');

% 선을 미리 생성 (plot 업데이트 최적화)
line_x = plot(ax(1), t(1), x(1), 'b');
line_y = plot(ax(2), t(1), y(1), 'r');
line_disturbance_u = plot(ax(3), t(1), disturbance_u(1), 'b');
line_estim_disturbance_u = plot(ax(3), t(1), estim_disturbance(1,1), 'r');
line_disturbance_v = plot(ax(4), t(1), disturbance_v(1), 'b');
line_estim_disturbance_v = plot(ax(4), t(1), estim_disturbance(2,1), 'r');
line_disturbance_r = plot(ax(5), t(1), disturbance_r(1), 'b');
line_estim_disturbance_r = plot(ax(5), t(1), estim_disturbance(3,1), 'r');
line_psi = plot(ax(6), t(1), psi(1), 'k');


% 세로선(Vertical Line) 추가
for i = 1:6
    vline_x(i) = xline(ax(i), t(1), '--k', 'LineWidth', 1.2);
end


%% Simulation loop
for i = 2:N
    if mod(i, control_update_steps) == 0 
        %% DOB
        tu = 10*sin(i*control_update_dt);
        tv = 15*sin(i*control_update_dt);
        tr = 10*sin(i*control_update_dt);

        fx = [(-Xu*u(i-1) - Xuu*sqrt(u(i-1)*u(i-1))*u(i-1) + tu)/M;
              (-Yv*v(i-1) -Yr*r(i-1) - Yvv*sqrt(v(i-1)*v(i-1))*v(i-1) + tv)/M;
              (-Nv*v(i-1) -Nr*r(i-1) - Nrrr*r(i-1)*r(i-1)*r(i-1) + tr)/I];        

        gain = -1.0;
        
        real_state = [u(i-1);v(i-1);r(i-1)];
        state_error = estim_state - real_state;
        estim_x_dot = estim_d + fx + gain*state_error;
        estim_state = estim_x_dot*control_update_dt + estim_state;

        
        w_cutoff = 0.5;
        pi = (1/gain)*(exp(gain*control_update_dt)-1.0);
        estim_d = -exp(gain*control_update_dt).*state_error/pi;
        
        before_filtered_d = filtered_d;
        filtered_d = before_filtered_d.*exp(-w_cutoff*control_update_dt) - estim_d.*(1-exp(-w_cutoff*control_update_dt));

    end

    % z_history(:,i) = z;
    estim_disturbance(:,i) = -filtered_d;

    disturbance_u(i) = 50/M;
    disturbance_v(i) = 20/M;
    disturbance_r(i) = 10/I;

    % disturbance_u(i) = 20*sin(i*0.1*dt)*sin(i*dt)/M;
    % disturbance_v(i) = 15*cos(0.8*dt)*sin(i*dt)/M;
    % disturbance_r(i) = 6*sin(0.5*i*dt)/I;    

    % Update states using Euler integration
    tau_u(i) = tu;
    tau_r(i) = tr;
    tau_v(i) = tv;
    
    x_dot = u(i-1)*cos(psi(i-1)) - v(i-1)*sin(psi(i-1));
    y_dot = u(i-1)*sin(psi(i-1)) + v(i-1)*cos(psi(i-1));
    u_dot = (-Xu*u(i-1) - Xuu*sqrt(u(i-1)*u(i-1))*u(i-1) + tau_u(i))/M + disturbance_u(i);
    v_dot = (-Yv*v(i-1) -Yr*r(i-1) - Yvv*sqrt(v(i-1)*v(i-1))*v(i-1) + tau_v(i))/M + disturbance_v(i);
    r_dot = (-Nv*v(i-1) -Nr*r(i-1) - Nrrr*r(i-1)*r(i-1)*r(i-1) + tau_r(i))/I + disturbance_r(i);
    
    x(i) = x(i-1) + x_dot * dt;
    y(i) = y(i-1) + y_dot * dt;
    psi(i) = psi(i-1) + r(i-1) * dt;
    u(i) = u(i-1) + u_dot * dt;
    v(i) = v(i-1) + v_dot * dt;
    r(i) = r(i-1) + r_dot * dt;

    % ** 실시간 선박 업데이트 **
    theta = psi(i); % 선박의 회전각
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)]; % 회전행렬 적용
    ship_rotated = R * [ship_x; ship_y]; % 회전된 선박 좌표
    
    set(ship_plot, 'XData', ship_rotated(1,:) + x(i), 'YData', ship_rotated(2,:) + y(i));
    
    % Update Simulation Time Text
    set(time_text, 'String', sprintf('Time: %.2f s', t(i)));

    % ** Acceptence Radius 업데이트 (선두와 선미 위치에 원 그리기) **
    center_x = [x(i) + lf * cos(psi(i)), x(i) + lb * cos(psi(i))]; % 선두, 선미의 x 위치
    center_y = [y(i) + lf * sin(psi(i)), y(i) + lb * sin(psi(i))]; % 선두, 선미의 y 위치
    
    % 선두와 선미에 빨간 원 추가
    set(circle_hf, 'Position', [center_x(1)-acceptance_rad, center_y(1)-acceptance_rad, 2*acceptance_rad, 2*acceptance_rad]);
    set(circle_hb, 'Position', [center_x(2)-acceptance_rad, center_y(2)-acceptance_rad, 2*acceptance_rad, 2*acceptance_rad]);
    

    % Subplot 업데이트 (선을 이어서 그림)
    set(line_x, 'XData', t(1:i), 'YData', x(1:i));
    set(line_y, 'XData', t(1:i), 'YData', y(1:i));
    set(line_disturbance_u, 'XData', t(1:i), 'YData', disturbance_u(1:i));
    set(line_estim_disturbance_u, 'XData', t(1:i), 'YData', estim_disturbance(1,1:i));
    set(line_disturbance_v, 'XData', t(1:i), 'YData', disturbance_v(1:i));
    set(line_estim_disturbance_v, 'XData', t(1:i), 'YData', estim_disturbance(2,1:i));
    set(line_disturbance_r, 'XData', t(1:i), 'YData', disturbance_r(1:i));
    set(line_estim_disturbance_r, 'XData', t(1:i), 'YData', estim_disturbance(3,1:i));
    set(line_psi, 'XData', t(1:i), 'YData', psi(1:i));


    % Update vertical lines
    for j = 1:6
        set(vline_x(j), 'Value', t(i));
    end

    drawnow;
    
    % Stop simulation if figure is closed
    if ~isvalid(fig)
        break;
    end
end
