clc; clear; close all;

%% Simulation parameters
dt = 0.01; % Simulation step (fine-grained)
control_update_dt = 0.1; % Control update interval
control_update_steps = control_update_dt / dt; % Control update every 10 steps
T = 40; % Simulation duration
t = 0:dt:T;
N = length(t);

%% Initialize states for x and y
x = zeros(1, N); u = zeros(1, N);
y = zeros(1, N); v = zeros(1, N);
psi = zeros(1, N); r = zeros(1, N);
psi(1) = -150*3.141592/180;
y(1) = 25;
x(1) = 5;

tau_u = zeros(1, N);
tau_r = zeros(1, N);
tau_v = zeros(1, N);

tu = 0;
tr = 0;
tv = 0;

Fv = 20;
lf = 3.5;
lb = -3;
bow = 1;

hp = 4;

alpha1 = 1;
alpha2 = 2;

alphac= 10;

Tu_con = 0;
Tv_con = 0;
Tr_con = 0;

acceptance_rad = 0.2;

eta_xd = 0;
eta_yd = 0;

xd = 0;
yd = 0;

a = 4;
b = 4;

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
plot( x_vals, y1_vals, 'r', 'LineWidth', 2);
plot( x_vals, y2_vals, 'm', 'LineWidth', 2);

% Add simulation time display (Top-left corner)
time_text = text(-45, 45, 'Time: 0.00 s', 'FontSize', 12, 'Color', 'r');

% Right: 3x2 Subplots
ax(1) = subplot(3,4,3); hold on; title('X Position');
ax(2) = subplot(3,4,4); hold on; title('Y Position');
ax(3) = subplot(3,4,7); hold on; title('Velocity (u, v)');
ax(4) = subplot(3,4,8); hold on; title('Angular Velocity (r)');
ax(5) = subplot(3,4,11); hold on; title('Control Inputs (\tau_u, \tau_r)');
ax(6) = subplot(3,4,12); hold on; title('Heading Angle (\psi)');

% 선을 미리 생성 (plot 업데이트 최적화)
line_x = plot(ax(1), t(1), x(1), 'b');
line_y = plot(ax(2), t(1), y(1), 'r');
line_u = plot(ax(3), t(1), u(1), 'g');
line_v = plot(ax(3), t(1), v(1), 'm');
line_r = plot(ax(4), t(1), r(1), 'k');
line_tau_u = plot(ax(5), t(1), tau_u(1), 'b');
line_tau_r = plot(ax(5), t(1), tau_r(1), 'r');
line_tau_v = plot(ax(5), t(1), tau_v(1), 'g');
line_psi = plot(ax(6), t(1), psi(1), 'k');


% 세로선(Vertical Line) 추가
for i = 1:6
    vline_x(i) = xline(ax(i), t(1), '--k', 'LineWidth', 1.2);
end


%% Simulation loop
for i = 2:N
    if mod(i, control_update_steps) == 0 
        %% Control Input Calculation
        u_dot = (-Xu*u(i-1) - Xuu*sqrt(u(i-1)*u(i-1))*u(i-1))/M;
        v_dot = (-Yv*v(i-1) -Yr*r(i-1) - Yvv*sqrt(v(i-1)*v(i-1))*v(i-1))/M;
        r_dot = (-Nv*v(i-1) -Nr*r(i-1) - Nrrr*r(i-1)*r(i-1)*r(i-1))/I;        

        %CBF
        % hf = y(i-1) + lf*sin(psi(i-1)) + 20 - acceptance_rad;
        % hf_dot = u(i-1)*sin(psi(i-1)) + v(i-1)*cos(psi(i-1)) + r(i-1)*lf*cos(psi(i-1));
        % hf_dotdot = u_dot*sin(psi(i-1)) + v_dot*cos(psi(i-1)) + lf*r_dot*cos(psi(i-1)) + u(i-1)*r(i-1)*cos(psi(i-1)) - v(i-1)*r(i-1)*sin(psi(i-1)) - lf*r(i-1)*r(i-1)*sin(psi(i-1));
        % hf_cbf = (hf + alpha1*hf_dot + alpha2*(hf_dot + alpha1*hf_dotdot))/(alpha1*alpha2);
        % 
        % hb = y(i-1) + lb*sin(psi(i-1)) + 20 - acceptance_rad;
        % hb_dot = u(i-1)*sin(psi(i-1)) + v(i-1)*cos(psi(i-1)) + r(i-1)*lb*cos(psi(i-1));
        % hb_dotdot = u_dot*sin(psi(i-1)) + v_dot*cos(psi(i-1)) + lb*r_dot*cos(psi(i-1)) + u(i-1)*r(i-1)*cos(psi(i-1)) - v(i-1)*r(i-1)*sin(psi(i-1)) - lb*r(i-1)*r(i-1)*sin(psi(i-1));
        % hb_cbf = (hb + alpha1*hb_dot + alpha2*(hb_dot + alpha1*hb_dotdot))/(alpha1*alpha2);

        %CBF
        
        % x_f = x(i-1) + lf*cos(psi(i-1));
        % y_f = y(i-1) + lf*sin(psi(i-1));
        % x_dot_f = u(i-1)*cos(psi(i-1)) - v(i-1)*sin(psi(i-1)) - lf*r(i-1)*sin(psi(i-1));
        % y_dot_f = u(i-1)*sin(psi(i-1)) + v(i-1)*cos(psi(i-1)) + lf*r(i-1)*cos(psi(i-1));
        % hf = y_f - a*log(x_f*x_f + 1);
        % hf_dot = yd - a*2*xd*xd/(xd*xd + 1);
        % etayf = - a*2*xd/(xd*xd + 1);
        % hf_cbf = hf + alpha1*hf_dot;
        x_f = x(i-1) + lf*cos(psi(i-1));
        y_f = y(i-1) + lf*sin(psi(i-1));
        x_dot_f = u(i-1)*cos(psi(i-1)) - v(i-1)*sin(psi(i-1)) - lf*r(i-1)*sin(psi(i-1));
        y_dot_f = u(i-1)*sin(psi(i-1)) + v(i-1)*cos(psi(i-1)) + lf*r(i-1)*cos(psi(i-1));        
        hf = y_f - a*log(x_f*x_f + 1);
        hf_dot = y_dot_f - 2*a*x_f*x_dot_f/(x_f*x_f + 1);
        etayf = -2*a*x_f/(x_f*x_f + 1);
        hf_dotdot = u_dot*sin(psi(i-1)) + v_dot*cos(psi(i-1)) + lf*r_dot*cos(psi(i-1)) + u(i-1)*r(i-1)*cos(psi(i-1)) - v(i-1)*r(i-1)*sin(psi(i-1)) - lf*r(i-1)*r(i-1)*sin(psi(i-1)) + 4*a*x_f*x_f*x_dot_f*x_dot_f/((x_f*x_f + 1)*(x_f*x_f + 1)) - 2*a*x_dot_f*x_dot_f/(x_f*x_f+1) + etayf*(-u(i-1)*r(i-1)*sin(psi(i-1)) - v(i-1)*r(i-1)*cos(psi(i-1)) - r(i-1)*r(i-1)*lf*cos(psi(i-1)));
        hf_cbf = (hf + (alpha1 + alpha2)*hf_dot + alpha1*alpha2*hf_dotdot)/(alpha1*alpha2);

        x_b = x(i-1) + lb*cos(psi(i-1));
        y_b = y(i-1) + lb*sin(psi(i-1));
        x_dot_b = u(i-1)*cos(psi(i-1)) - v(i-1)*sin(psi(i-1)) - lb*r(i-1)*sin(psi(i-1));
        y_dot_b = u(i-1)*sin(psi(i-1)) + v(i-1)*cos(psi(i-1)) + lb*r(i-1)*cos(psi(i-1));        
        hb = y_b - b*log(x_b*x_b + 1);
        hb_dot = y_dot_b - 2*b*x_b*x_dot_b/(x_b*x_b + 1);
        etayb = -2*b*x_f/(x_b*x_b + 1);
        hb_dotdot = u_dot*sin(psi(i-1)) + v_dot*cos(psi(i-1)) + lb*r_dot*cos(psi(i-1)) + u(i-1)*r(i-1)*cos(psi(i-1)) - v(i-1)*r(i-1)*sin(psi(i-1)) - lb*r(i-1)*r(i-1)*sin(psi(i-1)) + 4*b*x_b*x_b*x_dot_b*x_dot_b/((x_b*x_b + 1)*(x_b*x_b + 1)) - 2*b*x_dot_b*x_dot_b/(x_b*x_b+1) + etayb*(-u(i-1)*r(i-1)*sin(psi(i-1)) - v(i-1)*r(i-1)*cos(psi(i-1)) - r(i-1)*r(i-1)*lb*cos(psi(i-1)));
        hb_cbf = (hb + (alpha1 + alpha2)*hb_dot + alpha1*alpha2*hb_dotdot)/(alpha1*alpha2);

        %CLF
        eta_x = x(i-1) + hp*cos(psi(i-1));
        eta_y = y(i-1) + hp*sin(psi(i-1));
        eta_x_dot = u(i-1)*cos(psi(i-1)) - v(i-1)*sin(psi(i-1)) - r(i-1)*hp*sin(psi(i-1));
        eta_y_dot = u(i-1)*sin(psi(i-1)) + v(i-1)*cos(psi(i-1)) + r(i-1)*hp*cos(psi(i-1));

        clf_x = -u_dot*cos(psi(i-1)) + v_dot*sin(psi(i-1)) + r_dot*hp*sin(psi(i-1)) + u(i-1)*r(i-1)*sin(psi(i-1)) + v(i-1)*r(i-1)*cos(psi(i-1)) + hp*r(i-1)*r(i-1)*cos(psi(i-1)) - 5*(eta_x_dot - xd);
        clf_y = -u_dot*sin(psi(i-1)) - v_dot*cos(psi(i-1)) - r_dot*hp*cos(psi(i-1)) - u(i-1)*r(i-1)*cos(psi(i-1)) + v(i-1)*r(i-1)*sin(psi(i-1)) + hp*r(i-1)*r(i-1)*sin(psi(i-1)) - 5*(eta_y_dot - yd);

        clfxd = (eta_x - eta_xd)*(eta_x - eta_xd);
        clfyd = (eta_y - eta_yd)*(eta_y - eta_yd);

        %CBF
        %% MILP CBF
        % Gurobi 모델 생성
        model = struct();

        % 변수 정의: x1은 -1, 0, 1 사이의 값, x2, x3는 실수 변수
        

        % 목적 함수 정의 (비용 함수가 제곱 형태)
        % x1^2 + x2^2 + x3^2 (이차 목적 함수)
        model.Q = sparse([1, 0, 0, 0, 0, 0, 0, 0, 0;
                          0, 1, 0, 0, 0, 0, 0, 0, 0;
                          0, 0, 100000, 0, 0, 0, 0, 0, 0;
                          0, 0, 0, 100000, 0, 0, 0, 0, 0;
                          0, 0, 0, 0, 10, 0, 0, 0, 0;
                          0, 0, 0, 0, 0, 1000, 0, 0, 0;
                          0, 0, 0, 0, 0, 0, 100000, 0, 0; ...
                          0, 0, 0, 0, 0, 0, 0, 10000000, 0; ...
                          0, 0, 0, 0, 0, 0, 0, 0, 10000000;]);  % 대각선에 1을 넣어서 각 변수를 제곱하는 형태로 설정
        model.obj = [0, 0, 0, 0, 0, 0, 0, 0, 0];  % 1차 항은 0 (단지 제곱 항만 목적 함수로 사용)


        % 제약 조건 정의
        model.A = sparse([        
            (eta_x - eta_xd)*control_update_dt, 0, 1, 0, 0, 0, 0, 0, 0;
            0, (eta_y - eta_yd)*control_update_dt, 0, 1, 0, 0, 0, 0, 0;
            -1, 0, 0, 0, cos(psi(i-1))/M, -hp*sin(psi(i-1))/I, -Fv*sin(psi(i-1))/M - Fv*bow*hp*sin(psi(i-1))/I, 0, 0;
            0, -1, 0, 0, sin(psi(i-1))/M, hp*cos(psi(i-1))/I, Fv*cos(psi(i-1))/M + Fv*bow*hp*cos(psi(i-1))/I, 0, 0;
            % 0, 0, 0, 0, -sin(psi(i-1))/M, -lf*cos(psi(i-1))/I, -Fv*cos(psi(i-1))/M - lf*bow*Fv*cos(psi(i-1))/I, 1, 0;
            % 0, 0, 0, 0, -sin(psi(i-1))/M, -lb*cos(psi(i-1))/I, -Fv*cos(psi(i-1))/M - lb*bow*Fv*cos(psi(i-1))/I, 0, 1;  
            0, 0, 0, 0, -(sin(psi(i-1)) + etayf*cos(psi(i-1)))/M, -lf*(cos(psi(i-1)) - etayf*sin(psi(i-1)))/I, -Fv*(cos(psi(i-1)) - etayf*sin(psi(i-1)))/M - lf*bow*Fv*(cos(psi(i-1)) - etayf*sin(psi(i-1)))/I, 1, 0;
            0, 0, 0, 0, -(sin(psi(i-1)) + etayb*cos(psi(i-1)))/M, -lb*(cos(psi(i-1)) - etayb*sin(psi(i-1)))/I, -Fv*(cos(psi(i-1)) - etayb*sin(psi(i-1)))/M - lb*bow*Fv*(cos(psi(i-1)) - etayb*sin(psi(i-1)))/I, 0, 1;              
        ]);

        model.rhs = [-alphac*clfxd - (eta_x - eta_xd)*xd;-alphac*clfyd- (eta_y - eta_yd)*yd; clf_x; clf_y; hf_cbf; hb_cbf];
        model.sense = ['<','<','=','=', '<','<'];  
        model.modelsense = 'min';  % 최소화 문제
        model.lb = [-inf; -inf; -inf; -inf; -50; -20; -1.5; -inf; -inf];  % 변수 각각 음수 값이 허용
        model.ub = [inf; inf; inf; inf; 50; 20; 1.5; inf; inf];  % 상한 설정
        model.vtype = 'CCCCCCICC';   

        % 모델 최적화 실행
        result = gurobi(model,struct('OutputFlag', 0));

        % disp(['x1 (실수 변수): ', num2str(result.x(1))]);
        % disp(['x2 (실수 변수): ', num2str(result.x(2))]);
        % disp(['x3 (실수 변수): ', num2str(result.x(3))]);
        disp(['CBF heading: ', num2str(hf)]);
        disp(['slack heading : ', num2str(result.x(8))]);
        disp(['slack back : ', num2str(result.x(9))]);

        tu = result.x(5);  % Surge force
        tr = result.x(6) + Fv*bow*result.x(7);%result.x(3) + Fv*bow*result.x(1);    % Yaw torque
        tv = Fv*result.x(7);
        xd = xd + result.x(1)*control_update_dt;
        yd = yd + result.x(2)*control_update_dt;

    end

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
    
    % Update states using Euler integration
    tau_u(i) = tu;
    tau_r(i) = tr;
    tau_v(i) = tv;
    
    x_dot = u(i-1)*cos(psi(i-1)) - v(i-1)*sin(psi(i-1));
    y_dot = u(i-1)*sin(psi(i-1)) + v(i-1)*cos(psi(i-1));
    u_dot = (-Xu*u(i-1) - Xuu*sqrt(u(i-1)*u(i-1))*u(i-1) + tau_u(i))/M;
    v_dot = (-Yv*v(i-1) -Yr*r(i-1) - Yvv*sqrt(v(i-1)*v(i-1))*v(i-1) + tau_v(i))/M;
    r_dot = (-Nv*v(i-1) -Nr*r(i-1) - Nrrr*r(i-1)*r(i-1)*r(i-1) + tau_r(i))/I;
    
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
    set(line_u, 'XData', t(1:i), 'YData', u(1:i));
    set(line_v, 'XData', t(1:i), 'YData', v(1:i));
    set(line_r, 'XData', t(1:i), 'YData', r(1:i));
    set(line_tau_u, 'XData', t(1:i), 'YData', tau_u(1:i));
    set(line_tau_r, 'XData', t(1:i), 'YData', tau_r(1:i));
    set(line_tau_v, 'XData', t(1:i), 'YData', tau_v(1:i));
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
