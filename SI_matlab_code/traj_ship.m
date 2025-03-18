clc; clear; close all;

% 데이터를 불러옵니다.
data = readtable('030622.csv'); % 데이터 파일 경로
x = data.Var2;  % x 값 (데이터에서 추출)
y = data.Var3;  % y 값 (데이터에서 추출)
psi = data.Var4;
u = data.Var5;
v = data.Var6;
r = data.Var7;
u1 = data.Var14;
u2 = data.Var15;

% 최적화된 파라미터 값
Xu = 0.10531;
Xuu = 0.018405;
b1 = 0.00058466;

Yv = 8.7185e-09;
Yvv = 0.39199;
Yn = 1.1508e-08;
Nr = 9.1124e-08;
Nrr = 5.2726;
Nv = 6.1558e-09;
b2 = 0.0040635;
b3 = 0.31094;

dt = 0.1; 
start_idx = 5000;  % 시작 인덱스
end_idx = 10000;    % 끝 인덱스

% 시간 배열 생성 (dt = 0.1)
t = (start_idx:end_idx) * dt;  % 시간 배열 (0.1초 간격)

% 시간 영역에 해당하는 x, y 값 추출
x_selected = x(start_idx:end_idx);
y_selected = y(start_idx:end_idx);
psi_selected = psi(start_idx:end_idx);  % 실제 psi 값
u_selected = u(start_idx:end_idx);
v_selected = v(start_idx:end_idx);
r_selected = r(start_idx:end_idx);
u1_selected = u2(start_idx:end_idx);
u2_selected = u1(start_idx:end_idx);

pwm1_selected = arrayfun(@(x) convertThrustToPwm(x), u1_selected);
pwm2_selected = arrayfun(@(x) convertSteeringToPwm(x), u2_selected);

% 예측 값 초기화
psi_predicted = zeros(1, length(x_selected));
u_predicted = zeros(1, length(x_selected));
v_predicted = zeros(1, length(x_selected));
r_predicted = zeros(1, length(x_selected));
x_predicted = zeros(1, length(x_selected));
y_predicted = zeros(1, length(x_selected));

u_predicted(1) = u_selected(1); % 초기값 설정
v_predicted(1) = v_selected(1);
r_predicted(1) = r_selected(1);
x_predicted(1) = x_selected(1);
y_predicted(1) = y_selected(1);
psi_predicted(1) = psi_selected(1);

for i = 2:length(x_selected)
    % 시스템 동역학을 기반으로 다음 상태를 예측 (최적화된 계수 사용)
    u_predicted(i) = (-Xu*u_predicted(i-1) - Xuu*abs(u_predicted(i-1))*u_predicted(i-1) + b1*pwm1_selected(i-1)*pwm1_selected(i-1)*cos(b2*pwm2_selected(i-1)))*dt + u_predicted(i-1);    
    v_predicted(i) = (-Yv*v_predicted(i-1) - Yvv*abs(v_predicted(i-1))*v_predicted(i-1) - Yn*r_predicted(i-1) + b1*pwm1_selected(i)*pwm1_selected(i-1)*sin(b2*pwm2_selected(i-1)))*dt + v_predicted(i-1);
    r_predicted(i) = (-Nr*r_predicted(i-1) - Nrr*abs(r_predicted(i-1))*r_predicted(i-1) - Nv*v_predicted(i-1) - b3*b1*pwm1_selected(i-1)*pwm1_selected(i-1)*sin(b2*pwm2_selected(i-1)))*dt + r_predicted(i-1);
    
    % psi 값을 -pi와 pi 사이로 제한
    psi_predicted(i) = wrapToPi(psi_predicted(i-1) + r_predicted(i)*dt);
    
    % x, y 값 예측
    x_predicted(i) = x_predicted(i-1) + (u_predicted(i)*cos(psi_predicted(i)) - v_predicted(i)*sin(psi_predicted(i)))*dt;
    y_predicted(i) = y_predicted(i-1) + (u_predicted(i)*sin(psi_predicted(i)) + v_predicted(i)*cos(psi_predicted(i)))*dt; 
end

% x, y 값의 최소값과 최대값 구하기
x_min = min([x_selected', x_predicted]);
x_max = max([x_selected', x_predicted]);
y_min = min([y_selected', y_predicted]);
y_max = max([y_selected', y_predicted]);

% x, y 축 범위 설정 (각각 5만큼 더하고 빼기)
x_range = [x_min - 10, x_max + 10];
y_range = [y_min - 10, y_max + 10];

figure;

% 왼쪽: 궤적 비교 (전체 영역을 차지)
subplot(6,2,[1, 3, 5, 7, 9]); % 1행 2열 중 첫 번째 subplot
plot(x_selected, y_selected, 'b', 'LineWidth', 0.5, 'DisplayName', 'traj actual'); hold on;
plot(x_predicted, y_predicted, 'r', 'LineWidth', 0.5, 'DisplayName', 'traj pred');
xlabel('x');
ylabel('y');
title('Comparison of Actual and Predicted Trajectory');
grid on;
axis([x_range, y_range]); % x, y 축 범위 설정
axis equal;

% 오른쪽: 상태 변수 비교 (2행 3열로 나누어 표시)
subplot(6,2,2); % 첫 번째 subplot (psi 비교)
plot(t, psi_selected, 'b', 'DisplayName', 'psi actual'); hold on;
plot(t, psi_predicted, 'r--', 'DisplayName', 'psi predicted');
xlabel('Time [s]');
ylabel('psi');
legend;
title('Comparison of psi actual and psi predicted');

subplot(6,2,4); % 두 번째 subplot (u 비교)
plot(t, u_selected, 'b', 'DisplayName', 'u actual'); hold on;
plot(t, u_predicted, 'r--', 'DisplayName', 'u predicted');
xlabel('Time [s]');
ylabel('u');
legend;
title('Comparison of u actual and u predicted');

subplot(6,2,6); % 세 번째 subplot (v 비교)
plot(t, v_selected, 'b', 'DisplayName', 'v actual'); hold on;
plot(t, v_predicted, 'r--', 'DisplayName', 'v predicted');
xlabel('Time [s]');
ylabel('v');
legend;
title('Comparison of v actual and v predicted');

subplot(6,2,8); % 네 번째 subplot (r 비교)
plot(t, r_selected, 'b', 'DisplayName', 'r actual'); hold on;
plot(t, r_predicted, 'r--', 'DisplayName', 'r predicted');
xlabel('Time [s]');
ylabel('r');
legend;
title('Comparison of r actual and r predicted');

subplot(6,2,10); % 다섯 번째 subplot (u1 비교)
plot(t, pwm1_selected, 'b', 'DisplayName', 'u1 actual'); hold on;
xlabel('Time [s]');
ylabel('u1');
legend;
title('Comparison of u1 actual and u1 predicted');

subplot(6,2,12); % 여섯 번째 subplot (u2 비교)
plot(t, pwm2_selected, 'b', 'DisplayName', 'u2 actual'); hold on;
xlabel('Time [s]');
ylabel('u2');
legend;
title('Comparison of u2');