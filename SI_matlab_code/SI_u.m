clc; clear; close all;

% 데이터를 불러옵니다.
data = readtable('030623_1.csv'); % 데이터 파일 경로
psi = data.Var4;
u = data.Var5;
v = data.Var6;
r = data.Var7;
u1 = data.Var14;
u2 = data.Var15;

% 원하는 영역을 설정합니다.
start_idx = 5100; % 시작 인덱스
end_idx = 6500; % 끝 인덱스
dt = 0.1; % 시간 간격
t = (start_idx:end_idx)*dt; % 시간 배열 (선택된 구간)

% 선택된 구간에서 데이터 추출
psi_selected = psi(start_idx:end_idx);
u_selected = u(start_idx:end_idx);
v_selected = v(start_idx:end_idx);
r_selected = r(start_idx:end_idx);
u2_selected = u1(start_idx:end_idx);
u1_selected = u2(start_idx:end_idx);

pwm1_selected = arrayfun(@(x) convertThrustToPwm(x), u1_selected);
pwm2_selected = arrayfun(@(x) convertSteeringToPwm(x), u2_selected);

% YALMIP 변수를 정의합니다.
Xu = sdpvar(1,1); Xuu = sdpvar(1,1);
b1 = sdpvar(1,1); b2 = sdpvar(1,1); 

% 초기 추정값 설정
initial_guess = [0; 0; 0; 0]; % Xu, Xuu, Yv, Yvv, Yn, Nr, Nrr, Nv, b1, b2

% 최적화 문제 설정
objective = @(params) computeObjective(params, u_selected, pwm1_selected, pwm2_selected, dt);

% 제약 조건 설정: Xu > 0, Xuu > 0
nonlcon = @(params) constraints(params);  % 비선형 제약 함수

% 최적화 옵션
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'interior-point');

% fmincon을 사용하여 최적화 수행
[opt_params, fval] = fmincon(objective, initial_guess, [], [], [], [], [], [], nonlcon, options);

% 최적화된 파라미터 추출
Xu_opt = opt_params(1);
Xuu_opt = opt_params(2);
b1_opt = opt_params(3);
b2_opt = opt_params(4);


disp('Optimized Parameters:');
disp(['Xu: ', num2str(Xu_opt)]);
disp(['Xuu: ', num2str(Xuu_opt)]);
disp(['b1: ', num2str(b1_opt)]);
disp(['b2: ', num2str(b2_opt)]);


% 예측된 u, v, r 값 계산 (모델에 의해 예측된 값)
u_predicted = zeros(1, length(t));

u_predicted(1) = u_selected(1); % 초기값 설정

for i = 2:length(t)
    % 시스템 동역학을 기반으로 다음 상태를 예측 (최적화된 계수 사용)
    u_predicted(i) = (-Xu_opt*u_predicted(i-1) - Xuu_opt*abs(u_predicted(i-1))*u_predicted(i-1) + b1_opt*pwm1_selected(i-1)*pwm1_selected(i-1)*cos(b2_opt*pwm2_selected(i-1)))*0.1 + u_predicted(i-1);
end

% 실제 값과 예측 값을 비교하는 그래프 그리기
figure;

% u 값 비교
subplot(3,1,1);
plot(t, u_selected, 'b', 'DisplayName', 'u actual'); hold on;
plot(t, u_predicted, 'r--', 'DisplayName', 'u predicted');
xlabel('Time [s]');
ylabel('u');
legend;
title('Comparison of u actual and u predicted');

% u1 값 비교
subplot(3,1,2);
plot(t, pwm1_selected, 'b', 'DisplayName', 'u1 actual'); hold on;
xlabel('Time [s]');
ylabel('u1');
legend;
title('Comparison of u1 actual and u1 predicted');

% u2 값 비교
subplot(3,1,3);
plot(t, pwm2_selected, 'b', 'DisplayName', 'u2 actual'); hold on;
xlabel('Time [s]');
ylabel('u2');
legend;
title('Comparison of u2 actual and u2 predicted');

% 목적 함수 정의 (최소화하려는 오차 함수)
function objective = computeObjective(params, u_selected, pwm1_selected, pwm2_selected, dt)
    Xu = params(1);
    Xuu = params(2);
    b1 = params(3);
    b2 = params(4);

    objective = 0;
    u_next = u_selected(1);
    
    for i = 1:length(u_selected)-1
        % 시스템 동역학을 기반으로 다음 상태를 예측
        u_next = (-Xu*u_next - Xuu*abs(u_next).*u_next + b1*pwm1_selected(i)*pwm1_selected(i)*cos(b2*pwm2_selected(i)))*dt + u_next;
        % 실제 변화량과 예측값의 차이를 최소화하는 오차 함수
        u_diff = (u_next - u_selected(i+1))^2;
        % 오차 함수 추가
        objective = objective + 100*u_diff;
    end
end

% 제약 조건 함수 (Xu > 0, Xuu > 0)
function [c, ceq] = constraints(params)
    Xu = params(1);
    Xuu = params(2);

    % Xu > 0, Xuu > 0 의 조건
    c = [-Xu; -Xuu];  % 부등식 제약: Xu > 0, Xuu > 0
    ceq = [];  % 평등 제약은 없음
end


