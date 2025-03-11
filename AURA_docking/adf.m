clc;
clear;

% Gurobi 모델 생성
model = struct();

% 변수 정의: x1은 -1, 0, 1 사이의 값, x2, x3는 실수 변수
model.modelsense = 'min';  % 최소화 문제

% 목적 함수 정의 (비용 함수가 제곱 형태)
% x1^2 + x2^2 + x3^2 (이차 목적 함수)
model.Q = sparse([1, 0, 0;
                  0, 1, 0;
                  0, 0, 1]);  % 대각선에 1을 넣어서 각 변수를 제곱하는 형태로 설정
model.obj = [0, 0, 0];  % 1차 항은 0 (단지 제곱 항만 목적 함수로 사용)

% 제약 조건 정의
% x1 + 2*x2 <= 4
% 2*x1 + x3 <= 6
model.A = sparse([
    1, 2, 0;  % x1 + 2*x2 <= 4
    2, 0, 1;  % 2*x1 + x3 <= 6
    1, 0, 0;
    -1, 0, 0;
]);

model.rhs = [4; 6;1;1];  % 우변 벡터: b1 = 4, b2 = 6
model.sense = ['<'; '<';'<';'<'];  % 부등호 방향: <=

% 변수 타입 설정: x1은 정수, x2, x3은 실수
model.vtype = 'ICC';  % I: 정수, C: 연속 실수 (x1은 정수, x2, x3는 실수)



% 모델 최적화 실행
result = gurobi(model);

% 결과 출력
disp('최적화된 해:');
disp(['x1 (정수 변수): ', num2str(result.x(1))]);
disp(['x2 (실수 변수): ', num2str(result.x(2))]);
disp(['x3 (실수 변수): ', num2str(result.x(3))]);

disp(['최적화된 목적 함수 값: ', num2str(result.objval)]);

