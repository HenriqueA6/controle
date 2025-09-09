clear; clc; close all;

% Definição da variável de Laplace para a função de transferência
s = tf('s');

% Função de Transferência da planta linearizada
G = 1.62 / (s^2 + 0.081*s - 0.006561);
G.InputName = 'Vazão u';
G.OutputName = 'Nível h2';

%% 2. Definição dos Ganhos do Controlador PID
% Ganhos calculados analiticamente pelo método do LGR com cancelamento de polo.
Kp = 1;
Ki = 0.09;
Kd = 2.25;

% Criação do objeto do controlador PID
C = pid(Kp, Ki, Kd);

%% 3. Análise em Malha Fechada
% Sistema em malha aberta (controlador + planta)
L = C * G;

% Sistema em malha fechada (realimentação unitária)
T = feedback(L, 1);

% Exibição dos polos de malha fechada para verificação
polos_mf = pole(T);
disp('Polos de malha fechada:');
disp(polos_mf);

%% 4. Simulação e Visualização da Resposta ao Degrau
% Simulação da resposta a um degrau unitário na referência
figure;
step(T);
title('Resposta ao Degrau do Sistema Controlado com PID');
xlabel('Tempo (segundos)');
ylabel('Amplitude da Saída (nível h2)');
grid on;
legend('Resposta de h2');

% Exibição das métricas de desempenho no Command Window
info = stepinfo(T);
disp(' ');
disp('Métricas de Desempenho da Resposta ao Degrau:');
fprintf('Sobressinal (Overshoot): %.2f %%\n', info.Overshoot);
fprintf('Tempo de Assentamento (Settling Time): %.2f s\n', info.SettlingTime);
fprintf('Tempo de Subida (Rise Time): %.2f s\n', info.RiseTime);