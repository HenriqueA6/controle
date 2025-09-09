
clear; clc; close all;

%% 1. Definição da variável de Laplace para a F. Transf
s = tf('s');

% F. Transferência do modelo linearizado: G(s) = delta_H2(s) / delta_U(s)
G = 1.62 / (s^2 + 0.081*s - 0.006561);
G.InputName = 'Vazão u';
G.OutputName = 'Nível h2';

disp('Função de Transferência da Planta (Linearizada):');
display(G);

%% 2. Definição dos Ganhos do Controlador PI
% Ganhos calculados para priorizar o sobressinal baixo (xi = 0.7)
% através do cancelamento do polo estável da planta.
Kp = 0.00069;
Ki = 8.8e-5;

% Criação do objeto do controlador PI
C = pid(Kp, Ki); % Kd = 0

disp('Função de Transferência do Controlador PI:');
display(C);

%% 3. Análise em Malha Fechada
L = C * G;
T = feedback(L, 1);

polos_mf = pole(T);
disp(' ');
disp('Polos de malha fechada com o controlador PI:');
disp(polos_mf);

%% 4. Simulação e Visualização da Resposta ao Degrau
figure;
step(T);
title('Resposta ao Degrau com Controlador PI (Priorizando Baixo Sobressinal)');
xlabel('Tempo (segundos)');
ylabel('Amplitude da Saída (nível h2)');
grid on;
legend('Resposta de h2');

% Exibição das métricas de desempenho ~ bem  paia.
info = stepinfo(T);
disp(' ');
disp('Métricas de Desempenho da Resposta ao Degrau (PI):');
fprintf('Sobressinal (Overshoot): %.2f %%\n', info.Overshoot);
fprintf('Tempo de Assentamento (Settling Time, 2%%): %.2f s\n', info.SettlingTime);
fprintf('Tempo de Subida (Rise Time): %.2f s\n', info.RiseTime);