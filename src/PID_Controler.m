clear; clc; close all;

% Definição da variável de Laplace para a F. Transferência
s = tf('s');

% F. Transferência do modelo linearizado: G(s) = delta_H2(s) / delta_U(s)
G = 1.62 / (s^2 + 0.081*s - 0.006561);
G.InputName = 'Vazão u';
G.OutputName = 'Nível h2';
disp('F. Transf. do modelo linearizado:');
display(G);

%% 2. Definição dos Ganhos do Controlador PID
% Ganhos calculados analiticamente pelo método do LGR com cancelamento de
% polo -> MS < 5% e ts < 8s.
Kp = 2.51; % aumenta a agressividade da reação do sist., logo aumenta MS
Ki = 0.64; % elimina erro de reg. perm. 
Kd = 5.37; % caráter preditivo sobre o erro -> atenua ação de controle -> aumenta o amortecimento, logo diminui o MS

% controlador PID
C = pid(Kp, Ki, Kd);

disp('F. Transf. Controlador PID:');
display(C);

%% 3. Análise em MF
% Sistema em malha aberta (controlador + planta)
L = C * G;

% Sistema em MF (realimentação unitária)
T = feedback(L, 1);

% Exibição dos polos de malha fechada para verificação
polos_mf = pole(T);
disp(' ');
disp('Polos de MF:');
disp(polos_mf);
disp('Polos Dominantes devem estar próximos de -0.5 +/- 0.51j');

%% 4. Simulação e Visualização da Resposta ao Degrau
% Simulação da resposta a um degrau unitário na referência
figure;
step(T);
title('Resposta ao Degrau do Sistema Controlado com PID (Modelo Linear)');
xlabel('Tempo (segundos)');
ylabel('Amplitude da Saída (nível h2)');
grid on;
legend('Resposta de h2');

% Exibição das métricas de desempenho no Command Window
info = stepinfo(T);
disp(' ');
disp('Métricas de Desempenho da Resposta ao Degrau (Linear):');
fprintf('Overshoot (MS): %.2f %%\n', info.Overshoot);
fprintf('Tempo de Assentamento - Settling Time (tr): %.2f s\n', info.SettlingTime);
