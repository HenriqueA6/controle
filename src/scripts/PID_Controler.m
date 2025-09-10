clear; clc; close all;

%% 1. Definição da variável de Laplace para a F. Transf
s = tf('s');

% F. Transferência do modelo linearizado: G(s) = delta_H2(s) / delta_U(s)
G = 1.62 / (s^2 + 0.243*s + 0.006);
G.InputName = 'Vazão u';
G.OutputName = 'Nível h2';
disp('F. Transf. do modelo linearizado:');
display(G);

%% 2. Definição dos Ganhos do Controlador PID
% Ganhos calculados analiticamente pelo método do LGR com cancelamento de
% polo -> MS < 5% e tr < 8s.
Kp = 0.6; % atende requisitos em 0.6 -> aumentaman a agressividade da reação do sist., logo aumenta MS
Ki = 0.122; % atende requisitos em 0.122 -> elimina erro de reg. perm.
Kd = 1.75; % atende requisitos em 3 -> caráter preditivo sobre o erro: atenua ação de controle (aumenta o amortecimento), logo diminui o MS -> a partir de 2.3 o sist começa  a querer ter um carater de 1 ordem

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

%% 4. Simulação e Visualização da Resposta ao Degrau
% Simulação da resposta a um degrau unitário na referência
figure;
step(0.8*T);
title('Resposta ao Degrau do Sistema Controlado com PID (Modelo Linear)');
xlabel('Tempo');
ylabel('Amplitude da Saída (nível h2)');
grid on;
legend('Resposta de h2');

% Exibição das métricas de desempenho no Command Window
info = stepinfo(0.8*T);
MS = info.Overshoot;
zeta = sqrt(((log(MS/100))^2)/((pi^2)+((log(MS/100))^2)));
tr = info.RiseTime;
wn = (pi - acos(zeta))/ (tr*(sqrt(1-((zeta)^2)))); 
disp(' ');
disp('Métricas de Desempenho da Resposta ao Degrau (Linear):');
fprintf('Overshoot (MS): %.3f %%\n', MS);
fprintf('Tempo de Subida (tr): %.3f s\n', tr);
disp(' ');
disp('Parâmetros do sistema:');
fprintf('Fator de Amortecimento: %.3f \n', zeta);
fprintf('Frequência Natural: %.3f rad/s\n', wn);
