clear; clc; close all;

% Define 's' como a variável de Laplace (operador de transferência)
s = tf('s');

%% 1. Definição da Planta
% G(s) = 1,62 / (s^2 + 0,243*s + 0,006)
Kp_planta = 1.62;
den_planta_coefs = [1, 0.243, 0.006];
G = tf(Kp_planta, den_planta_coefs);

disp('F. Transf. da Planta G(s):');
disp(G);

%% 2. Requisitos de Desempenho
MS_desejado = 0.05; % 5%
tr_desejado = 8;    % 8s

disp(' ');
disp('Requisitos de Desempenho:');
fprintf('   --> Sobressinal (MS) < %.1f%%\n', MS_desejado*100);
fprintf('   --> Tempo de Subida (tr) < %.1fs\n', tr_desejado);

%% 3. Cálculo dos Polos Desejados
zeta_ideal = sqrt(log(MS_desejado)^2 / (pi^2 + log(MS_desejado)^2));
wn_ideal = (pi - acos(zeta_ideal)) / (tr_desejado * sqrt(1 - zeta_ideal^2));
polos_desejados = [-zeta_ideal*wn_ideal + 1j*wn_ideal*sqrt(1-zeta_ideal^2), ...
                   -zeta_ideal*wn_ideal - 1j*wn_ideal*sqrt(1-zeta_ideal^2)];

fprintf('\nFator de amortecimento ideal (zeta): %.3f\n', zeta_ideal);
fprintf('Frequência natural ideal (wn): %.3f rad/s\n', wn_ideal);
fprintf('Polos dominantes desejados: %.3f +/- j%.3f\n', real(polos_desejados(1)), imag(polos_desejados(1)));

%% 4. Projeto do Controlador PI
% C(s) = Kp + Ki/s
% O zero do controlador PI será posicionado para cancelar o polo mais lento da planta
% Polos da planta: roots([1, 0.243, 0.006]) = -0.0279 e -0.2151 -> polo mais lento é -0.0279.
% pensando em estabilidade posiciona-se o zero próximo ao polo mais lento, em -0.03.
zero_pi = -0.03;
polo_pi = 0; % ação integradora -> erro de regime nulo

% O controlador PI tem a forma C(s) = Kp * (s - zero_pi) / s
% A FT de MA C(s)G(s) = Kp * (s + 0.03) / s * (1.62 / (s^2 + 0.243s + 0.006))
% A equacao caracteristica é: 1 + C(s)G(s) = 0
% s * (s^2 + 0.243s + 0.006) + Kp * (s + 0.03) * 1.62 = 0
% s^3 + 0.243s^2 + 0.006s + 1.62*Kp*s + 1.62*Kp*0.03 = 0
% s^3 + 0.243s^2 + (0.006 + 1.62*Kp)s + 0.0486*Kp = 0

% A eq. caracteristica desejada é de 3 ordem, com os polos desejados e o polo remanescente da planta
polos_remanescentes = roots(den_planta_coefs);
polo_remanescente_planta = min(real(polos_remanescentes)); 

% Equação característica desejada: (s - p_desejado1)(s - p_desejado2)(s - p_remanescente)
polos_desejados_completo = [polos_desejados(1), polos_desejados(2), polo_remanescente_planta];
DCE = poly(polos_desejados_completo);

disp(' ');
disp('Equação Característica Desejada (DCE):');
fprintf('   s^3 + %.3fs^2 + %.3fs + %.3f = 0\n', DCE(2), DCE(3), DCE(4));

%% 5. Cálculo dos Ganhos Kp e Ki por Comparação de Coeficientes
% comparar a eq. real com a desejada: s^3 + 0.243s^2 + (0.006 + 1.62*Kp)s + 0.0486*Kp = 0
% s^3 + DCE(2)s^2 + DCE(3)s + DCE(4) = 0

% Calculando Kp
Kp = (DCE(3) - 0.006) / 1.62;

% Calculando Ki a partir de Kp e do termo constante
Ki = (0.0486 * Kp) / 1.62;

% safeguard de verificação
Ki_verificacao = Kp * (-zero_pi);

fprintf('\nGanhos do Controlador PI (Calculados):\n');
fprintf('   --> Ganho Proporcional (Kp): %.3f\n', Kp);
fprintf('   --> Ganho Integral (Ki): %.3f\n', Ki);
fprintf('   --> Ki_safeguard = %.3f\n', Ki_verificacao);


%% 6. Simulação e Resposta ao Degrau
% Controlador PI C(s)
C_PI = Kp + Ki/s;

% Sistema de MA
MF = feedback(C_PI * G, 1);

disp(' ');
disp('Resposta do Sistema em Malha Fechada com o Controlador PI:');
step(0.8 * MF); % Simulação para um degrau de 0.8m
title('Resposta ao Degrau de 0.8m com Controlador PI');
grid on;
stepinfo(0.8*MF)