clear; clc; close all;

%% 1. Definição da Função de Transferência em Malha Aberta (Planta)
% G(s) = 1,62 / (s^2 + 0,243*s + 0,006)
Kp_planta = 1.62; % n(s)
den_planta = [1, 0.243, 0.006]; % d(s)

% Criação do objeto de função de transferência G(s)
G = tf(Kp_planta, den_planta);

disp('Função de Transferência da Planta G(s):');
disp(G);

%% 2. Definição dos Requisitos de Desempenho
% Sobressinal Máximo (MS) < 5%
% Tempo de Subida (tr) < 8s
% Erro em Regime Permanente: zero para entrada degrau (garantido pela ação integral do PID)

MS_desejado = 0.05; % MS em fração (5%)
tr_desejado = 8;    % Tempo de subida em segundos

disp(' ');
disp('Requisitos de Desempenho:');
fprintf('   - Sobressinal (MS) < %.1f%%\n', MS_desejado*100);
fprintf('   - Tempo de Subida (tr) < %.1fs\n', tr_desejado);

%% 3. Cálculo dos Parâmetros dos Polos Dominantes Desejados
% Fator de Amortecimento (zeta) baseado no MS
zeta_ideal = sqrt(log(MS_desejado)^2 / (pi^2 + log(MS_desejado)^2));
fprintf('\nFator de amortecimento ideal (zeta): %.3f\n', zeta_ideal);

% Frequência Natural (wn) baseada no tr e zeta
% Formula: tr = (pi - acos(zeta)) / (wn * sqrt(1-zeta^2))
wn_ideal = (pi - acos(zeta_ideal)) / (tr_desejado * sqrt(1 - zeta_ideal^2));
fprintf('Frequência natural ideal (wn): %.3f rad/s\n', wn_ideal);

% Posição dos Polos Desejados no plano s
sigma_d = zeta_ideal * wn_ideal;
wd_d = wn_ideal * sqrt(1 - zeta_ideal^2);
polos_desejados = [-sigma_d + 1j*wd_d, -sigma_d - 1j*wd_d];
fprintf('Polos dominantes desejados: %.3f +/- j%.3f\n', real(polos_desejados(1)), imag(polos_desejados(1)));

% Definição de um polo adicional rápido (para um sistema de 3a ordem)
%       5x mais rápido que a parte real dos polos dominantes.
polo_adicional = -5 * sigma_d;
fprintf('Polo adicional rápido: %.3f\n', polo_adicional);

%% 4. Cálculo da Equação Característica Desejada (DCE)
% Polos complexos conjugados: (s + sigma_d - j*wd_d)(s + sigma_d + j*wd_d) = s^2 + 2*sigma_d*s + (sigma_d^2 + wd_d^2)
polo_complexo = [1, 2*sigma_d, sigma_d^2 + wd_d^2]; % = [1, 2*sigma_d, wn_ideal^2]

% Polo real adicional: (s - polo_adicional) = (s + abs(polo_adicional))
polo_real = [1, -polo_adicional]; % = [1, abs(polo_adicional)]

% Multiplicação dos polinômios usando conv()
coefs_DCE = conv(polo_complexo, polo_real);

disp(' ');
disp('Eq. Característica Desejada (DCE):');
fprintf('   s^3 + %.3fs^2 + %.3fs + %.3f = 0\n', coefs_DCE(2), coefs_DCE(3), coefs_DCE(4));

%% 5. Cálculo dos Ganhos do PID por Comparação de Coeficientes
% Eq. Característica do sistema com PID é: s^3 + (a_p + Kp_planta*Kd)s^2 + (b_p + Kp_planta*Kp)s + Kp_planta*Ki = 0
% onde a_p = 0.243, b_p = 0.006 e Kp_planta = 1.62
% Comparando com a DCE: s^3 + A*s^2 + B*s + C = 0

A_desejado = coefs_DCE(2);
B_desejado = coefs_DCE(3);
C_desejado = coefs_DCE(4);

a_planta = den_planta(2);
b_planta = den_planta(3);
Kp_planta_val = Kp_planta;

% Calculando os ganhos
Kd = (A_desejado - a_planta) / Kp_planta_val;
Kp = (B_desejado - b_planta) / Kp_planta_val;
Ki = C_desejado / Kp_planta_val;

%% 6. Apresentação dos Ganhos do PID
disp(' ');
disp('Ganhos do Controlador PID (Calculados):');
fprintf('   -> Ganho Derivativo (Kd): %.3f\n', Kd);
fprintf('   -> Ganho Proporcional (Kp): %.3f\n', Kp);
fprintf('   -> Ganho Integral (Ki): %.3f\n', Ki);

% Criação do objeto de F. Transf. do PID
num_PID = [Kd, Kp, Ki];
den_PID = [1, 0]; % Polo na origem
C_PID = tf(num_PID, den_PID);

disp(' ');
disp('F. Transf. do Controlador PID C(s):');
disp(C_PID);

%% 7. Verificação da Eq. Característica em MF
% 1 + C(s)G(s) = 0  => s*den_planta + num_PID*Kp_planta = 0
% s^3 + (0.243 + 1.62*Kd)*s^2 + (0.006 + 1.62*Kp)*s + 1.62*Ki = 0
MF_den_calc = [1, (0.243 + 1.62*Kd), (0.006 + 1.62*Kp), (1.62*Ki)];

disp(' ');
disp('Verificação: Eq. Característica do Sistema em MF:');
fprintf('   s^3 + %.3fs^2 + %.3fs + %.3f = 0\n', MF_den_calc(2), MF_den_calc(3), MF_den_calc(4));

%% 8. Análise da Resposta ao Degrau
% Sistema em MF
T_MF = feedback(C_PID*G, 1);

figure;
step(0.8*T_MF);
title('Resposta ao Degrau do Sistema em MF');
grid on;

% Informações sobre a resposta
stepinfo(0.8*T_MF)