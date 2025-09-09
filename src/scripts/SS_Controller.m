clear; clc; close all;
%% 1.Matrizes do Espaço de Estados: x_dot = Ax + Bu, y = Cx + Du
% Onde x = [delta_h1; delta_h2]
A = [-0.081, 0.081; 0.081, -0.162];
B = [20; 0];
C = [0, 1];
D = 0;

% Espaço de estados
sys_ma = ss(A, B, C, D);
sys_ma.StateName = {'delta_h1', 'delta_h2'};
sys_ma.InputName = {'delta_u'};
sys_ma.OutputName = {'delta_h2'};

fprintf('Matriz A:\n');
disp(A);
fprintf('Matriz B:\n');
disp(B);
fprintf('Sistema em malha aberta definido.\n\n');

%% 2. REQUISITOS DE DESEMPENHO E ALOCAÇÃO DE POLOS
disp('--> Obtendo Polos desejados a partir dos requisitos de desempenho...');

% Requisitos de desempenho para a malha fechada
MS_max = 5;      % MS máximo [%]
ts_max = 8;      % tr (critério 2%) [s]

% Cálculo do fator de amortecimento (xi) a partir do sobressinal
xi = -log(MS_max / 100) / sqrt(pi^2 + log(MS_max / 100)^2);
fprintf('Fator de amortecimento mínimo (xi): %.3f\n', xi);

% A partir do tempo de assentamento, xi*wn > 4/ts
xi_wn = 4 / ts_max;
fprintf('Parte real mínima dos polos (xi*wn): %.3f\n', xi_wn);

% Escolha de projeto 
xi_proj = 0.7; % -> xi >= 0.69
xi_wn_proj = 0.6; % -> xi*wn > 0.5

% Cálculo da frequência natural (wn) e frequência amortecida (wd)
wn_proj = xi_wn_proj / xi_proj;
wd_proj = wn_proj * sqrt(1 - xi_proj^2);

fprintf('Polo do controlador projetado para: xi=%.2f, wn=%.2f rad/s\n', xi_proj, wn_proj);

p_controlador = [-xi_wn_proj + wd_proj*1i, -xi_wn_proj - wd_proj*1i];
fprintf('Polos do controlador alocados em: %.3f +/- %.3fi\n\n', -xi_wn_proj, wd_proj);


%% 3. PROJETO DO REGULADOR
% Objetivo: Encontrar K tal que os autovalores de (A-BK) sejam p_controlador -> lei de controle é u = -K*x.
disp('--> Projetando o ganho de realimentação de estados K...');

K = place(A, B, p_controlador);

fprintf('Ganho de realimentação de estados (K) calculado:\n');
disp(K);
fprintf('K = [%.4f, %.4f]\n\n', K(1), K(2));

%% 4. PROJETO DO OBSERVADOR DE ESTADOS
% Objetivo: Encontrar Ke tal que os autovalores de (A-KeC) sejam estáveis -> lei de controle com o observador é u = -K*x_hat.
disp('--> Projetando o ganho do observador de estados Ke...');

% Regra de Ouro: polos do observador de 5 a 10 VEZES MAIS RÁPIDOS
fator_rapidez = 5;
p_observador = fator_rapidez * real(p_controlador(1));
p_observador = [p_observador, p_observador - 0.1]; % Polos reais e distintos

fprintf('Polos do observador alocados em: %.2f e %.2f\n', p_observador(1), p_observador(2));

% O cálculo de Ke é dual ao de K. Usa-se (A', C') e o resultado é transposto.
Ke = place(A', C', p_observador)';

fprintf('Ganho do observador (Ke) calculado:\n');
disp(Ke);
fprintf('\n');

%% 5. PROJETO DA PRÉ-COMPENSAÇÃO PARA RASTREAMENTO DE REFERÊNCIA
% Objetivo: Calcular os ganhos Nx e Nu para garantir erro nulo em regime permanente para uma referência degrau.
disp('--> Calculando ganhos Nx e Nu...');

% Monta a matriz aumentada para resolver o sistema
M = [A, B; C, D];

% Resolve para [Nx; Nu] = M^(-1) * [0; 0; ...; 1]
N_bar = inv(M) * [zeros(size(A, 1), 1); 1];

Nx = N_bar(1:size(A, 1));
Nu = N_bar(end);

fprintf('N-ganhos calculados:\n');
fprintf('Nx:\n');
disp(Nx);
fprintf('Nu: %.4f\n\n', Nu);

disp('Ganhos K, Ke, Nx e Nu Calculados!');