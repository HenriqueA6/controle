clear; clc; close all;

%valores atuais de requisitos calculados a partir da f. transferencia
wn_atual = 1.26;   % 0.077
csi_atual = 0.158;
pm_atual = 10.9;
ms_atual = 60.49;
tr_atual = 22.746;

mag_atual = 0.0269;
mag_esperada = 18.72; % wn = 0.4 tem 18.72 de mag. aproximadamente

%requisitos
tr = 8;
msideal = 5;
csi_ideal= sqrt(((log(msideal/100))^2)/((pi^2)+((log(msideal/100))^2)));
wn = (pi - acos(csi_ideal))/ (tr*(sqrt(1-((csi_ideal)^2))));

% margem de fase ideal:
pm_ideal = 100* csi_ideal;
% angulo de corte
ang_corte = 180 - pm_ideal;

% tabela de requisitos:
% margem de fase:
fprintf('requisitos do sistema:\n');
fprintf('MS = 5\nTr = 8s\n');
fprintf('PM = %f\nCsi =%f\nWn = %f\n', pm_ideal,csi_ideal,wn)

%tabela de requisitos atuais:
fprintf('\nValores atuais:\n');
fprintf('MS = %f\nTr = %f\nPM = %f\nCsi =%f\nWn = %f\n',ms_atual,tr_atual,pm_atual,csi_atual,wn_atual)

%funcao transferencia da nossa planta linearizada
s = tf('s');
G = 1.62/ (s^2 + 0.243*s + 0.006); 

figure
bode(G)

%1º passo: vamos deslocar o grafico para baixo para assim atender wn necessario
%ajustando o ganho k
ganho_db = mag_atual - mag_esperada;
k = 10^(ganho_db/20);                     % 0<K<1 --> para poder descer o gráfico
H =(k* 1.62)/ (s^2 + 0.243*s + 0.006);    % H = k*G

figure  %apos o ajuste de ganho k 
bode(H)

fprintf('\nparametros para ajuste:\n');
fprintf('controlador proporcionar com ganho k para ajuste da frequencia de corte:\nk = %f',k);

%parametro de wn atendido, com wn = 0.402 
%apos o ganho k temos a seguinte pm:
pm_atualk = 31.7;

%2º passo: projetar um controlador lead para adequar a PM necessária:

%projetando lead:
phi_max = pm_ideal - pm_atualk;
alpha = ((1- sind(phi_max))/(1+ sind(phi_max)));
Kc = 0.49 ; %Kc = sqrt(alpha);
T = 1/(wn*Kc);

fprintf('\n \nparametros do lead:\n');
fprintf('phi max = %f\nalpha = %f\nKc = %f\nT = %f',phi_max,alpha,Kc,T);

%funcao do lead:
figure
bode(C)
C = ((0.4)*(T*s+0.19907))/( alpha*T*s + 0.822);

% funcao do K*G*Clead
E = (1.62/ (s^2 + 0.243*s + 0.006))*(0.1162) * (2.16 *((s+ 0.19907)/(s + 0.882)));

figure
bode(E)

%3º PASSO: projetar um controlador lag para adequar aos requisitos
%restantes, como ess = 0 para entradas degrau

%funcao transferencia do sistema final, k*Clead*Clag*G:
L = (1.62 / (s^2 + 0.243*s + 0.006)) * (0.2509 * (s + 0.19907) / (s + 0.882)) * ((s + 0.04) / s);

figure
bode(L)








