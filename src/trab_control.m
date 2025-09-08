%valores atuais de requisitos calculados a partir da funcao de
%transferência
wn_atual = 0.077;
csi_atual = 0.364;
pm_atual = 3.64;
ms_atual = 14.328;
tr_atual = 32.612;
mag_atual = 0.0269;
mag_esperada = 19.9;

% frequencia de corte no 0bd atual = 1,27 rad/s --> jogando no grafico de
% fase - 176º

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


s = tf('s');

G = 1.62/ ((s - 0.047)*(s+0.128)); 

bode(G)

%ajustando o ganho k
ganho_db = mag_atual - mag_esperada;
k = 10^(-19.8731/20);
fprintf('\nparametros para ajuste:\n');
fprintf('controlador proporcionar com ganho k para ajuste da frequencia de corte:\nk = %f',k);

%projetando lead:
phi_max = pm_ideal - pm_atual;
alpha = ((1- sind(phi_max))/(1+ sind(phi_max)));
Kc = 0.69 ; %Kc = sqrt(alpha);
T = 1/(wn*Kc);
fprintf('\n \nparametros do lead:\n');
fprintf('phi max = %f\nalpha = %f\nKc = %f\nT = %f',phi_max,alpha,Kc,T);
%C =( ((Kc*T)*s + 1) / ((((alpha)*T)*s) + 1)) ;




