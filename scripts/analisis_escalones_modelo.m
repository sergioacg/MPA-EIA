% analisis_escalones_modelo.m  -  Ganancia K, matrices A, B, C, retardo y validacion
% Manual de la MPA (2026-2), seccion 8.4-8.5. Sistemas de Control en Tiempo Continuo, EIA.
% Se ejecuta DESPUES de analisis_pulsos.m (usa y, t, Ts, wn y zeta).

ym  = @(a,b) mean(y(t >= a & t < b));                       % promedio en [a, b)
Ksub = (ym(tSub+Tesp-5, tSub+Tesp) - ym(tSub-5, tSub)) / dU;
Kbaj = (ym(tBaj+Tesp-5, tBaj+Tesp) - ym(tBaj-5, tBaj)) / (-dU);
fprintf('K subiendo = %.3f   K bajando = %.3f  grados/%%\n', Ksub, Kbaj)

% --- parte 2: matrices, retardo y validacion
K = Ksub;
A = [0 1; -wn^2 -2*zeta*wn];   B = [0; K*wn^2];   C = [1 0];   D = 0;
G = ss(A, B, C, D, 'StateName', {'theta','omega'}, 'StateUnit', {'grados','grados/s'}, ...
       'InputName', 'u', 'InputUnit', '%', 'OutputName', 'theta', 'OutputUnit', 'grados');
% --- datos de validacion: escalon de bajada, en desviacion (u aplicado de verdad)
u  = double(u_ap(:));
w  = t >= tBaj-5 & t < tBaj+Tesp;
tv = t(w) - t(find(w,1));
dy = y(w) - ym(tBaj-5, tBaj);    du = u(w) - u0;
% --- retardo: el que mejor alinea modelo y medida
dl = 0:Ts:0.3;   J = zeros(size(dl));
for i = 1:numel(dl)
    G.InputDelay = dl(i);   J(i) = norm(dy - lsim(G, du, tv));
end
[~, ib] = min(J);  retardo = dl(ib);  G.InputDelay = retardo;
ys  = lsim(G, du, tv);
fit = 100*(1 - norm(dy - ys)/norm(dy - mean(dy)));
plot(tv, dy, tv, ys, '--', 'LineWidth', 1.2); grid on
legend('medido', 'modelo'); xlabel('t [s]'); ylabel('\Delta\theta [grados]')
fprintf('retardo = %.2f s   fit de validacion = %.0f %%\n', retardo, fit)
save(sprintf('MPA%02d_modelo_%s.mat', modulo, datestr(now,'yyyymmdd')), ...
     'A','B','C','D','wn','zeta','K','u0','retardo','fit')
