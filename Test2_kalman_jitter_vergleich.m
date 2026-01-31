%% =====================  Jitter-Analyse der Trajektorie  ==========================
% Verwendete Datenstruktur:
% log.green_raw  (Nx2)
% log.green_filt (Nx2)
% log.t          (Nx1)   Zeitstempel (optional)

clc;

load('log_kalman_session_20251210_124453.mat');

P_raw  = log.green_raw;    % ungefilterte Trajektorie
P_kf   = log.green_filt;   % Kalman-gefilterte Trajektorie

%% 1. NaN entfernen (falls in manchen Frames keine Detektion vorhanden ist)
valid_raw = all(~isnan(P_raw),2);
valid_kf  = all(~isnan(P_kf),2);

P_raw = P_raw(valid_raw,:);
P_kf  = P_kf(valid_kf,:);

%% 2. Frame-zu-Frame-Verschiebung (Schrittweite) berechnen
d_raw = vecnorm(diff(P_raw), 2, 2);   % Raw: Schrittweite pro Frame
d_kf  = vecnorm(diff(P_kf),  2, 2);   % KF:  Schrittweite pro Frame

%% 3. Jitter berechnen (Standardabweichung der Schrittweite)
jitter_raw = std(d_raw);
jitter_kf  = std(d_kf);

fprintf("=========== Jitter der grünen Trajektorie ===========\n");
fprintf("Raw-Jitter : %.6f m\n", jitter_raw);
fprintf("KF-Jitter  : %.6f m\n", jitter_kf);
fprintf("Verbesserung: %.2f %%\n", (1 - jitter_kf/jitter_raw)*100);

%% 4. Jitter-Vergleich plotten
figure;
subplot(2,1,1);
plot(d_raw, 'r'); hold on;
plot(d_kf, 'g');
legend('Raw Schrittweite', 'Kalman Schrittweite');
xlabel('Frame-Index'); ylabel('Schrittweite (m)');
title('Vergleich der Schrittweite (Frame-zu-Frame-Verschiebung)');

subplot(2,1,2);
bar([jitter_raw, jitter_kf]);
set(gca,'XTickLabel',{'Raw','Kalman'});
ylabel('Jitter (Std. der Schrittweite)');
title('Jitter-Vergleich');
grid on;
