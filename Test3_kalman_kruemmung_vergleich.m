%% =============== Krümmungs-Kontinuitätsanalyse (Raw vs. Kalman) ===============
% Benötigte Datenstruktur:
% log.t           (Nx1)
% log.green_raw   (Nx2)   [x_raw, y_raw]
% log.green_filt  (Nx2)   [x_kf,  y_kf]

clc;

load('log_kalman_session_20251210_124830.mat');

t_all   = log.t(:);
P_raw   = log.green_raw;
P_filt  = log.green_filt;

% NaN-Frames entfernen (wenn kein Punkt detektiert wurde)
valid_raw  = all(~isnan(P_raw),2);
valid_filt = all(~isnan(P_filt),2);

t_raw   = t_all(valid_raw);
P_raw   = P_raw(valid_raw,:);
t_kf    = t_all(valid_filt);
P_kf    = P_filt(valid_filt,:);

%% Hilfsfunktion: Krümmung aus t und P berechnen
compute_curvature = @(t, P) local_compute_curvature(t, P);

[k_raw, t_raw_mid]  = compute_curvature(t_raw,  P_raw);
[k_kf,  t_kf_mid]   = compute_curvature(t_kf,   P_kf);

%% Nur den überlappenden Zeitbereich behalten (fairer Vergleich)
t_min = max([t_raw_mid(1),  t_kf_mid(1)]);
t_max = min([t_raw_mid(end),t_kf_mid(end)]);

idx_raw = (t_raw_mid >= t_min) & (t_raw_mid <= t_max);
idx_kf  = (t_kf_mid  >= t_min) & (t_kf_mid  <= t_max);

k_raw_c = k_raw(idx_raw);
k_kf_c  = k_kf(idx_kf);

%% Kennzahlen: mittlere Krümmung & Standardabweichung & normierte Schwankung
mean_k_raw = mean(k_raw_c);
mean_k_kf  = mean(k_kf_c);

std_k_raw  = std(k_raw_c);
std_k_kf   = std(k_kf_c);

rel_std_raw = std_k_raw / mean_k_raw;
rel_std_kf  = std_k_kf  / mean_k_kf;

fprintf("============= Krümmungs-Kontinuität (Grün) =============\n");
fprintf("Mittlere Krümmung (Raw) : %.6f  1/m\n", mean_k_raw);
fprintf("Mittlere Krümmung (KF)  : %.6f  1/m\n", mean_k_kf);
fprintf("Std. Krümmung (Raw)     : %.6f  1/m\n", std_k_raw);
fprintf("Std. Krümmung (KF)      : %.6f  1/m\n", std_k_kf);
fprintf("Rel. Std. (Raw)         : %.6f\n", rel_std_raw);
fprintf("Rel. Std. (KF)          : %.6f\n", rel_std_kf);
fprintf("Verbesserung der Std.   : %.2f %%\n", (1 - std_k_kf/std_k_raw)*100);

%% Visualisierung: Krümmung über der Zeit
figure;
subplot(2,1,1);
plot(t_raw_mid, k_raw, 'r'); hold on;
plot(t_kf_mid,  k_kf,  'g');
xlabel('t [s]'); ylabel('\kappa [1/m]');
legend('Raw','Kalman');
title('Krümmung über der Zeit (grüne Trajektorie)');

subplot(2,1,2);
bar([std_k_raw, std_k_kf]);
set(gca,'XTickLabel',{'Raw','Kalman'});
ylabel('Std. der Krümmung [1/m]');
title('Krümmungs-Kontinuität (kleiner ist besser)');
grid on;

%% =============== Lokale Funktion: Krümmung aus t und P berechnen ===========================
function [kappa, t_mid2] = local_compute_curvature(t, P)
    % Eingabe:
    %   t : Nx1 Zeitstempel
    %   P : Nx2 [x, y]
    % Ausgabe:
    %   kappa : Krümmungssequenz (M×1)
    %   t_mid2: Zeitpunkte der Krümmung (M×1)

    x = P(:,1);
    y = P(:,2);

    % Erste Differenz: Geschwindigkeit
    dt1 = diff(t);                    % (N-1)x1
    vx  = diff(x) ./ dt1;             % (N-1)x1
    vy  = diff(y) ./ dt1;
    t_mid1 = t(1:end-1) + dt1/2;      % Zeitpunkte der Geschwindigkeit

    % Zweite Differenz: Beschleunigung
    dt2 = diff(t_mid1);               % (N-2)x1
    ax  = diff(vx) ./ dt2;            % (N-2)x1
    ay  = diff(vy) ./ dt2;
    t_mid2 = t_mid1(1:end-1) + dt2/2; % Zeitpunkte von Beschleunigung/Krümmung

    vx_c = vx(1:end-1);
    vy_c = vy(1:end-1);

    % Krümmung: κ = |vx*ay - vy*ax| / (vx^2 + vy^2)^(3/2)
    num = abs(vx_c .* ay - vy_c .* ax);
    den = (vx_c.^2 + vy_c.^2).^(3/2);

    % Division durch 0 vermeiden (zu kleine Geschwindigkeit)
    eps_v = 1e-6;
    valid = den > eps_v;

    kappa = NaN(size(den));
    kappa(valid) = num(valid) ./ den(valid);

    % NaN entfernen
    valid2 = ~isnan(kappa);
    kappa  = kappa(valid2);
    t_mid2 = t_mid2(valid2);
end
