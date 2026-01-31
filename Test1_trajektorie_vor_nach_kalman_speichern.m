function Test1_trajektorie_vor_nach_kalman_speichern
    clc; close all;

    % ==========================================================
    % 1. Kameraintrinsiken und Homographien H laden (cam1 + cam2)
    %    Benötigte Dateien:
    %    params/cameraParams_cam1.mat
    %    params/cameraParams_cam2.mat
    %    params/H_room_cam1.mat
    %    params/H_room_cam2.mat
    % ==========================================================
    tmp = load('params/cameraParams_cam1.mat'); 
    cameraParams_cam1 = tmp.cameraParams;

    tmp = load('params/cameraParams_cam2.mat');
    cameraParams_cam2 = tmp.cameraParams;

    tmp = load('params/H_room_cam1.mat');
    H_room_cam1 = tmp.H;

    tmp = load('params/H_room_cam2.mat');
    H_room_cam2 = tmp.H;

    fprintf("✔ Intrinsiken und Homographien für cam1 & cam2 wurden geladen.\n");

    % ==========================================================
    % 2. HSV-Schwellenwerte (Grün + Rot)
    % ==========================================================
    % Grün
    hMin_g=0.680; hMax_g=0.944;
    sMin_g=0.121; sMax_g=0.447;
    vMin_g=0.592; vMax_g=0.933;
    
    % Rot (Wrap-Around, wird in hsvRange() behandelt)
    hMin_r = 0.988;  hMax_r = 0.033;
    sMin_r = 0.471;  sMax_r = 0.740;
    vMin_r = 0.937;  vMax_r = 1.000;

    % ==========================================================
    % 3. Zwei Kameras öffnen (IDs an den Rechner anpassen)
    % ==========================================================
    cam1 = webcam(2);
    cam2 = webcam(3);

    cam1.Resolution='1920x1080';
    cam2.Resolution='1920x1080';
    pause(1);

    I1_init = snapshot(cam1);
    I2_init = snapshot(cam2);

    % ==========================================================
    % 4. Separates Fenster für CAM1
    % ==========================================================
    fig1 = figure('Name','CAM1 (ID=1)','NumberTitle','off');
    set(fig1,'Position',[100 150 900 600]);
    hImg1 = imshow(I1_init); title('CAM1');
    hold on;
    hCam1_green = plot(NaN,NaN,'go','MarkerSize',10,'LineWidth',2);
    hCam1_red   = plot(NaN,NaN,'ro','MarkerSize',10,'LineWidth',2);

    % ==========================================================
    % 5. Separates Fenster für CAM2
    % ==========================================================
    fig2 = figure('Name','CAM2 (ID=2)','NumberTitle','off');
    set(fig2,'Position',[1100 150 900 600]);
    hImg2 = imshow(I2_init); title('CAM2');
    hold on;
    hCam2_green = plot(NaN,NaN,'go','MarkerSize',10,'LineWidth',2);
    hCam2_red   = plot(NaN,NaN,'ro','MarkerSize',10,'LineWidth',2);

    fprintf("Start: Dual-Kamera-Farbtracking (nur Anzeige, keine Trajektorie; Logging Raw + Kalman) ... (Ctrl+C oder Fenster schließen zum Stop)\n");

    % ==========================================================
    % 6. Kalmanfilter initialisieren (Zustand [x y vx vy]^T)
    % ==========================================================
    dt = 0.05;
    alpha = 0.9;    % Geschwindigkeits-Dämpfungsfaktor

    A  = [1 0 dt 0;
          0 1 0  dt;
          0 0 alpha 0;
          0 0 0 alpha];

    Hk = [1 0 0 0;
          0 1 0 0];

    Q = diag([1e-4, 1e-4, 1e-3, 1e-3]);   % Prozessrauschen
    R = diag([0.02, 0.02]);               % Messrauschen

    xg = [0;0;0;0]; Pg = eye(4); kalman_g_init = false;
    xr = [0;0;0;0]; Pr = eye(4); kalman_r_init = false;

    % Zur Berechnung von dt
    prevTime = tic;
    t_start  = tic;   % Relative Zeit (s)

    % Synchronisationstoleranz
    MAX_SYNC = 0.030;

    % ==========================================================
    % 7. Log-Struktur (für die spätere Auswertung)
    % ==========================================================
    log = struct();
    log.t            = [];   % Zeitstempel (s), t=0 ab Start
    log.green_raw    = [];   % [x_raw, y_raw] fusionierter grüner Punkt
    log.green_filt   = [];   % [x_kalman, y_kalman]
    log.red_raw      = [];   % [x_raw, y_raw] fusionierter roter Punkt
    log.red_filt     = [];   % [x_kalman, y_kalman]

    frameIdx = 0;

    % ==========================================================
    % 8. Hauptschleife (Pseudo-Sync + Fusion + Kalman + nur Logging)
    % ==========================================================
    while ishandle(fig1) && ishandle(fig2)

        % ---------- dt aktualisieren und A updaten ----------
        dt = toc(prevTime);
        prevTime = tic;
        if dt <= 0
            dt = 0.05;
        elseif dt > 0.2
            dt = 0.2;
        end

        alpha = 0.9;
        A = [1 0 dt 0;
             0 1 0  dt;
             0 0 alpha 0;
             0 0 0 alpha];

        %% -------------------- Synchrones Capturing --------------------
        t0 = tic;
        I1 = snapshot(cam1);
        t1 = toc(t0);

        I2 = snapshot(cam2);
        t2 = toc(t0);

        if abs(t1 - t2) > MAX_SYNC
            % Synchronisationsabweichung zu groß -> Frame überspringen
            drawnow limitrate;
            continue;
        end

        %% Entzerren
        I1u = undistortImage(I1,cameraParams_cam1);
        I2u = undistortImage(I2,cameraParams_cam2);

        %% -------------------- CAM1 Detektion --------------------
        hsv1 = rgb2hsv(I1u);

        % Grün
        mask_g1 = hsvRange(hsv1, hMin_g,hMax_g, sMin_g,sMax_g, vMin_g,vMax_g);
        [cx1_g, cy1_g, A1_g] = findLargestCentroid(mask_g1);

        % Rot
        mask_r1 = hsvRange(hsv1, hMin_r,hMax_r, sMin_r,sMax_r, vMin_r,vMax_r);
        [cx1_r, cy1_r, A1_r] = findLargestCentroid(mask_r1);

        %% -------------------- CAM2 Detektion --------------------
        hsv2 = rgb2hsv(I2u);

        mask_g2 = hsvRange(hsv2, hMin_g,hMax_g, sMin_g,sMax_g, vMin_g,vMax_g);
        [cx2_g, cy2_g, A2_g] = findLargestCentroid(mask_g2);

        mask_r2 = hsvRange(hsv2, hMin_r,hMax_r, sMin_r,sMax_r, vMin_r,vMax_r);
        [cx2_r, cy2_r, A2_r] = findLargestCentroid(mask_r2);

        %% -------------------- Abbildung auf die Raumebene --------------------
        P1_g = projectPoint(cx1_g, cy1_g, H_room_cam1);
        P2_g = projectPoint(cx2_g, cy2_g, H_room_cam2);

        P1_r = projectPoint(cx1_r, cy1_r, H_room_cam1);
        P2_r = projectPoint(cx2_r, cy2_r, H_room_cam2);

        %% -------------------- Fusionsstrategie: größte Fläche --------------------
        P_g = fuseByArea(P1_g, A1_g, P2_g, A2_g);   % [x y] oder []
        P_r = fuseByArea(P1_r, A1_r, P2_r, A2_r);

        %% -------------------- Kalman: Grün --------------------
        if ~isempty(P_g)
            z_g = P_g(:);
        else
            z_g = [];
        end

        [xg, Pg, kalman_g_init, Pg_filt] = kalman2D_step( ...
            xg, Pg, z_g, kalman_g_init, A, Hk, Q, R);

        %% -------------------- Kalman: Rot --------------------
        if ~isempty(P_r)
            z_r = P_r(:);
        else
            z_r = [];
        end

        [xr, Pr, kalman_r_init, Pr_filt] = kalman2D_step( ...
            xr, Pr, z_r, kalman_r_init, A, Hk, Q, R);

        %% -------------------- Logging (Raw + Kalman) --------------------
        frameIdx = frameIdx + 1;
        t_now = toc(t_start);    % Relative Zeit (s)

        % Standard: fehlende Werte mit NaN füllen
        g_raw  = [NaN, NaN];
        g_filt = [NaN, NaN];
        r_raw  = [NaN, NaN];
        r_filt = [NaN, NaN];

        if ~isempty(P_g)
            g_raw = P_g;
        end
        if ~isempty(Pg_filt)
            g_filt = Pg_filt;
        end

        if ~isempty(P_r)
            r_raw = P_r;
        end
        if ~isempty(Pr_filt)
            r_filt = Pr_filt;
        end

        log.t(frameIdx,1)          = t_now;
        log.green_raw(frameIdx,:)  = g_raw;
        log.green_filt(frameIdx,:) = g_filt;
        log.red_raw(frameIdx,:)    = r_raw;
        log.red_filt(frameIdx,:)   = r_filt;

        %% -------------------- Kamerafenster aktualisieren (nur Punkte, keine Trajektorie) --------------------
        set(hImg1,'CData',I1u);
        set(hImg2,'CData',I2u);

        set(hCam1_green,'XData',cx1_g,'YData',cy1_g);
        set(hCam1_red,  'XData',cx1_r,'YData',cy1_r);

        set(hCam2_green,'XData',cx2_g,'YData',cy2_g);
        set(hCam2_red,  'XData',cx2_r,'YData',cy2_r);

        drawnow limitrate;
    end

    % Schleife beendet -> Log speichern
    clear cam1 cam2;

    ts = datestr(now,'yyyymmdd_HHMMSS');
    fname = sprintf('log_kalman_session_%s.mat', ts);
    save(fname,'log');
    fprintf("Logdatei wurde gespeichert: %s\n", fname);
end

%% ================== Hilfsfunktionen =====================

function mask = hsvRange(hsv, hmin,hmax, smin,smax, vmin,vmax)
    h = hsv(:,:,1); s = hsv(:,:,2); v = hsv(:,:,3);

    if hmin <= hmax
        maskH = (h >= hmin & h <= hmax);
    else
        maskH = (h >= hmin | h <= hmax);   % Rot-Überlauf behandeln (Wrap-Around)
    end

    mask = maskH & (s>=smin & s<=smax) & (v>=vmin & v<=vmax);
    mask = imclose(mask,strel('disk',10));
    mask = imopen(mask,strel('disk',5));
    mask = imfill(mask,'holes');
end

function [cx,cy,area] = findLargestCentroid(mask)
    stats = regionprops(mask,'Area','Centroid');
    if isempty(stats)
        cx=NaN; cy=NaN; area=0;
    else
        [~,i] = max([stats.Area]);
        cx = stats(i).Centroid(1);
        cy = stats(i).Centroid(2);
        area = stats(i).Area;
    end
end

function P = projectPoint(u,v,H)
    if isnan(u) || isnan(v)
        P=[];
    else
        p = H * [u;v;1];
        P = [p(1)/p(3), p(2)/p(3)];
    end
end

function P = fuseByArea(P1,A1,P2,A2)
    if A1==0 && A2==0
        P=[];
    elseif A1>=A2
        P=P1;
    else
        P=P2;
    end
end

% ================== 2D-Kalmanfilter =====================
function [x, P, initialized, pos_out] = kalman2D_step(x, P, z, initialized, A, H, Q, R)

    pos_out = [];

    if ~initialized
        if isempty(z)
            return;
        else
            x = [z(1); z(2); 0; 0];
            P = eye(4);
            initialized = true;
            pos_out = z(:).';
            return;
        end
    end

    % Prädiktion
    x = A * x;
    P = A * P * A' + Q;

    % Update nur bei Messung
    if ~isempty(z)
        z_pred = H * x;
        y = z - z_pred;
        S = H * P * H' + R;
        K = P * H' / S;
        x = x + K * y;
        P = (eye(size(P)) - K * H) * P;
    end

    pos_out = x(1:2).';
end
