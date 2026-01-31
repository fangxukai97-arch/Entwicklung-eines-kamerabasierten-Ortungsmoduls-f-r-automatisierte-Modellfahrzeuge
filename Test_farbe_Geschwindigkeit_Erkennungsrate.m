function Test_farbe_Geschwindigkeit_Erkennungsrate
    clc; close all;

    % ==========================================================
    % 1. Kameraintrinsiken und Homographien H laden (cam1 + cam2)
    % ==========================================================
    tmp = load('params/cameraParams_cam1.mat');
    cameraParams_cam1 = tmp.cameraParams;

    tmp = load('params/cameraParams_cam2.mat');
    cameraParams_cam2 = tmp.cameraParams;

    tmp = load('params/H_room_cam1.mat');
    H_room_cam1 = tmp.H;

    tmp = load('params/H_room_cam2.mat');
    H_room_cam2 = tmp.H;

    fprintf("Intrinsiken und Homographien für cam1 & cam2 wurden geladen.\n");

    % ==========================================================
    % 2. HSV-Schwellenwerte (Grün + Rot)
    % ==========================================================
    % Grün
    hMin_g=0.680; hMax_g=0.944;
    sMin_g=0.121; sMax_g=0.447;
    vMin_g=0.592; vMax_g=0.933;

    % Rot
    hMin_r = 0.988;  hMax_r = 0.033;
    sMin_r = 0.471;  sMax_r = 0.740;
    vMin_r = 0.937;  vMax_r = 1.000;

    % ==========================================================
    % 3. Zwei Kameras öffnen (IDs = 2, 3)
    % ==========================================================
    cam1 = webcam(2);
    cam2 = webcam(3);

    cam1.Resolution='1920x1080';
    cam2.Resolution='1920x1080';
    pause(1);

    I1_init = snapshot(cam1);
    I2_init = snapshot(cam2);

    % ==========================================================
    % 4. Fenster CAM1
    % ==========================================================
    fig1 = figure('Name','CAM1','NumberTitle','off');
    set(fig1,'Position',[100 150 900 600]);
    hImg1 = imshow(I1_init); title('CAM1');
    hold on;
    hCam1_green = plot(NaN,NaN,'go','MarkerSize',10,'LineWidth',2);
    hCam1_red   = plot(NaN,NaN,'ro','MarkerSize',10,'LineWidth',2);

    % ==========================================================
    % 5. Fenster CAM2
    % ==========================================================
    fig2 = figure('Name','CAM2','NumberTitle','off');
    set(fig2,'Position',[1100 150 900 600]);
    hImg2 = imshow(I2_init); title('CAM2');
    hold on;
    hCam2_green = plot(NaN,NaN,'go','MarkerSize',10,'LineWidth',2);
    hCam2_red   = plot(NaN,NaN,'ro','MarkerSize',10,'LineWidth',2);

    fprintf("Start: Dual-Kamera-Farbtracking ... (Ctrl+C oder Fenster schließen zum Stop)\n");

    % Synchronisationstoleranz
    MAX_SYNC = 0.030;

    % ==========================================================
    % 6. Log-Struktur (nur rohe fusionierte Ergebnisse)
    % ==========================================================
    log = struct();
    log.t           = [];   % Zeitstempel (s) ab Start
    log.green_raw   = [];   % [x y] fusionierter grüner Punkt (NaN = fehlt)
    log.red_raw     = [];   % [x y] fusionierter roter Punkt (NaN = fehlt)
    log.sync_ok     = [];   % 1=Sync ok, 0=Frame übersprungen (für Replay)

    frameIdx = 0;

    % ==========================================================
    % 7. Statistik: Erfolgsrate & mittlere Geschwindigkeit (Fusion)
    % ==========================================================
    t_start  = tic;     % Startzeit der Ausführung
    tGlobal  = tic;     % dito, für Geschwindigkeits-dt (alternativ t_start)

    nTotal   = 0;       % Gesamtanzahl der Schleifendurchläufe (optional)
    nSynced  = 0;       % Anzahl synchronisierter Frames (Nenner)
    nDet_g   = 0;       % Erfolgreiche Frames (Grün)
    nDet_r   = 0;       % Erfolgreiche Frames (Rot)

    % Mittlere Geschwindigkeit: Summe Strecke / Summe Zeit (nur bei aufeinanderfolgenden gültigen Messungen)
    lastPg = []; lastTg = NaN;  distSum_g = 0; timeSum_g = 0;
    lastPr = []; lastTr = NaN;  distSum_r = 0; timeSum_r = 0;

    % ==========================================================
    % 8. Hauptschleife (Sync + Fusion + nur Logging)
    % ==========================================================
    while ishandle(fig1) && ishandle(fig2)

        nTotal = nTotal + 1;
        tNow = toc(tGlobal);  % aktuelle Zeit (s)

        %% -------------------- Synchrones Capturing --------------------
        t0 = tic;
        I1 = snapshot(cam1);
        t1 = toc(t0);

        I2 = snapshot(cam2);
        t2 = toc(t0);

        if abs(t1 - t2) > MAX_SYNC

            frameIdx = frameIdx + 1;
            log.t(frameIdx,1)         = toc(t_start);
            log.green_raw(frameIdx,:) = [NaN, NaN];
            log.red_raw(frameIdx,:)   = [NaN, NaN];
            log.sync_ok(frameIdx,1)   = 0;

            drawnow limitrate;
            continue;
        end

        % Sync bestanden
        nSynced = nSynced + 1;

        %% Entzerren
        I1u = undistortImage(I1,cameraParams_cam1);
        I2u = undistortImage(I2,cameraParams_cam2);

        %% -------------------- CAM1 Detektion --------------------
        hsv1 = rgb2hsv(I1u);

        mask_g1 = hsvRange(hsv1, hMin_g,hMax_g, sMin_g,sMax_g, vMin_g,vMax_g);
        [cx1_g, cy1_g, A1_g] = findLargestCentroid(mask_g1);

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

        %% -------------------- Statistik: Erfolgsrate + mittlere Geschwindigkeit --------------------
        % Grün
        if ~isempty(P_g)
            nDet_g = nDet_g + 1;
            if ~isempty(lastPg) && ~isnan(lastTg)
                dtv = tNow - lastTg;
                if dtv > 0
                    dsv = norm(P_g - lastPg);
                    distSum_g = distSum_g + dsv;
                    timeSum_g = timeSum_g + dtv;
                end
            end
            lastPg = P_g;
            lastTg = tNow;
        end

        % Rot
        if ~isempty(P_r)
            nDet_r = nDet_r + 1;
            if ~isempty(lastPr) && ~isnan(lastTr)
                dtv = tNow - lastTr;
                if dtv > 0
                    dsv = norm(P_r - lastPr);
                    distSum_r = distSum_r + dsv;
                    timeSum_r = timeSum_r + dtv;
                end
            end
            lastPr = P_r;
            lastTr = tNow;
        end

        %% -------------------- Logging --------------------
        frameIdx = frameIdx + 1;
        log.t(frameIdx,1) = toc(t_start);

        g_raw = [NaN, NaN];
        r_raw = [NaN, NaN];
        if ~isempty(P_g), g_raw = P_g; end
        if ~isempty(P_r), r_raw = P_r; end

        log.green_raw(frameIdx,:) = g_raw;
        log.red_raw(frameIdx,:)   = r_raw;
        log.sync_ok(frameIdx,1)   = 1;

        %% -------------------- Kamerafenster aktualisieren --------------------
        set(hImg1,'CData',I1u);
        set(hImg2,'CData',I2u);

        set(hCam1_green,'XData',cx1_g,'YData',cy1_g);
        set(hCam1_red,  'XData',cx1_r,'YData',cy1_r);

        set(hCam2_green,'XData',cx2_g,'YData',cy2_g);
        set(hCam2_red,  'XData',cx2_r,'YData',cy2_r);

        drawnow limitrate;
    end

    % ==========================================================
    % 9. Ende: Log speichern + Statistik ausgeben
    % ==========================================================
    clear cam1 cam2;

    % Erfolgsrate (bezogen auf synchronisierte Frames)
    if nSynced > 0
        successRate_g = nDet_g / nSynced;
        successRate_r = nDet_r / nSynced;
    else
        successRate_g = NaN;
        successRate_r = NaN;
    end

    % Mittlere Geschwindigkeit (m/s)
    if timeSum_g > 0
        vavg_g = distSum_g / timeSum_g;
    else
        vavg_g = NaN;
    end
    if timeSum_r > 0
        vavg_r = distSum_r / timeSum_r;
    else
        vavg_r = NaN;
    end

    % Statistik im Log ablegen (für Post-Processing)
    log.stats = struct();
    log.stats.nTotal  = nTotal;
    log.stats.nSynced = nSynced;
    log.stats.nDet_g  = nDet_g;
    log.stats.nDet_r  = nDet_r;
    log.stats.successRate_g = successRate_g;
    log.stats.successRate_r = successRate_r;
    log.stats.vavg_g = vavg_g;
    log.stats.vavg_r = vavg_r;

    ts = datestr(now,'yyyymmdd_HHMMSS');

    % Unterscheidung zu Kalman-Logs: raw
    fname = sprintf('log_raw_session_%s.mat', ts);
    save(fname,'log');

    fprintf("\n================= Statistikergebnisse =================\n");
    fprintf("Synced Frames (nSynced)       : %d\n", nSynced);
    fprintf("Fahrzeug-Detektionen (nDet_g) : %d\n", nDet_g);
    fprintf("Erfolgsrate                   : %.2f %%\n", 100*successRate_g);
    fprintf("Mittlere Geschwindigkeit (m/s): %.4f\n", vavg_g);
    fprintf("RAW-Log wurde gespeichert: %s\n", fname);
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
