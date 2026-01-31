function step4_multifarbe_kalman_ros
    clc; close all;

    % ===================== ROS-Konfiguration =====================
    PI_IP   = "192.168.50.11";         % ROS Master (z.B. Raspberry Pi)
    TOPIC   = "/multicar/poses";       % Topic-Name
    FRAMEID = "map";                   % Frame-ID im ROS-Header

    % ===================== 1) Parameter laden ====================
    % Benötigte Dateien:
    % params/cameraParams_cam1.mat
    % params/cameraParams_cam2.mat
    % params/H_room_cam1.mat
    % params/H_room_cam2.mat
    tmp = load('params/cameraParams_cam1.mat');
    cameraParams_cam1 = tmp.cameraParams;

    tmp = load('params/cameraParams_cam2.mat');
    cameraParams_cam2 = tmp.cameraParams;

    tmp = load('params/H_room_cam1.mat');
    H_room_cam1 = tmp.H;

    tmp = load('params/H_room_cam2.mat');
    H_room_cam2 = tmp.H;

    fprintf("✔ Kamera-Parameter & Homographien (cam1+cam2) geladen.\n");

    % ===================== 2) HSV-Schwellen ======================
    % Grün
    hMin_g=0.680; hMax_g=0.944;
    sMin_g=0.121; sMax_g=0.447;
    vMin_g=0.592; vMax_g=0.933;

    % Rot (Hue-Überlauf -> wird in hsvRange() behandelt)
    hMin_r = 0.988;  hMax_r = 0.033;
    sMin_r = 0.471;  sMax_r = 0.740;
    vMin_r = 0.937;  vMax_r = 1.000;

    % ===================== 3) Kameras öffnen =====================
    % Hinweis: IDs ggf. an deinen Rechner anpassen!
    cam1 = webcam(2);
    cam2 = webcam(3);

    cam1.Resolution='1920x1080';
    cam2.Resolution='1920x1080';
    pause(1);

    I1_init = snapshot(cam1);
    I2_init = snapshot(cam2);

    % ===================== 4) Visualisierung =====================
    fig1 = figure('Name','CAM1','NumberTitle','off');
    set(fig1,'Position',[100 150 900 600]);
    hImg1 = imshow(I1_init); title('CAM1');
    hold on;
    hCam1_green = plot(NaN,NaN,'go','MarkerSize',10,'LineWidth',2);
    hCam1_red   = plot(NaN,NaN,'ro','MarkerSize',10,'LineWidth',2);

    fig2 = figure('Name','CAM2','NumberTitle','off');
    set(fig2,'Position',[1100 150 900 600]);
    hImg2 = imshow(I2_init); title('CAM2');
    hold on;
    hCam2_green = plot(NaN,NaN,'go','MarkerSize',10,'LineWidth',2);
    hCam2_red   = plot(NaN,NaN,'ro','MarkerSize',10,'LineWidth',2);

    figMapFilt = figure('Name','Room Coordinates - Kalman','NumberTitle','off');
    set(figMapFilt,'Position',[650 100 700 700]);
    grid on; axis equal; hold on;
    xlabel('X (m)'); ylabel('Y (m)');
    xlim([0 8]); ylim([0 8]);
    title('Trajektorie (nur Kalman-Filter)');
    hRoom_green_filt_path = plot(NaN,NaN,'g-','LineWidth',2);
    hRoom_green_filt_mk   = plot(NaN,NaN,'go','MarkerFaceColor','g');
    hRoom_red_filt_path   = plot(NaN,NaN,'r-','LineWidth',2);
    hRoom_red_filt_mk     = plot(NaN,NaN,'ro','MarkerFaceColor','r');

    greenX_filt=[]; greenY_filt=[];
    redX_filt=[];   redY_filt=[];

    % ===================== 5) ROS initialisieren =================
    try, rosshutdown; catch, end
    rosinit("http://" + PI_IP + ":11311");

    pub = rospublisher(TOPIC, "geometry_msgs/PoseArray");
    msg = rosmessage(pub);

    % WICHTIG: zwei unabhängige Pose-Objekte (kein repmat!)
    poseA = rosmessage("geometry_msgs/Pose");   % Fahrzeug A = Grün
    poseB = rosmessage("geometry_msgs/Pose");   % Fahrzeug B = Rot

    % Wir nutzen effektiv nur X/Y; Z=0 und Orientierung neutral
    poseA.Position.Z = 0;  poseA.Orientation.W = 1;
    poseB.Position.Z = 0;  poseB.Orientation.W = 1;

    % Cleanup, damit Kamera/ROS sauber beendet werden
    cleaner = onCleanup(@()cleanupAll(cam1, cam2));

    fprintf("Start: Dual-Kamera Farberkennung + Kalman + ROS Publish ... (Ctrl+C zum Stop)\n");

    % ===================== 6) Kalman-Initialisierung =============
    % Zustandsvektor: [x y vx vy]^T
    dt = 0.05;
    alpha = 0.9;    % Geschwindigkeits-Dämpfung

    A  = [1 0 dt 0;
          0 1 0  dt;
          0 0 alpha 0;
          0 0 0 alpha];

    Hk = [1 0 0 0;
          0 1 0 0];

    Q = diag([1e-4, 1e-4, 1e-3, 1e-3]);  % Prozessrauschen
    R = diag([0.02, 0.02]);              % Messrauschen

    xg = [0;0;0;0]; Pg = eye(4); kalman_g_init = false;
    xr = [0;0;0;0]; Pr = eye(4); kalman_r_init = false;

    prevTime = tic;

    % Sychronisation-Toleranz zwischen den beiden Snapshots
    MAX_SYNC = 0.030;

    % ===================== 7) Hauptschleife ======================
    while ishandle(fig1) && ishandle(fig2) && ishandle(figMapFilt)

        % ---- dt aktualisieren und Zustandsmatrix A updaten ----
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

        % ---- Synchronisation/Abgreifen beider Kameras ----
        t0 = tic;
        I1 = snapshot(cam1);
        t1 = toc(t0);

        I2 = snapshot(cam2);
        t2 = toc(t0);

        if abs(t1 - t2) > MAX_SYNC
            fprintf("⚠ Δt=%.3f s zu groß, Frame übersprungen.\n", abs(t1 - t2));
            drawnow; 
            continue;
        end

        % ---- Entzerren (Undistortion) ----
        I1u = undistortImage(I1, cameraParams_cam1);
        I2u = undistortImage(I2, cameraParams_cam2);

        % ================= CAM1: Detektion =================
        hsv1 = rgb2hsv(I1u);

        mask_g1 = hsvRange(hsv1, hMin_g,hMax_g, sMin_g,sMax_g, vMin_g,vMax_g);
        [cx1_g, cy1_g, A1_g] = findLargestCentroid(mask_g1);

        mask_r1 = hsvRange(hsv1, hMin_r,hMax_r, sMin_r,sMax_r, vMin_r,vMax_r);
        [cx1_r, cy1_r, A1_r] = findLargestCentroid(mask_r1);

        % ================= CAM2: Detektion =================
        hsv2 = rgb2hsv(I2u);

        mask_g2 = hsvRange(hsv2, hMin_g,hMax_g, sMin_g,sMax_g, vMin_g,vMax_g);
        [cx2_g, cy2_g, A2_g] = findLargestCentroid(mask_g2);

        mask_r2 = hsvRange(hsv2, hMin_r,hMax_r, sMin_r,sMax_r, vMin_r,vMax_r);
        [cx2_r, cy2_r, A2_r] = findLargestCentroid(mask_r2);

        % ================= Projektion in Raumkoordinaten =================
        P1_g = projectPoint(cx1_g, cy1_g, H_room_cam1);
        P2_g = projectPoint(cx2_g, cy2_g, H_room_cam2);

        P1_r = projectPoint(cx1_r, cy1_r, H_room_cam1);
        P2_r = projectPoint(cx2_r, cy2_r, H_room_cam2);

        % ================= Fusion: größte Fläche gewinnt =================
        P_g = fuseByArea(P1_g, A1_g, P2_g, A2_g);   % [x y] oder []
        P_r = fuseByArea(P1_r, A1_r, P2_r, A2_r);

        % ================= Kalman: Grün (Fahrzeug A) =================
        if ~isempty(P_g), z_g = P_g(:); else, z_g = []; end

        [xg, Pg, kalman_g_init, Pg_filt] = kalman2D_step( ...
            xg, Pg, z_g, kalman_g_init, A, Hk, Q, R);

        if ~isempty(Pg_filt)
            Pg_filt(1) = min(max(Pg_filt(1), 0), 8);
            Pg_filt(2) = min(max(Pg_filt(2), 0), 8);

            greenX_filt = [greenX_filt Pg_filt(1)];
            greenY_filt = [greenY_filt Pg_filt(2)];
            set(hRoom_green_filt_path,'XData',greenX_filt,'YData',greenY_filt);
            set(hRoom_green_filt_mk,'XData',Pg_filt(1),'YData',Pg_filt(2));
        end

        % ================= Kalman: Rot (Fahrzeug B) =================
        if ~isempty(P_r), z_r = P_r(:); else, z_r = []; end

        [xr, Pr, kalman_r_init, Pr_filt] = kalman2D_step( ...
            xr, Pr, z_r, kalman_r_init, A, Hk, Q, R);

        if ~isempty(Pr_filt)
            Pr_filt(1) = min(max(Pr_filt(1), 0), 8);
            Pr_filt(2) = min(max(Pr_filt(2), 0), 8);

            redX_filt = [redX_filt Pr_filt(1)];
            redY_filt = [redY_filt Pr_filt(2)];
            set(hRoom_red_filt_path,'XData',redX_filt,'YData',redY_filt);
            set(hRoom_red_filt_mk,'XData',Pr_filt(1),'YData',Pr_filt(2));
        end

        % ================= ROS Publish (Kalman-Output) =================
        % Header aktualisieren
        msg.Header.Stamp   = rostime("now");
        msg.Header.FrameId = FRAMEID;

        % Fahrzeug A (Grün)
        if ~isempty(Pg_filt)
            poseA.Position.X = Pg_filt(1);
            poseA.Position.Y = Pg_filt(2);
        else
            poseA.Position.X = 0;
            poseA.Position.Y = 0;
        end

        % Fahrzeug B (Rot)
        if ~isempty(Pr_filt)
            poseB.Position.X = Pr_filt(1);
            poseB.Position.Y = Pr_filt(2);
        else
            poseB.Position.X = 0;
            poseB.Position.Y = 0;
        end

        % Immer zwei Posen senden (A,B); fehlende Messungen -> 0
        msg.Poses = [poseA, poseB];
        send(pub, msg);

        % ================= Anzeige updaten =================
        set(hImg1,'CData',I1u);
        set(hImg2,'CData',I2u);

        set(hCam1_green,'XData',cx1_g,'YData',cy1_g);
        set(hCam1_red,  'XData',cx1_r,'YData',cy1_r);

        set(hCam2_green,'XData',cx2_g,'YData',cy2_g);
        set(hCam2_red,  'XData',cx2_r,'YData',cy2_r);

        drawnow limitrate;
    end
end

% ===================== Hilfsfunktionen =====================

function cleanupAll(cam1, cam2)
    % Ressourcen sauber freigeben
    try, clear cam1 cam2; catch, end
    try, rosshutdown; catch, end
end

function mask = hsvRange(hsv, hmin,hmax, smin,smax, vmin,vmax)
    % HSV-Schwellenbildung inkl. Rot-Überlauf (Hue wrap-around)
    h = hsv(:,:,1); s = hsv(:,:,2); v = hsv(:,:,3);

    if hmin <= hmax
        maskH = (h >= hmin & h <= hmax);
    else
        maskH = (h >= hmin | h <= hmax);
    end

    mask = maskH & (s>=smin & s<=smax) & (v>=vmin & v<=vmax);

    % Morphologische Nachbearbeitung
    mask = imclose(mask, strel('disk',10));
    mask = imopen(mask,  strel('disk',5));
    mask = imfill(mask, 'holes');
end

function [cx,cy,area] = findLargestCentroid(mask)
    % Größtes zusammenhängendes Segment finden und dessen Schwerpunkt berechnen
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
    % Pixelpunkt (u,v) via Homographie in Raumebene projizieren
    if isnan(u) || isnan(v)
        P=[];
    else
        p = H * [u; v; 1];
        P = [p(1)/p(3), p(2)/p(3)];
    end
end

function P = fuseByArea(P1,A1,P2,A2)
    % Fusionsstrategie: Kamera mit größerer Segmentfläche gewinnt
    if A1==0 && A2==0
        P=[];
    elseif A1>=A2
        P=P1;
    else
        P=P2;
    end
end

function [x, P, initialized, pos_out] = kalman2D_step(x, P, z, initialized, A, H, Q, R)
    % 2D-Kalmanfilter: Zustand [x y vx vy]^T
    pos_out = [];

    % Initialisierung beim ersten Messwert
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
