function step5_apriltag_kalman_ros
% ========================================================================
% - Erkennt zwei Tag-IDs
% - Fusioniert die Position (wenn beide Kameras verfügbar -> Mittelwert)
% - Kalman-Vorhersage + Update (Update nur bei Messung)
% - Sendet PoseArray an ROS: Position aus Kalman, Orientierung aus Tag-Ausrichtung
% ========================================================================

    clc; close all;

    % -------------------- Benutzer-Konfiguration -------------------------
    ROS_MASTER_IP = "192.168.50.11";          % IP vom ROS-Master (z.B. Raspberry Pi)
    TOPIC         = "/multicar/poses";        % PoseArray Topic
    FRAME_ID      = "map";                    % Frame-ID in ROS

    TAG_FAMILY    = "tag36h11";
    TAG_IDS       = [0 1];                   % zwei Tag-IDs
    tagSide       = 195/1000;                % Tag-Kantenlänge [m]

    CAM_ID_2      = 2;                       % Kamera-Index 2
    CAM_ID_3      = 3;                       % Kamera-Index 3
    RESOLUTION    = "1920x1080";

    MAX_SYNC = 0.030;                        % 30 ms 
    ARROW_LEN     = 0.5;                     % Pfeillänge im Room-Map Plot [m]

    % -------------------- 1) Kalibrierung laden -------------------------
    fprintf(" Lade Kalibrierungsparameter...\n");

    tmp = load("params/cameraParams_cam1.mat");  cameraParams_cam2 = tmp.cameraParams;
    tmp = load("params/cameraParams_cam2.mat");  cameraParams_cam3 = tmp.cameraParams;

    tmp = load("params/H_room_cam1.mat");        H_room_cam2 = tmp.H;
    tmp = load("params/H_room_cam2.mat");        H_room_cam3 = tmp.H;

    fprintf(" Kalibrierung für Cam2 & Cam3 geladen.\n");

    % -------------------- 2) Kameras öffnen ------------------------------
    cam2 = webcam(CAM_ID_2);
    cam3 = webcam(CAM_ID_3);
    cam2.Resolution = RESOLUTION;
    cam3.Resolution = RESOLUTION;
    pause(1);

    I2_init = snapshot(cam2);
    I3_init = snapshot(cam3);

    % Cleanup, damit Kamera/ROS sauber beendet werden
    cleaner = onCleanup(@()cleanupAll(cam2, cam3));

    % -------------------- 3) Kamerafenster -------------------------------
    fig2 = figure("Name","CAM2 (ID=2)","NumberTitle","off");
    hImg2 = imshow(I2_init); title("CAM2 - Live");
    set(fig2,"Position",[100,150,900,600]);

    fig3 = figure("Name","CAM3 (ID=3)","NumberTitle","off");
    hImg3 = imshow(I3_init); title("CAM3 - Live");
    set(fig3,"Position",[1100,150,900,600]);

    % -------------------- 4) Room-Map Fenster ----------------------------
    numTags = numel(TAG_IDS);
    tagColors = [0 0 1; 1 0 0];  

    figMap = figure("Name","Room Map");
    axis([0 8 0 8]); axis equal; grid on; hold on;
    xlabel("X (m)"); ylabel("Y (m)");
    title("AprilTag Tracking (Fusion + Kalman + ROS1)");

    hPaths  = cell(numTags,1);
    hPoints = cell(numTags,1);
    hDirs   = cell(numTags,1);

    for i = 1:numTags
        c = tagColors( min(i,size(tagColors,1)) , :);
        hPaths{i}  = animatedline("Color",c,"LineWidth",1.5);
        hPoints{i} = plot(NaN,NaN,"o","MarkerSize",10,"LineWidth",2, ...
                          "MarkerEdgeColor","k","MarkerFaceColor",c);
        hDirs{i}   = quiver(NaN,NaN,0,0,0,"Color",c,"LineWidth",2,"MaxHeadSize",1.5);
    end
    legend(arrayfun(@(x)sprintf("Tag %d",x), TAG_IDS, "UniformOutput", false));

    % -------------------- 5) ROS1 initialisieren -------------------------
    fprintf("\nROS Master: http://%s:11311\n", ROS_MASTER_IP);
    fprintf("Topic: %s\n\n", TOPIC);

    try, rosshutdown; catch, end
    rosinit("http://" + ROS_MASTER_IP + ":11311");

    pub = rospublisher(TOPIC, "geometry_msgs/PoseArray");
    msg = rosmessage(pub);

    % -------------------- 6) Kalman-Initialisierung ----------------------
    % Zustand: [x y vx vy]^T
    dt    = 0.05;
    alpha = 0.9;

    Hk = [1 0 0 0;
          0 1 0 0];

    Q = diag([1e-4, 1e-4, 1e-3, 1e-3]);   % Prozessrauschen
    R = diag([0.02, 0.02]);               % Messrauschen (anpassen, falls nötig)

    % Pro Tag: eigener Filterzustand
    xKF   = cell(numTags,1);
    PKF   = cell(numTags,1);
    initKF= false(numTags,1);

    for i = 1:numTags
        xKF{i} = [0;0;0;0];
        PKF{i} = eye(4);
    end

    prevTime = tic;

    % -------------------- 7) Statistik -----------------------------------
    totalFrames    = 0;
    detectedFrames = 0;

    fprintf("Live-Tracking läuft... (Fenster schließen zum Beenden)\n");

    % -------------------- 8) Hauptschleife -------------------------------
    while ishandle(fig2) && ishandle(fig3) && ishandle(figMap)

        totalFrames = totalFrames + 1;

        % ---- dt berechnen und Zustandsmatrix A aktualisieren ----
        dt = toc(prevTime);
        prevTime = tic;

        if dt <= 0
            dt = 0.05;
        elseif dt > 0.2
            dt = 0.2;
        end

        A = [1 0 dt 0;
             0 1 0  dt;
             0 0 alpha 0;
             0 0 0 alpha];

        % ---- Synchronisation ----
        t0 = tic;
        I2 = snapshot(cam2); t2 = toc(t0);
        I3 = snapshot(cam3); t3 = toc(t0);

        if abs(t2 - t3) > MAX_SYNC
    
            continue;
        end

        % ---- Undistortion ----
        I2u = undistortImage(I2, cameraParams_cam2, "OutputView","full");
        I3u = undistortImage(I3, cameraParams_cam3, "OutputView","full");
        set(hImg2,"CData",I2u);
        set(hImg3,"CData",I3u);

        % ---- AprilTag Detection pro Kamera ----
        [id2, loc2] = readAprilTag(I2u, TAG_FAMILY, cameraParams_cam2.Intrinsics, tagSide);
        [id3, loc3] = readAprilTag(I3u, TAG_FAMILY, cameraParams_cam3.Intrinsics, tagSide);

        detectedThisFrame = false;

        % PoseArray: pro Tag eine Pose (sauber: jede Pose als eigenes Objekt erzeugen)
        posesOut = repmat(rosmessage("geometry_msgs/Pose"), 1, numTags);
        for i = 1:numTags
            posesOut(i) = rosmessage("geometry_msgs/Pose");
            posesOut(i).Position.Z = 0;
            posesOut(i).Orientation.W = 1;
        end

        for i = 1:numTags
            tagID = TAG_IDS(i);

            % ------------------ Cam2: Tag suchen --------------------------
            idx2 = find(id2 == tagID, 1);
            if ~isempty(idx2)
                corners2 = loc2(:,:,idx2);
                P2 = projectToRoom(corners2, H_room_cam2);   % [x y]
            else
                corners2 = [];
                P2 = [];
            end

            % ------------------ Cam3: Tag suchen --------------------------
            idx3 = find(id3 == tagID, 1);
            if ~isempty(idx3)
                corners3 = loc3(:,:,idx3);
                P3 = projectToRoom(corners3, H_room_cam3);   % [x y]
            else
                corners3 = [];
                P3 = [];
            end

            % ------------------ Fusion der Position -----------------------
            if ~isempty(P2) && ~isempty(P3)
                P_meas = (P2 + P3) / 2;   % Mittelwert, wenn beide verfügbar
                detectedThisFrame = true;
            elseif ~isempty(P2)
                P_meas = P2;
                detectedThisFrame = true;
            elseif ~isempty(P3)
                P_meas = P3;
                detectedThisFrame = true;
            else
                P_meas = [];              % keine Messung für diesen Tag
            end

            % ------------------ Kalman: Vorhersage + (optional) Update ----
            if ~isempty(P_meas)
                z = P_meas(:);    % [x;y]
            else
                z = [];
            end

            [xKF{i}, PKF{i}, initKF(i), P_filt] = kalman2D_step( ...
                xKF{i}, PKF{i}, z, initKF(i), A, Hk, Q, R);

            % Falls Filter noch nicht initialisiert und keine Messung: 0 senden, Plot leeren
            if ~initKF(i)
                set(hPoints{i},"XData",NaN,"YData",NaN);
                set(hDirs{i},"XData",NaN,"YData",NaN,"UData",0,"VData",0);

                posesOut(i).Position.X = 0;
                posesOut(i).Position.Y = 0;
                posesOut(i).Orientation.X = 0;
                posesOut(i).Orientation.Y = 0;
                posesOut(i).Orientation.Z = 0;
                posesOut(i).Orientation.W = 1;
                continue;
            end

            % Gefilterte Position (auch bei fehlender Messung -> Prädiktion)
            P = P_filt;  % [x y]
            P(1) = min(max(P(1), 0), 8);
            P(2) = min(max(P(2), 0), 8);

            % ------------------ Orientierung bestimmen ---------------------
            % Orientierung basiert auf den Ecken der Kamera, die detektiert hat.
            % Wenn keine Messung vorhanden ist, bleibt Orientierung neutral.
            if ~isempty(corners2)
                corners = corners2; H = H_room_cam2;
            elseif ~isempty(corners3)
                corners = corners3; H = H_room_cam3;
            else
                corners = [];
                H = [];
            end

            if ~isempty(corners)
                % Eckenreihenfolge: typischerweise [TL; BL; BR; TR]
                tl = corners(1,:);
                tr = corners(4,:);

                dirVec = directionVectorInRoom(tl, tr, H);
                if norm(dirVec) > 0
                    dirUnit = dirVec / norm(dirVec);
                else
                    dirUnit = [1 0];
                end

                u = dirUnit(1) * ARROW_LEN;
                v = dirUnit(2) * ARROW_LEN;

                yaw = atan2(dirUnit(2), dirUnit(1));
                q = yawToQuatZ(yaw);
            else
                % keine aktuelle Orientierung -> neutral
                u = 0; v = 0;
                q = [0 0 0 1];
            end

            % ------------------ Plot aktualisieren -------------------------
            addpoints(hPaths{i}, P(1), P(2));
            set(hPoints{i},"XData",P(1),"YData",P(2));
            set(hDirs{i},"XData",P(1),"YData",P(2),"UData",u,"VData",v);

            % ------------------ ROS Pose füllen ----------------------------
            posesOut(i).Position.X = P(1);
            posesOut(i).Position.Y = P(2);
            posesOut(i).Position.Z = 0;

            posesOut(i).Orientation.X = q(1);
            posesOut(i).Orientation.Y = q(2);
            posesOut(i).Orientation.Z = q(3);
            posesOut(i).Orientation.W = q(4);
        end

        if detectedThisFrame
            detectedFrames = detectedFrames + 1;
        end

        % ------------------ ROS Publish ----------------------------------
        msg.Header.Stamp   = rostime("now");
        msg.Header.FrameId = FRAME_ID;
        msg.Poses = posesOut;   % immer numTags Posen (Reihenfolge wie TAG_IDS)
        send(pub, msg);

        drawnow limitrate;
    end

    % -------------------- Statistik --------------------------------------
    successRate = detectedFrames / max(totalFrames,1) * 100;
    fprintf("\n================= Tracking Ende =================\n");
    fprintf("Gesamtframes: %d\n", totalFrames);
    fprintf("Frames mit mind. 1 Detektion: %d\n", detectedFrames);
    fprintf("Detektionsrate: %.2f %%\n", successRate);
    fprintf("=================================================\n");
end

% ========================================================================
% Cleanup: Kameras freigeben + ROS trennen
% ========================================================================
function cleanupAll(cam2, cam3)
    try, clear cam2 cam3; catch, end
    try, rosshutdown; catch, end
end

% ========================================================================
% Hilfsfunktion: Mittelpunkt in Room-Koordinaten aus 4 Eckpunkten
% ========================================================================
function P = projectToRoom(corners, H)
    corners_h = [corners, ones(4,1)];
    room_h = corners_h * H.';
    room = room_h(:,1:2) ./ room_h(:,3);
    P = mean(room, 1);
end

% ========================================================================
% Hilfsfunktion: Richtungsvektor (TL -> TR) in Room-Koordinaten
% ========================================================================
function dirVec = directionVectorInRoom(tl_px, tr_px, H)
    tl_h = [tl_px 1] * H.';
    tr_h = [tr_px 1] * H.';

    tl_room = tl_h(1:2) / tl_h(3);
    tr_room = tr_h(1:2) / tr_h(3);

    dirVec = (tr_room - tl_room);
end

% ========================================================================
% Hilfsfunktion: Yaw (rad) -> Quaternion (nur Rotation um Z)
% Rückgabe: [qx qy qz qw]
% ========================================================================
function q = yawToQuatZ(yaw)
    qx = 0;
    qy = 0;
    qz = sin(yaw/2);
    qw = cos(yaw/2);
    q = [qx qy qz qw];
end

% ========================================================================
% 2D-Kalmanfilter Schritt: Zustand [x y vx vy]^T
% - Initialisierung beim ersten Messwert
% - Vorhersage immer, Update nur wenn z vorhanden
% - pos_out = gefilterte/ vorhergesagte Position [x y]
% ========================================================================
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

    % Vorhersage
    x = A * x;
    P = A * P * A' + Q;

    % Update nur bei Messung
    if ~isempty(z)
        y = z - H * x;
        S = H * P * H' + R;
        K = P * H' / S;
        x = x + K * y;
        P = (eye(size(P)) - K * H) * P;
    end

    pos_out = x(1:2).';
end
