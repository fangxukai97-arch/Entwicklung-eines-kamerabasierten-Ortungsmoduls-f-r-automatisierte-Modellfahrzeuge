function Test_Apriltag_erkennungsrate
    % ============================================================
    %   - Gesamte mittlere Geschwindigkeit v_avg = sum(ds)/sum(dt)
    %     (für jedes Tag)
    % ============================================================

    clc; close all;
    fprintf("Kalibrierparameter werden geladen...\n");

    %% ============================================================
    % 1. Kalibrierparameter laden (cam2 / cam3)
    % ============================================================
    tmp = load('params/cameraParams_cam1.mat');
    cameraParams_cam2 = tmp.cameraParams;

    tmp = load('params/cameraParams_cam2.mat');
    cameraParams_cam3 = tmp.cameraParams;

    tmp = load('params/H_room_cam1.mat');
    H_room_cam2 = tmp.H;

    tmp = load('params/H_room_cam2.mat');
    H_room_cam3 = tmp.H;

    fprintf("✔ Kalibrierdateien für cam2 & cam3 wurden geladen.\n");

    %% ============================================================
    % 2. Kameras öffnen (ID = 2 & 3)
    % ============================================================
    cam2 = webcam(2);
    cam3 = webcam(3);

    cam2.Resolution = '1920x1080';
    cam3.Resolution = '1920x1080';
    pause(1);

    I2_init = snapshot(cam2);
    I3_init = snapshot(cam3);

    %% ============================================================
    % 3. Zwei getrennte Kamerafenster erstellen
    % ============================================================

    % --- CAM2 ---
    fig2 = figure('Name','CAM2 (ID=2)','NumberTitle','off');
    set(fig2,'Position',[100,150,900,600]);
    hImg2 = imshow(I2_init);
    title('CAM2 - Live');
    ax2 = gca; hold(ax2,'on');

    % --- CAM3 ---
    fig3 = figure('Name','CAM3 (ID=3)','NumberTitle','off');
    set(fig3,'Position',[1100,150,900,600]);
    hImg3 = imshow(I3_init);
    title('CAM3 - Live');
    ax3 = gca; hold(ax3,'on');

    %% ============================================================
    % 4. AprilTag-Parameter + Raumkarten-Fenster
    % ============================================================
    tagFamily = 'tag25h9';
    tagIDs    = 0;                    % zu trackende Tag-ID(s)
    tagSide   = 115/1000;             % Tag-Kantenlänge (m)

    numTags = numel(tagIDs);
    tagColors = [0 0 1;               % Tag0: Blau
                 1 0 0];              % Tag1: Rot

    % --- Raumkarte ---
    figMap = figure('Name','Room Map','NumberTitle','off');
    axis([0 8 0 8]); axis equal; grid on; hold on;
    xlabel('X (m)');
    ylabel('Y (m)');
    title('AprilTag Tracking (Multi-Cam Fusion)');

    hPaths  = cell(numTags,1);
    hPoints = cell(numTags,1);
    hDirs   = cell(numTags,1);

    for i = 1:numTags
        hPaths{i}  = animatedline('Color', tagColors(i,:), 'LineWidth', 1.5);
        hPoints{i} = plot(NaN,NaN,'o','MarkerSize',10,'LineWidth',2,...
                          'MarkerEdgeColor','k','MarkerFaceColor',tagColors(i,:));
        hDirs{i}   = quiver(NaN,NaN,0,0,0,'Color',tagColors(i,:),...
                            'LineWidth',2,'MaxHeadSize',1.5);
    end

    legend(arrayfun(@(x)sprintf('Tag %d',x), tagIDs,'UniformOutput',false));

    %% ============================================================
    % 5. Overlay in den Kamerabildern
    % ============================================================
    cam2Poly = gobjects(numTags,1);
    cam2Txt  = gobjects(numTags,1);
    for i = 1:numTags
        cam2Poly(i) = plot(ax2, NaN, NaN, '-', 'LineWidth', 2, 'Color', tagColors(i,:));
        cam2Txt(i)  = text(ax2, NaN, NaN, '', 'Color', tagColors(i,:), ...
                           'FontSize', 14, 'FontWeight','bold', 'Interpreter','none');
    end

    cam3Poly = gobjects(numTags,1);
    cam3Txt  = gobjects(numTags,1);
    for i = 1:numTags
        cam3Poly(i) = plot(ax3, NaN, NaN, '-', 'LineWidth', 2, 'Color', tagColors(i,:));
        cam3Txt(i)  = text(ax3, NaN, NaN, '', 'Color', tagColors(i,:), ...
                           'FontSize', 14, 'FontWeight','bold', 'Interpreter','none');
    end

    %% ============================================================
    % 6. Erfolgsraten-Statistikvariablen
    % ============================================================
    totalFrames    = 0;
    detectedFrames = 0;   % Frames, in denen mindestens ein Tag detektiert wurde

    %% ============================================================
    % 7. Statistik der mittleren Geschwindigkeit (pro Tag: sum(ds)/sum(dt))
    % ============================================================
    tGlobal = tic;
    lastP   = cell(numTags,1);
    lastT   = nan(numTags,1);
    distSum = zeros(numTags,1);
    timeSum = zeros(numTags,1);

    %% ============================================================
    % 8. Initialisierung der Log-Struktur (Datensatz)
    % ============================================================
    log = struct();
    log.meta = struct();
    log.meta.tagFamily = tagFamily;
    log.meta.tagIDs    = tagIDs;
    log.meta.tagSide_m = tagSide;
    log.meta.camIDs    = [2, 3];
    log.meta.MAX_ALLOWED_SYNC_GAP = 0.030;
    log.meta.created   = datestr(now,'yyyy-mm-dd HH:MM:SS');

    log.t       = [];         % (N x 1)
    log.sync_ok = [];         % (N x 1)

    % Separate Speicherung je Tag
    log.tag = struct([]);
    for i = 1:numTags
        log.tag(i).id        = tagIDs(i);
        log.tag(i).P_cam2    = [];   % (N x 2) Raumkoordinaten
        log.tag(i).P_cam3    = [];
        log.tag(i).P_fused   = [];
        log.tag(i).det_cam2  = [];   % (N x 1) 0/1
        log.tag(i).det_cam3  = [];
        log.tag(i).det_fused = [];
    end

    frameIdx = 0;

    %% ============================================================
    % 9. Hauptschleife
    % ============================================================
    MAX_SYNC = 0.030;  % 30 ms

    while ishandle(fig2) && ishandle(fig3) && ishandle(figMap)

        totalFrames = totalFrames + 1;
        tNow = toc(tGlobal);

        t0 = tic;
        I2 = snapshot(cam2); t2 = toc(t0);
        I3 = snapshot(cam3); t3 = toc(t0);
        delta_t = abs(t2 - t3);

        % Neuen Log-Eintrag anlegen (auch wenn nicht synchron, für Post-Processing)
        frameIdx = frameIdx + 1;
        log.t(frameIdx,1) = tNow;

        if delta_t > MAX_SYNC
            log.sync_ok(frameIdx,1) = 0;

            % In diesem Frame: alle Tags als fehlend loggen
            for i = 1:numTags
                log.tag(i).P_cam2(frameIdx,:)    = [NaN NaN];
                log.tag(i).P_cam3(frameIdx,:)    = [NaN NaN];
                log.tag(i).P_fused(frameIdx,:)   = [NaN NaN];
                log.tag(i).det_cam2(frameIdx,1)  = 0;
                log.tag(i).det_cam3(frameIdx,1)  = 0;
                log.tag(i).det_fused(frameIdx,1) = 0;
            end

            drawnow limitrate;
            continue;
        end

        log.sync_ok(frameIdx,1) = 1;

        %% --- Entzerren ---
        I2u = undistortImage(I2, cameraParams_cam2, 'OutputView','full');
        I3u = undistortImage(I3, cameraParams_cam3, 'OutputView','full');

        set(hImg2,'CData',I2u);
        set(hImg3,'CData',I3u);

        %% --- AprilTag-Detektion ---
        [id2, loc2] = readAprilTag(I2u, tagFamily, cameraParams_cam2.Intrinsics, tagSide);
        [id3, loc3] = readAprilTag(I3u, tagFamily, cameraParams_cam3.Intrinsics, tagSide);

        %% --- Tag-Markierungen im Kamerabild anzeigen (Viereck + ID) ---
        for i = 1:numTags
            tagID = tagIDs(i);

            % CAM2 overlay
            idx2 = find(id2 == tagID, 1);
            if ~isempty(idx2)
                corners2 = loc2(:,:,idx2);  % 4x2
                x2 = [corners2(:,1); corners2(1,1)];
                y2 = [corners2(:,2); corners2(1,2)];
                set(cam2Poly(i), 'XData', x2, 'YData', y2, 'Visible','on');
                set(cam2Txt(i),  'Position', [corners2(1,1), corners2(1,2)], ...
                                 'String', sprintf('Tag %d', tagID), 'Visible','on');
            else
                set(cam2Poly(i), 'XData', NaN, 'YData', NaN, 'Visible','off');
                set(cam2Txt(i),  'Position', [NaN NaN], 'String','', 'Visible','off');
            end

            % CAM3 overlay
            idx3 = find(id3 == tagID, 1);
            if ~isempty(idx3)
                corners3 = loc3(:,:,idx3);  % 4x2
                x3 = [corners3(:,1); corners3(1,1)];
                y3 = [corners3(:,2); corners3(1,2)];
                set(cam3Poly(i), 'XData', x3, 'YData', y3, 'Visible','on');
                set(cam3Txt(i),  'Position', [corners3(1,1), corners3(1,2)], ...
                                 'String', sprintf('Tag %d', tagID), 'Visible','on');
            else
                set(cam3Poly(i), 'XData', NaN, 'YData', NaN, 'Visible','off');
                set(cam3Txt(i),  'Position', [NaN NaN], 'String','', 'Visible','off');
            end
        end

        detectedThisFrame = false;

        %% --- Verarbeitung pro Tag
        for i = 1:numTags
            tagID = tagIDs(i);

            % ========= cam2 =========
            idx2 = find(id2 == tagID, 1);
            if ~isempty(idx2)
                corners2 = loc2(:,:,idx2);
                P2 = projectToRoom(corners2, H_room_cam2);  % 1x2
                det2 = 1;
            else
                P2 = [];
                corners2 = [];
                det2 = 0;
            end

            % ========= cam3 =========
            idx3 = find(id3 == tagID, 1);
            if ~isempty(idx3)
                corners3 = loc3(:,:,idx3);
                P3 = projectToRoom(corners3, H_room_cam3);
                det3 = 1;
            else
                P3 = [];
                corners3 = [];
                det3 = 0;
            end

            % ========= Fusion =========
            if ~isempty(P2) && ~isempty(P3)
                P = (P2 + P3) / 2;
                detF = 1;
                detectedThisFrame = true;
            elseif ~isempty(P2)
                P = P2;
                detF = 1;
                detectedThisFrame = true;
            elseif ~isempty(P3)
                P = P3;
                detF = 1;
                detectedThisFrame = true;
            else
                % Tag fehlt in diesem Frame: in der Karte ausblenden, aber NaN loggen
                set(hPoints{i},'XData',NaN,'YData',NaN);
                set(hDirs{i},'XData',NaN,'YData',NaN,'UData',0,'VData',0);

                log.tag(i).P_cam2(frameIdx,:)    = [NaN NaN];
                log.tag(i).P_cam3(frameIdx,:)    = [NaN NaN];
                log.tag(i).P_fused(frameIdx,:)   = [NaN NaN];
                log.tag(i).det_cam2(frameIdx,1)  = det2;
                log.tag(i).det_cam3(frameIdx,1)  = det3;
                log.tag(i).det_fused(frameIdx,1) = 0;
                continue;
            end

            % ========= Geschwindigkeitsakkumulation (v_avg = sum(ds)/sum(dt)) =========
            if ~isempty(lastP{i}) && ~isnan(lastT(i))
                dtv = tNow - lastT(i);
                if dtv > 0
                    dsv = norm(P - lastP{i});
                    distSum(i) = distSum(i) + dsv;
                    timeSum(i) = timeSum(i) + dtv;
                end
            end
            lastP{i} = P;
            lastT(i) = tNow;

            % ========= Richtungsabschätzung (über Eckpunkte einer Kamera) =========
            if ~isempty(corners2)
                corners = corners2;  H = H_room_cam2;
            else
                corners = corners3;  H = H_room_cam3;
            end

            % Eckpunktreihenfolge: [oben-links; unten-links; unten-rechts; oben-rechts]
            tl = corners(1,:);
            tr = corners(4,:);

            tl_h = [tl 1] * H.';
            tr_h = [tr 1] * H.';

            tl_room = tl_h(1:2) / tl_h(3);
            tr_room = tr_h(1:2) / tr_h(3);

            dirVec = tr_room - tl_room;
            if norm(dirVec) > 0
                dirVec = dirVec / norm(dirVec);
            end

            arrow_len = 0.5;
            u = dirVec(1) * arrow_len;
            v = dirVec(2) * arrow_len;

            % ========= Raumkarte updaten =========
            addpoints(hPaths{i}, P(1), P(2));
            set(hPoints{i},'XData',P(1),'YData',P(2));
            set(hDirs{i},'XData',P(1),'YData',P(2),'UData',u,'VData',v);

            % ========= Logging =========
            if det2 == 1
                log.tag(i).P_cam2(frameIdx,:) = P2;
            else
                log.tag(i).P_cam2(frameIdx,:) = [NaN NaN];
            end
            if det3 == 1
                log.tag(i).P_cam3(frameIdx,:) = P3;
            else
                log.tag(i).P_cam3(frameIdx,:) = [NaN NaN];
            end

            log.tag(i).P_fused(frameIdx,:)   = P;
            log.tag(i).det_cam2(frameIdx,1)  = det2;
            log.tag(i).det_cam3(frameIdx,1)  = det3;
            log.tag(i).det_fused(frameIdx,1) = detF;
        end

        if detectedThisFrame
            detectedFrames = detectedFrames + 1;
        end

        drawnow limitrate;
    end

    clear cam2 cam3;

    %% ============================================================
    % 10. Ende: Statistik & Datensatz speichern
    % ============================================================
    successRate = detectedFrames / totalFrames * 100;

    % Mittlere Geschwindigkeit pro Tag
    vAvg = nan(numTags,1);
    for i = 1:numTags
        if timeSum(i) > 0
            vAvg(i) = distSum(i) / timeSum(i);
        end
    end

    % Aggregierte mittlere Geschwindigkeit (alle Tags zusammen)
    if sum(timeSum) > 0
        vAvgAll = sum(distSum) / sum(timeSum);
    else
        vAvgAll = NaN;
    end

    % In log.stats speichern (für spätere Auswertung)
    log.stats = struct();
    log.stats.totalFrames    = totalFrames;
    log.stats.detectedFrames = detectedFrames;
    log.stats.successRate_percent = successRate;

    log.stats.distSum_m    = distSum;
    log.stats.timeSum_s    = timeSum;
    log.stats.vAvg_mps     = vAvg;
    log.stats.vAvgAll_mps  = vAvgAll;

    % Datei speichern
    ts = datestr(now,'yyyymmdd_HHMMSS');
    fname = sprintf('log_apriltag_raw_session_%s.mat', ts);
    save(fname, 'log');

    %% Ausgabe
    fprintf("\n================= Tracking beendet =================\n");
    fprintf("Alle Frames           : %d\n", totalFrames);
    fprintf("Erfolgreiche Detektionen: %d\n", detectedFrames);
    fprintf("AprilTag Erfolgsrate  : %.2f%%\n", successRate);

    fprintf("\n================= Mittlere Geschwindigkeit =================\n");
    for i = 1:numTags
        fprintf("Tag %d: v_avg = %.4f m/s (dist=%.3f m, time=%.3f s)\n", ...
            tagIDs(i), vAvg(i), distSum(i), timeSum(i));
    end
    fprintf("Alle Tags (aggregiert): v_avg = %.4f m/s\n", vAvgAll);

    fprintf("\nDatensatz wurde gespeichert: %s\n", fname);
    fprintf("============================================\n");
end

%% =====================================================
% Hilfsfunktion: Vier Eckpunkte auf Raumebene projizieren
% und den Mittelpunkt in Raumkoordinaten bestimmen
%% =====================================================
function P = projectToRoom(corners, H)
    corners_h = [corners, ones(4,1)];
    world_h = corners_h * H.';
    world = world_h(:,1:2) ./ world_h(:,3);
    P = mean(world,1);
end
