function step3_H_matrix_erzeugen

    % ======================================================
    % Für verschiedene Kameras werden getrennte Extrinsiken
    % (Homographie-Matrix H) bestimmt.
    % camTag anpassen: cam1 / cam2 /
    % ======================================================

    % ======== Hier ändern: Kamera-Tag zur Unterscheidung =========
    camTag = 'cam1';   % ← auf cam2 ändern, um die zweite Kamera zu kalibrieren
    % =============================================================

    clc;

    %% === Kameraintrinsiken laden ===
    paramFile = fullfile('params', sprintf('cameraParams_%s.mat', camTag));
    assert(isfile(paramFile), ...
        'Datei mit Kameraintrinsiken nicht gefunden: %s\nBitte zuerst step2_calibrate_camera_generic ausführen.', paramFile);

    load(paramFile, 'cameraParams');
    fprintf('Kameraintrinsiken geladen: %s\n', paramFile);

    %% === Kamera auswählen (per Index, nicht per Name) ===
    cams = webcamlist;
    disp('Verfügbare Kameras:');
    disp(cams);

    % camID vom Benutzer abfragen: 1 / 2 / 3 (muss physisch der gleichen Kamera wie bei der Intrinsik-Kalibrierung entsprechen)
    camID = input(sprintf('Bitte Kamera-ID (1/2/3) auswählen für die Extrinsik-Kalibrierung von %s: ', camTag));

    cam = webcam(camID);
    fprintf('Ausgewählte Kamera #%d: %s\n', camID, cam.Name);

    %% === Auflösung einstellen ===
    try
        cam.Resolution = '1920x1080';
        fprintf('Auflösung gesetzt auf: %s\n', cam.Resolution);
    catch
        warning('Die Auflösung 1920x1080 wird nicht unterstützt. Automatisches Fallback:');
        disp(cam.AvailableResolutions);
        cam.Resolution = cam.AvailableResolutions{end};
        fprintf('Zurückgefallen auf: %s\n', cam.Resolution);
    end

    %% === Ein Kalibrierbild aufnehmen ===
    pause(2);
    I = snapshot(cam);
    clear cam;
    fprintf('Ein Bild wurde aufgenommen.\n');

    % Entzerren
    Iu = undistortImage(I, cameraParams);

    figure; imshow(Iu);
    title('4 Referenzpunkte anklicken und die Raumkoordinaten eingeben (Einheit: Meter)');
    hold on;

    %% === Referenzpunkte manuell auswählen ===
    nPoints = 4;
    imgPts = zeros(nPoints, 2);
    roomPts = zeros(nPoints, 2);

    for k = 1:nPoints
        [x, y] = ginput(1);
        imgPts(k,:) = [x, y];

        prompt = sprintf('Bitte Raumkoordinaten für Referenzpunkt %d eingeben [X Y] (m):', k);
        answer = inputdlg(prompt, 'Eingabe der Raumkoordinaten', [1 40], {'0 0'});
        coords = str2num(answer{1}); %#ok<ST2NM>

        if numel(coords) ~= 2
            error('Falsches Koordinatenformat. Bitte zwei Zahlen eingeben, z.B.: 1.2 3.4');
        end
        roomPts(k,:) = coords;

        plot(x, y, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
        text(x + 10, y, sprintf('(%.2f, %.2f)m', coords(1), coords(2)), ...
            'Color', 'y', 'FontWeight', 'bold', 'FontSize', 10);
    end
    hold off;

    %% === Homographie-Matrix H mit DLT berechnen ===
    A = [];
    for i = 1:nPoints
        u = imgPts(i,1); v = imgPts(i,2);
        X = roomPts(i,1); Y = roomPts(i,2);
        A = [A;
             -u, -v, -1, 0, 0, 0, u*X, v*X, X;
              0, 0, 0, -u, -v, -1, u*Y, v*Y, Y];
    end

    [~, ~, V] = svd(A);
    H = reshape(V(:, end), 3, 3)';
    H = H / H(3,3);
    fprintf('Homographie-Matrix H wurde berechnet.\n');

    %% === Unter kamerabezogenem Dateinamen speichern ===
    if ~exist('params','dir'), mkdir('params'); end
    saveName = fullfile('params', sprintf('H_room_%s.mat', camTag));
    save(saveName, 'H');

    fprintf('Homographie-Matrix gespeichert unter: %s\n', saveName);

    %% === Mapping-Verifikation ===
    pTest = [imgPts ones(size(imgPts,1),1)] * H';
    pTest = pTest ./ pTest(:,3);

    fprintf('\n Verifikation: Abgebildete Raumkoordinaten der Bildpunkte (sollten nahe an den Eingabewerten liegen):\n');
    disp(pTest(:,1:2));

    %% === Raum-Punkte visualisieren ===
    figure;
    plot(roomPts(:,1), roomPts(:,2), 'ro-', 'LineWidth', 2, 'MarkerSize', 10);
    axis equal; grid on;
    xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('Raum-Referenzpunkte (%s)', camTag));
end
