function step1_capture_calib_images 
    % ==========================================
    % Allgemeines Skript zur Aufnahme von Kalibrierbildern
    % ==========================================

    % ======== ① Nur diese zwei Zeilen ändern, um die Kamera zu unterscheiden =======
    camID  = 2;          % Kamera-ID: 1 oder 2
    camTag = 'cam2';     % Ausgabeordner: cam1 / cam2
    % ==============================================================================

    clc;

    % === Ausgabeverzeichnis, z.B. data/calib/cam1 ===
    outDir = fullfile('data','calib', camTag);
    if ~exist(outDir,'dir')
        mkdir(outDir);
    end

    % === Alle verfügbaren Kameras anzeigen ===
    cams = webcamlist;
    disp('Verfügbare Kameras:');
    disp(cams);

    % === Kamera mit der ID camID erzwingen ===
    try
        cam = webcam(camID);
        fprintf('Ausgewählte Kamera #%d: %s\n', camID, cam.Name);
    catch
        error('Kamera mit der ID %d kann nicht geöffnet werden. Bitte USB-Reihenfolge prüfen.', camID);
    end

    % === Auflösung einstellen ===
    try
        cam.Resolution = '1920x1080';
        fprintf('Aktuelle Auflösung: %s\n', cam.Resolution);
    catch
        warning('Die Kamera unterstützt 1920x1080 nicht. Automatisches Fallback:');
        disp(cam.AvailableResolutions);
        cam.Resolution = cam.AvailableResolutions{end};
        fprintf('Zurückgefallen auf: %s\n', cam.Resolution);
    end

    % === Vorschaufenster ===
    hFig = figure('Name', ...
        sprintf('Calibration Capture [%s] (SPACE=speichern, Q=beenden)', camTag), ...
        'NumberTitle','off');
    set(hFig,'KeyPressFcn',@(s,e)setappdata(s,'key',e.Key));

    I = snapshot(cam);
    hIm = imshow(I);
    title('SPACE drücken zum Speichern, Q drücken zum Beenden');

    idxImg = 51;
    while ishandle(hFig)
        I = snapshot(cam);
        set(hIm,'CData',I);
        drawnow limitrate;

        k = getappdata(hFig,'key');
        setappdata(hFig,'key',[]);

        if isempty(k)
            continue;
        end

        switch lower(k)
            case 'space'
                fname = fullfile(outDir, sprintf('%s_%03d.png', camTag, idxImg));
                imwrite(I, fname);
                title(['Gespeichert: ', fname]);
                fprintf('Gespeichert: %s\n', fname);
                idxImg = idxImg + 1;

            case {'q','escape'}
                fprintf('Aufnahme vom Benutzer beendet.\n');
                break;
        end
    end

    clear cam;
    close(hFig);
    fprintf('Kamera wurde geschlossen.\n');
end
