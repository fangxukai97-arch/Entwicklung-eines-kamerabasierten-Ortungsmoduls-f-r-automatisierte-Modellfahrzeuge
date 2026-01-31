function step2_intrinsik_kalibrierung
    
    % ======== ① Nur diese zwei Zeilen ändern, um verschiedene Kameras zu unterscheiden ========
    camTag       = 'cam2';      % cam1 / cam2 verwenden
    squareSizeMM = 31;          % Schachbrett-Quadratgröße (mm)
    % =========================================================================================

    clc;

    % === Bildordner, z.B. data/calib/cam1 ===
    imgFolder = fullfile('data','calib','cam2');
    assert(isfolder(imgFolder), 'Kalibrierbildordner nicht gefunden: %s', imgFolder);

    % === Bilder einlesen ===
    imds = imageDatastore(imgFolder);
    imageFileNames = imds.Files;
    fprintf('Aus %s wurden %d Bilder geladen.\n', imgFolder, numel(imageFileNames));

    % === Schachbrett-Eckpunkte extrahieren ===
    [imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);

    fprintf('Anzahl der Bilder mit erkannten Eckpunkten: %d / %d\n', ...
        nnz(imagesUsed), numel(imageFileNames));
    fprintf('Anzahl der inneren Eckpunkte: %d × %d\n', boardSize(2), boardSize(1));

    if nnz(imagesUsed) == 0
        error('‼ Kein Schachbrett erkannt. Bitte Aufnahmewinkel und Beleuchtung prüfen.');
    end

    % === Größe des ersten gültigen Bildes bestimmen ===
    I0 = imread(imageFileNames{find(imagesUsed,1)});
    imageSize = [size(I0,1), size(I0,2)];

    % === Weltkoordinaten des Schachbretts erzeugen ===
    worldPoints = generateCheckerboardPoints(boardSize, squareSizeMM);

    % === Kalibrierung ===
    fprintf('Kameraintrinsiken werden berechnet...\n');
    cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
        'ImageSize', imageSize, ...
        'EstimateSkew', false, ...
        'NumRadialDistortionCoefficients', 3, ...
        'WorldUnits', 'mm');

    fprintf('Reprojektionsfehler (Mean Reprojection Error): %.3f px\n', cameraParams.MeanReprojectionError);

    % === Ergebnisse speichern ===
    if ~exist('params','dir'), mkdir('params'); end
    saveName = fullfile('params', sprintf('cameraParams_%s.mat', camTag));
    save(saveName, 'cameraParams');

    fprintf('Kameraparameter gespeichert: %s\n', saveName);

    % === Visualisierung ===
    figure; showReprojectionErrors(cameraParams);
    title(sprintf('Reprojection Errors (%s)', camTag));

    figure; showExtrinsics(cameraParams, 'CameraCentric');
    title(sprintf('Extrinsics (%s)', camTag));
end
