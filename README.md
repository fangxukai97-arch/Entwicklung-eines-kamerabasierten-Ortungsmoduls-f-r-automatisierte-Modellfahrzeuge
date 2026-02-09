# Multi-Kamera-Tracking (MATLAB)

Dieses Repository enthält MATLAB-Skripte zur kamerabasierten Positionsbestimmung von Modellfahrzeugen im Versuchsraum.  
Die Hauptpipeline ist in **step1 bis step5** gegliedert. Für die nachgelagerte Auswertung/Analyse werden separate **Test- Skripte** verwendet.

---

## Ordner- / Dateistruktur (Übersicht)

### Hauptpipeline (step1–step5)
Diese Skripte bilden den vollständigen Ablauf von der Datenerfassung bis zur Echtzeit-Positionsausgabe:

1. **`step1_capture_calib_images.m`**  
   Aufnahme und Speicherung von Kalibrierbildern (z. B. Schachbrettmuster) für die Kamera(s).

2. **`step2_intrinsik_kalibrierung.m`**  
   Intrinsische Kalibrierung (Kameramatrix, Verzerrungsparameter) auf Basis der aufgenommenen Kalibrierbilder.

3. **`step3_H_matrix_erzeugen.m`**  
   Berechnung der Homographiematrix **H** zur Abbildung zwischen Pixelkoordinaten und Welt-/Raumkoordinaten (Bodenebene).

4. **`step4_multifarbe_kalman_ros.m`**  
   Hauptprogramm für Echtzeit-Fahrzeugerkennung per **HSV/Farbsegmentierung**, Koordinatenfusion und Kalman-Filterung; zusätzlich Ausgabe/Publishing der Positionen über ROS.

5. **`step5_apriltag_kalman_ros.m`**  
   Alternative Hauptpipeline zur Echtzeit-Erkennung über **AprilTag**, inkl. Positionsbestimmung, Fusion, Kalman-Filterung und ROS-Ausgabe.

---

### Auswertung / Tests (Test*)
Die folgenden Skripte dienen der Offline-Analyse der Messergebnisse und der Vergleichsauswertung:

- **`Test_Apriltag_erkennungsrate.m`**  
  Auswertung der Erkennungsrate im AprilTag-basierten Verfahren.

- **`Test_farbe_Geschwindigkeit_Erkennungsrate.m`**  
  Analyse des Zusammenhangs zwischen Fahrzeuggeschwindigkeit und Erkennungsrate für die Farberkennung (HSV).

- **`Test1_trajektorie_vor_nach_kalman_speichern.m`**  
  Vergleich und Speicherung von Trajektorien **vor** und **nach** Kalman-Filterung.

- **`Test2_kalman_jitter_vergleich.m`**  
  Vergleich des Jitters (Positionsrauschen) mit/ohne Filter bzw. zwischen Varianten.

- **`Test3_kalman_kruemmung_vergleich.m`**  
  Analyse der Trajektorienglättung über Krümmungskennwerte (z. B. Vergleich vor/nach Filterung).

---

## Empfohlener Ablauf (Quick Start)

1. **Kalibrierdaten erfassen**  
   `step1_capture_calib_images.m`

2. **Intrinsische Kalibrierung durchführen**  
   `step2_intrinsik_kalibrierung.m`

3. **Homographie bestimmen**  
   `step3_H_matrix_erzeugen.m`

4. **Echtzeitbetrieb starten (eine Methode wählen)**  
   - Farberkennung: `step4_multifarbe_kalman_ros.m`  
   - AprilTag: `step5_apriltag_kalman_ros.m`

5. **Ergebnisse offline auswerten**  
   Auswertungsskripte `Test*.m` nach Bedarf ausführen.

---

## Hinweise
- Die step-Skripte bauen logisch aufeinander auf (Kalibrierung → Homographie → Tracking/Fusion/Filter → ROS-Ausgabe).
- Die Test-Skripte sind für die Analyse von Messdaten/Logs gedacht und können unabhängig vom Live-Betrieb verwendet werden.
