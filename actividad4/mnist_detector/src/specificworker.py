#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import os
import traceback
import time
import numpy as np
import cv2
import torch

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console

# Importamos la clase base y las interfaces
from genericworker import *
import interfaces as ifaces

# --- IMPORTACIÓN DE LA RED ---
try:
    from src.net import Net
except ImportError:
    try:
        from net import Net
    except ImportError:
        print("❌ Error CRÍTICO: No se encuentra 'net.py'.")
        sys.exit(-1)

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = configData["Period"]["Compute"]

        # Variables de estado
        self.detected_number = -1
        self.detected_x_center = -1
        self.last_roi_img = None

        if startup_check:
            self.startup_check()
        else:
            self.started_camera = False
            while not self.started_camera:
                try:
                    print("Connecting to Camera360RGB...")
                    self.rgb_original = self.camera360rgb_proxy.getROI(-1, -1, -1, -1, -1, -1)
                    print("Connected to Camera360RGB")
                    self.started_camera = True
                except Ice.Exception as e:
                    traceback.print_exc()
                    print("Waiting for camera...", e)
                    time.sleep(1)

            # --- CARGA DEL MODELO ---
            self.device = torch.device("cpu")
            self.model = Net().to(self.device)
            self.model_loaded = False

            # RUTA CORREGIDA SEGÚN TU INDICACIÓN
            current_dir = os.path.dirname(os.path.abspath(__file__))
            # Buscamos en ../dnn/my_network.pt
            model_path = os.path.join(current_dir, "../dnn/my_network.pt")

            if os.path.exists(model_path):
                try:
                    self.model.load_state_dict(torch.load(model_path, map_location=self.device))
                    self.model.eval()
                    self.model_loaded = True
                    console.print(f"[bold green]✅ MODELO CARGADO CORRECTAMENTE: {model_path}[/bold green]")
                    # ... dentro del try de carga del modelo ...
                    self.model.load_state_dict(torch.load(model_path, map_location=self.device))
                    self.model.eval()
                    self.model_loaded = True

                    # --- VERIFICACIÓN DE PESOS ---
                    # Si esto imprime "nan", el modelo está roto.
                    if hasattr(self.model, 'conv1'):
                        print("DEBUG PESOS (Media capa 1):", self.model.conv1.weight.mean().item())

                    console.print(f"[bold green]✅ MODELO CARGADO CORRECTAMENTE: {model_path}[/bold green]")
                except Exception as e:
                    console.print(f"[bold red]❌ ERROR FATAL cargando pesos: {e}[/bold red]")
                    # Si falla la carga, mejor salir que predecir basura
                    sys.exit(-1)


            else:
                console.print(f"[bold red]⛔ NO EXISTE EL ARCHIVO: {model_path}[/bold red]")
                console.print("El componente se cerrará para evitar predecir con una red vacía.")
                sys.exit(-1)

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    @QtCore.Slot()
    def compute(self):
        # Seguridad extra
        if not self.model_loaded: return

        try:
            # 1. Obtener imagen
            image = self.camera360rgb_proxy.getROI(-1, -1, -1, -1, -1, -1)
            color = np.frombuffer(image.image, dtype=np.uint8).reshape(image.height, image.width, 3)

            # 2. Detectar ROI (Cuadro Negro)
            rect = self.detect_black_square(color)
            debug_img = color.copy()

            if rect is not None:
                x1, y1, x2, y2 = rect
                cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

                roi = color[y1:y2, x1:x2]

                # 3. PREPROCESADO PRO (Estilo MNIST estricto)
                tensor_input, processed_img, digit_raw = self.preprocess_mnist_strict(roi)
                self.last_roi_img = processed_img

                if tensor_input is not None:
                    pred = -1
                    info = ""

                    # --- ANÁLISIS ---

                    # A. Geometría de Cabeza (Para el 1 vs 7)
                    is_top_narrow, top_ratio = self.analyze_top_width(digit_raw)

                    # B. Aspect Ratio Global
                    h, w = digit_raw.shape
                    global_ratio = float(w) / h

                    # CASO 1: Es un 1 por geometría pura (muy flaco)
                    if global_ratio < 0.45:
                        pred = 1
                        info = f"Geo:1 (R:{global_ratio:.2f})"

                    # CASO 2: Es un 1 por cabeza estrecha (evita confusión con 7)
                    elif is_top_narrow:
                        pred = 1
                        info = f"Head:1 (TR:{top_ratio:.2f})"

                    # CASO 3: Red Neuronal
                    else:
                        with torch.no_grad():
                            output = self.model(tensor_input)
                            pred = output.argmax(dim=1, keepdim=True).item()

                        # Filtro de coherencia: Si dice 1 pero es gordo...
                        if pred == 1 and global_ratio > 0.65:
                            info = f"DNN:1? (Suspicious)"
                        else:
                            info = f"DNN:{pred}"

                    # 5. Guardar resultados
                    self.detected_number = int(pred)
                    self.detected_x_center = int((x1 + x2) / 2)

                    # Visualización
                    cv2.putText(debug_img, info, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                    # Miniatura (Lo que ve la red)
                    mini = cv2.cvtColor(processed_img, cv2.COLOR_GRAY2BGR)
                    mini = cv2.resize(mini, (100, 100), interpolation=cv2.INTER_NEAREST)
                    debug_img[0:100, 0:100] = mini
                    cv2.putText(debug_img, "INPUT", (5, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            else:
                self.detected_number = -1
                self.detected_x_center = -1

            cv2.imshow("Detector", debug_img)
            cv2.waitKey(1)

        except Exception as e:
            traceback.print_exc()

    # -------------------------------------------------------------------------
    # 1. Detección del marco negro
    # -------------------------------------------------------------------------
    def detect_black_square(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY_INV, 11, 2)
        kernel = np.ones((3, 3), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_rect = None
        max_area = 0
        h_img, w_img = img.shape[:2]

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 800 or area > (w_img * h_img * 0.9): continue

            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / h

            if 0.5 < aspect_ratio < 2.0 and area > max_area:
                hull = cv2.convexHull(cnt)
                hull_area = cv2.contourArea(hull)
                solidity = float(area) / hull_area if hull_area > 0 else 0

                if solidity > 0.35:
                    max_area = area
                    # Recorte 20%
                    margin_x = int(w * 0.20)
                    margin_y = int(h * 0.20)
                    x1 = max(0, x + margin_x)
                    y1 = max(0, y + margin_y)
                    x2 = min(w_img, x + w - margin_x)
                    y2 = min(h_img, y + h - margin_y)

                    if x2 > x1 and y2 > y1:
                        best_rect = (x1, y1, x2, y2)
        return best_rect

    # -------------------------------------------------------------------------
    # 2. Preprocesado "Bajada de Calidad" + Normalización
    # -------------------------------------------------------------------------
    def preprocess_mnist_strict(self, roi):
        # A. Binarización
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        # B. Dilatación suave (unir trazos)
        kernel = np.ones((3, 3), np.uint8)
        thresh = cv2.dilate(thresh, kernel, iterations=1)

        # C. Recorte exacto
        coords = cv2.findNonZero(thresh)
        if coords is None: return None, thresh, thresh
        x, y, w, h = cv2.boundingRect(coords)
        digit = thresh[y:y + h, x:x + w]

        # D. NORMALIZACIÓN a 20x20
        canvas = np.zeros((28, 28), dtype=np.uint8)
        max_side = max(w, h)
        scale = 20.0 / max_side
        new_w = int(w * scale)
        new_h = int(h * scale)
        new_w = max(1, new_w)
        new_h = max(1, new_h)

        # Usamos INTER_AREA para simular baja resolución/grises
        resized_digit = cv2.resize(digit, (new_w, new_h), interpolation=cv2.INTER_AREA)

        # Centrar
        x_off = (28 - new_w) // 2
        y_off = (28 - new_h) // 2
        canvas[y_off:y_off + new_h, x_off:x_off + new_w] = resized_digit

        # E. Tensor y Normalización
        # Pasamos a float 0..1
        tensor = torch.from_numpy(canvas).float() / 255.0

        # APLICAR NORMALIZACIÓN MNIST (La misma que en el entrenamiento)
        # mean=0.1307, std=0.3081
        tensor = (tensor - 0.1307) / 0.3081

        # Añadir dimensiones batch y canal
        tensor = tensor.unsqueeze(0).unsqueeze(0)

        return tensor.to(self.device), canvas, digit

    # -------------------------------------------------------------------------
    # 3. Análisis Geométrico (Guardaespaldas del 1)
    # -------------------------------------------------------------------------
    def analyze_top_width(self, digit_img):
        h, w = digit_img.shape
        limit_y = int(h * 0.3)
        top_part = digit_img[0:limit_y, :]

        coords = cv2.findNonZero(top_part)
        if coords is None: return False, 0.0

        _, _, tw, _ = cv2.boundingRect(coords)
        ratio = float(tw) / w
        return (ratio < 0.5), ratio

    # -------------------------------------------------------------------------
    # Interfaz
    # -------------------------------------------------------------------------
    def MNIST_getNumber(self):
        res = ifaces.RoboCompMNIST.TDigit()
        res.detectedNumber = self.detected_number
        res.x = self.detected_x_center
        if res.detectedNumber != -1:
            # Opcional: imprimir para debug
            pass
        return res

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)