#!/usr/bin/env python3
"""
Nodo ROS 2 per la Rilevazione e il Tracciamento di Oggetti
utilizzando YOLOv8 (Ultralytics) su un robot mobile (come LIMO).

Questo nodo si iscrive ai topic della telecamera (immagine a colori e profondità),
esegue l'inferenza YOLO per identificare un oggetto target specificato e,
in base alla sua posizione e distanza, invia comandi di velocità per
tracciare l'oggetto o fermarsi se è troppo vicino.
"""
import os
os.environ["TORCH_CPP_LOG_LEVEL"] = "ERROR"

import time
import cv2
import numpy as np
import rclpy                               # type: ignore
from rclpy.node import Node                # type: ignore
            
from sensor_msgs.msg import Image          # type: ignore
from cv_bridge import CvBridge             # type: ignore
from geometry_msgs.msg import Twist        # type: ignore

from vision_msgs.msg import (              # type: ignore
    Detection2D,
    ObjectHypothesisWithPose,
    BoundingBox2D
)

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


class LimoYoloNode(Node):
    """
    Nodo principale ROS 2 per l'elaborazione delle immagini e il controllo
    del movimento basato sulle rilevazioni YOLO.

    Gestisce l'inferenza YOLO, l'estrazione della profondità per l'oggetto
    rilevato e il loop di controllo per il tracciamento e l'arresto di sicurezza.
    """
    def __init__(self):
        super().__init__("limo_yolo")

        # ----------------------------------------------------
        # Parametri
        # ----------------------------------------------------
        self.declare_parameter("image_topic", "/image")
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("score_thresh", 0.5)             # Soglia di confidenza minima per non scartare una predizione
        self.declare_parameter("filter_class", "elephant")      # Classe target da individuare
        self.declare_parameter("safe_distance", 1.0)            # Distanza di sicurezza a cui fermarsi rispetto al target

        self.img_topic = self.get_parameter("image_topic").value
        self.model_path = self.get_parameter("model").value
        self.score_thresh = self.get_parameter("score_thresh").value
        self.filter_class = self.get_parameter("filter_class").value.lower()
        self.safe_distance = self.get_parameter("safe_distance").value

        # ----------------------------------------------------
        # Variabili di stato
        # ----------------------------------------------------
        self.bridge = CvBridge()
        self.model = None
        self.model_names = {}

        self.last_detections = []

        self.target_detected = False
        self.target_confidence = 0.0
        self.target_last_seen_time = time.time()
        self.stopped = False   

        self.latest_depth = None


        # Variabili per il tracciamento (calcolate in detect_target)
        self.target_cx = 0      # Centro x (colonna) del bounding box target
        self.target_cy = 0      # Centro y (riga) del bounding box target

        # ----------------------------------------------------
        # ROS I/O
        # ----------------------------------------------------
        self.sub_image = self.create_subscription(
            Image, self.img_topic, self.on_image, 10
        )

        self.cmd_pub = self.create_publisher(
            Twist, "/cmd_vel", 10
        )

        self.sub_depth = self.create_subscription(
            Image,
            "/camera/depth/image_raw",
            self.on_depth,
            10
        )

        # Per pubblicare immagini con bounding box sovrapposta 
        self.pub_image = self.create_publisher(
            Image,
            "/yolo/annotated_image",
            10
        )

        # Control loop (20 Hz)
        self.control_timer = self.create_timer(
            0.05, self.control_loop
        )

        # Carica YOLO
        self.load_model()
        self.get_logger().info(
            f"Limo YOLO node ready. Tracking: {self.filter_class}"
        )

    # ----------------------------------------------------
    # CARICAMENTO YOLO
    # ----------------------------------------------------
    def load_model(self):
        """
        Carica il modello YOLOv8 specificato dal parametro 'model'.
        Esegue la fusione dei modelli per un'inferenza più veloce.

        Raises:
            RuntimeError: Se il pacchetto 'ultralytics' non è installato.
        """
        if YOLO is None:
            raise RuntimeError("Ultralytics YOLO not installed")

        t0 = time.time()
        self.model = YOLO(self.model_path)
        self.model.fuse()
        self.model_names = self.model.names

        dt = time.time() - t0
        self.get_logger().info(
            f"Loaded YOLO model '{self.model_path}' in {dt:.2f}s"
        )

    # ----------------------------------------------------
    # CALLBACK DELLA CAMERA
    # ----------------------------------------------------
    def on_image(self, msg: Image):
        """
        Callback per la ricezione delle immagini a colori.

        Esegue i seguenti passaggi:
        1. Conversione dell'immagine ROS in formato OpenCV (BGR).
        2. Esecuzione dell'inferenza YOLO.
        3. Aggiornamento dello stato di rilevamento del target tramite `detect_target`.
        4. Disegno del bounding box e del centro sull'immagine se il target è rilevato.
        5. Pubblicazione dell'immagine annotata.

        Args:
            msg (Image): Messaggio immagine ROS 2.
        """
        
        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        h, w = img_bgr.shape[:2]
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

        # Run YOLO inference
        results = self.model.predict(
            img_rgb,
            imgsz=320,
            conf=self.score_thresh,
            verbose=False
        )

        if not results:
            return

        r = results[0]
        if r.boxes is None:
            return

        boxes = r.boxes
        xyxy = boxes.xyxy.cpu().numpy()
        conf = boxes.conf.cpu().numpy()
        cls_ids = boxes.cls.cpu().numpy()

        
        self.detect_target(
            xyxy, conf, cls_ids,
            h, w,
            target_class=self.filter_class,
            threshold_score=self.score_thresh
        )

        # Sovrappone bounding box sull'immagine se l'oggetto target è stato individuato
        if self.target_detected:

            x = int(self.target_cx)
            y = int(self.target_cy)

            
            for i in range(len(xyxy)):
                x1, y1, x2, y2 = xyxy[i]
                cls_name = str(self.model_names.get(int(cls_ids[i]), "unknown")).lower()

                if cls_name == self.filter_class:
                    x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

                    # Contorno della bounding box
                    cv2.rectangle(
                        img_bgr,
                        (x1, y1),
                        (x2, y2),
                        (0, 255, 0),
                        2
                    )

                    # Centro della bounding box
                    cv2.circle(
                        img_bgr,
                        (self.target_cx, self.target_cy),
                        5,
                        (0, 0, 255),
                        -1
                    )

                    # Aggiunge la label (etichetta) relativa alla classe
                    label = f"{self.filter_class} {self.target_confidence:.2f}"
                    cv2.putText(
                        img_bgr,
                        label,
                        (x1, max(y1 - 10, 0)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )

                    break

            # Pubblica l'immagine con annotazione (bounding box)
            out_msg = self.bridge.cv2_to_imgmsg(img_bgr, encoding="bgr8")
            self.pub_image.publish(out_msg)


    def on_depth(self, msg: Image):
        """
        Callback per la ricezione delle immagini di profondità.
        Memorizza l'array numpy della profondità.

        Args:
            msg (Image): Messaggio immagine ROS 2 (codifica 'passthrough').
        """
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warning(f"Depth cv_bridge failed: {e}")


    def detect_target(self, xyxy: np.ndarray, conf: np.ndarray, cls_ids: np.ndarray, image_h: int, image_w: int,
                    target_class: str, threshold_score: float):
        """
        Analizza i risultati YOLO, identifica il target specificato e aggiorna
        le variabili di stato del nodo (`target_detected`, `target_cx`, ecc.).

        Applica una logica di rilassamento della soglia (metà `threshold_score`)
        se l'oggetto è già stato precedentemente rilevato per migliorarne la stabilità.

        Args:
            xyxy (np.ndarray): Coordinate dei bounding box (x1, y1, x2, y2).
            conf (np.ndarray): Punteggi di confidenza.
            cls_ids (np.ndarray): ID delle classi.
            image_h (int): Altezza dell'immagine.
            image_w (int): Larghezza dell'immagine.
            target_class (str): Nome della classe target da tracciare.
            threshold_score (float): Soglia di confidenza minima.
        """

        # Itera per ogni bounding box
        for i in range(len(xyxy)):


            x1, y1, x2, y2 = xyxy[i]

            # Estrae la confidenza per l'elemento corrente
            score = None    # TODO

            # Estrae la classe per l'elemento corrente
            class_idx = None # TODO
            cls_name = str(self.model_names.get(int(class_idx), "unknown")).lower()

            # Considera solo gli elementi appartenenti alla classe target
            # TODO
            
            # Considera l'elemento corrente solo se la soglia di confidenza è sufficientemente elevata
            # TODO

            # Rilassa la soglia di confidenza se il bersaglio è stato già individuato (per stabilità)
            if self.target_detected and score < (threshold_score / 2):
                continue

            # L'obiettivo è stato individuato correttamente
            self.target_confidence = None   # TODO
            self.target_last_seen_time = time.time()

            # Calcola il centro della bounding box
            self.target_cx = None # TODO
            self.target_cy = None # TODO


            self.target_detected = None # TODO

            return  

    # Control loop
    def control_loop(self):
        """
        Loop di controllo periodico (20 Hz) che determina i comandi di movimento.

        Logica:
        1. Se il target è perso per più di 1 secondo, entra in modalità ricerca.
        2. Se il target non è rilevato, il robot ruota sul posto (cerca).
        3. Se il target è rilevato E la profondità è disponibile:
            a. Controlla la distanza di sicurezza (`safe_distance`).
            b. Se troppo vicino, pubblica l'arresto e imposta `self.stopped = True`.
        4. Altrimenti (target rilevato e distanza OK), il robot avanza.
        """
        now = time.time()

        if self.stopped:
            return

        # Se è passato più di un secondo dall'ultima volta in cui il bersaglio è stato individuato, è considerato perso
        if True:    # TODO
            self.target_detected = None     # TODO
            self.latest_depth = None

        # Se il bersaglio non è stato individuato, ruota sul posto
        
        # TODO

        if self.latest_depth is not None and self.target_detected:

            # Estrae la profondità (in metri) nel centro del box target
            dist = self.latest_depth[self.target_cy, self.target_cx]

            # Se il robot è abbastanza vicino al target: stop (controllare anche se dist > 0.0 per evitare valori invalidi)

            # TODO


        self.stopped = False
        
        # Altrimenti avanza
        speed = 0.4
        # TODO

    # ----------------------------------------------------
    # METODI DI SUPPORTO
    # ----------------------------------------------------
    def publish_twist(self, linear=0.0, angular=0.0):
        """
        Pubblica un comando di velocità lineare e angolare sul topic /cmd_vel.

        Args:
            linear (float): Velocità lineare in X (m/s).
            angular (float): Velocità angolare in Z (rad/s).
        """
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def publish_stop(self):
        """Pubblica un comando di velocità nullo (arresto del robot)."""
        self.publish_twist(0.0, 0.0)


# ----------------------------------------------------
# MAIN
# ----------------------------------------------------
def main():
    """
    Funzione principale per l'esecuzione del nodo.
    Inizializza ROS 2, crea il nodo e avvia lo spin.
    Garantisce l'arresto del robot al termine.
    """
    rclpy.init()
    node = LimoYoloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()   
        rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()