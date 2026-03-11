#!/usr/bin/env python3
import os
os.environ["TORCH_CPP_LOG_LEVEL"] = "ERROR"

import time
import math
import cv2                                                                       # type: ignore
import numpy as np                                                               # type: ignore
import rclpy                                                                     # type: ignore
from rclpy.node import Node                                                      # type: ignore
from sensor_msgs.msg import Image, LaserScan                                     # type: ignore
from geometry_msgs.msg import Twist                                              # type: ignore
from cv_bridge import CvBridge                                                   # type: ignore
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data     # type: ignore

try:
    from ultralytics import YOLO                                                 # type: ignore
except ImportError:
    YOLO = None


class FiniteStateMachine(Node):
    def __init__(self):
        super().__init__("limo_yolo")

        # PARAMETRI ROS
        self.declare_parameter("image_topic", "/image")
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("score_thresh", 0.8)             # Soglia di confidenza minima da soddisfare
        self.declare_parameter("filter_class", "bottle")   # Classe target da individuare
        self.declare_parameter("turn_speed_max", 1.5)           # Velocità angolare massima
        self.declare_parameter("bb_ratio_threshold", 0.1)      # Rapporto tra bounding box e immagine da usare come proxy di distanza

        self.img_topic = self.get_parameter("image_topic").value
        self.model_path = self.get_parameter("model").value
        self.score_thresh = self.get_parameter("score_thresh").value
        self.filter_class = self.get_parameter("filter_class").value.lower()
        self.turn_speed_max = self.get_parameter("turn_speed_max").value
        self.bb_ratio_threshold = self.get_parameter("bb_ratio_threshold").value

        # CAMERA E IMMAGINE
        self.image_w = None         # Ampiezza immagine (width)

        self.bridge = CvBridge()

        # TARGET
        self.model = None
        self.model_names = {}

        self.target_detected = False                # Indica se il target è individuato
        self.target_centered = False                # Indica se il target è stato centrato
        self.target_box_ratio = None                # Rapporto corrente tra area della bounding box e dell'immagine
        self.target_last_seen_time = time.time()    # Timestamp per gestire il timeout (target perso)

        # CONTROLLO MOVIMENTO
        self.rate_hz = 20
        self.center_deadband_ratio = 0.3      # ±10% of image width
        self.center_turn_speed = 0.4

        # Velocità
        self.forward_speed = 0.1        # Velocità lineare
        self.angular_speed = 0.2        # Velocità angolare

        # VARIABILE DI STATO
        self.state = "FORWARD"  # Indica lo stato interno della FSM

        # SEARCH E SCAN
        self.search_straight_distance = 1.0 # Distanza in rettilineo da percorrere in fase "FORWARD"
        self.move_forward_duration = self.search_straight_distance / self.forward_speed

        # Tempo di inizio ricerca in rettilineo (FORWARD) 
        self.search_start_time = None

        # SCAN basato su open loop 
        self.scan_phase = 0
        self.scan_start_time = None

        self.scan_right_duration = math.pi/4 / self.angular_speed
        self.scan_left_duration = math.pi/2 / self.angular_speed

        # RILEVAMENTO OSTACOLI
        self.ranges = []
        self.direction = -1     # Destra di default

        self.obstacle_detected = False

        self.end_avoiding_time = None

        self.obstacle_threshold = 0.2   # Distanza di sicurezza a cui evitare un ostacolo
        self.rejoin_distance = 0.8      # Distanza da percorrere in fase di evitamento ostacolo
        self.current_distance = np.inf  # Distanza corrente da eventuali ostacoli

        # Logica di stop
        self.stopped = False

        # -----------------------------
        # ROS I/O
        # -----------------------------
        # qos = QoSProfile(
        #     reliability=QoSReliabilityPolicy.,
        #     history=QoSHistoryPolicy.KEEP_LAST,
        #     depth=10
        # )

        self.sub_image = self.create_subscription(Image, "/camera/color/decompressed", self.on_image, 10)     # Riceve le immagini della camera
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)          # Riceve il LIDAR
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)                             # Invia le velocità
        self.pub_image = self.create_publisher(Image, "/yolo/annotated_image", 10)              # Pubblica le immagini annotate con bounding box

        self.control_timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)           # Ciclo di controllo principale

        self.load_model()
        self.get_logger().info(f"Node loaded. Tracking: {self.filter_class}")

    # -----------------------------
    # CARICAMENTO MODELLO YOLO
    # -----------------------------
    def load_model(self):
        if YOLO is None:
            raise RuntimeError("Ultralytics YOLO not installed")

        t0 = time.time()
        self.model = YOLO(self.model_path)
        self.model.fuse()
        self.model_names = self.model.names
        self.get_logger().info(f"Loaded YOLO model '{self.model_path}' in {time.time() - t0:.2f}s")

    # -----------------------------
    # CALLBACK PER CAMERA
    # -----------------------------
    def on_image(self, msg: Image):
        """
        Callback per la ricezione delle immagini a colori.

        Esegue l'inferenza YOLO, chiama `detect_target` per processare i risultati
        e sovrappone i bounding box rilevati sull'immagine prima di ripubblicarla.

        Args:
            msg (Image): Messaggio immagine ROS 2.
        """

        # Converte da ROS a OpenCV
        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge failed: {e}")
            return

        h, w = img_bgr.shape[:2]

        if self.image_w is None:
            self.image_w = w

        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

        # Lancia il modello YOLO
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

        # Sovrappone all'immagine la bounding box se il bersaglio è stato individuato
        if self.target_detected:

            x = int(self.target_cx)
            y = int(self.target_cy)

            
            for i in range(len(xyxy)):
                x1, y1, x2, y2 = xyxy[i]
                cls_name = str(self.model_names.get(int(cls_ids[i]), "unknown")).lower()

                if cls_name == self.filter_class:
                    x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

                    # Disegna il contorno della bounding box
                    cv2.rectangle(
                        img_bgr,
                        (x1, y1),
                        (x2, y2),
                        (0, 255, 0),
                        2
                    )

                    # Disegna il punto centrale
                    cv2.circle(
                        img_bgr,
                        (self.target_cx, self.target_cy),
                        5,
                        (0, 0, 255),
                        -1
                    )

                    # Aggiunge l'etichetta della classe
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

            # Pubblica l'immagine annotata
            out_msg = self.bridge.cv2_to_imgmsg(img_bgr, encoding="bgr8")
            self.pub_image.publish(out_msg)

    # -----------------------------
    # CALLBACK PER LIDAR
    # -----------------------------
    def on_scan(self, msg: LaserScan):

        self.ranges = np.array(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

        # Pulisce i valori nan e inf
        for i in range(len(self.ranges)):
            if self.ranges[i] is math.inf or self.ranges[i] is math.nan:
                self.ranges[i] = 5.0

        # Lista degli angoli
        self.angles = [self.angle_min + i * self.angle_increment for i in range(len(self.ranges))]

        # Lista di indici
        self.obstacle_indices = [idx for idx in range(len(self.ranges)) if self.ranges[idx] < self.obstacle_threshold and abs(self.angles[idx]) < math.pi/3.]
        
        if self.obstacle_indices:
            self.min_angle = self.angles[min(self.obstacle_indices)]
            self.max_angle = self.angles[max(self.obstacle_indices)]
            
            obstacle_values = [self.ranges[idx] for idx in self.obstacle_indices]
            self.current_distance = min(obstacle_values)

        else:
            self.current_distance = 10


    # -----------------------------
    # METODI DI CONTROLLO
    # -----------------------------
    def normalize_angle(self, angle: float) -> float:
        """
        Normalizza un angolo all'intervallo [-pi, pi).
        """
        return (angle + math.pi) % (2 * math.pi) - math.pi
    def compute_center_error(self):
        """
        Calcola l'errore orizzontale normalizzato rispetto al centro dell'immagine:
        -1.0 = il target si trova all'estrema sinistra
         0.0 = il target è perfettamente al centro
        +1.0 = il target si trova all'estrema destra
        
        Ritorna:
            float: Errore normalizzato nell'intervallo [-1.0, 1.0].
        """
        image_center_x = self.image_w / 2.0
        return (self.target_cx - image_center_x) / image_center_x

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
    
    def angle_to_index(self, angle: float) -> int:
        """
        Converts an angle (rad) into the corresponding LaserScan index.
        """
        return int((angle - self.angle_min) / self.angle_increment)

    def get_sector_min_distance(self, angle_min, angle_max):

        if self.ranges is None:
            return np.inf

        i_min = int((angle_min - self.angle_min) / self.angle_increment)
        i_max = int((angle_max - self.angle_min) / self.angle_increment)

        # Clamp to valid indices
        i_min = max(0, min(i_min, len(self.ranges) - 1))
        i_max = max(0, min(i_max, len(self.ranges) - 1))

        # Ensure correct order
        if i_min > i_max:
            i_min, i_max = i_max, i_min

        sector = self.ranges[i_min:i_max]

        # Remove invalid values
        sector = sector[np.isfinite(sector)]
        sector = sector[sector > 0.0]

        if len(sector) == 0:
            return np.inf

        return np.min(sector)

    # -----------------------------
    # RILEVAMENTO OSTACOLI
    # -----------------------------
    def detect_obstacle(self):
        """
        Verifica la presenza di ostacoli.

        Prioritizza l'evitamento: un ostacolo è rilevato se la distanza LiDAR
        frontale è inferiore alla soglia, A MENO CHE l'oggetto rilevato da YOLO
        sia proprio in quel punto.
        """
        
        self.obstacle_detected = self.current_distance < self.obstacle_threshold

    # -----------------------------
    # RILEVAMENTO TARGET
    # -----------------------------
    def detect_target(self, xyxy: np.ndarray, conf: np.ndarray, cls_ids: np.ndarray, image_h: int, image_w: int,
                    target_class: str, threshold_score: float):
        """
        Analizza i risultati YOLO, identifica l'oggetto target specificato
        e aggiorna le variabili di stato (`target_detected`, `target_cx`, ecc.).

        Prioritizza il target e rilassa la soglia di confidenza se è già tracciato.

        Args:
            xyxy (np.ndarray): Coordinate dei bounding box (x1, y1, x2, y2).
            conf (np.ndarray): Punteggi di confidenza.
            cls_ids (np.ndarray): ID delle classi.
            image_h (int): Altezza dell'immagine.
            image_w (int): Larghezza dell'immagine.
            target_class (str): Nome della classe target da tracciare.
            threshold_score (float): Soglia di confidenza minima.
        """

        for i in range(len(xyxy)):
            x1, y1, x2, y2 = xyxy[i]
            score = float(conf[i])
            cls_name = str(self.model_names.get(int(cls_ids[i]), "unknown")).lower()

            # Debug
            # self.get_logger().info(f"Detected: {cls_name} (score={score:.2f})")

            # Prosegue solo se classe e confidenza sono corrette
            if cls_name != target_class:
                continue
            
            if (not self.target_detected and score < threshold_score) or \
               (self.target_detected and score < (threshold_score / 2)):
                continue

            # Il target è stato individuato
            self.target_confidence = score
            self.target_last_seen_time = time.time()

            # Aggiorna il centro del target
            self.target_cx = int((x1 + x2) / 2)
            self.target_cy = int((y1 + y2) / 2)

            # Calcola larghezza e altezza del box in pixel
            box_w = x2 - x1
            box_h = y2 - y1

            # Calcola il rapporto di area 
            image_area = image_w * image_h
            target_area = box_w * box_h
            self.target_box_ratio = target_area / image_area

            # Debug log 
            # self.get_logger().info(f"Target ratio: {self.target_box_ratio:.4f}")

            self.target_detected = True

            return


    # -----------------------------
    # METODI DI STATO
    # -----------------------------
    def move_forward(self):

        """
        Gestisce la fase di avanzamento rettilineo durante la modalità di ricerca.
        
        Il metodo esegue le seguenti operazioni:
        1. Verifica che lo stato corrente sia effettivamente "FORWARD".
        2. Memorizza la posizione iniziale (odometria) al primo avvio della manovra.
        3. Calcola la distanza percorsa rispetto al punto di partenza.
        4. Se la distanza percorsa è uguale o superiore a 'search_straight_distance':
           - Arresta il robot.
           - Resetta tutti i parametri di navigazione e orientamento.
           - Passa allo stato "SCAN" per iniziare la rotazione di ricerca.
        5. Se la distanza non è stata ancora raggiunta, continua a pubblicare
           un comando di velocità lineare costante.
        """
        
        # Controllo di sicurezza: se lo stato interno non è FORWARD -> ritorna subito
        if self.state != "FORWARD": 
            return
        
        print("Moving forward")
        
        # Salva il tempo di inizio ricerca in rettilineo
        if self.search_start_time is None:
            self.search_start_time = time.time()

        if time.time() - self.search_start_time > self.move_forward_duration:    # Prosegue dritto per la durata indicata, poi scansiona
            self.publish_stop()

            # Resetta i parametri di scansione 
            self.scan_start_time = None
            self.scan_phase = 0

            # Resetta i parametri di stato
            self.search_start_time = None

            # Aggiorna lo stato interno
            self.state = "SCAN"

            return

        # Altrimenti, prosegue dritto
        self.publish_twist(self.forward_speed, 0.0)

        return


    def scan(self):

        if self.state != "SCAN":
            return

        now = time.time()

        # Initialize scan
        if self.scan_start_time is None:
            self.scan_start_time = now
            self.scan_phase = 0

        elapsed = now - self.scan_start_time

        # PHASE 0 → Rotate right 45°
        if self.scan_phase == 0:
            if elapsed < self.scan_right_duration:
                self.publish_twist(0.0, -self.angular_speed)
            else:
                self.scan_phase = 1
                self.scan_start_time = now

        # PHASE 1 → Rotate left 90°
        elif self.scan_phase == 1:
            if elapsed < self.scan_left_duration:
                self.publish_twist(0.0, self.angular_speed)
            else:
                self.scan_phase = 2
                self.scan_start_time = now

        # PHASE 2 → Rotate right 45° (realign)
        elif self.scan_phase == 2:
            if elapsed < self.scan_right_duration:
                self.publish_twist(0.0, -self.angular_speed)
            else:
                self.publish_stop()

                # Reset scan variables
                self.scan_start_time = None
                self.scan_phase = 0

                self.state = "FORWARD"


    def approach(self):

        """
        Gestisce l'avvicinamento finale al target rilevato e centrato.
        
        Il metodo monitora costantemente il rapporto tra l'area della bounding box 
        e l'area totale dell'immagine (target_box_ratio). 
        - Se il rapporto supera la soglia 'bb_ratio_threshold', il robot considera 
          l'obiettivo raggiunto e passa allo stato "INTERACT".
        - Altrimenti, continua ad avanzare verso il target a velocità costante.
        """
        
        # Controlla se la bounding box è abbastanza grande -> il robot è abbastanza vicino
        if self.target_box_ratio is not None:
            if self.target_box_ratio >= self.bb_ratio_threshold:
                self.publish_stop()
                self.state = "INTERACT"
                return

        # Altrimenti prosegue dritto 
        self.publish_twist(self.forward_speed, 0.0)

        return

    def interact(self):
        """
        Conclude la missione una volta raggiunto l'obiettivo.
        
        Esegue le seguenti azioni finali:
        1. Arresta completamente ogni movimento del robot.
        2. Invia un log informativo indicando il completamento del task.
        3. Imposta il flag 'self.stopped' su True per interrompere il loop di controllo.
        """
        self.publish_stop()
        # Pubblica un messaggio finale
        self.get_logger().info(f"Reached {self.filter_class}. Task completed...")
        self.stopped = True

        return

    def avoid(self):
        
        
        if self.end_avoiding_time is None:
            duration = (math.pi / 2) / self.angular_speed
                
            self.start_time = time.time()
            self.end_avoiding_time = self.start_time + duration
            
            print(f"Obstacle! Performing 90° right turn. Duration: {duration:.2f}s")

        current_time = time.time()

        if current_time < self.end_avoiding_time:
            # Continue rotating right
            self.publish_twist(0.0, self.angular_speed * self.direction)

            return
        else:
            # Rotation complete: Reset state
            print("Turn complete. Resuming...")
            self.end_avoiding_time = None
            self.publish_twist(0.0, 0.0)

        # Resetta le variabili di stato FORWARD
        self.search_start_time = None

        self.state = "FORWARD"

    def align_to_target(self):
        """
        Esegue l'allineamento del robot verso l'obiettivo rilevato (Visual Servoing).
        
        Il metodo utilizza l'errore di centraggio calcolato sui pixel dell'immagine:
        1. Calcola l'errore normalizzato tra il centro della bounding box e il 
           centro del frame della camera.
        2. Verifica se l'errore rientra nella 'zona morta' (center_deadband_ratio):
           - Se centrato: arresta la rotazione, imposta il flag 'target_centered' 
             e passa allo stato "APPROACH".
        3. Se non centrato: applica una velocità angolare proporzionale all'errore, 
           limitata dal valore massimo 'turn_speed_max', per orientare il robot 
           verso il target.
        """

        if self.image_w is None or not self.target_detected:
            self.publish_stop()
            return

        error = self.compute_center_error()

        # Deadband: il target è centrato
        if abs(error) < self.center_deadband_ratio:
            self.publish_stop()

            self.target_centered = True

            self.state = "APPROACH"
            return

        # Rotazione proporzionale
        angular_z = self.center_turn_speed * error

        # Calcola la velocità angolare
        angular_z = max(min(angular_z, self.turn_speed_max),
                        -self.turn_speed_max)

        self.publish_twist(0.0, angular_z)


    # -----------------------------
    # CONTROL LOOP
    # -----------------------------
    def control_loop(self):
        """
        Ciclo di controllo principale eseguito a 20 Hz. 
        Gestisce la logica decisionale del robot e le transizioni tra gli stati.

        Il metodo opera secondo la seguente gerarchia di priorità:
        1. VALIDAZIONE DATI: Verifica la disponibilità di odometria e dati LiDAR.
        2. SICUREZZA: Se il robot è in stato di arresto ('stopped'), pubblica velocità nulla.
        3. GESTIONE TARGET PERSO: Se il target non viene rilevato per più di 1 secondo, 
           resetta i parametri di inseguimento e forza il ritorno allo stato "SCAN".
        4. RILEVAMENTO OSTACOLI: Se il LiDAR rileva un pericolo frontale, lo stato 
           "AVOID" assume la priorità massima su qualsiasi altra azione.
        5. AGGANCIO TARGET: Se il target è visibile ma non centrato, imposta lo 
           stato "CENTER_TARGET" (a meno che non si sia già in fase di approccio finale).
        6. ESECUZIONE FSM: Smista l'esecuzione ai metodi specifici ("AVOID", "APPROACH", 
           "FORWARD", "SCAN", "CENTER_TARGET", "INTERACT") in base allo stato attivo.
        """

        now = time.time()
        
        if self.stopped:
            self.publish_stop()
            return
        
        print(self.state)
        
        '''
        # Se il bersaglio non è individuato da almeno un secondo, è considerato perso
        if now - self.target_last_seen_time > 1 and self.target_detected and self.state != "CENTER_TARGET":
            self.target_detected = False
            self.target_box_ratio = None
            self.target_centered = False

            # Resetta le variabili di stato SCAN
            self.scan_phase = 0
            self.scan_start_time = None

            self.state = "SCAN"

            return
        '''

        # Controlla se vi sono ostacoli da evitare
        self.detect_obstacle()

        if self.obstacle_detected:
            self.state = "AVOID"
        
        # Se il target viene rilevato, occorre centrarlo e avanzare
        if not self.target_centered:
            if self.target_detected and self.state not in ["APPROACH", "INTERACT"]:
                self.state = "CENTER_TARGET"


        # Logica di macchina a stati finiti (FSM)
        if self.state == "AVOID":
            self.avoid()
        elif self.state == "APPROACH":
            self.approach()
        elif self.state == "FORWARD": 
            self.move_forward()
        elif self.state == "SCAN":
            self.scan()
        elif self.state == "CENTER_TARGET":
            self.align_to_target()
        elif self.state == "INTERACT":
            self.interact()
        
        return

# -----------------------------
# MAIN
# -----------------------------
def main():
    rclpy.init()
    node = FiniteStateMachine()
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