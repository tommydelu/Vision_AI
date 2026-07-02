#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

import matplotlib
matplotlib.use('Agg') # ⚠️ FONDAMENTALE per non far crashare la VM senza grafica
import matplotlib.pyplot as plt

CUBE_SIZE = 0.04
OUTPUT_DIR = '/home/delu/Desktop/vision_ai/yumi_ws'

class CompletePerceptionBridge(Node):
    def __init__(self):
        super().__init__('locate_cube_node_MACOS')
        
        self.get_logger().info("Connessione a CoppeliaSim sul Mac...")
        self.client = RemoteAPIClient('192.168.1.12') 
        self.sim = self.client.require('sim')
        self.get_logger().info("Connesso a CoppeliaSim con successo!")

        self.h_right_target = self.sim.getObject('/rightTarget')
        self.h_left_target = self.sim.getObject('/leftTarget')
        self.h_right_tip = self.sim.getObject('/rightTip')
        self.h_left_tip = self.sim.getObject('/leftTip')
        self.h_sensor = self.sim.getObject('/Vision_sensor2')
        # --- Camera ORIGINALE usata dalla logica di scan in yumi_cube_node ---
        # /sensor_pose alimenta sensor_front_position() in yumi_cube_node.cpp,
        # che calcola dove posizionare il cubo per la scansione. Quella logica
        # è calibrata sulla posa della camera originale, non su Vision_sensor2
        # (che è vicina al tavolo, per il solo rilevamento del centro cubo).
        # Se pubblichiamo /sensor_pose da Vision_sensor2, yumi_cube_node calcola
        # un punto di scan completamente diverso e sbagliato.
        self.h_sensor_scan = self.sim.getObject('/Vision_sensor')
        self.h_cube = self.sim.getObject('/RubickCube') 

        self.h_right_joints = [self.sim.getObject(f'/rightJoint{i}') for i in range(1, 8)]
        self.h_left_joints = [self.sim.getObject(f'/leftJoint{i}') for i in range(1, 8)]

        self.pub_pose_R = self.create_publisher(Pose, '/yumi/right/current_pose', 10)
        self.pub_pose_L = self.create_publisher(Pose, '/yumi/left/current_pose', 10)
        self.pub_joints_R = self.create_publisher(JointState, '/yumi/right/current_joint_state', 10)
        self.pub_joints_L = self.create_publisher(JointState, '/yumi/left/current_joint_state', 10)
        
        self.pub_cube_pcl = self.create_publisher(Pose, '/get_cube_pose', 10)      
        self.pub_cube_gt = self.create_publisher(Pose, '/cube_pose', 10)          
        self.pub_sensor_gt = self.create_publisher(Pose, '/sensor_pose', 10)      

        self.create_subscription(Pose, '/yumi/right/desired_pose', self.target_pose_R_cb, 10)
        self.create_subscription(Pose, '/yumi/left/desired_pose', self.target_pose_L_cb, 10)
        self.create_subscription(JointState, '/yumi/right/desired_joint_state', self.target_joints_R_cb, 10)
        self.create_subscription(JointState, '/yumi/left/desired_joint_state', self.target_joints_L_cb, 10)

        self.detection_done = False
        self.timer = self.create_timer(0.02, self.main_loop)

    def target_pose_R_cb(self, msg):
        pose = [msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.sim.setObjectPose(self.h_right_target, -1, pose)

    def target_pose_L_cb(self, msg):
        pose = [msg.position.x, msg.position.y, msg.position.z, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        self.sim.setObjectPose(self.h_left_target, -1, pose)

    def target_joints_R_cb(self, msg):
        for i, pos in enumerate(msg.position[:7]):
            self.sim.setJointTargetPosition(self.h_right_joints[i], pos)

    def target_joints_L_cb(self, msg):
        for i, pos in enumerate(msg.position[:7]):
            self.sim.setJointTargetPosition(self.h_left_joints[i], pos)

    def get_and_pub_pose(self, handle, publisher):
        p = self.sim.getObjectPose(handle, -1)
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = p[0], p[1], p[2]
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = p[3], p[4], p[5], p[6]
        publisher.publish(msg)
        return msg

    def main_loop(self):
        try:
            self.get_and_pub_pose(self.h_right_tip, self.pub_pose_R)
            self.get_and_pub_pose(self.h_left_tip, self.pub_pose_L)
            self.get_and_pub_pose(self.h_cube, self.pub_cube_gt)
            self.get_and_pub_pose(self.h_sensor_scan, self.pub_sensor_gt)

            js_R = JointState()
            js_R.header.stamp = self.get_clock().now().to_msg()
            js_R.position = [self.sim.getJointPosition(h) for h in self.h_right_joints]
            self.pub_joints_R.publish(js_R)

            js_L = JointState()
            js_L.header.stamp = self.get_clock().now().to_msg()
            js_L.position = [self.sim.getJointPosition(h) for h in self.h_left_joints]
            self.pub_joints_L.publish(js_L)

            if not self.detection_done:
                self.get_logger().info("Scatto la foto e calcolo la Point Cloud...")
                self.process_point_cloud_oneshot()

        except Exception as e:
            self.get_logger().error(f"Errore nel loop del bridge: {e}")

    def process_point_cloud_oneshot(self):
        res_x, res_y = self.sim.getVisionSensorResolution(self.h_sensor)
        angle = self.sim.getObjectFloatParam(self.h_sensor, self.sim.visionfloatparam_perspective_angle)
        far_clip = self.sim.getObjectFloatParam(self.h_sensor, self.sim.visionfloatparam_far_clipping)
        
        depth_data, _ = self.sim.getVisionSensorDepth(self.h_sensor, 1, [0, 0], [res_x, res_y])
        if not depth_data: 
            return
            
        depth_img = np.frombuffer(depth_data, dtype=np.float32).reshape((res_y, res_x))
        Z = np.flipud(depth_img)
        
        cx, cy = res_x / 2.0, res_y / 2.0
        fx = cx / np.tan(angle / 2.0)
        fy = cy / np.tan(angle / 2.0)
        
        u, v = np.meshgrid(np.arange(res_x), np.arange(res_y))
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        
        points = np.stack((X, Y, Z), axis=-1).reshape(-1, 3)
        valid_points = points[Z.flatten() < (far_clip - 0.01)] 
        
        if len(valid_points) < 100: 
            return
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(valid_points)
        # ---------------------------------------------------------
        # --- FIX CONVENZIONE ASSI ---
        # Il frame locale del vision sensor in CoppeliaSim ha (per
        # documentazione ufficiale): Z che punta nella direzione guardata,
        # Y verso l'alto, X verso SINISTRA. Questo è diverso dalla
        # convenzione "pinhole" usata sopra per proiettare i pixel
        # (X cresce a destra nell'immagine, Y cresce verso il basso).
        # Per convertire pinhole -> frame locale CoppeliaSim: si inverte
        # X (destra->sinistra) e Y (giù->su); Z resta invariato (avanti
        # resta avanti). Prima venivano invertiti Y e Z invece di X e Y:
        # bug sistematico che spiega gli errori di posizione ricorrenti
        # in world frame, simili in magnitudine indipendentemente da
        # quale sensore/scena venisse usato.
        # ---------------------------------------------------------
        pcd.transform([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        
        # ---------------------------------------------------------
        # --- RANSAC TAVOLO ---
        # Dai dati raw sappiamo che il tavolo è un unico piano ben definito
        # e il cubo sporge di circa 8-10cm sopra di esso: soglia stretta va
        # benissimo perché il tavolo qui è pulito e ben rappresentato.
        # ---------------------------------------------------------
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.004, ransac_n=3, num_iterations=1000)
        a, b, c, d = plane_model
        cube_cloud = pcd.select_by_index(inliers, invert=True)
        
        if len(cube_cloud.points) < 10: 
            self.get_logger().warn("Nessun punto rimasto dopo la rimozione del tavolo.")
            return

        # ---------------------------------------------------------
        # --- FILTRO PER SALTO DI PROFONDITA' ---
        # Dai dati raw: cubo ~0.17-0.19m, tavolo ~0.20-0.26m. Il cubo sta
        # tutto DALLA STESSA PARTE del piano del tavolo (dalla parte della
        # camera). Usiamo la distanza con segno dal piano per tenere solo
        # i punti chiaramente sollevati (evita eventuali residui di rumore
        # appena sopra/sotto il piano del tavolo).
        # ---------------------------------------------------------
        pts_remaining = np.asarray(cube_cloud.points)
        signed_dist = (a * pts_remaining[:, 0] + b * pts_remaining[:, 1] + c * pts_remaining[:, 2] + d)
        # Ci assicuriamo che il segno positivo indichi "sollevato verso la camera"
        # (controlliamo il segno medio; se negativo, invertiamo il criterio)
        if np.median(signed_dist) < 0:
            keep_mask = signed_dist < -0.003
        else:
            keep_mask = signed_dist > 0.003
        idx_keep = np.where(keep_mask)[0]
        cube_cloud = cube_cloud.select_by_index(idx_keep.tolist())

        self.get_logger().info(f"Punti cubo dopo filtro profondità: {len(cube_cloud.points)}")

        if len(cube_cloud.points) < 10:
            self.get_logger().warn("Troppo pochi punti dopo il filtro di profondità.")
            return

        # ---------------------------------------------------------
        # --- ISOLAMENTO DEL CUBO PER DIMENSIONE (DBSCAN) ---
        # Rete di sicurezza: se restasse rumore isolato o parti del gripper,
        # teniamo solo il cluster di dimensione compatibile con CUBE_SIZE.
        # ---------------------------------------------------------
        isolated = self.isolate_cube_cluster(cube_cloud)
        if isolated is not None and len(isolated.points) >= 10:
            cube_cloud = isolated
        else:
            self.get_logger().warn("DBSCAN non ha isolato un cluster valido: uso tutti i punti rimasti dopo il filtro profondità.")

        pts_cube_final = np.asarray(cube_cloud.points)

        # ---------------------------------------------------------
        # --- STIMA CENTRO MULTI-FACCIA ---
        # Invece di assumere che si veda sempre e solo la faccia superiore
        # (centroide + normale del tavolo), rileviamo le facce effettivamente
        # visibili nel cluster - utile quando si inquadra uno spigolo/angolo
        # del cubo invece della faccia piana dall'alto.
        # ---------------------------------------------------------
        geometric_center_local, normal = self.estimate_center_multi_face(cube_cloud)
        if geometric_center_local is None:
            self.get_logger().warn("Impossibile rilevare facce nel cluster del cubo; uso centroide semplice come fallback.")
            centroid = pts_cube_final.mean(axis=0)
            fallback_normal = np.array([a, b, c])
            fallback_normal = fallback_normal / np.linalg.norm(fallback_normal)
            if np.median(signed_dist) < 0:
                fallback_normal = -fallback_normal
            geometric_center_local = centroid - fallback_normal * (CUBE_SIZE / 2.0)
            normal = fallback_normal

        # ---------------------------------------------------------
        # --- DEBUG: SALVATAGGIO DI 4 VISTE PNG ---
        # ---------------------------------------------------------
        try:
            all_pts = np.vstack((pts_cube_final, geometric_center_local.reshape(1, 3)))
            max_range = max(np.array([all_pts[:,0].max()-all_pts[:,0].min(),
                                  all_pts[:,1].max()-all_pts[:,1].min(),
                                  all_pts[:,2].max()-all_pts[:,2].min()]).max() / 2.0, 0.01)
            mid_x = (all_pts[:,0].max()+all_pts[:,0].min()) * 0.5
            mid_y = (all_pts[:,1].max()+all_pts[:,1].min()) * 0.5
            mid_z = (all_pts[:,2].max()+all_pts[:,2].min()) * 0.5
            
            viste = [
                (30, 45, "1_isometrica"),
                (0, 0, "2_lato_X"),
                (0, 90, "3_lato_Y"),
                (90, -90, "4_alto")
            ]
            
            for elev, azim, nome_vista in viste:
                fig = plt.figure(figsize=(8, 6))
                ax = fig.add_subplot(111, projection='3d')
                ax.scatter(pts_cube_final[:, 0], pts_cube_final[:, 1], pts_cube_final[:, 2], c='green', s=8, label='Punti Cubo')
                ax.scatter(geometric_center_local[0], geometric_center_local[1], geometric_center_local[2], c='red', s=100, marker='o', label='Centro Stimato')
                ax.set_xlim(mid_x - max_range, mid_x + max_range)
                ax.set_ylim(mid_y - max_range, mid_y + max_range)
                ax.set_zlim(mid_z - max_range, mid_z + max_range)
                ax.set_title(f'Debug Cubo - {nome_vista.replace("_", " ").title()}')
                ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
                ax.view_init(elev=elev, azim=azim)
                plt.savefig(f'{OUTPUT_DIR}/debug_cubo_{nome_vista}.png')
                plt.close(fig)
            self.get_logger().info("Salvate 4 immagini di debug.")
        except Exception as e_img:
            self.get_logger().error(f"Impossibile salvare l'immagine di debug: {e_img}")

        # 2. Posa del sensore rispetto al mondo
        sensor_pose = self.sim.getObjectPose(self.h_sensor, -1)
        sensor_position = sensor_pose[:3]
        sensor_quaternion = sensor_pose[3:] 
        
        rot_matrix_sensor = R.from_quat(sensor_quaternion).as_matrix()
        geometric_center_global = np.dot(rot_matrix_sensor, geometric_center_local) + sensor_position

        # 3. Orientamento locale del cubo (asse Z = normale del tavolo, cioè faccia superiore)
        z_axis = normal
        temp_x = np.array([1.0, 0.0, 0.0]) if abs(z_axis[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        x_axis = temp_x - np.dot(temp_x, z_axis) * z_axis
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        rot_matrix_local = np.column_stack((x_axis, y_axis, z_axis))
        quat_local = R.from_matrix(rot_matrix_local).as_quat() 
        
        r_sensor = R.from_quat(sensor_quaternion)
        r_local = R.from_quat(quat_local)
        quat_global = (r_sensor * r_local).as_quat()

        msg = Pose()
        msg.position.x = float(geometric_center_global[0])
        msg.position.y = float(geometric_center_global[1])
        msg.position.z = float(geometric_center_global[2])
        msg.orientation.x = float(quat_global[0])
        msg.orientation.y = float(quat_global[1])
        msg.orientation.z = float(quat_global[2])
        msg.orientation.w = float(quat_global[3])
        
        self.pub_cube_pcl.publish(msg)
        self.detection_done = True
        self.get_logger().info(f"Posa globale inviata a ROS 2! X:{msg.position.x:.3f} Y:{msg.position.y:.3f} Z:{msg.position.z:.3f}")

    def estimate_center_multi_face(self, cloud):
        """
        Rileva iterativamente le facce visibili nel cluster del cubo (possono
        essere 1, 2 o 3 a seconda dell'angolo di vista - es. uno spigolo
        mostra 2 facce a V) e stima il centro come media di
        (centroide_faccia + normale_faccia * CUBE_SIZE/2) su tutte le facce
        trovate. La normale di ciascuna faccia viene orientata "verso la
        camera" sfruttando il fatto che, nel frame LOCALE del sensore, la
        camera stessa si trova all'origine (0,0,0): quindi la normale
        corretta è quella che punta dal centroide della faccia verso
        l'origine, non una direzione fissa come "l'alto del tavolo" (che
        funziona solo se si vede esclusivamente la faccia superiore).
        """
        remaining = cloud
        face_centroids = []
        face_normals = []

        for _ in range(3):  # al massimo 3 facce visibili su un cubo
            if len(remaining.points) < 20:
                break
            plane_model, inliers = remaining.segment_plane(distance_threshold=0.003, ransac_n=3, num_iterations=500)
            if len(inliers) < 15:
                break
            face_cloud = remaining.select_by_index(inliers)
            centroid = np.asarray(face_cloud.points).mean(axis=0)
            normal = np.array(plane_model[:3])
            normal = normal / np.linalg.norm(normal)

            # Orienta la normale verso la camera (origine nel frame locale)
            direction_to_cam = -centroid
            if np.dot(normal, direction_to_cam) < 0:
                normal = -normal

            face_centroids.append(centroid)
            face_normals.append(normal)

            remaining = remaining.select_by_index(inliers, invert=True)
            if len(remaining.points) < 15:
                break

        if not face_centroids:
            return None, None

        self.get_logger().info(f"Facce rilevate per la stima del centro: {len(face_centroids)}")

        center_estimate = np.zeros(3)
        for c, n in zip(face_centroids, face_normals):
            # n punta verso la camera (fuori dal cubo): per arrivare al
            # centro del cubo bisogna andare nella direzione OPPOSTA,
            # cioè sottrarre n, non sommarla.
            center_estimate += c - n * (CUBE_SIZE / 2.0)
        center_estimate /= len(face_centroids)

        # Normale "rappresentativa" per l'orientamento pubblicato: quella
        # della faccia con più punti (probabilmente la più frontale/grande)
        representative_normal = face_normals[0]

        return center_estimate, representative_normal

    def isolate_cube_cluster(self, cloud):
        pts = np.asarray(cloud.points)
        if len(pts) < 10:
            return None
        labels = np.array(cloud.cluster_dbscan(eps=CUBE_SIZE * 1.2, min_points=6, print_progress=False))
        if labels.max() < 0:
            return None
        best_idx, best_score = None, None
        for lbl in range(labels.max() + 1):
            idx = np.where(labels == lbl)[0]
            cluster_pts = pts[idx]
            extent = cluster_pts.max(axis=0) - cluster_pts.min(axis=0)
            max_extent = extent.max()
            self.get_logger().info(f"Cluster {lbl}: {len(idx)} punti, estensione max {max_extent*100:.2f} cm")
            if max_extent > CUBE_SIZE * 2.5:
                continue
            score = abs(max_extent - CUBE_SIZE)
            if best_score is None or score < best_score:
                best_score, best_idx = score, idx
        if best_idx is None:
            return None
        return cloud.select_by_index(best_idx.tolist())


def main(args=None):
    rclpy.init(args=args)
    node = CompletePerceptionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()