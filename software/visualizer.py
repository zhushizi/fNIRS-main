"""
visualizer.py
==================
该脚本使用 Flask 与 Plotly 对 fNIRS 数据进行实时可视化。
它会连接上游服务器获取数据，并通过 Web 界面提供交互。
同时支持演示模式，以便在无真实硬件数据时进行测试。
脚本还包含 3D 脑网格创建、激活数据处理与可视化更新等功能。
"""

import sys
import signal
import time
import os
import logging
import subprocess
import threading
from queue import Queue

import serial
from flask import Flask, jsonify, send_from_directory, request
from flask_socketio import SocketIO
import plotly.graph_objs as go
from plotly.offline import plot
import numpy as np
import nibabel as nib
from scipy.spatial import cKDTree
import socketio as sio_client_lib
import pandas as pd

import config

# 检查命令行参数中是否包含 demo 标志
demo_mode = any(arg.lower() == 'demo' for arg in sys.argv[1:])
if demo_mode:
    logging.info("Demo mode is active.")
else:
    logging.info("Demo mode is not active.")
    ser = serial.Serial(
        config.SERIAL_PORT,
        baudrate=config.BAUD_RATE,
        timeout=config.TIMEOUT
    )

# 配置日志
logging.basicConfig(level=logging.DEBUG)

# -----------------------------------------------------
# Flask 与 Socket.IO 初始化
# -----------------------------------------------------
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# -----------------------------------------------------
# 全局变量与数据队列（用于处理后的数据包）
# -----------------------------------------------------
data_queue = Queue(maxsize=20)
data_lock = threading.Lock()  # 用于线程同步的锁
reader_thread = None

# -----------------------------------------------------
# 上游 Socket.IO 客户端配置（接收 processed_data）
# -----------------------------------------------------
sio_client = sio_client_lib.Client()

@sio_client.event
def connect():
    """
    当客户端连接到上游服务器时调用。
    """
    logging.info("Connected to upstream server for data.")

@sio_client.event
def disconnect():
    """
    当客户端与上游服务器断开连接时调用。
    """
    logging.info("Disconnected from upstream server.")

@sio_client.event
def processed_data(data):
    """收到新的处理后数据包时调用。"""
    activation_data = np.array(data['concentrations'])
    logging.info(f"Received new data: {activation_data}")
    if activation_data.ndim == 1:
        activation_data = activation_data.reshape(-1, 1)
    with data_lock:
        if data_queue.full():
            data_queue.get()  # 队列满时移除最旧数据
        data_queue.put(activation_data)

    # 如果当前处于 mBLL 模式，立即更新图形
    if current_mode == 'mBLL':
        update_graphs(activation_data)

def get_most_recent_packet():
    """
    获取队列中最新的数据包。
    """
    with data_lock:
        if not data_queue.empty():
            return list(data_queue.queue)[-1]
    return None

def signal_handler():
    """
    优雅退出处理器。
    """
    print("Exiting gracefully...")
    if sio_client.connected:
        sio_client.disconnect()
    app.quit()

# 注册 SIGINT 信号处理器
signal.signal(signal.SIGINT, signal_handler)

# -----------------------------------------------------
# fNIRS 数据处理与脑网格相关函数
# -----------------------------------------------------
def compute_vertex_normals(vertices, triangles):
    """计算网格中每个顶点的近似法向量。"""
    normals = np.zeros(vertices.shape, dtype=float)
    for tri in triangles:
        v0, v1, v2 = vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]
        n = np.cross(v1 - v0, v2 - v0)
        norm = np.linalg.norm(n)
        if norm:
            n = n / norm
        normals[tri[0]] += n
        normals[tri[1]] += n
        normals[tri[2]] += n
    norms = np.linalg.norm(normals, axis=1)[:, None]
    normals = normals / (norms + 1e-8)
    return normals

def create_flat_cylinder_mesh(center, normal, radius, height=1.0, resolution=20, angle=0.0):
    """
    创建一个扁平圆柱（类似瓶盖）的顶点与面片。
    先在原点按 z 轴方向生成，再旋转到指定法向并平移到 center。

    `angle` 表示在与法向对齐前，圆周先额外旋转的角度（弧度）。
    """
    theta = np.linspace(0, 2*np.pi, resolution, endpoint=False) + angle
    circle_bottom = np.column_stack((radius * np.cos(theta),
                                     radius * np.sin(theta),
                                     np.zeros(resolution)))
    circle_top = np.column_stack((radius * np.cos(theta),
                                  radius * np.sin(theta),
                                  np.full(resolution, height)))
    vertices = np.vstack((circle_bottom, circle_top))

    faces = []
    for i in range(resolution):
        next_i = (i + 1) % resolution
        faces.append([i, next_i, i + resolution])
        faces.append([next_i, next_i + resolution, i + resolution])

    bottom_center_index = len(vertices)
    vertices = np.vstack((vertices, np.array([0, 0, 0])))
    for i in range(resolution):
        next_i = (i+1) % resolution
        faces.append([bottom_center_index, next_i, i])

    top_center_index = len(vertices)
    vertices = np.vstack((vertices, np.array([0, 0, height])))
    for i in range(resolution):
        next_i = (i+1) % resolution
        faces.append([top_center_index, i+resolution, next_i+resolution])

    def rotation_matrix_from_vectors(a, b):
        a = a / np.linalg.norm(a)
        b = b / np.linalg.norm(b)
        v = np.cross(a, b)
        c = np.dot(a, b)
        if np.linalg.norm(v) < 1e-8:
            return np.eye(3)
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]],
                         [v[2], 0, -v[0]],
                         [-v[1], v[0], 0]])
        R = np.eye(3) + kmat + np.dot(kmat, kmat) * ((1 - c) / (s**2))
        return R

    R = rotation_matrix_from_vectors(np.array([0, 0, 1]), normal)
    vertices_rotated = np.dot(vertices, R.T)
    vertices_translated = vertices_rotated + center
    return vertices_translated, faces


def preload_static_data():
    """
    从文件加载脑网格静态数据。
    """
    coords = np.loadtxt('BrainMesh_Ch2_smoothed.nv', skiprows=1, max_rows=53469)
    x, y, z = coords.T
    triangles = np.loadtxt('BrainMesh_Ch2_smoothed.nv', skiprows=53471, dtype=int)
    triangles_zero_offset = triangles - 1
    i, j, k = triangles_zero_offset.T
    aal_img = nib.load('aal.nii')
    aal_data = aal_img.get_fdata()
    affine = aal_img.affine
    return coords, x, y, z, i, j, k, aal_data, affine

def initialize_sensor_positions(coords):
    """
    定义自定义传感器位置，并为每个节点设置旋转角。
    返回发射器与探测器的独立数组。
    """
    # 发射器位置（8 个节点）
    emitter_positions = np.array([
        [-55.0, 21.0, 25.0],
        [-38.0, -47.0, 60.0],
        [16.0, -15.0, 78.0],
        [19.0, -100.0, -4.0],
        [-11.0, -75.0, 57.0],
        [-6.0, 59.0, 36.0],
        [52.0, -51.0, 51.0],
        [55.0, 21.0, 25.0],
    ])
    emitter_angles = np.array([0, 0, 0, 0, 0, 0, 0, 0])

    # 探测器位置（16 个节点）
    detector_positions = np.array([
        [-38.0, 54.0, 18.0],    # 第 1 组
        [-65.0, -29.0, 33.0],

        [-43.0, -5.0, 58.0],    # 第 2 组
        [-51.0, -69.0, 29.0],

        [15.0, 28.0, 60.0],     # 第 3 组
        [-15.0, 28.0, 60.0],

        [20.0, -95.0, 30.0],    # 第 4 组
        [-20.0, -95.0, 30.0],

        [14.0, -43.0, 80.0],    # 第 5 组
        [-15.0, -41.0, 78.0],

        [15.0, 70.0, -3.0],     # 第 6 组
        [-15.0, 70.0, -3.0],

        [50.0, -74.0, 21.0],    # 第 7 组
        [66.0, -22.0, 38.0],

        [41.0, -13.0, 66.0],    # 第 8 组
        [43.0, 50.0, 22.0],
    ])
    detector_angles = np.zeros(len(detector_positions))

    return emitter_positions, emitter_angles, detector_positions, detector_angles

def map_points_to_regions(points, affine, aal_data):
    """
    将传感器位置映射到脑区编号。
    """
    voxel_coords = np.round(np.linalg.inv(affine) @
                            np.column_stack((points, np.ones(points.shape[0]))).T).T[:, :3]
    voxel_coords = voxel_coords.astype(int)
    regions = []
    for voxel in voxel_coords:
        if (0 <= voxel[0] < aal_data.shape[0] and
            0 <= voxel[1] < aal_data.shape[1] and
            0 <= voxel[2] < aal_data.shape[2]):
            regions.append(aal_data[tuple(voxel)])
        else:
            regions.append(-1)
    return np.array(regions)

def filter_coordinates_to_surface(coords, surface_coords, threshold=2.0):
    """
    筛选靠近脑表面的坐标点。
    """
    tree = cKDTree(surface_coords)
    distances, _ = tree.query(coords)
    return coords[distances <= threshold]

def create_static_brain_mesh():
    """
    创建包含传感器节点的静态 3D 脑网格。
    发射器绘制为扁平圆柱帽（默认白色），探测器绘制为黑色标记。
    """
    fig = go.Figure()

    # 添加脑网格
    fig.add_trace(go.Mesh3d(
        x=x, y=y, z=z,
        i=i, j=j, k=k,
        color='lightpink',
        opacity=0.5,
        name='Brain Mesh',
        showscale=False
    ))

    # 构建 KD-tree 并计算顶点法向量
    vertices = np.column_stack((x, y, z))
    triangles = np.column_stack((i, j, k))
    vertex_normals = compute_vertex_normals(vertices, triangles)
    tree = cKDTree(vertices)

    # 绘制发射器
    _, emitter_indices = tree.query(emitter_positions)
    emitter_normals = vertex_normals[emitter_indices]
    for pos, angle, norm in zip(emitter_positions, emitter_angles, emitter_normals):
        vertices_cap, faces_cap = create_flat_cylinder_mesh(pos,
                                                            norm,
                                                            radius=10,
                                                            height=2,
                                                            resolution=20,
                                                            angle=angle)
        faces_cap = np.array(faces_cap)
        i_cap, j_cap, k_cap = faces_cap[:, 0], faces_cap[:, 1], faces_cap[:, 2]
        fig.add_trace(go.Mesh3d(
            x=vertices_cap[:, 0],
            y=vertices_cap[:, 1],
            z=vertices_cap[:, 2],
            i=i_cap, j=j_cap, k=k_cap,
            color='white',  # 发射器默认颜色
            opacity=1,
            name='Emitter'
        ))

    # 绘制探测器
    _, detector_indices = tree.query(detector_positions)
    detector_normals = vertex_normals[detector_indices]
    for pos, angle, norm in zip(detector_positions, detector_angles, detector_normals):
        vertices_cap, faces_cap = create_flat_cylinder_mesh(pos,
                                                            norm,
                                                            radius=10,
                                                            height=2,
                                                            resolution=20,
                                                            angle=angle)
        faces_cap = np.array(faces_cap)
        i_cap, j_cap, k_cap = faces_cap[:, 0], faces_cap[:, 1], faces_cap[:, 2]
        fig.add_trace(go.Mesh3d(
            x=vertices_cap[:, 0],
            y=vertices_cap[:, 1],
            z=vertices_cap[:, 2],
            i=i_cap, j=j_cap, k=k_cap,
            color='black',
            opacity=1,
            name='Detector'
        ))

    fig.update_layout(
        # title="3D Brain Mesh with Sensor Nodes",
        scene=dict(xaxis_visible=False, yaxis_visible=False, zaxis_visible=False),
        width=800,
        height=800,
        showlegend=True
    )
    return fig


# -----------------------------------------------------
# 全局变量
# -----------------------------------------------------

# 预加载静态数据
coords, x, y, z, i, j, k, aal_data, affine = preload_static_data()
emitter_positions, emitter_angles, detector_positions,detector_angles = initialize_sensor_positions(coords)
combined_positions = np.vstack((emitter_positions, detector_positions))
regions = map_points_to_regions(combined_positions, affine, aal_data)
brain_mesh_fig = create_static_brain_mesh()
emitter_states = [True] * len(emitter_positions)

# 用于存储累积激活数据的全局变量
activation_history = None
processing_proc = None
running_processes = []
current_mode = None
current_sources = []

# 控制数据定义（发射器与 MUX 控制状态）
control_data = {
    'emitter_control_override_enable': 0,
    'emitter_control_state': 0,
    'emitter_pwm_control_h': 0,
    'emitter_pwm_control_l': 0,
    'mux_control_override_enable': 0,
    'mux_control_state': 0
}

# 传感器分组定义
sensor_groups = [
    {"group_id": 1, "emitter_index": 0, "detector_indices": [0, 1]},
    {"group_id": 2, "emitter_index": 1, "detector_indices": [2, 3]},
    {"group_id": 3, "emitter_index": 2, "detector_indices": [4, 5]},
    {"group_id": 4, "emitter_index": 3, "detector_indices": [6, 7]},
    {"group_id": 5, "emitter_index": 4, "detector_indices": [8, 9]},
    {"group_id": 6, "emitter_index": 5, "detector_indices": [10, 11]},
    {"group_id": 7, "emitter_index": 6, "detector_indices": [12, 13]},
    {"group_id": 8, "emitter_index": 7, "detector_indices": [14, 15]},
]

# 传感器映射：对应每个 HbO 传感器（0-23）
sensor_mapping = [
    0, 8, 9,    # 第1组: emitter0, detector0, detector1
    1, 10, 11,  # 第2组: emitter1, detector2, detector3
    2, 12, 13,  # 第3组: emitter2, detector4, detector5
    3, 14, 15,  # 第4组: emitter3, detector6, detector7
    4, 16, 17,  # 第5组: emitter4, detector8, detector9
    5, 18, 19,  # 第6组: emitter5, detector10, detector11
    6, 20, 21,  # 第7组: emitter6, detector12, detector13
    7, 22, 23   # 第8组: emitter7, detector14, detector15
]


# -----------------------------------------------------
# 脑区映射预计算
# -----------------------------------------------------

# 预计算传感器到脑区的映射（长度 24）
sensor_region = [regions[sensor_mapping[i]] for i in range(len(sensor_mapping))]

# 为感兴趣脑区预计算“脑区到传感器索引”的映射
region_to_sensor_indices = {}
for i, reg in enumerate(sensor_region):
    if reg > 0:
        region_to_sensor_indices.setdefault(reg, []).append(i)

# 为每个脑区预计算筛选后的坐标（固定阈值，例如 2.0）
surface_coords = np.column_stack((x, y, z))
region_filtered = {}
# 从传感器映射中提取唯一脑区 ID（仅保留 > 0）
unique_sensor_regions = np.unique([r for r in sensor_region if r > 0])
for reg in unique_sensor_regions:
    region_mask = aal_data == reg
    region_voxels = np.argwhere(region_mask)
    region_world_coords = nib.affines.apply_affine(affine, region_voxels)
    filtered_coords = filter_coordinates_to_surface(region_world_coords,
                                                    surface_coords,
                                                    threshold=2.0)
    region_filtered[reg] = filtered_coords


# -----------------------------------------------------
# 脑网格更新辅助函数
# -----------------------------------------------------
def update_highlighted_regions(fig, hbo_values):
    """
    根据激活数据更新脑网格中的高亮脑区。
    使用预计算的传感器-脑区映射、脑区-传感器索引，
    以及预筛选的脑区坐标。
    """
    # 收集被激活传感器对应的脑区 ID（hbo 值 < 0）
    highlighted_region_ids = []
    for i, value in enumerate(hbo_values):
        if value < 0 and sensor_region[i] > 0:
            highlighted_region_ids.append(sensor_region[i])
    highlighted_region_ids = np.unique(highlighted_region_ids)

    highlighted_coords = []
    highlighted_values = []
    for reg in highlighted_region_ids:
        filtered_coords = region_filtered.get(reg)
        if filtered_coords is not None and filtered_coords.size > 0:
            # 计算映射到该脑区的传感器平均值
            indices = region_to_sensor_indices.get(reg, [])
            if indices:
                avg_val = np.mean([hbo_values[i] for i in indices])
            else:
                avg_val = 0
            highlighted_coords.append(filtered_coords)
            highlighted_values.extend([avg_val] * len(filtered_coords))

    if highlighted_coords:
        highlighted_coords = np.vstack(highlighted_coords)
        highlighted_values = np.array(highlighted_values)
        # 移除旧高亮轨迹并添加新轨迹
        fig.data = [trace for trace in fig.data if trace.name != 'Highlighted Regions']
        fig.add_trace(go.Scatter3d(
            x=highlighted_coords[:, 0],
            y=highlighted_coords[:, 1],
            z=highlighted_coords[:, 2],
            mode='markers',
            marker=dict(
                size=2,
                color='red',
                opacity=0.1
            ),
            name='Highlighted Regions'
        ))
    return fig


def highlight_sensor_group(fig, group_id):
    """
    通过附加轨迹高亮指定传感器组。
    先移除已有组高亮，再高亮当前所选组。
    """
    # 移除之前的组高亮
    fig.data = [trace for trace in fig.data if trace.name != "Group Highlight"]

    group = next((g for g in sensor_groups if g["group_id"] == group_id), None)
    if group is None:
        return fig

    emitter_idx = group["emitter_index"]
    detector_indices = group["detector_indices"]

    emitter_coord = emitter_positions[emitter_idx]
    detector_coords = detector_positions[detector_indices]

    # 高亮发射器
    fig.add_trace(go.Scatter3d(
        x=[emitter_coord[0]],
        y=[emitter_coord[1]],
        z=[emitter_coord[2]],
        mode='markers',
        marker=dict(size=14, color='yellow', symbol='circle'),
        showlegend=False,
        name="Group Highlight"
    ))

    # 高亮探测器
    fig.add_trace(go.Scatter3d(
        x=detector_coords[:, 0],
        y=detector_coords[:, 1],
        z=detector_coords[:, 2],
        mode='markers',
        marker=dict(size=12, color='yellow', symbol='circle'),
        showlegend=False,
        name="Group Highlight"
    ))

    # 绘制发射器到探测器的连线
    for det in detector_coords:
        fig.add_trace(go.Scatter3d(
            x=[emitter_coord[0], det[0]],
            y=[emitter_coord[1], det[1]],
            z=[emitter_coord[2], det[2]],
            mode='lines',
            line=dict(color='yellow', width=4),
            showlegend=False,
            name="Group Highlight"
        ))
    return fig


def update_graphs(latest_packet):
    """
    使用最新数据包更新脑网格。
    当收到上游服务器新数据时调用。
    """
    global brain_mesh_fig
    if latest_packet is None:
        return

    # 处理输入数据
    activation_data = np.array(latest_packet)
    if activation_data.ndim == 1:
        activation_data = activation_data.reshape(-1, 1)
    hbo_values = activation_data[::2]  # 当前为 24 个值的数组

    brain_mesh_fig = update_highlighted_regions(brain_mesh_fig, hbo_values)
    socketio.emit('brain_mesh_update', {
        'brain_mesh': brain_mesh_fig.to_json()
    })


def stop_serial_reader():
    """
    平滑停止串口读取线程。
    """
    global ser
    try:
        ser.close()
        logging.info("Serial connection closed for record mode.")
    except Exception as e:
        logging.error(f"Error closing serial port: {e}")

def reinit_serial_connection():
    """
    重新初始化串口连接。
    """
    global ser
    try:
        ser = serial.Serial(
            config.SERIAL_PORT,
            baudrate=config.BAUD_RATE,
            timeout=config.TIMEOUT
        )
        logging.info("Serial connection reinitialized.")
    except Exception as e:
        logging.error(f"Error reinitializing serial port: {e}")


# -------------------- Flask Routes --------------------

@app.route('/')
def index():
    """
    提供主页面 HTML。
    页面包含 Plotly 图表及其他 UI 元素。
    """
    return send_from_directory('.', 'index.html')

@app.route('/update_graphs')
def update_graphs_route():
    """
    使用最新数据包更新脑网格。
    当收到上游服务器新数据时调用。
    """
    return jsonify({
        'brain_mesh': brain_mesh_fig.to_json(),
    })

@app.route('/select_group/<int:group_id>')
def select_group(group_id):
    """
    在脑网格上高亮指定传感器组。
    """
    global brain_mesh_fig
    brain_mesh_fig = highlight_sensor_group(brain_mesh_fig, group_id)
    return jsonify({'brain_mesh': brain_mesh_fig.to_json()})

@app.route('/update_emitter_states', methods=['POST'])
def update_emitter_states():
    """
    根据收到的 JSON 数据更新发射器状态。
    """
    global emitter_states
    data = request.json
    emitter_states = data['emitter_states']
    return jsonify({'status': 'success'})

@app.route('/update_control_data', methods=['POST'])
def update_control_data():
    """
    更新发射器与 MUX 的控制数据。
    该函数接收包含控制信息的 JSON，并更新全局 control_data 字典。
    若非 demo 模式，还会将更新后的字节写入串口。
    """
    global control_data
    data = request.json
    control_data.update(data)
    print(f"Control data updated: {control_data}")
    values_list = list(control_data.values())
    data_bytes = bytes(values_list)
    if not demo_mode:
        ser.write(data_bytes)
    return jsonify({'status': 'success'})

@app.route('/start_processing', methods=['POST'])
def start_processing():
    """
    根据所选模式启动处理流程。
    """
    global processing_proc, current_mode, current_sources
    data = request.get_json()
    mode = data.get('mode')      # "live" 或 "record"
    current_sources = data.get('sources', [])

    # 实时读取模式仅支持 ADC
    if mode == 'live':
        current_mode = 'adc_live'
        # demo 模式下使用模拟服务器
        if demo_mode:
            proc1 = subprocess.Popen(['python', 'adc_mock_server.py'])
            time.sleep(1)  # 等待服务器初始化
            proc2 = subprocess.Popen(['python', 'adc_client.py'])
            running_processes.extend([proc1, proc2])
        else:
            stop_serial_reader()
            proc = subprocess.Popen(['python', 'adc_live.py'])
            running_processes.append(proc)
        return jsonify({'status': 'ADC mode started'})

    # 录制与可视化模式支持 ADC 与 mBLL
    if mode == 'record':
        current_mode = 'record'
        # demo 模式下跳过 fNIRS_processing.py 启动
        if demo_mode:
            return jsonify({'status': 'demo mode active, processing skipped'})
        try:
            # 停止 visualizer 自身的串口读取
            stop_serial_reader()
            # 启动 fNIRS_processing.py（该进程会自行建立串口连接）
            proc = subprocess.Popen(['python', 'fNIRS_processing.py'])
            running_processes.extend([proc])
            return jsonify({'status': 'processing started'})
        except Exception as e:
            return jsonify({'status': 'error', 'message': str(e)}), 500
    else:
        return jsonify({'status': 'error', 'message': 'Invalid mode selected.'}), 400


@app.route('/stop_processing', methods=['POST'])
def stop_processing():
    """
    停止处理流程并重新初始化串口连接。
    该函数向子进程发送 SIGUSR1，以便其平滑退出。
    """
    global running_processes
    # 向每个子进程发送 SIGUSR1，使其平滑停止
    for proc in running_processes:
        os.kill(proc.pid, signal.SIGUSR1)
    running_processes = []
    # 短暂等待，确保 fNIRS_processing.py 已关闭串口连接
    time.sleep(1)
    # 重新初始化串口连接并恢复读取
    try:
        if not ser.is_open:
            reinit_serial_connection()
        else:
            logging.info("Serial connection is already open; no need to reinitialize.")
    except Exception as e:
        logging.error(f"Error checking serial connection: {e}")
    return jsonify({'status': 'processing stop signal sent and serial reinitialized'})


@app.route('/download/<source>')
def download_file(source):
    """
    根据 source 参数下载对应 CSV 文件。
    source 可为 'ADC' 或 'mBLL'。
    """
    # 将 source 映射到对应 CSV 文件名（可按需调整）
    csv_map = {
        'ADC': 'all_groups.csv',
        'mBLL': 'processed_output.csv'
    }
    filename = csv_map.get(source)
    if not filename:
        return jsonify({'status': 'error', 'message': 'Invalid source.'}), 400
    # 获取用户指定下载名；若未提供则使用默认文件名
    download_name = request.args.get('filename', filename)
    data_dir = os.path.dirname(os.path.abspath(__file__))

    return send_from_directory(data_dir, filename, as_attachment=True, download_name=download_name)


@app.route('/view_static/ADC')
def view_static_adc_plotly():
    """
    生成 ADC 静态可视化 HTML。
    为每个分组创建 Plotly 图，并拼接后返回完整 HTML。
    """
    # 读取 CSV 数据
    data_dir = 'sample_data' if demo_mode else '.'
    csv_path = os.path.join(data_dir, 'all_groups.csv')
    df = pd.read_csv(csv_path)

    # 为每个分组分别构建图表
    figures_html = ""
    for i in range(8):
        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x = df["Time (s)"],
            y = df[f"G{i}_Short"],
            mode = 'lines',
            name = 'Short'
        ))
        fig.add_trace(go.Scatter(
            x = df["Time (s)"],
            y = df[f"G{i}_Long1"],
            mode = 'lines',
            name = 'Long1'
        ))
        fig.add_trace(go.Scatter(
            x = df["Time (s)"],
            y = df[f"G{i}_Long2"],
            mode = 'lines',
            name = 'Long2'
        ))

        fig.update_layout(
            title = f"Group {i+1}",
            xaxis_title = "Time (s)",
            yaxis_title = "Value",
            autosize=True,
            margin=dict(l=50, r=50, t=50, b=50)
        )

        # 生成当前图的 HTML div（不重复包含 Plotly.js）
        div = plot(fig,
                   output_type='div',
                   include_plotlyjs=False,
                   config={'displayModeBar': True,
                           'responsive': True})
        figures_html += f"<div style='margin-bottom:50px;'>{div}</div>"

    # 在 head 中只引入一次 Plotly.js
    html = f"""
    <html>
      <head>
         <title>ADC Static Plots</title>
         <meta charset="UTF-8">
         <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
         <style>
           body {{
             margin: 0;
             padding: 20px;
             background-color: #eef2f7;
             font-family: 'Helvetica Neue', Helvetica, Arial, sans-serif;
           }}
         </style>
      </head>
      <body>
         {figures_html}
      </body>
    </html>
    """
    return html

@app.route('/view_static/mBLL')
def view_static_mbll_plotly():
    """
    从 CSV 加载并可视化 mBLL 数据。
    该函数生成包含 Plotly 图表的静态 HTML 页面。
    """
    # 读取 CSV 数据
    data_dir = 'sample_data' if demo_mode else '.'
    csv_path = os.path.join(data_dir, 'processed_output.csv')
    df = pd.read_csv(csv_path)

    # 为每个分组分别构建图表
    figures_html = ""
    for i in range(1, 9):
        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x = df["Time"],
            y = df[f"S{i}_D1_hbo"],
            mode = 'lines',
            name = 'D1_hbo'
        ))
        fig.add_trace(go.Scatter(
            x = df["Time"],
            y = df[f"S{i}_D1_hbr"],
            mode = 'lines',
            name = 'D1_hbr'
        ))
        fig.add_trace(go.Scatter(
            x = df["Time"],
            y = df[f"S{i}_D2_hbo"],
            mode = 'lines',
            name = 'D2_hbo'
        ))
        fig.add_trace(go.Scatter(
            x = df["Time"],
            y = df[f"S{i}_D2_hbr"],
            mode = 'lines',
            name = 'D2_hbr'
        ))
        fig.add_trace(go.Scatter(
            x = df["Time"],
            y = df[f"S{i}_D3_hbo"],
            mode = 'lines',
            name = 'D3_hbo'
        ))
        fig.add_trace(go.Scatter(
            x = df["Time"],
            y = df[f"S{i}_D3_hbr"],
            mode = 'lines',
            name = 'D3_hbr'
        ))

        fig.update_layout(
            title = f"Group {i}",
            xaxis_title = "Time (s)",
            # yaxis_title = "Value",
            autosize=True,
            margin=dict(l=50, r=50, t=50, b=50)
        )

        # 生成当前图的 HTML div（不重复包含 Plotly.js）
        div = plot(fig,
                   output_type='div',
                   include_plotlyjs=False,
                   config={'displayModeBar': True, 'responsive': True})
        figures_html += f"<div style='margin-bottom:50px;'>{div}</div>"

    # 在 head 中只引入一次 Plotly.js
    html = f"""
    <html>
      <head>
         <title>ADC Static Plots</title>
         <meta charset="UTF-8">
         <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
         <style>
           body {{
             margin: 0;
             padding: 20px;
             background-color: #eef2f7;
             font-family: 'Helvetica Neue', Helvetica, Arial, sans-serif;
           }}
         </style>
      </head>
      <body>
         {figures_html}
      </body>
    </html>
    """
    return html


@app.route('/view_animation/ADC')
def view_animation_adc():
    """
    在独立进程中启动 ADC 动画脚本。
    """
    args = [sys.executable, 'adc_animation.py']
    if demo_mode:
        args.append('demo')
    subprocess.Popen(args)
    return ('', 204)

@app.route('/view_animation/mBLL')
def view_animation_mbll():
    """
    在独立进程中启动 mBLL 动画脚本。
    """
    subprocess.Popen([sys.executable, 'mBLL_animation.py'])
    args = [sys.executable, 'mBLL_animation.py']
    if demo_mode:
        args.append('demo')
    subprocess.Popen(args)
    return ('', 204)

# -----------------------------------------------------
# 在后台线程中启动上游客户端
# -----------------------------------------------------
def run_socketio_client():
    """
    在独立线程中运行 Socket.IO 客户端。
    """
    connected = False
    while not connected:
        try:
            logging.info("Attempting to connect to server at http://127.0.0.1:5000")
            sio_client.connect('http://127.0.0.1:5000', transports=['websocket'])
            connected = True
            sio_client.wait()  # 保持客户端持续运行
        except Exception as e:
            logging.error(f"Connection failed: {e}. Retrying in 1 second...")
            time.sleep(1)

# -----------------------------------------------------
# 主入口：启动 Flask/Socket.IO 服务与客户端线程
# -----------------------------------------------------
if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    # 在 8050 端口运行 Flask-SocketIO 服务
    socketio.run(app, debug=True, use_reloader=False, port=8050)
