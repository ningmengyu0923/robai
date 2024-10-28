import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State
import plotly.graph_objs as go
from plotly.subplots import make_subplots
import json
import asyncio
import websockets
from multiprocessing import Process, Manager
from collections import deque

# 初始化 Dash 应用
app = dash.Dash(__name__)

# WebSocket 服务器地址
WEBSOCKET_URI = "ws://192.168.10.134:8765"  # 替换为实际的 WebSocket 服务器地址

# 从 WebSocket 获取数据
async def fetch_data(queue):
    async with websockets.connect(WEBSOCKET_URI) as websocket:
        await websocket.send("Client connected")
        k = 0
        while True:
            data = await websocket.recv()
            k += 1
            if k % 100 == 0:
                data = json.loads(data)
                queue.append(data)  # 将数据放入共享的列表

def websocket_process(shared_data):
    asyncio.run(fetch_data(shared_data))

# Dash 布局
app.layout = html.Div([
    dcc.Dropdown(
        id='joint-selection-dropdown',
        options=[{'label': f'Joint {i + 1}', 'value': i} for i in range(8)],
        value=0,  # 默认选择第一个关节
        multi=False,
        style={'width': '50%', 'margin': '20px'}
    ),
    dcc.Graph(id='live-update-graph'),
    dcc.Interval(id='interval-component', interval=100, n_intervals=0),  # 每秒更新一次
])

# 更新图表的回调
@app.callback(
    Output('live-update-graph', 'figure'),
    Input('interval-component', 'n_intervals'),
    State('joint-selection-dropdown', 'value')
)
def update_graph(n, selected_joint):
    # 读取共享的数据
    new_data = {
        'timestamps': [],
        'cur_pos': [],
        'cur_vel': [],
        'cur_trq': [],
        'cur_following_err': [],
        'servo_err': [],
        'servo_temperature': [],
        'pos_cmd': [],
        'vel_cmd': []
    }

    if len(shared_data) > 0:
        for _ in range(len(shared_data)):  # 批量读取数据
            latest_data = shared_data.pop(0)  # 使用 pop(0) 从队列头部读取数据
            timestamp = latest_data['Timestamp']

            # 更新存储数据
            new_data['timestamps'].append(timestamp)

            # 更新 y 数据
            for metric in ['cur_pos', 'cur_vel', 'cur_trq', 'cur_following_err',
                           'servo_err', 'servo_temperature', 'pos_cmd', 'vel_cmd']:
                new_data[metric].append(latest_data[metric][selected_joint])

    # 创建图表
    fig = make_subplots(rows=4, cols=2,
                        subplot_titles=("Current Position", "Current Velocity", "Current Torque",
                                        "Following Error", "Servo Error", "Servo Temperature",
                                        "Position Command", "Velocity Command"))

    metrics = ['cur_pos', 'cur_vel', 'cur_trq', 'cur_following_err',
               'servo_err', 'servo_temperature', 'pos_cmd', 'vel_cmd']

    for i, metric in enumerate(metrics):
        y_data = new_data[metric]  # 获取最新数据
        row = i // 2 + 1
        col = i % 2 + 1
        fig.add_trace(go.Scatter(x=new_data['timestamps'], y=y_data, mode='lines+markers',
                                 name=f'{metric} - Joint {selected_joint + 1}',
                                 marker=dict(size=3), line=dict(width=1)), row=row, col=col)

    fig.update_layout(title_text='Live Data from WebSocket', height=800)
    return fig

# 启动进程
if __name__ == '__main__':
    from multiprocessing import freeze_support

    freeze_support()

    # 使用 Manager 创建一个共享的列表
    with Manager() as manager:
        shared_data = manager.list()  # 使用 Manager 的 list
        websocket_proc = Process(target=websocket_process, args=(shared_data,))
        websocket_proc.start()

        try:
            # 运行 Dash 服务器
            app.run_server(host='0.0.0.0', port=8053, debug=False)
        except KeyboardInterrupt:
            print("Shutting down...")
        finally:
            websocket_proc.terminate()  # 确保在程序结束时关闭 WebSocket 进程
            websocket_proc.join()
