import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State
import plotly.graph_objs as go
from plotly.subplots import make_subplots
import redis
import json


# 初始化 Dash 应用
app = dash.Dash(__name__)

# Redis 连接设置
redis_client = redis.Redis(host='192.168.10.45', port=6379, db=0)

# 从 Redis 获取关节数量
def get_joint_count():
    latest_data = redis_client.zrange('sensor_data_sorted', -1, -1)  # 获取最新一条数据
    if latest_data:
        data = json.loads(latest_data[0].decode('utf-8'))
        return len(data['cur_pos'])  # 假设 'cur_pos' 代表关节数量
    return 1

# Dash 布局
app.layout = html.Div([
    dcc.Dropdown(
        id='data-selection-dropdown',
        multi=False,
        style={'width': '50%', 'margin': '20px'}
    ),
    dcc.Graph(id='live-update-graph'),
    dcc.Interval(id='interval-component', interval=200, n_intervals=0),  # 每秒更新一次
    html.Button("Pause", id='pause-button', n_clicks=0,
                style={'position': 'absolute', 'top': '10px', 'right': '10px', 'backgroundColor': 'blue',
                       'color': 'white'}),
    dcc.Store(id='pause-state', data=False),  # 存储暂停状态
    dcc.Store(id='joint-count', data=1),  # 存储关节数量
    dcc.Store(id='latest-data', data=[])  # 存储最新数据
])

# 从 Redis 获取数据
def fetch_latest_data():
    """从 Redis 有序集合获取最新的 3000 条数据，按 Timestamp 顺序"""
    latest_data = redis_client.zrange('sensor_data_sorted', -1000, -1)  # 获取最新的 3000 条数据
    latest_data = [json.loads(data.decode('utf-8')) for data in latest_data]

    # 提取 Timestamps 和各个度量
    timestamps = [data['Timestamp'] for data in latest_data]
    metrics = {key: [data[key] for data in latest_data] for key in
               ['cur_pos', 'cur_vel', 'cur_trq', 'cur_following_err', 'servo_err',
                'servo_temperature', 'pos_cmd', 'vel_cmd']}
    return timestamps, metrics

# 更新关节数量的回调
@app.callback(
    Output('data-selection-dropdown', 'options'),
    Output('data-selection-dropdown', 'value'),
    Output('joint-count', 'data'),
    Input('interval-component', 'n_intervals'),
    State('joint-count', 'data')
)
def update_dropdown_options(n, joint_count):
    new_joint_count = get_joint_count()
    if new_joint_count != joint_count:  # 如果关节数量变化，更新下拉菜单选项
        options = [{'label': f'Data Group {i + 1}', 'value': f'Group {i + 1}'} for i in range(new_joint_count)]
        return options, 'Group 1', new_joint_count
    return dash.no_update, dash.no_update, joint_count

# 更新图表
@app.callback(
    Output('live-update-graph', 'figure'),
    Input('interval-component', 'n_intervals'),
    Input('pause-state', 'data'),
    Input('data-selection-dropdown', 'value'),
    State('joint-count', 'data'),
    State('latest-data', 'data')  # 使用存储的最新数据
)
def update_graph_callback(n, is_paused, selected_group, joint_count, latest_data):
    if is_paused:
        return dash.no_update  # 如果暂停，不更新图形

    # 如果最新数据为空，从 Redis 获取
    if not latest_data:
        timestamps, metrics = fetch_latest_data()
        latest_data = {'timestamps': timestamps, 'metrics': metrics}
    timestamps, metrics = latest_data['timestamps'], latest_data['metrics']

    fig = make_subplots(
        rows=4, cols=2,
        vertical_spacing=0.1,  # 调整折线图间距
        subplot_titles=("Current Position", "Current Velocity", "Current Torque", "Following Error",
                        "Servo Error", "Servo Temperature", "Position Command", "Velocity Command"))

    try:
        for i in range(joint_count):
            if selected_group == f'Group {i + 1}':
                # 添加每个度量的散点图，并设置 hovertemplate 显示时间
                fig.add_trace(go.Scatter(
                    x=timestamps,
                    y=[pos[i] for pos in metrics['cur_pos']],
                    mode='lines+markers',
                    line={'width': 1},
                    name='Cur Pos',
                    marker=dict(size=3),
                    hovertemplate='Time: %{x:.0f}<br>Cur Pos: %{y:.2f}<extra></extra>'  # 添加时间
                ), row=1, col=1)

                fig.add_trace(go.Scatter(
                    x=timestamps,
                    y=[vel[i] for vel in metrics['cur_vel']],
                    mode='lines+markers',
                    line={'width': 1},
                    name='Cur Vel',
                    marker=dict(size=3),
                    hovertemplate='Time: %{x}<br>Cur Vel: %{y:.2f}<extra></extra>'  # 添加时间
                ), row=1, col=2)

                fig.add_trace(go.Scatter(
                    x=timestamps,
                    y=[trq[i] for trq in metrics['cur_trq']],
                    mode='lines+markers',
                    line={'width': 1},
                    name='Cur Trq',
                    marker=dict(size=3),
                    hovertemplate='Time: %{x}<br>Cur Trq: %{y:.2f}<extra></extra>'  # 添加时间
                ), row=2, col=1)

                fig.add_trace(go.Scatter(
                    x=timestamps,
                    y=[err[i] for err in metrics['cur_following_err']],
                    mode='lines+markers',
                    line={'width': 1},
                    name='Following Err',
                    marker=dict(size=3),
                    hovertemplate='Time: %{x}<br>Following Err: %{y:.2f}<extra></extra>'  # 添加时间
                ), row=2, col=2)

                fig.add_trace(go.Scatter(
                    x=timestamps,
                    y=[serr[i] for serr in metrics['servo_err']],
                    mode='lines+markers',
                    line={'width': 1},
                    name='Servo Error',
                    marker=dict(size=3),
                    hovertemplate='Time: %{x}<br>Servo Error: %{y:.2f}<extra></extra>'  # 添加时间
                ), row=3, col=1)

                fig.add_trace(go.Scatter(
                    x=timestamps,
                    y=[setm[i] for setm in metrics['servo_temperature']],
                    mode='lines+markers',
                    line={'width': 1},
                    name='Servo Temperature',
                    marker=dict(size=3),
                    hovertemplate='Time: %{x}<br>Servo Temperature: %{y:.2f}<extra></extra>'  # 添加时间
                ), row=3, col=2)

                fig.add_trace(go.Scatter(
                    x=timestamps,
                    y=[cmd[i] for cmd in metrics['pos_cmd']],
                    mode='lines+markers',
                    line={'width': 1},
                    name='Pos Cmd',
                    marker=dict(size=3),
                    hovertemplate='Time: %{x}<br>Position Cmd: %{y:.2f}<extra></extra>'  # 添加时间
                ), row=4, col=1)

                fig.add_trace(go.Scatter(
                    x=timestamps,
                    y=[cmd[i] for cmd in metrics['vel_cmd']],
                    mode='lines+markers',
                    line={'width': 1},
                    name='Vel Cmd',
                    marker=dict(size=3),
                    hovertemplate='Time: %{x}<br>Velocity Cmd: %{y:.2f}<extra></extra>'  # 添加时间
                ), row=4, col=2)

    except IndexError:
        print(f"Index error when processing group {selected_group}")

    fig.update_layout(
        height=1200,  # 增加整个图表的高度，默认值可以根据实际需求调整
        title_text='Live Data from Redis')

    return fig

# 设置暂停按钮回调
@app.callback(
    Output('pause-state', 'data'),
    Output('pause-button', 'style'),
    Input('pause-button', 'n_clicks'),
    State('pause-state', 'data')
)
def toggle_pause(n_clicks, is_paused):
    if n_clicks > 0:
        is_paused = not is_paused
    button_color = 'gray' if is_paused else 'blue'
    return is_paused, {'backgroundColor': button_color, 'color': 'white', 'position': 'absolute', 'top': '15px',
                       'right': '15px'}

# 运行 Dash 服务器
if __name__ == '__main__':
    app.run_server(host='0.0.0.0', port=8051, debug=False)
