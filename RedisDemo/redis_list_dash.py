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
LastMsg = redis_client.lrange('sensor_data', 0, 1)[0]  # 获取一条最新数据
RosNum = len(json.loads(LastMsg.decode('utf-8'))['cur_pos'])

# 从 Redis 获取关节数量
def get_joint_count():
    latest_data = redis_client.lrange('sensor_data', 0, 1)  # 获取一条最新数据
    if latest_data:
        data = json.loads(latest_data[0].decode('utf-8'))
        return len(data['cur_pos'])  # 假设 'cur_pos' 代表关节数量
    return 1
# Dash 布局
app.layout = html.Div([
    dcc.Dropdown(
        id='data-selection-dropdown',
        options=[{'label': f'Data Group {i + 1}', 'value': f'Group {i + 1}'} for i in range(RosNum)],
        multi=False,
        value='Group 1',  # 默认选择第一个数据组
        style={'width': '50%', 'margin': '20px'}
    ),
    dcc.Graph(id='live-update-graph'),
    dcc.Interval(id='interval-component', interval=3000, n_intervals=0),  # 每秒更新一次
    html.Button("Pause", id='pause-button', n_clicks=0,
                style={'position': 'absolute', 'top': '10px', 'right': '10px', 'backgroundColor': 'blue',
                       'color': 'white'}),
    dcc.Store(id='pause-state', data=False)  # 存储暂停状态
])


def fetch_latest_data():
    """从 Redis 获取最新的 3000 条数据"""
    latest_data = redis_client.lrange('sensor_data', 0, 30000)  # 获取最新的 3000 条数据
    latest_data = [json.loads(data.decode('utf-8')) for data in latest_data]
    timestamps = [data['Timestamp'][-12:-3] for data in latest_data]
    metrics = {key: [data[key] for data in latest_data] for key in
               ['cur_pos', 'cur_vel', 'cur_trq', 'cur_following_err', 'servo_err',
                'servo_temperature', 'pos_cmd', 'vel_cmd']}

    return timestamps, metrics


def update_graph(n, is_paused, selected_group):
    if is_paused:
        return dash.no_update  # 如果暂停，不更新图形

    timestamps, metrics = fetch_latest_data()

    # 创建子图
    fig = make_subplots(
        rows=4, cols=2,
        vertical_spacing=0.2, # 调整垂直间距，默认是0.02，你可以增加到0.1或更高
        subplot_titles=(
        "Current Position", "Current Velocity", "Current Torque", "Following Error",
        "Servo Error", "Servo Temperature", "Position Command", "Velocity Command"))

    # 添加数据到子图
    try:
        for i in range(RosNum):
            if selected_group == f'Group {i + 1}':
                fig.add_trace(go.Scatter(x=timestamps, y=[pos[i] for pos in metrics['cur_pos']],
                                         mode='lines+markers',line={'width': 1},  # 设置线条宽度为1
                                         name=f'Cur Pos',
                                         marker=dict(size=3)),  # 设置点的大小
                              row=1, col=1)
                fig.add_trace(go.Scatter(x=timestamps, y=[vel[i] for vel in metrics['cur_vel']],
                                         mode='lines+markers',line={'width': 1},  # 设置线条宽度为1
                                         name=f'Cur Vel',
                                         marker=dict(size=3)),  # 设置点的大小
                              row=1, col=2)
                fig.add_trace(go.Scatter(x=timestamps, y=[trq[i] for trq in metrics['cur_trq']],
                                         mode='lines+markers',line={'width': 1},  # 设置线条宽度为1
                                         name=f'Cur Trq',
                                         marker=dict(size=3)),  # 设置点的大小
                              row=2, col=1)
                fig.add_trace(go.Scatter(x=timestamps, y=[err[i] for err in metrics['cur_following_err']],
                                         mode='lines+markers',line={'width': 1},  # 设置线条宽度为1
                                         name=f'Following Err',
                                         marker=dict(size=3)),  # 设置点的大小
                              row=2, col=2)

                fig.add_trace(go.Scatter(x=timestamps, y=[serr[i] for serr in metrics['servo_err']],
                                         mode='lines+markers',line={'width': 1},  # 设置线条宽度为1
                                         name=f'Servo Error',
                                         marker=dict(size=3)),  # 设置点的大小
                              row=3, col=1)
                fig.add_trace(go.Scatter(x=timestamps, y=[setm[i] for setm in metrics['servo_temperature']],
                                         mode='lines+markers',line={'width': 1},  # 设置线条宽度为1
                                         name=f'Servo Temperature',
                                         marker=dict(size=3)),  # 设置点的大小
                              row=3, col=2)

                fig.add_trace(go.Scatter(x=timestamps, y=[cmd[i] for cmd in metrics['pos_cmd']],
                                         mode='lines+markers',line={'width': 1},  # 设置线条宽度为1
                                         name=f'Pos Cmd',
                                         marker=dict(size=3)),  # 设置点的大小
                              row=4, col=1)
                fig.add_trace(go.Scatter(x=timestamps, y=[cmd[i] for cmd in metrics['vel_cmd']],
                                         mode='lines+markers',line={'width': 1},  # 设置线条宽度为1
                                         name=f'Vel Cmd',
                                         marker=dict(size=3)),  # 设置点的大小
                              row=4, col=2)

                fig.update_yaxes(range=[-1000, 1000], row=1, col=1)  # 设置适当的最小值和最大值


    except IndexError:
        print(f"Index error when processing group {selected_group}")

    # 更新布局
    fig.update_layout(height=800, title_text='Live Data from Redis')
    return fig


# 设置 Dash 回调
@app.callback(
    Output('live-update-graph', 'figure'),
    Input('interval-component', 'n_intervals'),
    Input('pause-state', 'data'),
    Input('data-selection-dropdown', 'value')
)
def update_graph_callback(n, is_paused, selected_group):
    return update_graph(n, is_paused, selected_group)


# 设置暂停按钮回调
@app.callback(
    Output('pause-state', 'data'),
    Output('pause-button', 'style'),
    Input('pause-button', 'n_clicks'),
    State('pause-state', 'data')
)
def toggle_pause(n_clicks, is_paused):
    if n_clicks > 0:  # 确保有点击发生
        is_paused = not is_paused  # 每次点击后切换暂停状态
    button_color = 'gray' if is_paused else 'blue'
    return is_paused, {'backgroundColor': button_color, 'color': 'white', 'position': 'absolute', 'top': '15px', 'right': '15px'}


# 运行 Dash 服务器
if __name__ == '__main__':

    app.run_server(host='0.0.0.0', debug=True,)
