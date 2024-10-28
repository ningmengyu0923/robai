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

# Dash 布局
app.layout = html.Div([
    dcc.Dropdown(
        id='data-selection-dropdown',
        options=[{'label': f'Data Group {i + 1}', 'value': f'Group {i + 1}'} for i in range(6)],
        multi=False,
        # value=[f'Group {i + 1}' for i in range(6)],  # 默认选择所有数据组
        value='Group 1',  # 默认选择第一个数据组
        style={'width': '50%', 'margin': '20px'}
    ),
    dcc.Graph(id='live-update-graph'),
    dcc.Interval(id='interval-component', interval=4000, n_intervals=0),  # 每秒更新5次
    html.Button("Pause", id='pause-button', n_clicks=0,
                style={'position': 'absolute', 'top': '10px', 'right': '10px', 'backgroundColor': 'blue',
                       'color': 'white'}),
    dcc.Store(id='pause-state', data=False)  # 存储暂停状态
])


def fetch_latest_data():
    """从 Redis 获取最新的 30 条数据并提取相应指标"""
    # 从 Redis 列表获取最新的 30 条数据
    latest_data = redis_client.lrange('sensor_data', 0, 1000)  # 获取最新的 30 条数据
    latest_data = [json.loads(data.decode('utf-8')) for data in latest_data]



    timestamps = [data['Timestamp'] for data in latest_data]
    metrics = {key: [data[key] for data in latest_data] for key in
               ['cur_pos', 'cur_vel', 'cur_trq', 'cur_following_err', 'servo_err',
                'servo_temperature', 'pos_cmd', 'vel_cmd']}
    return timestamps,metrics


def update_graph(n, is_paused, selected_groups):
    if is_paused:
        return dash.no_update  # 如果暂停，不更新图形

    timestamps,metrics = fetch_latest_data()



    # 创建子图
    fig = make_subplots(rows=4, cols=2, subplot_titles=(
        "Current Position", "Current Velocity",
        "Current Torque", "Following Error",
        "Servo Error", "Servo Temperature",
        "Position Command", "Velocity Command"
    ))

    # 添加所有数据到子图
    for i in range(4):
        group_name = f'Group {i + 1}'
        if group_name in selected_groups:
            try:

                fig.add_trace(go.Scatter(x=timestamps, y=[pos[i] for pos in metrics['cur_pos']],
                                         mode='lines+markers', name=f'Cur Pos {i + 1}'),
                              row=1, col=1)
                fig.add_trace(go.Scatter(x=timestamps, y=[vel[i] for vel in metrics['cur_vel']],
                                         mode='lines+markers', name=f'Cur Vel {i + 1}'),
                              row=1, col=2)
                fig.add_trace(go.Scatter(x=timestamps, y=[trq[i] for trq in metrics['cur_trq']],
                                         mode='lines+markers', name=f'Cur Trq {i + 1}'),
                              row=2, col=1)
                fig.add_trace(go.Scatter(x=timestamps, y=[err[i] for err in metrics['cur_following_err']],
                                         mode='lines+markers', name=f'Following Err {i + 1}'),
                              row=2, col=2)

                fig.add_trace(go.Scatter(x=timestamps, y=[serr[i] for serr in metrics['servo_err']],
                                         mode='lines+markers', name=f'Servo Error {i + 1}'),
                              row=3, col=1)
                fig.add_trace(go.Scatter(x=timestamps, y=[setm[i] for setm in metrics['servo_temperature']],
                               mode='lines+markers', name=f'Servo Temperature {i + 1}'),
                    row=3, col=2)


                fig.add_trace(go.Scatter(x=timestamps, y=[cmd[i] for cmd in metrics['pos_cmd']],
                                         mode='lines+markers', name=f'Pos Cmd {i + 1}'),
                              row=4, col=1)
                fig.add_trace(go.Scatter(x=timestamps, y=[cmd[i] for cmd in metrics['vel_cmd']],
                                         mode='lines+markers', name=f'Vel Cmd {i + 1}'),
                              row=4, col=2)

            except IndexError:
                print(f"Index error when processing group {group_name}")

    # 更新布局
    fig.update_layout(height=800, title_text='Live Data from Redis')
    return fig


# 设置 Dash 回调
@app.callback(
    Output('live-update-graph', 'figure'),
    Input('interval-component', 'n_intervals'),
    Input('pause-state', 'data'),  # 获取暂停状态
    Input('data-selection-dropdown', 'value')  # 获取选择的数据组
)
def update_graph_callback(n, is_paused, selected_groups):
    return update_graph(n, is_paused, selected_groups)


# 设置暂停按钮回调
@app.callback(
    Output('pause-state', 'data'),Output('pause-button', 'style'),  # 更新按钮样式
    Input('pause-button', 'n_clicks'),State('pause-state', 'data'))
def toggle_pause(n_clicks, is_paused):
    if n_clicks % 2 == 1:  # 每次点击切换状态
        is_paused = not is_paused
    button_color = 'gray' if is_paused else 'blue'  # 根据状态设置按钮颜色
    return is_paused, {'position': 'absolute', 'top': '10px', 'right': '10px', 'backgroundColor': button_color, 'color': 'white'}

# 运行 Dash 服务器
if __name__ == '__main__':
    app.run_server(host = '0.0.0.0',debug=True)
