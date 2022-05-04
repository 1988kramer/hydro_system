from datetime import datetime, timedelta
import numpy as np
import os

import dash
import dash_core_components as dcc
import dash_html_components as html
import plotly
from dash.dependencies import Input, Output


external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']

app = dash.Dash(__name__, external_stylesheets=external_stylesheets)
app.layout = html.Div(
    html.Div([
        html.H4('Hydro System Status'),
        dcc.Graph(id='live-update-graph'),
        dcc.Interval(
            id='interval-component',
            interval=60*1000, # in milliseconds
            n_intervals=0
        )
    ])
)


def get_data(name):
    today_str = datetime.today().strftime('_%d_%m_%Y')
    yesterday_str = (datetime.today() - timedelta(1)).strftime('_%d_%m_%Y')
    directory = '/home/pi/logs/'
    data_today_fname = directory + name + today_str + '.csv'
    data_yesterday_fname = directory + name + yesterday_str + '.csv'
    data_filt_today_fname = directory + name + '_filtered' + today_str + '.csv'
    data_filt_yesterday_fname = directory + name + '_filtered' + yesterday_str + '.csv'

    data = np.loadtxt(data_today_fname, delimiter=',')
    if os.path.isfile(data_yesterday_fname):
        data_yesterday = np.loadtxt(data_yesterday_fname, delimiter=',')
        data = np.concatenate((data_yesterday,data))

    data_filt = np.loadtxt(data_filt_today_fname, delimiter=',')
    if os.path.isfile(data_filt_yesterday_fname):
        data_filt_yesterday = np.loadtxt(data_filt_yesterday_fname, delimiter=',')
        data_filtered = np.concatenate((data_filt_yesterday, data_filt))

    data = data[data[-1,0] - data[:,0] < 84600.0]
    data[:,0] -= data[0,0]
    data[:,0] -= data[-1,0]
    data[:,0] /= 3600.
    data_filtered = data_filtered[data_filtered[-1,0] - data_filtered[:,0] < 84600.0]
    data_filtered[:,0] -= data_filtered[0,0]
    data_filtered[:,0] -= data_filtered[-1,0]
    data_filtered[:,0] /= 3600.

    return data, data_filtered

def get_y_range(data, offset = 1):
    lower = max(np.min(data[:,1]) - offset/2.0, np.median(data[:,1] - offset))
    upper = min(np.max(data[:,1]) + offset/2.0, np.median(data[:,1] + offset))
    return [lower, upper]


# Multiple components can update everytime interval gets fired.
@app.callback(Output('live-update-graph', 'figure'),
              Input('interval-component', 'n_intervals'))
def update_plots_live(n):
    
    water_temp, water_temp_filtered = get_data('water_temp')
    pH, pH_filtered = get_data('pH')
    air_temp, air_temp_filtered = get_data('air_temp')
    humidity, humidity_filtered = get_data('humidity')
    water_depth, water_depth_filtered = get_data('depth')

    deg_sign = u'\N{DEGREE SIGN}'

    water_temp_title = 'Water Temperature (current: %.2f' % water_temp_filtered[-1,1]
    water_temp_title += deg_sign + 'C)'
    pH_title = 'Water pH (current: %.2f)' % pH_filtered[-1,1]
    air_temp_title = 'Air Temperature (current: %.2f' % air_temp_filtered[-1,1]
    air_temp_title += deg_sign + 'C)'
    humidity_title = 'Relative Humidity (current: %.2f' % humidity_filtered[-1,1]
    humidity_title +=  '%)'
    depth_title = 'Water Volume (current: %.1f L)' % water_depth[-1,1]

    # Create the graph with subplots
    fig = plotly.subplots.make_subplots(rows=5, cols=1, vertical_spacing=0.1,
            subplot_titles=(pH_title, water_temp_title, air_temp_title, humidity_title, depth_title))
    fig['layout']['margin'] = {
        'l': 30, 'r': 10, 'b': 30, 't': 30
    }
    fig['layout']['legend'] = {'x': 0, 'y': 1, 'xanchor': 'left'}

    fig.append_trace({
        'x': water_temp[:,0],
        'y': water_temp[:,1],
        'name': 'Raw',
        'mode': 'lines',
        'type': 'scatter',
        'line': {'dash': 'dash', 'color': 'red'}
    }, 2, 1)
    fig.append_trace({
        'x': water_temp_filtered[:,0],
        'y': water_temp_filtered[:,1],
        'name': 'Filtered',
        'mode': 'lines',
        'type': 'scatter',
        'line': {'dash': 'solid', 'color': 'blue'}
    }, 2, 1)
    fig.update_yaxes(range=get_y_range(water_temp_filtered, 5), row=2, col=1)
    
    fig.append_trace({
        'x': pH[:,0],
        'y': pH[:,1],
        'name': 'Raw pH',
        'mode': 'lines',
        'type': 'scatter',
        'line': {'dash': 'dash', 'color': 'red'},
        'showlegend': False
    }, 1, 1)
    fig.append_trace({
        'x': pH_filtered[:,0],
        'y': pH_filtered[:,1],
        'name': 'Filtered pH',
        'mode': 'lines',
        'type': 'scatter',
        'line': {'dash': 'solid', 'color': 'blue'},
        'showlegend': False
    }, 1, 1)
    fig.update_yaxes(range=get_y_range(pH_filtered, 1), row=1, col=1)

    fig.append_trace({
        'x': air_temp[:,0],
        'y': air_temp[:,1],
        'name': 'air temp raw',
        'mode': 'lines',
        'type': 'scatter',
        'line': {'dash': 'dash', 'color': 'red'},
        'showlegend': False
    }, 3, 1)
    fig.append_trace({
        'x': air_temp_filtered[:,0],
        'y': air_temp_filtered[:,1],
        'name': 'air temp filtered',
        'mode': 'lines',
        'type': 'scatter',
        'line': {'dash': 'solid', 'color': 'blue'},
        'showlegend': False
    }, 3, 1)
    fig.update_yaxes(range=get_y_range(air_temp_filtered, 5), row=3, col=1)

    fig.append_trace({
        'x': humidity[:,0],
        'y': humidity[:,1],
        'name': 'humdity raw',
        'mode': 'lines',
        'type': 'scatter',
        'line': {'dash': 'dash', 'color': 'red'},
        'showlegend': False
    }, 4, 1)
    fig.append_trace({
        'x': humidity_filtered[:,0],
        'y': humidity_filtered[:,1],
        'name': 'humidity filtered',
        'mode': 'lines',
        'type': 'scatter',
        'line': {'dash': 'solid', 'color': 'blue'},
        'showlegend': False
    }, 4, 1)
    fig.update_yaxes(range=get_y_range(humidity_filtered, 5), row=4, col=1)

    fig.append_trace({
        'x': water_depth[:,0],
        'y': water_depth[:,1],
        'name': 'water volume raw',
        'mode': 'lines',
        'type': 'scatter',
        'line': {'dash': 'dash', 'color': 'red'},
        'showlegend': False
    }, 5, 1)
    fig.append_trace({
        'x': water_depth_filtered[:,0],
        'y': water_depth_filtered[:,1],
        'name': 'water volume filtered',
        'mode': 'lines',
        'type': 'scatter',
        'line': {'dash': 'solid', 'color': 'blue'},
        'showlegend': False
    }, 5, 1)
    fig.update_yaxes(range=get_y_range(water_depth_filtered, 5), row=5, col=1)

    fig.update_yaxes(title_text='temperature (' + deg_sign + 'C)', row=2, col=1)
    fig.update_yaxes(title_text='pH', row=1, col=1)
    fig.update_yaxes(title_text='temperature (' + deg_sign + 'C)', row=3, col=1)
    fig.update_yaxes(title_text='Humidity (%)', row=4, col=1)
    fig.update_yaxes(title_text='volume (L)',row=5,col=1)
    fig.update_xaxes(title_text='time (hours)', row=1, col=1)
    fig.update_xaxes(title_text='time (hours)', row=2, col=1)
    fig.update_xaxes(title_text='time (hours)', row=3, col=1)
    fig.update_xaxes(title_text='time (hours)', row=4, col=1)
    fig.update_xaxes(title_text='time (hours)', row=5, col=1)
    fig.update_layout(height=1200)


    return fig


if __name__ == '__main__':
    app.run_server(debug=False, port=8080, host='0.0.0.0')
