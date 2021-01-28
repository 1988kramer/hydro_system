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

    data_today = np.loadtxt(data_today_fname, delimiter=',')
    if os.path.isfile(data_yesterday_fname):
        data_yesterday = np.loadtxt(data_yesterday_fname, delimiter=',')
        data = np.concatenate((data_yesterday,data_today))
    else:
        data = data_today
    data_filt_today = np.loadtxt(data_filt_today_fname, delimiter=',')
    if os.path.isfile(data_filt_yesterday_fname):
        data_filt_yesterday = np.loadtxt(data_filt_yesterday_fname, delimiter=',')
        data_filtered = np.concatenate((data_filt_yesterday, data_filt_today))
    else:
        data_filtered = data_filt_today

    data = data[data[-1,0] - data[:,0] < 86400.0]
    data[:,0] -= data[0,0]
    data[:,0] /= 3600.
    data_filtered = data_filtered[data_filtered[-1,0] - data_filtered[:,0] < 86400.0]
    data_filtered[:,0] -= data_filtered[0,0]
    data_filtered[:,0] /= 3600.

    return data, data_filtered


# Multiple components can update everytime interval gets fired.
@app.callback(Output('live-update-graph', 'figure'),
              Input('interval-component', 'n_intervals'))
def update_plots_live(n):
    
    water_temp, water_temp_filtered = get_data('water_temp')
    pH, pH_filtered = get_data('pH')
    air_temp, air_temp_filtered = get_data('air_temp')
    humidity, humidity_filtered = get_data('humidity')

    deg_sign = u'\N{DEGREE SIGN}'

    water_temp_title = 'Water Temperature (current: %.2f' % water_temp_filtered[-1,1]
    water_temp_title += deg_sign + 'C)'
    pH_title = 'Water pH (current: %.2f)' % pH_filtered[-1,1]
    air_temp_title = 'Air Temperature (current: %.2f' % air_temp_filtered[-1,1]
    air_temp_title += deg_sign + 'C)'
    humidity_title = 'Relative Humidity (current: %.2f' % humidity_filtered[-1,1]
    humidity_title += deg_sign + '%)'

    # Create the graph with subplots
    fig = plotly.subplots.make_subplots(rows=4, cols=1, vertical_spacing=0.1,
            subplot_titles=(pH_title, water_temp_title, air_temp_title, humidity_title))
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

    fig.update_yaxes(title_text='temperature (' + deg_sign + 'C)',row=1,col=1)
    fig.update_yaxes(title_text='pH',row=3,col=1)
    fig.update_yaxes(title_text='temperature (' + deg_sign + 'C)',row=2,col=1)
    fig.update_yaxes(title_text='Humidity (%)',row=4,col=1)
    fig.update_layout(height=1200)


    return fig


if __name__ == '__main__':
    app.run_server(debug=False, port=8080, host='0.0.0.0')
