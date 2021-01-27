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
    
    temp, temp_filtered = get_data('temp')
    pH, pH_filtered = get_data('pH')
    deg_sign = u'\N{DEGREE SIGN}'

    temp_title = 'Water Temperature (current: %.2f' % temp_filtered[-1,1]
    temp_title += deg_sign + 'C)'
    pH_title = 'Water pH (current: %.2f)' % pH_filtered[-1,1]
    # Create the graph with subplots
    fig = plotly.subplots.make_subplots(rows=2, cols=1, vertical_spacing=0.2,
            subplot_titles=(temp_title, pH_title))
    fig['layout']['margin'] = {
        'l': 30, 'r': 10, 'b': 30, 't': 30
    }
    fig['layout']['legend'] = {'x': 0, 'y': 1, 'xanchor': 'left'}

    fig.append_trace({
        'x': temp[:,0],
        'y': temp[:,1],
        'name': 'Raw',
        'mode': 'lines',
        'type': 'scatter',
        'line': {'dash': 'dash', 'color': 'red'}
    }, 1, 1)
    fig.append_trace({
        'x': temp_filtered[:,0],
        'y': temp_filtered[:,1],
        'name': 'Filtered',
        'mode': 'lines',
        'type': 'scatter',
        'line': {'dash': 'solid', 'color': 'blue'}
    }, 1, 1)
    
    fig.append_trace({
        'x': pH[:,0],
        'y': pH[:,1],
        'name': 'Raw pH',
        'mode': 'lines',
        'type': 'scatter',
        'line': {'dash': 'dash', 'color': 'red'},
        'showlegend': False
    }, 2, 1)
    fig.append_trace({
        'x': pH_filtered[:,0],
        'y': pH_filtered[:,1],
        'name': 'Filtered pH',
        'mode': 'lines',
        'type': 'scatter',
        'line': {'dash': 'solid', 'color': 'blue'},
        'showlegend': False
    }, 2, 1)

    fig.update_yaxes(title_text='temperature (' + deg_sign + 'C)',row=1,col=1)
    fig.update_yaxes(title_text='pH',row=2,col=1)

    return fig


if __name__ == '__main__':
    app.run_server(debug=False, port=8080, host='0.0.0.0')
