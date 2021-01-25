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
        html.Div(id='live-update-text'),
        dcc.Graph(id='live-update-graph'),
        dcc.Interval(
            id='interval-component',
            interval=1*1000, # in milliseconds
            n_intervals=0
        )
    ])
)

@app.callback(Output('live-update-text', 'children'),
              Input('interval-component', 'n_intervals'))
def update_metrics(n):

    style = {'padding': '5px', 'fontSize': '16px'}
    return [
        html.Span('Temperature: {0:.2f}'.format(25.0), style=style),
        html.Span('pH: {0:.2f}'.format(7.0), style=style)
    ]



# Multiple components can update everytime interval gets fired.
@app.callback(Output('live-update-graph', 'figure'),
              Input('interval-component', 'n_intervals'))
def update_plots_live(n):
    
    today_str = datetime.today().strftime('_%d_%m_%Y')
    yesterday_str = (datetime.today() - timedelta(1)).strftime('_%d_%m_%Y')
    directory = '/home/pi/logs/'
    filenames = [[directory + 'temp' + yesterday_str + '.csv',
                  directory + 'temp' + today_str + '.csv'],
                 [directory + 'temp_filtered' + yesterday_str + '.csv',
                  directory + 'temp_filtered' + today_str + '.csv']]

    temp_today = np.loadtxt(filenames[0][1], delimiter=',')
    if os.path.isfile(filenames[0][0]):
        temp_yesterday = np.loadtxt(filenames[0][0], delimiter=',')
        temp = np.concatenate((temps_yesterday,temps_today),axis=0)
    else:
        temp = temp_today

    temp = temp[temp[:,0] - temp[-1,0] < 60.0]
    temp[:,0] = temp[:,0] - temp[0,0]

    
    temp_filtered_today = np.loadtxt(filenames[1][1], delimiter=',')
    if os.path.isfile(filenames[1][0]):
        temp_filtered_yesterday = np.loadtxt(filenames[1][0], delimiter=',')
        temp_filtered = np.concatenate((temps_filtered_yesterday,temps_filtered_today),axis=0)
    else:
        temp_filtered = temp_filtered_today

    temp_filtered = temp_filtered[temp_filtered[:,0] - temp_filtered[-1,0] < 60.0]
    temp_filtered[:,0] = temp_filtered[:,0] - temp_filtered[0,0]

    temp_sigmas = 2.0 * np.sqrt(temp_filtered[:,2])
    temp_sig_upper = temp_filtered[:,1] + temp_sigmas
    temp_sig_lower = temp_filtered[:,1] - temp_sigmas


    # Create the graph with subplots
    fig = plotly.tools.make_subplots(rows=1, cols=1, vertical_spacing=0.2)
    fig['layout']['margin'] = {
        'l': 30, 'r': 10, 'b': 30, 't': 10
    }
    fig['layout']['legend'] = {'x': 0, 'y': 1, 'xanchor': 'left'}

    fig.append_trace({
        'x': temp[:,0],
        'y': temp[:,1],
        'name': 'Raw Temperature',
        'mode': 'lines',
        'type': 'scatter'
    }, 1, 1)
    fig.append_trace({
        'x': temp_filtered[:,0],
        'y': temp_filtered[:,1],
        'name': 'Filtered Temperature',
        'mode': 'lines',
        'type': 'scatter'
    }, 1, 1)
    fig.append_trace({
        'x': temp_filtered[:,0],
        'y': temp_sig_upper,
        'name': 'Temp StdDev Upper',
        'mode': 'lines',
        'type': 'scatter'
    }, 1, 1)
    fig.append_trace({
        'x': temp_filtered[:,0],
        'y': temp_sig_lower,
        'name': 'Temp StdDev Lower',
        'mode': 'lines',
        'type': 'scatter'
    }, 1, 1)
    '''
    fig.append_trace({
        'x': pH[:,0],
        'y': pH[:,1],
        'name': 'Raw pH',
        'mode': 'lines',
        'type': 'scatter'
    }, 2, 1)
    fig.append_trace({
        'x': pH_filtered[:,0],
        'y': pH_filtered[:,1],
        'name': 'Filtered pH',
        'mode': 'lines',
        'type': 'scatter'
    }, 2, 1)
    fig.append_trace({
        'x': pH_filtered[:,0],
        'y': pH__sig_upper,
        'name': 'pH StdDev Upper'
        'mode': 'lines',
        'type': 'scatter'
    }, 2, 1)
    fig.append_trace({
        'x': pH_filtered[:,0],
        'y': pH__sig_lower,
        'name': 'pH StdDev Lower'
        'mode': 'lines',
        'type': 'scatter'
    }, 2, 1)
    '''
    return fig


if __name__ == '__main__':
    app.run_server(debug=False, port=8080, host='0.0.0.0')
