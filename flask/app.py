from flask import Flask 
from flask import render_template
from datetime import datetime
import numpy as np 

app = Flask(__name__)

def get_data(name):
  today_str = datetime.today().strftime('_%d_%m_%Y')
  directory = '/home/pi/logs/'
  data_fname = directory + name + '_filtered' + today_str + '.csv'
  
  data = np.loadtxt(data_fname, delimiter=',')

  return data[-1,1]

@app.route('/')
def time_now():
  pH = get_data('pH')
  pH_str = '%.2f' % pH
  air_temp = get_data('air_temp')
  air_temp_str = '%.1f' % air_temp
  water_temp = get_data('water_temp')
  water_temp_str = '%.1f' % water_temp
  humidity = get_data('humidity')
  humidity_str = '%.1f' % humidity
  return render_template('time.html',vals=[pH_str,water_temp_str,air_temp_str,humidity_str])
