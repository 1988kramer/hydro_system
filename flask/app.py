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
  temp = get_data('temp')
  temp_str = '%.2f' % temp
  return render_template('time.html',vals=[pH_str,temp_str])
