from flask import Flask 
from flask import render_template
from datetime import datetime

app = Flask(__name__)

@app.route('/')
def time_now():
  now = datetime.now().strftime("%H:%M")
  return render_template('time.html',now=now)