#!/usr/bin/env python
from flask import Flask
from flask import render_template

app = Flask(__name__)

@app.route("/")
def showCamera():
    return render_template('index.html') 

if __name__ == "__main__":
#    app.debug = True
    app.run(host='0.0.0.0', port=10000)
