from dorna2 import Dorna
from flask import Flask, render_template, request, session, redirect, url_for


# create the Dorna object
robot = Dorna()

app = Flask(__name__)
app.secret_key = 'Anirudh'
robot = Dorna()
connected = robot.connect(host="192.168.1.101", port=443)


@app.route('/')
@app.route('/home')
def index():
    return render_template('index.html', log_message=session.pop('log_message',''))
    
@app.route('/connect', methods=['GET', 'POST'])
def connect():
    global robot, connected

    if request.method == 'POST':
        robot = Dorna()
        connected = robot.connect(host="192.168.1.101", port=443)

        session['log_message'] = 'Robot connected successfully'

    return redirect(url_for('index'))


@app.route('/move', methods=['GET', 'POST'])
def move():
    global robot, connected

    '''if not connected:
        session['log_message'] = 'Error: Robot not connected
         return 'Error: Robot not connected'''

    joint = request.form['joint']

    if joint == 'joint0+':
        #robot.play(timeout=-1, cmd="jmove", rel=1, j0=10, vel=50)
        #print("Moved j0")
        robot.set_output(0, 1) 
    elif joint == 'joint0-':
        robot.play(timeout=-1, cmd="jmove", rel=1, j0=-10, vel=50)
        print("Moved j0")
    elif joint == 'joint1+':
        robot.play(timeout=-1, cmd="jmove", rel=1, j1=+10, vel=50)
        print("Moved j0")
    elif joint == 'joint1-':
        robot.play(timeout=-1, cmd="jmove", rel=1, j1=-10, vel=50)
        print("Moved j0")
    elif joint == 'joint2+':
        robot.play(timeout=-1, cmd="jmove", rel=1, j2=10, vel=50)
        print("Moved j0")
    elif joint == 'joint2-':
        robot.play(timeout=-1, cmd="jmove", rel=1, j2=-10, vel=50)
        print("Moved j0")
    elif joint == 'joint3+':
        robot.play(timeout=-1, cmd="jmove", rel=1, j3=10, vel=50)
        print("Moved j0")
    elif joint == 'joint3-':
        robot.play(timeout=-1, cmd="jmove", rel=1, j3=-10, vel=50)
        print("Moved j0")
    elif joint == 'joint4+':
        robot.play(timeout=-1, cmd="jmove", rel=1, j4=10, vel=50)
        print("Moved j0")
    elif joint == 'joint4-':
        robot.play(timeout=-1, cmd="jmove", rel=1, j4=-10, vel=50)
        print("Moved j0")
    elif joint == 'home':
        robot.play(timeout=-1, cmd="jmove", rel=0, j0=180, j1=180, j2=-142, j3=135, j4=0, vel=50)
    elif joint == 'zero':
        robot.play(timeout=-1, cmd="jmove", rel=0, j0=0, j1=0, j2=-0, j3=0, j4=0, vel=50)



    session['log_message'] = 'Move commond executed successfully'
    return redirect(url_for('index'))

         

    

#robot.play(timeout=-1, cmd="jmove", rel=1, j0=60, vel=50)

    

if __name__ == '__main__':
	app.run()