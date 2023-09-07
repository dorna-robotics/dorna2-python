from flask import Flask, render_template, request, redirect, url_for, session
from dorna2 import Dorna
import json
import time


# create the Dorna object
robot = Dorna()

app = Flask(__name__)
app.secret_key = 'Anirudh'
robot = Dorna()
connected = robot.connect(host="192.168.1.101", port=443)

# Path to the file where the values will be stored
VALUES_FILE = 'values.json'

# Initialize the values dictionary
values = {}

try:
    with open(VALUES_FILE, 'r') as f:
        file_content = f.read()
        if file_content:
            values = json.loads(file_content)
except FileNotFoundError:
    pass
except json.JSONDecodeError:
    pass

# Index route to render the main page
@app.route('/')
def index():
    return render_template('index3.html', log_message=session.pop('log_message',''))

# Route to handle form submission
@app.route('/submit', methods=['POST'])
def submit():
    if request.method == 'POST':
        if request.form.get('saveButton') == 'saveButton':
            # Retrieve data from the submitted form
            dropdown_value = request.form.get('dropdownValue')
            input_field_value = request.form.get('inputFieldValue')

            values[dropdown_value] = input_field_value

            with open(VALUES_FILE, 'w') as f:
                json.dump(values, f)

            session['log_message'] = 'Variable saved as ' + str(dropdown_value) + " " + str(input_field_value)

    # Process the data or perform desired actions
    # Add your logic here

    return redirect(url_for('index'))

# Route to handle loading tweezers button
@app.route('/load_tweezers', methods=['POST'])
def load_tweezers():
    if request.method == 'POST':
        if request.form.get('loadTweezersButton') == 'loadTweezersButton':
            robot.play(timeout=-1, cmd="jmove", rel=0, j0=values['j0posl'] , j1=values['j1posl'] , j2=values['j2posl'] , j3=values['j3posl'] , j4=values['j4posl'], vel=values['velocity'], accel=values['acceleration'], jerk=values['jerk'])
            
            #robot.jmove(rel=0, j0=1.654, j1=59.895, j2=-87.483, j3=-59.940, j4=0, vel= 800, accel=10000, jerk=25000)


            session['log_message'] = 'Tweezers loaded'
        if request.form.get('loadTweezersSetButton') == 'loadTweezersSetButton':
            positions = robot.get_all_joint()
            print(positions)
            values['j0posl'] = positions[0]
            values['j1posl'] = positions[1]
            values['j2posl'] = positions[2]
            values['j3posl'] = positions[3]
            values['j4posl'] = positions[4]
            session['log_message'] = 'Positions set'

            with open(VALUES_FILE, 'w') as f:
                json.dump(values, f)

    return redirect(url_for('index'))  # Example response

# Route to handle spray position button
@app.route('/spray_position', methods=['POST'])
def spray_position():
    if request.method == 'POST':
        if request.form.get('sprayPositionButton') == 'sprayPositionButton':
            robot.play(timeout=-1, cmd="jmove", rel=0, j0=values['j0poss'], j1=values['j1poss'], j2=values['j2poss'], j3=values['j3poss'], j4=values['j4poss'], vel=values['velocity'], accel=values['acceleration'], jerk=values['jerk'])
            
            #robot.jmove(rel=0, j0=0, j1=18.680, j2=-3.409, j3=-102.780, j4=0, vel= 800, accel=10000, jerk=25000)

            
            session['log_message'] = 'Robot moved to spray position'
        if request.form.get('sprayPositionSetButton') == 'sprayPositionSetButton':
            positions = robot.get_all_joint()
            print(positions)
            values['j0poss'] = positions[0]
            values['j1poss'] = positions[1]
            values['j2poss'] = positions[2]
            values['j3poss'] = positions[3]
            values['j4poss'] = positions[4]
            session['log_message'] = 'Positions set'

            with open(VALUES_FILE, 'w') as f:
                json.dump(values, f)

    return redirect(url_for('index'))  # Example response

# Route to handle plunge button
@app.route('/plunge', methods=['POST'])
def plunge():
    if request.method == 'POST':
        if request.form.get('plungeButton') == 'plungeButton':
            if request.form.get('sprayCheckbox') == 'sprayCheckbox' and request.form.get('blotCheckbox') == 'blotCheckbox':
                robot.set_output(0, 0)
                time.sleep(3)
                #time.sleep(blot_time)
                robot.set_output(0, 1) 
                robot.play(timeout=-1, cmd="lmove", rel=0, j0=values['j0posp'], j1=values['j1posp'], j2=values['j2posp'], j3=values['j3posp'], j4=values['j4posp'], vel=values['velocity'], accel=values['acceleration'], jerk=values['jerk'])
                
                #robot.jmove(rel=0, j0=0, j1=18.373, j2=-31.984, j3=-73.912, j4=0, vel= 800, accel=10000, jerk=25000)
                
                session['log_message'] = 'Grid sprayed and robot successfully plunged'
            elif request.form.get('blotCheckbox') == 'blotCheckbox':
                robot.set_output(0, 0)
                time.sleep(3)
                #time.sleep(blot_time)
                robot.set_output(0, 1)
                robot.play(timeout=-1, cmd="lmove", rel=0, j0=values['j0posp'], j1=values['j1posp'], j2=values['j2posp'], j3=values['j3posp'], j4=values['j4posp'], vel=values['velocity'], accel=values['acceleration'], jerk=values['jerk'])
                
                #robot.jmove(rel=0, j0=0, j1=18.373, j2=-31.984, j3=-73.912, j4=0, vel= 800, accel=10000, jerk=25000)
                
                session['log_message'] = 'Grid blotted and robot successfully plunged'
            else:
                session['log_message'] = 'Proper options not selected'
            
        if request.form.get('plungeSetButton') == 'plungeSetButton':
            positions = robot.get_all_joint()
            print(positions)
            values['j0posp'] = positions[0]
            values['j1posp'] = positions[1]
            values['j2posp'] = positions[2]
            values['j3posp'] = positions[3]
            values['j4posp'] = positions[4]
            session['log_message'] = 'Positions set'

            with open(VALUES_FILE, 'w') as f:
                json.dump(values, f)

    return redirect(url_for('index'))  # Example response

# Route to handle store grid button
@app.route('/store_grid', methods=['POST'])
def store_grid():
    if request.method == 'POST':
        if request.form.get('storeGridButton') == 'storeGridButton' and request.form.get('storePositionFieldValue') == '1':
            #robot.play(timeout=-1, cmd="jmove", rel=0, j0=5.428, j1=29.745, j2=-51.892, j3=-65.385, j4=0, vel= 500, accel=3000, jerk=25000)
            #robot.play(timeout=-1, cmd="jmove", rel=0, j0=6.75, j1=25.623, j2=-51.699, j3=-61.447, j4=0, vel= 500, accel=3000, jerk=25000)
            
            start = time.time()
            robot.lmove(rel=0, j0=0, j1=28.183, j2=-46.996, j3=-68.715, j4=0, vel= 800, accel=10000, jerk=25000, timeout=0)
            robot.lmove(rel=0, j0=0, j1=31.495, j2=-65.662, j3=-52.166, j4=0)
            robot.log(time.time()-start)
            
            session['log_message'] = 'Grid successfully stored in position 1'
        elif request.form.get('storeGridButton') == 'storeGridButton' and request.form.get('storePositionFieldValue') == 2:
            robot.play(timeout=-1, cmd="jmove", rel=0, j0=0, j1=0, j2=-0, j3=0, j4=0, vel=values['velocity'], accel=values['acceleration'], jerk=values['jerk'])
            session['log_message'] = 'Grid successfully stored in position 2'
        elif request.form.get('storeGridButton') == 'storeGridButton' and request.form.get('storePositionFieldValue') == 3:
            robot.play(timeout=-1, cmd="jmove", rel=0, j0=0, j1=0, j2=-0, j3=0, j4=0, vel=values['velocity'], accel=values['acceleration'], jerk=values['jerk'])
            session['log_message'] = 'Grid successfully stored in position 3'
        elif request.form.get('storeGridButton') == 'storeGridButton' and request.form.get('storePositionFieldValue') == 4:
            robot.play(timeout=-1, cmd="jmove", rel=0, j0=0, j1=0, j2=-0, j3=0, j4=0, vel=values['velocity'], accel=values['acceleration'], jerk=values['jerk'])
            session['log_message'] = 'Grid successfully stored in position 4'
        else:
            session['log_message'] = 'Proper parameters not given'

    return redirect(url_for('index'))  # Example response

@app.route('/connect', methods=['GET', 'POST'])
def connect():
    global robot, connected

    if request.method == 'POST':
        if request.form.get('controlRobot') == 'connectButton':
            robot = Dorna()
            connected = robot.connect(host="192.168.1.101", port=443)
            if connected:
                session['log_message'] = 'Robot connected successfully'
            else:
                session['log_message'] = 'Could not connect to robot'

        
        elif request.form.get('controlRobot') == 'powerOnButton':
            robot.set_motor(1)
            robot.set_output(0, 1)
            robot.play(cmd="pid", pid=0)
            session['log_message'] = 'Robot turned on and motors turned on'
        
        elif request.form.get('controlRobot') == 'resetButton':
            session['log_message'] = 'Grid successfully stored'
        
        elif request.form.get('controlRobot') == 'lowPowerButton':
            
            values['velocity'] = 100
            values['acceleration'] = 1000
            values['jerk'] = 3000

            with open(VALUES_FILE, 'w') as f:
                json.dump(values, f)

            session['log_message'] = 'Low Power Mode on'
        
        elif request.form.get('controlRobot') == 'highPowerButton':

            values['velocity'] = 800
            values['acceleration'] = 10000
            values['jerk'] = 20000

            with open(VALUES_FILE, 'w') as f:
                json.dump(values, f)

            session['log_message'] = 'High Power Mode on'
        
        elif request.form.get('controlRobot') == 'configurationButton':
            session['log_message'] = 'Configuration window'
            return redirect(url_for('configuration'))
        
        elif request.form.get('controlRobot') == 'robotManagerButton':
            #session['log_message'] = 'Dorna GUI'
            return redirect("http://192.168.1.100/", code=302)
        
        elif request.form.get('controlRobot') == 'ioManagerButton':
            #session['log_message'] = 'I/O manager'
            return redirect(url_for('index'))
        
        elif request.form.get('controlRobot') == 'powerOffButton':
            robot.play(timeout=-1, cmd="jmove", rel=0, j0=180, j1=180, j2=-142, j3=135, j4=0, vel=values['velocity'], accel=values['acceleration'], jerk=values['jerk'])
            robot.play(cmd="pid", pid=1)
            robot.set_motor(0)
            robot.close()
            session['log_message'] = 'Robot powered off'
        
        elif request.form.get('controlRobot') == 'zero':
            robot.play(timeout=-1, cmd="jmove", rel=0, j0=0, j1=0, j2=-0, j3=0, j4=0, vel=values['velocity'], accel=values['acceleration'], jerk=values['jerk'])
            session['log_message'] = 'Robot moved to zero'

    return redirect(url_for('index'))



# Run the Flask application
if __name__ == '__main__':
    app.run(debug=True, port=5000, host='0.0.0.0')