from flask import Flask, render_template, request, redirect, url_for, session
from dorna2 import Dorna

# create the Dorna object
robot = Dorna()

app = Flask(__name__)
app.secret_key = 'Anirudh'
robot = Dorna()
connected = robot.connect(host="192.168.1.101", port=443)

# Index route to render the main page
@app.route('/')
def index():
    return render_template('index2.html', log_message=session.pop('log_message',''))

# Route to handle form submission
@app.route('/submit', methods=['POST'])
def submit():
    if 'saveButton' in request.form:
        # Retrieve data from the submitted form
        dropdown_value = request.form['dropdownValue']
        input_field_value = request.form['inputFieldValue']

    print(dropdown_value)
    print(input_field_value)

    # Process the data or perform desired actions
    # Add your logic here

    return 'Data submitted successfully'  # Example response

# Route to handle loading tweezers button
@app.route('/load_tweezers', methods=['POST'])
def load_tweezers():
    xpos = 0
    ypos = 0
    zpos = 0
    apos = 0
    bpos = 0
    if request.method == 'POST':
        if request.form.get('loadTweezersButton') == 'loadTweezersButton':
            robot.play(timeout=-1, cmd="lmove", rel=0, x=xpos, y=ypos, z=zpos, a=apos, b=bpos, vel=50)
            session['log_message'] = 'Tweezers loaded'
        if request.form.get('loadTweezersSetButton') == 'loadTweezersSetButton':
            positions = robot.get_all_pose()
            print(positions)
            xpos = positions[0]
            ypos = positions[1]
            zpos = positions[2]
            apos = positions[3]
            bpos = positions[4]
            session['log_message'] = 'Positions set'

    return redirect(url_for('index'))  # Example response

# Route to handle spray position button
@app.route('/spray_position', methods=['POST'])
def spray_position():
    if request.method == 'POST':
        if request.form.get('sprayPositionButton') == 'sprayPositionButton':
            robot.play(timeout=-1, cmd="jmove", rel=0, j0=0, j1=0, j2=-0, j3=0, j4=0, vel=50)
            session['log_message'] = 'Robot moved to spray position'

    return redirect(url_for('index'))  # Example response

# Route to handle plunge button
@app.route('/plunge', methods=['POST'])
def plunge():
    if request.method == 'POST':
        if request.form.get('plungeButton') == 'plungeButton':
            if request.form.get('sprayCheckbox') == 'sprayCheckbox' and request.form.get('blotCheckbox') == 'blotCheckbox':
                robot.play(timeout=-1, cmd="jmove", rel=0, j0=0, j1=0, j2=-0, j3=0, j4=0, vel=50)
                session['log_message'] = 'Grid sprayed and robot successfully plunged'
            elif request.form.get('blotCheckbox') == 'blotCheckbox':
                robot.play(timeout=-1, cmd="jmove", rel=0, j0=0, j1=0, j2=-0, j3=0, j4=0, vel=50)
                session['log_message'] = 'Grid blotted and robot successfully plunged'

    return redirect(url_for('index'))  # Example response

# Route to handle store grid button
@app.route('/store_grid', methods=['POST'])
def store_grid():
    if request.method == 'POST':
        if request.form.get('storeGridButton') == 'storeGridButton':
            robot.play(timeout=-1, cmd="jmove", rel=0, j0=0, j1=0, j2=-0, j3=0, j4=0, vel=50)
            session['log_message'] = 'Grid successfully stored'

    return redirect(url_for('index'))  # Example response

@app.route('/connect', methods=['GET', 'POST'])
def connect():
    global robot, connected

    if request.method == 'POST':
        if request.form.get('controlRobot') == 'connectButton':
            robot = Dorna()
            connected = robot.connect(host="192.168.1.101", port=443)
            session['log_message'] = 'Robot connected successfully'
        
        elif request.form.get('controlRobot') == 'powerOnButton':
            robot.set_motor(1)
            session['log_message'] = 'Robot turned on and motors turned on'
        
        elif request.form.get('controlRobot') == 'resetButton':
            session['log_message'] = 'Grid successfully stored'
        
        elif request.form.get('controlRobot') == 'highPowerButton':
            session['log_message'] = 'Grid successfully stored'
        
        elif request.form.get('controlRobot') == 'configurationButton':
            session['log_message'] = 'Configuration window'
        
        elif request.form.get('controlRobot') == 'robotManagerButton':
            session['log_message'] = 'Dorna GUI'
        
        elif request.form.get('controlRobot') == 'ioManagerButton':
            session['log_message'] = 'I/O manager'
        
        elif request.form.get('controlRobot') == 'powerOffButton':
            robot.play(timeout=-1, cmd="jmove", rel=0, j0=180, j1=180, j2=-142, j3=135, j4=0, vel=50)
            robot.set_motor(0)
            session['log_message'] = 'Grid successfully stored'

    return redirect(url_for('index'))

# Run the Flask application
if __name__ == '__main__':
    app.run()