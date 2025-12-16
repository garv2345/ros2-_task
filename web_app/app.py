import os
import subprocess
import sys
from flask import Flask, render_template, request, redirect, url_for

app = Flask(__name__)

# Configure folders
UPLOAD_FOLDER = 'uploads'
app.config['UPLOAD_FOLDER'] = UPLOAD_FOLDER

# Ensure upload folder exists
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/upload', methods=['POST'])
def upload_file():
    if 'file' not in request.files:
        return redirect(request.url)

    file = request.files['file']
    if file.filename == '':
        return redirect(request.url)

    if file:
        # 1. Save the file
        filepath = os.path.join(app.config['UPLOAD_FOLDER'], file.filename)
        file.save(filepath)

        # 2. Run the Checker Script
        # We assume checker.py is in the same folder as app.py
        checker_path = os.path.join(os.getcwd(), 'checker.py')
        try:
            # Run using the same python executable
            result = subprocess.run(
                [sys.executable, checker_path, filepath],
                capture_output=True,
                text=True
            )
            report = result.stdout
            if not report:
                report = "Checker ran but produced no output. Check if checker.py is present."
        except Exception as e:
            report = f"Error running checker: {str(e)}"

        return render_template('index.html', report=report)

@app.route('/simulate', methods=['POST'])
def simulate():
    # 3. Run the Simulation Node
    try:
        # We assume simple_move.py is in the same folder
        sim_path = os.path.join(os.getcwd(), 'simple_move.py')
        subprocess.Popen([sys.executable, sim_path])
        message = "Simulation Command Sent! Check Gazebo window."
    except Exception as e:
        message = f"Error starting simulation: {str(e)}"

    return render_template('index.html', simulation_status=message)

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')
