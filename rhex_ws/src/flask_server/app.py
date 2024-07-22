from flask import Flask, jsonify
import subprocess
import os

app = Flask(__name__)

@app.route('/')
def home():
    return 'Welcome to the Flask Server!'

@app.route('/run', methods=['POST'])
def run_code():
    try:
        # Set the working directory
        working_directory = '/home/rhex/robot/rhex-api/examples'  # Update to the correct path
        os.chdir(working_directory)

        # Check if the binary file exists
        binary_file = './basic_walk'
        if not os.path.isfile(binary_file):
            return jsonify({'error': f"Binary file '{binary_file}' does not exist in directory '{working_directory}'."}), 404

        # Run the binary
        run_command = [binary_file]
        run_result = subprocess.run(run_command, capture_output=True, text=True)
        
        return jsonify({
            'run_output': run_result.stdout,
            'run_errors': run_result.stderr,
            'returncode': run_result.returncode
        })

    except Exception as e:
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
