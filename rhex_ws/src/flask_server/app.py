from flask import Flask, jsonify
import subprocess
import os

app = Flask(__name__)
print("here")

@app.route('/')
def home():
    return 'Welcome to the Flask Server!\n'

@app.route('/connect', methods=['POST'])
def run_connect():
    try:
        # Set the working directory
        run_dir = '/home/rhex/mnt/rhex_ws/src/controls'

        # Check if the directory exists
        if not os.path.exists(run_dir):
            return jsonify({'error': f'Directory not found: {run_dir}'}), 500
        
        os.chdir(run_dir)

        # Print the contents of the current directory
        dir_contents = os.listdir('.')
        print(f"Current directory contents: {dir_contents}")

        # Run the ./connect command
        run_command = ['./connect']
        run_result = subprocess.run(run_command, capture_output=True, text=True)
        
        return jsonify({
            'run_output': run_result.stdout,
            'run_errors': run_result.stderr,
            'returncode': run_result.returncode
        })

    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/disconnect', methods=['POST'])
def run_disconnect():
    try:
        # Set the working directory
        run_dir = '/home/rhex/mnt/rhex_ws/src/controls'

        # Check if the directory exists
        if not os.path.exists(run_dir):
            return jsonify({'error': f'Directory not found: {run_dir}'}), 500
        
        os.chdir(run_dir)

        # Print the contents of the current directory
        dir_contents = os.listdir('.')
        print(f"Current directory contents: {dir_contents}")

        # Run the ./disconnect command
        run_command = ['./disconnect']
        run_result = subprocess.run(run_command, capture_output=True, text=True)
        
        return jsonify({
            'run_output': run_result.stdout,
            'run_errors': run_result.stderr,
            'returncode': run_result.returncode
        })

    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/sit', methods=['POST'])
def run_sit():
    try:
        # Set the working directory
        run_dir = '/home/rhex/mnt/rhex_ws/src/controls'
        
        # Check if the directory exists
        if not os.path.exists(run_dir):
            return jsonify({'error': f'Directory not found: {run_dir}'}), 500
        
        os.chdir(run_dir)

        # Print the contents of the current directory
        dir_contents = os.listdir('.')
        print(f"Current directory contents: {dir_contents}")

        # Run the ./sit command
        run_command = ['./sit']
        run_result = subprocess.run(run_command, capture_output=True, text=True)
        
        return jsonify({
            'run_output': run_result.stdout,
            'run_errors': run_result.stderr,
            'returncode': run_result.returncode
        })

    except Exception as e:
        return jsonify({'error': str(e)}), 500

@app.route('/stand', methods=['POST'])
def run_stand():
    try:
        # Set the working directory
        run_dir = '/home/rhex/mnt/rhex_ws/src/controls'
        
        # Check if the directory exists
        if not os.path.exists(run_dir):
            return jsonify({'error': f'Directory not found: {run_dir}'}), 500
        
        os.chdir(run_dir)

        # Print the contents of the current directory
        dir_contents = os.listdir('.')
        print(f"Current directory contents: {dir_contents}")

        # Run the ./stand command
        run_command = ['./stand']
        run_result = subprocess.run(run_command, capture_output=True, text=True)
        
        return jsonify({
            'run_output': run_result.stdout,
            'run_errors': run_result.stderr,
            'returncode': run_result.returncode
        })
    except Exception as e:
        return jsonify({'error': str(e)}), 500
        
@app.route('/compile', methods=['POST'])
def compile_code():
    try:
        # Set the working directory
        compile_dir = '/home/rhex/robot/rhex-api/examples'
        
        # Check if the directory exists
        if not os.path.exists(compile_dir):
            return jsonify({'error': f'Directory not found: {compile_dir}'}), 500
        
        os.chdir(compile_dir)

        # Print the contents of the current directory
        dir_contents = os.listdir('.')
        print(f"Current directory contents: {dir_contents}")

        # Compile the code
        compile_command = [
            'g++', '-o', 'basic_walk', 'basic_walk.cc',
            '-I../include', '-I/usr/include/gstreamer-1.0', '-I/usr/include/glib-2.0',
            '-I/usr/lib/x86_64-linux-gnu/glib-2.0/include', '-L../lib',
            '-L/usr/lib/x86_64-linux-gnu', '-lrhexapi', '-lgstreamer-1.0',
            '-lgobject-2.0', '-lglib-2.0', '-lssl', '-lcrypto', '-lgstvideo-1.0',
            '-lgstrtp-1.0', '-largon2'
        ]

        compile_result = subprocess.run(compile_command, capture_output=True, text=True)
        
        if compile_result.returncode != 0:
            return jsonify({
                'output': compile_result.stdout,
                'errors': compile_result.stderr,
                'returncode': compile_result.returncode
            }), 500

        # Run the compiled binary
        run_command = ['./basic_walk']
        run_result = subprocess.run(run_command, capture_output=True, text=True)
        
        return jsonify({
            'run_output': run_result.stdout,
            'run_errors': run_result.stderr,
            'returncode': run_result.returncode
        })

    except Exception as e:
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5001)