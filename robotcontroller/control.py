from robotcontroller.controller import control, get_robot_communication
from flask_socketio import SocketIO, emit
from flask import Flask, request, send_from_directory,  Response, render_template, jsonify
from flask import jsonify
from multiprocessing import Process, Queue

global task_queue
global control_process
task_queue = None
control_process = None
if task_queue is None:
    task_queue = Queue()
if control_process is None:
    print("CREATING CONTROL PROCESS")
    robot_communication = get_robot_communication(True)
    control_process = control(task_queue, robot_communication)

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
app.config['SESSION_TYPE'] = 'filesystem'
app.config['TEMPLATES_AUTO_RELOAD'] = True


@app.route('/message', methods=['POST'])
def index():
    message = request.get_json()
    task_queue.put(message)
    return 'OK', 200

if __name__ == '__main__':
    socketio = SocketIO(app, async_mode=None)
    # NOTE: debug and reloader can not be used as otherwise the file is loaded twice causing loading the controller also twice
    socketio.run(app, host='0.0.0.0', debug=True, use_reloader=False, port=6000)
    #move_message = { 'type': 'MOVE_TO_STEPS', 'steps': [1000, 10000, 10000, 10000] }







