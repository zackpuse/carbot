#!/usr/bin/env python3
import http.server
import socketserver
import os
import json
import subprocess
import cgi

PORT = 8080
DIRECTORY = os.path.dirname(os.path.abspath(__file__))

os.chdir(DIRECTORY)

class CustomHandler(http.server.SimpleHTTPRequestHandler):
    def do_POST(self):
        if self.path == '/api/calibrate':
            self.handle_calibrate()
        elif self.path == '/api/save':
            self.handle_save()
        else:
            self.send_error(404, "Endpoint not found")

    def handle_calibrate(self):
        # Run auto_hsv_web.py --action calibrate
        script_path = "/ssd/ros_ws/src/carbot_auto_hsv/scripts/auto_hsv_web.py"
        try:
            # We must use the car_brain python environment AND source ROS setup
            bash_cmd = "source /opt/ros/melodic/setup.bash && source /ssd/ros_ws/devel/setup.bash && /ssd/envs/car_brain/bin/python {} --action calibrate".format(script_path)
            cmd = ["bash", "-c", bash_cmd]
            result = subprocess.check_output(cmd, stderr=subprocess.STDOUT)
            
            # GStreamer prints to stdout, so we must find the JSON line
            json_response = "{}"
            for line in reversed(result.decode('utf-8').strip().split('\n')):
                if line.startswith('{') and line.endswith('}'):
                    json_response = line
                    break
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json_response.encode('utf-8'))
        except subprocess.CalledProcessError as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            error_json = json.dumps({"error": "Script failed", "details": e.output.decode('utf-8')})
            self.wfile.write(error_json.encode('utf-8'))
        except Exception as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            error_json = json.dumps({"error": str(e)})
            self.wfile.write(error_json.encode('utf-8'))

    def handle_save(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        data = json.loads(post_data.decode('utf-8'))
        
        script_path = "/ssd/ros_ws/src/carbot_auto_hsv/scripts/auto_hsv_web.py"
        try:
            bash_cmd = "source /opt/ros/melodic/setup.bash && source /ssd/ros_ws/devel/setup.bash && /ssd/envs/car_brain/bin/python {} --action save --lower {} {} {} --upper {} {} {}".format(
                script_path, data['lower'][0], data['lower'][1], data['lower'][2], data['upper'][0], data['upper'][1], data['upper'][2])
            cmd = ["bash", "-c", bash_cmd]
            result = subprocess.check_output(cmd, stderr=subprocess.STDOUT)
            json_response = "{}"
            for line in reversed(result.decode('utf-8').strip().split('\n')):
                if line.startswith('{') and line.endswith('}'):
                    json_response = line
                    break
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json_response.encode('utf-8'))
        except subprocess.CalledProcessError as e:
            self.send_response(500)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            error_json = json.dumps({"error": "Save failed", "details": e.output.decode('utf-8')})
            self.wfile.write(error_json.encode('utf-8'))

if __name__ == '__main__':
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("0.0.0.0", PORT), CustomHandler) as httpd:
        print("Dashboard serving at http://0.0.0.0:%d" % PORT)
        print("Akses dari browser: http://<jetson-ip>:%d" % PORT)
        httpd.serve_forever()
