import socket
import json
import threading
import queue
from time import sleep

class fanucrobot:
    def __init__(self):
        self.ip = None
        self.port = None
        self.socket = None
        self.is_connected = False
        self.send_lock = threading.RLock()
        self.cmd_lock = threading.RLock()
        self.timely_fifo = queue.Queue()
        self.response_fifo = queue.Queue()
        self.receive_thread = None
        self.running = False
        self.configuration = {"Front":1, "Up":1, "Left":0, "Flip":0, "Turn4":0, "Turn5":0, "Turn6":0}
        
    def connect(self, ip):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((ip, 16001))
            
            connect_cmd = {"Communication":"FRC_Connect"}
            sock.send((json.dumps(connect_cmd) + "\r\n").encode('utf-8'))
            
            response = sock.recv(1024).decode('utf-8')
            sock.close()
            
            response_data = json.loads(response)
            
            if (response_data.get("Communication") != "FRC_Connect" or 
                response_data.get("ErrorID") != 0):
                self.is_connected = False
                print(f"FRC_Connect error: {response_data}")
                return False
            
            new_port = response_data.get("PortNumber")
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((ip, new_port))
            
            abort_cmd = {"Command":"FRC_Abort"}
            self.socket.send((json.dumps(abort_cmd) + "\r\n").encode('utf-8'))
            
            abort_response = self.socket.recv(1024).decode('utf-8')
            abort_data = json.loads(abort_response)
            
            if (abort_data.get("Command") != "FRC_Abort" or 
                abort_data.get("ErrorID") != 0):
                print(f"FRC_Abort error: {abort_response}")
            
            reset_cmd = {"Command":"FRC_Reset"}
            self.socket.send((json.dumps(reset_cmd) + "\r\n").encode('utf-8'))
            
            reset_response = self.socket.recv(1024).decode('utf-8')
            reset_data = json.loads(reset_response)
            
            if (reset_data.get("Command") != "FRC_Reset" or 
                reset_data.get("ErrorID") != 0):
                print(f"FRC_Reset error: {reset_response}")
            
            init_cmd = {"Command":"FRC_Initialize"}
            self.socket.send((json.dumps(init_cmd) + "\r\n").encode('utf-8'))
            
            init_response = self.socket.recv(1024).decode('utf-8')
            init_data = json.loads(init_response)
            
            if (init_data.get("Command") != "FRC_Initialize" or 
                init_data.get("ErrorID") != 0):
                self.socket.close()
                self.is_connected = False
                print(f"FRC_Initialize error: {init_response}")
                return False
            
            self.ip = ip
            self.port = new_port
            self.is_connected = True
            
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_thread_function)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            
            return True
                
        except Exception as e:
            print(f"Connection error: {e}")
            if hasattr(self, 'socket') and self.socket:
                self.socket.close()
            self.is_connected = False
            return False
    
    def send_message(self, message):
        if not self.is_connected or not self.socket:
            return False
            
        try:
            with self.send_lock:
                msg_str = json.dumps(message) + "\r\n"
                self.socket.send(msg_str.encode('utf-8'))
                return True
        except Exception as e:
            print(f"Send message error: {e}")
            return False
            
    def _receive_thread_function(self):
        buffer = ""
        while self.running and self.is_connected and self.socket:
            try:
                data = self.socket.recv(1024).decode('utf-8')
                if not data:
                    break
                    
                buffer += data
                
                while "\r\n" in buffer:
                    message, buffer = buffer.split("\r\n", 1)
                    if message:
                        try:
                            msg_data = json.loads(message)
                            
                            command = msg_data.get("Command")
                            if command:
                                self.timely_fifo.put(msg_data)
                            elif msg_data.get("Instruction") or msg_data.get("Communication") == "FRC_SystemFault":
                                self.response_fifo.put(msg_data)
                        except json.JSONDecodeError as e:
                            print(f"JSON decode error: {e}")
            
            except Exception as e:
                print(f"Receive thread error: {e}")
                break
    def _clear_queue(self, q):
        while not q.empty():
            try:
                q.get_nowait()
            except queue.Empty:
                continue
        return q
    
    def send_command_wait_response(self, command, timeout=5):
        if not self.is_connected or not self.socket:
            return False
            
        try:
            with self.cmd_lock:
                self._clear_queue(self.timely_fifo)
                self.send_message(command)
                return self.timely_fifo.get(timeout=timeout)
        except Exception as e:
            print(f"Send command error: {e}")
            return None
    
    def disconnect(self):
        try:
            self.running = False
            
            if hasattr(self, 'socket') and self.socket:
                self.socket.close()
                self.socket = None
            
            if hasattr(self, 'receive_thread') and self.receive_thread and self.receive_thread.is_alive():
                self.receive_thread.join(timeout=2.0)
            
            self.is_connected = False
            
            print("Disconnected successfully")
            return True
            
        except Exception as e:
            print(f"Disconnect error: {e}")
            return False
    
    def get_pose(self):
        if not self.is_connected or not self.socket:
            return None
            
        try:
            pose_data = self.send_command_wait_response({"Command":"FRC_ReadCartesianPosition"})
            
            if (pose_data.get("Command") == "FRC_ReadCartesianPosition" and
                pose_data.get("ErrorID") == 0 and
                pose_data.get("Position")):
                position = pose_data.get("Position")
                Configuration = pose_data.get("Configuration")
                self.configuration = Configuration
                return [
                    position.get("X", 0),
                    position.get("Y", 0),
                    position.get("Z", 0),
                    position.get("W", 0),
                    position.get("P", 0),
                    position.get("R", 0)
                ]
            else:
                print(f"Error in pose response: {pose_data}")
                return None
                
        except queue.Empty:
            print("Timeout waiting for pose response")
            return None
        except Exception as e:
            print(f"Get pose error: {e}")
            return None
    
    def get_joint(self):
        if not self.is_connected or not self.socket:
            return None
            
        try:
            joint_data = self.send_command_wait_response({"Command":"FRC_ReadJointAngles"})
            
            if (joint_data.get("Command") == "FRC_ReadJointAngles" and
                joint_data.get("ErrorID") == 0 and
                joint_data.get("JointAngle")):
                joint_angle = joint_data.get("JointAngle")
                return [
                    joint_angle.get("J1", 0),
                    joint_angle.get("J2", 0),
                    joint_angle.get("J3", 0),
                    joint_angle.get("J4", 0),
                    joint_angle.get("J5", 0),
                    joint_angle.get("J6", 0)
                ]
            else:
                print(f"Error in joint response: {joint_data}")
                return None
                
        except queue.Empty:
            print("Timeout waiting for joint response")
            return None
        except Exception as e:
            print(f"Get joint error: {e}")
            return None

    def reset(self):
        if not self.is_connected or not self.socket:
            return False
            
        try:
            abort_data = self.send_command_wait_response({"Command":"FRC_Abort"})
            if not (abort_data.get("Command") == "FRC_Abort" and
                abort_data.get("ErrorID") == 0):
                print(f"Error in abort response: {abort_data}")

            reset_data = self.send_command_wait_response({"Command":"FRC_Reset"})        
            if not (reset_data.get("Command") == "FRC_Reset" and
                reset_data.get("ErrorID") == 0):
                print(f"Error in reset response: {reset_data}")

            init_data = self.send_command_wait_response({"Command":"FRC_Initialize"})
            if not (init_data.get("Command") == "FRC_Initialize" and
                init_data.get("ErrorID") == 0):
                print(f"Error in initialize response: {init_data}")
                return False

            tool_data = self.send_command_wait_response({"Command": "FRC_SetUFrameUTool", "UFrameNumber": 1, "UToolNumber": 1})
            if not (tool_data.get("Command") == "FRC_SetUFrameUTool" and
                tool_data.get("ErrorID") == 0):
                print(f"Error in tool response: {tool_data}")
                return False
            return True
            
        except queue.Empty:
            print("Timeout waiting for reset response")
            return False
        except Exception as e:
            print(f"Reset error: {e}")
            return False

    def _send_motion_stream(
        self,
        instruction,
        points,
        build_payload_fn,
        speed_type,
        speed,
        cnt=100
    ):
        if not points:
            print("_send_motion_stream: empty points")
            return False

        self._clear_queue(self.response_fifo)

        if not isinstance(points[0], (list, tuple)):
            points = [points]

        send_count = 0
        recv_count = 0
        seq = 1
        total = len(points)

        if "JointMotion" in instruction:
            accepted_resp_instructions = ("FRC_JointMotion", "FRC_JointMotionJRep")
        elif "LinearMotion" in instruction:
            accepted_resp_instructions = ("FRC_LinearMotion", "FRC_LinearMotionJRep")
        else:
            accepted_resp_instructions = (instruction,)

        for i, point in enumerate(points):
            term_type = "FINE" if i == total - 1 else "CNT"

            payload = build_payload_fn(point)
            if not isinstance(payload, dict):
                print("_send_motion_stream: build_payload_fn must return dict")
                return False

            msg = {}
            msg["Instruction"] = instruction
            msg["SequenceID"] = seq

            for k, v in payload.items():
                msg[k] = v

            msg["SpeedType"] = speed_type
            msg["Speed"] = speed
            msg["TermType"] = term_type
            msg["TermValue"] = cnt if term_type == "CNT" else 0

            if not self.send_message(msg):
                print(f"Failed to send {instruction} seq={seq}")
                return False

            send_count += 1
            seq += 1

            if send_count - recv_count >= 8:
                try:
                    resp = self.response_fifo.get(timeout=60)
                    recv_count += 1
                    if not (resp.get("Instruction") in accepted_resp_instructions and resp.get("ErrorID") == 0):
                        print(f"{instruction} stream error: {resp}")
                        return False
                except queue.Empty:
                    print(f"Timeout waiting for {instruction} response")
                    return False

        while recv_count < send_count:
            try:
                resp = self.response_fifo.get(timeout=60)
                recv_count += 1
                if not (resp.get("Instruction") in accepted_resp_instructions and resp.get("ErrorID") == 0):
                    print(f"{instruction} final error: {resp}")
                    return False
            except queue.Empty:
                print(f"Timeout waiting for remaining {instruction} responses")
                return False

        return True

    def moveJ(self, type, val, speed=100, cnt=100):
        if not self.is_connected or not self.socket:
            return False

        if self.reset() is False:
            return False

        if type == "joint":
            def build_payload(point):
                return {
                    "JointAngle": {
                        "J1": float(point[0]), "J2": float(point[1]), "J3": float(point[2]),
                        "J4": float(point[3]), "J5": float(point[4]), "J6": float(point[5])
                    }
                }

            return self._send_motion_stream(
                instruction="FRC_JointMotionJRep",
                points=val,
                build_payload_fn=build_payload,
                speed_type="Percent",
                speed=speed,
                cnt=cnt
            )

        elif type == "pose":
            self.get_pose()
            def build_payload(point):
                return {
                    "Configuration": self.configuration,
                    "Position": {
                        "X": float(point[0]), "Y": float(point[1]), "Z": float(point[2]),
                        "W": float(point[3]), "P": float(point[4]), "R": float(point[5])
                    }
                }

            return self._send_motion_stream(
                instruction="FRC_JointMotion",
                points=val,
                build_payload_fn=build_payload,
                speed_type="Percent",
                speed=speed,
                cnt=cnt
            )
        else:
            print("moveJ type must be 'joint' or 'pose'")
            return False

    def moveL(self, type, val, speed=100, cnt=100):
        if not self.is_connected or not self.socket:
            return False

        if self.reset() is False:
            return False

        speed = int(speed * 20)

        if type == "joint":
            def build_payload(point):
                return {
                    "JointAngle": {
                        "J1": float(point[0]), "J2": float(point[1]), "J3": float(point[2]),
                        "J4": float(point[3]), "J5": float(point[4]), "J6": float(point[5])
                    }
                }

            return self._send_motion_stream(
                instruction="FRC_LinearMotionJRep",
                points=val,
                build_payload_fn=build_payload,
                speed_type="mmSec",
                speed=speed,
                cnt=cnt
            )

        elif type == "pose":
            self.get_pose()
            def build_payload(point):
                return {
                    "Configuration": self.configuration,
                    "Position": {
                        "X": float(point[0]), "Y": float(point[1]), "Z": float(point[2]),
                        "W": float(point[3]), "P": float(point[4]), "R": float(point[5])
                    }
                }

            return self._send_motion_stream(
                instruction="FRC_LinearMotion",
                points=val,
                build_payload_fn=build_payload,
                speed_type="mmSec",
                speed=speed,
                cnt=cnt
            )
        else:
            print("moveL type must be 'joint' or 'pose'")
            return False