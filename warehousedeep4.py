
from controller import Robot
from collections import deque
import time

# ---------------------------
# Database
# ---------------------------
class WarehouseDatabase:
    def __init__(self):
        # Node definitions with coordinates and connections
        self.nodes = {
            'N1':  {'x': 5.95, 'y': 6, 'type': 'start', 'connections': ['N2']},
            'N2':  {'x': 3, 'y': 6, 'type': 'intersection', 'connections': ['N1', 'N3', 'N10']},
            'N3':  {'x': 3, 'y': 3, 'type': 'intersection', 'connections': ['N2', 'N4', 'N7']},
            'N4':  {'x': 3, 'y': 0, 'type': 'intersection', 'connections': ['N3', 'N5', 'N8']},
            'N5':  {'x': 3, 'y': -3, 'type': 'intersection', 'connections': ['N4', 'N6', 'N9']},
            'N6':  {'x': 3, 'y': -6, 'type': 'intersection', 'connections': ['N5', 'N26','N14']},
            'N7':  {'x': 2, 'y': 3, 'type': 'item_pickup', 'connections': ['N3']},
            'N8':  {'x': 2, 'y': 0, 'type': 'item_pickup', 'connections': ['N4']},
            'N9':  {'x': 2, 'y': -3, 'type': 'item_pickup', 'connections': ['N5']},
            'N10': {'x': 0, 'y': 6, 'type': 'intersection', 'connections': ['N2', 'N11', 'N18']},
            'N11': {'x': 0, 'y': 3, 'type': 'intersection', 'connections': ['N10', 'N12', 'N15']},
            'N12': {'x': 0, 'y': 0, 'type': 'intersection', 'connections': ['N11', 'N13', 'N16']},
            'N13': {'x': 0, 'y': -3, 'type': 'intersection', 'connections': ['N12', 'N14', 'N17']},
            'N14': {'x': 0, 'y': -6, 'type': 'intersection', 'connections': ['N13', 'N6', 'N22']},
            'N15': {'x': -1, 'y': 3, 'type': 'item_pickup', 'connections': ['N11']},
            'N16': {'x': -1, 'y': 0, 'type': 'item_pickup', 'connections': ['N12']},
            'N17': {'x': -1, 'y': -3, 'type': 'item_pickup', 'connections': ['N13']},
            'N18': {'x': -3, 'y': 6, 'type': 'intersection', 'connections': ['N10', 'N19']},
            'N19': {'x': -3, 'y': 3, 'type': 'intersection', 'connections': ['N18', 'N20', 'N23']},
            'N20': {'x': -3, 'y': 0, 'type': 'intersection', 'connections': ['N19', 'N21', 'N24']},
            'N21': {'x': -3, 'y': -3, 'type': 'intersection', 'connections': ['N20', 'N22', 'N25']},
            'N22': {'x': -3, 'y': -6, 'type': 'intersection', 'connections': ['N21', 'N14']},
            'N23': {'x': -4, 'y': 3, 'type': 'item_pickup', 'connections': ['N19']},
            'N24': {'x': -4, 'y': 0, 'type': 'item_pickup', 'connections': ['N20']},
            'N25': {'x': -4, 'y': -3, 'type': 'item_pickup', 'connections': ['N21']},
            'N26': {'x': 6, 'y': -6, 'type': 'drop_zone', 'connections': ['N6']}
        }

        self.items = {
            'ITEM_A': {'location': 'N7', 'picked': False},
            'ITEM_B': {'location': 'N8', 'picked': False},
            'ITEM_C': {'location': 'N9', 'picked': False},
            'ITEM_D': {'location': 'N15', 'picked': False},
            'ITEM_E': {'location': 'N16', 'picked': False},
            'ITEM_F': {'location': 'N17', 'picked': False},
            'ITEM_G': {'location': 'N23', 'picked': False},
            'ITEM_H': {'location': 'N24', 'picked': False},
            'ITEM_I': {'location': 'N25', 'picked': False}
        }

    def find_path(self, start, end):
        """BFS shortest path including start and end"""
        if start not in self.nodes or end not in self.nodes:
            return None

        queue = deque([(start, [start])])
        visited = set([start])

        while queue:
            current, path = queue.popleft()
            if current == end:
                return path
            for neighbor in self.nodes[current]['connections']:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        return None

    def get_item_path(self, item_code, current_position):
        if item_code not in self.items:
            return None
        item_node = self.items[item_code]['location']
        return self.find_path(current_position, item_node)

    def get_dropoff_path(self, current_position):
        return self.find_path(current_position, 'N26')

    def get_return_path(self, current_position):
        return self.find_path(current_position, 'N1')

# ---------------------------
# Sensor Manager
# ---------------------------
class SensorManager:
    def __init__(self, robot, timestep):
        self.robot = robot
        self.timestep = timestep

        # Initialize 11 IR sensors D0..D10
        self.ir_sensors = []
        for i in range(11):
            name = f'D{i}'
            dev = robot.getDevice(name)
            dev.enable(self.timestep)
            self.ir_sensors.append(dev)

        # Camera
        try:
            self.camera = robot.getDevice('camera')
            self.camera.enable(self.timestep)
        except Exception:
            self.camera = None

    def read_line_sensors(self):
        values = []
        for sensor in self.ir_sensors:
            raw = sensor.getValue()
            # threshold tuned for typical IR range in Webots; adjust if needed
            normalized = 1 if raw < 800 else 0
            values.append(normalized)
        return values

    def detect_intersection(self):
        sensor_values = self.read_line_sensors()
        
        # Count sensors on line
        on_line_count = sum(1 for v in sensor_values if v == 0)
        
        # FULL INTERSECTION - most sensors detect line
        if on_line_count >= 8:
            return 'FULL_INTERSECTION'
            
        # Check for specific intersection patterns
        left_side = sensor_values[0:3]  # D0, D1, D2
        right_side = sensor_values[8:11] # D8, D9, D10
        center = sensor_values[4:7]     # D4, D5, D6
        
        # T-INTERSECTION - both sides detect line
        if (sum(left_side) == 0 and sum(right_side) == 0 and 
            sum(center) <= 1):  # Center might be partially on line
            return 'T_INTERSECTION'
            
        # LEFT TURN - left side detects line strongly
        if (sum(left_side) == 0 and  # All left sensors on line
            sum(right_side) >= 2 and  # Most right sensors off line
            sum(center[:2]) == 0):    # Left-center sensors on line
            return 'LEFT_INTERSECTION'
            
        # RIGHT TURN - right side detects line strongly  
        if (sum(right_side) == 0 and  # All right sensors on line
            sum(left_side) >= 2 and   # Most left sensors off line
            sum(center[1:]) == 0):    # Right-center sensors on line
            return 'RIGHT_INTERSECTION'
            
        return 'NO_INTERSECTION'

    def get_line_position(self):
        sensor_values = self.read_line_sensors()
        total = 0
        weight_sum = 0
        for i, v in enumerate(sensor_values):
            if v == 0:  # on line
                weight = i - 5  # Center weight at sensor 5
                total += weight
                weight_sum += 1
        if weight_sum == 0:
            return None
        return total / weight_sum

    def detect_item_color(self, timeout=5.0):
        """Simple central-pixel red detection. Returns 'red' or None."""
        if not self.camera:
            return None
        start = self.robot.getTime()
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        cx = width // 2
        cy = height // 2
        
        while self.robot.getTime() - start < timeout:
            img = self.camera.getImage()
            # get central pixel RGB
            try:
                r = self.camera.imageGetRed(img, width, cx, cy)
                g = self.camera.imageGetGreen(img, width, cx, cy)
                b = self.camera.imageGetBlue(img, width, cx, cy)
            except Exception:
                return None

            # simple threshold for red
            if r > 180 and g < 120 and b < 120:
                return 'red'

            # step simulation while waiting
            self.robot.step(self.timestep)
        return None

# ---------------------------
# Motion Controller (Mecanum)
# ---------------------------
class MotionController:
    def __init__(self, robot, timestep, sensor_manager):
        self.robot = robot
        self.timestep = timestep
        self.sensor_manager = sensor_manager

        # Use wheel names from option B
        wheel_names = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'back_left_wheel_joint',
            'back_right_wheel_joint'
        ]
        self.motors = []
        for name in wheel_names:
            m = robot.getDevice(name)
            m.setPosition(float('inf'))
            m.setVelocity(0.0)
            self.motors.append(m)

        # PID for line following
        self.kp = 0.8    #0.8
        self.ki = 0.002
        self.kd = 0.1
        self.integral = 0.0
        self.previous_error = 0.0

        self.base_speed = 3.0
        self.max_speed = 6.0

    def clamp(self, v):
        return max(min(v, self.max_speed), -self.max_speed)

    def follow_line(self, line_position):
        if line_position is None:
            self.stop()
            return False
        error = line_position
        self.integral += error
        derivative = error - self.previous_error
        correction = self.kp * error + self.ki * self.integral + self.kd * derivative

        vx = 0.0  # strafe
        vy = self.base_speed
        omega = -correction*0.1

        # mecanum kinematics (simplified)
        w1 = vy - vx - omega  # front-left
        w2 = vy + vx + omega  # front-right
        w3 = vy + vx - omega  # back-left
        w4 = vy - vx + omega  # back-right

        # clamp and set
        self.motors[0].setVelocity(self.clamp(w1))
        self.motors[1].setVelocity(self.clamp(w2))
        self.motors[2].setVelocity(self.clamp(w3))
        self.motors[3].setVelocity(self.clamp(w4))

        self.previous_error = error
        return True

    def set_mecanum_velocity(self, vx, vy, omega):
        w1 = vy - vx - omega
        w2 = vy + vx + omega
        w3 = vy + vx - omega
        w4 = vy - vx + omega
        self.motors[0].setVelocity(self.clamp(w1))
        self.motors[1].setVelocity(self.clamp(w2))
        self.motors[2].setVelocity(self.clamp(w3))
        self.motors[3].setVelocity(self.clamp(w4))

    def stop(self):
        for m in self.motors:
            m.setVelocity(0.0)

    def execute_smart_turn(self, turn_type):
        print(f"Executing {turn_type} turn...")
        
        if turn_type == 'STRAIGHT':
            # Just move forward a bit to clear intersection
            self.set_mecanum_velocity(0, 2.0, 0)
            start = self.robot.getTime()
            while self.robot.getTime() - start < 0.4:
                self.robot.step(self.timestep)
            self.stop()
            return
            
        elif turn_type == 'LEFT':
            # Rotate 90 degrees counter-clockwise
            self.set_mecanum_velocity(0, 0, 2.0)  # CCW rotation
            rotation_time = 16.5  # Adjust this based on your robot's rotation speed
            
        elif turn_type == 'RIGHT':
            # Rotate 90 degrees clockwise  
            self.set_mecanum_velocity(0, 0, -2.0)  # CW rotation
            rotation_time = 16.5  # Adjust this based on your robot's rotation speed
            
        elif turn_type == 'BACK':
            # Rotate 180 degrees
            self.set_mecanum_velocity(0, 0, 2.0)  # CCW rotation
            rotation_time = 32  # Double the time for 180 degrees
            
        else:
            self.stop()
            return

        # Execute rotation for fixed time
        start_time = self.robot.getTime()
        while self.robot.getTime() - start_time < rotation_time:
            self.robot.step(self.timestep)
            
        self.stop()
        
        # Small forward movement to align with new line
        self.set_mecanum_velocity(0, 1.5, 0)
        start_time = self.robot.getTime()
        while self.robot.getTime() - start_time < 2:
            self.robot.step(self.timestep)
        self.stop()

# ---------------------------
# Navigation System
# ---------------------------
class NavigationSystem:
    def __init__(self, database, sensor_manager, motion_controller, robot, timestep):
        self.database = database
        self.sensors = sensor_manager
        self.motion = motion_controller
        self.robot = robot
        self.timestep = timestep

        self.current_node = 'N1'
        self.current_path = []
        self.current_mission = None
        self.state = 'IDLE'
        self.intersection_detected = False
        
        # Cooldown system - using step counter approach
        self.cooldown_counter = 0
        self.cooldown_steps = 80 # Adjust based on timestep (e.g., 32ms × 80 ≈ 2.5 seconds)
        
        # Direction tracking
        self.current_direction = 'SOUTH'  # Robot starts facing -X (South)

    def start_mission(self, item_code):
        if item_code not in self.database.items:
            print(f" Unknown item: {item_code}")
            return False
        path_to_item = self.database.get_item_path(item_code, self.current_node)
        if not path_to_item:
            print(" No path to item")
            return False
        self.current_mission = {
            'item_code': item_code,
            'target_node': self.database.items[item_code]['location'],
            'path_to_item': path_to_item[:],
            'state': 'GOING_TO_ITEM'
        }
        self.current_path = path_to_item[:]
        self.state = 'NAVIGATING'
        print(f" Mission started: {item_code}")
        print(f" Path: {' - '.join(self.current_path)}")
        return True

    def update_navigation(self):
        if self.state != 'NAVIGATING':
            return
        
        # Handle cooldown period
        if self.cooldown_counter > 0:
            self.cooldown_counter -= 1
            # Continue line following but skip intersection detection
            line_pos = self.sensors.get_line_position()
            self.motion.follow_line(line_pos)
            
            # Debug: Show cooldown status occasionally
            if self.cooldown_counter % 20 == 0:
                print(f"Cooldown: {self.cooldown_counter} steps remaining")
            return
        
        # Normal operation - line following with intersection detection
        line_pos = self.sensors.get_line_position()
        self.motion.follow_line(line_pos)
        
        # Intersection detection (only when NOT in cooldown)
        inter = self.sensors.detect_intersection()
        if inter != 'NO_INTERSECTION' and not self.intersection_detected:
            self.intersection_detected = True
            self.handle_intersection(inter)
        elif inter == 'NO_INTERSECTION':
            self.intersection_detected = False

    def handle_intersection(self, inter_type):
        if not self.current_path or len(self.current_path) < 2:
            return
        
        # START COOLDOWN PERIOD
        self.cooldown_counter = self.cooldown_steps
        print(f"Intersection detected - Starting cooldown ({self.cooldown_steps} steps)")
        
        # Determine next node from path
        if self.current_node == self.current_path[0]:
            next_node = self.current_path[1]
        else:
            next_node = self.current_path[0]

        action = self.calculate_turn_action(self.current_node, next_node)
        print(f"Intersection type: {inter_type}. Action: {action} | From {self.current_node} to {next_node}")
        print(f"Current direction: {self.current_direction}")

        # Stop and execute turn
        self.motion.stop()
        
        # Small wait so robot entirely inside intersection before turning
        start = self.robot.getTime()
        while self.robot.getTime() - start < 0.15:
            self.robot.step(self.timestep)
            
        # Execute turn
        self.motion.execute_smart_turn(action)
        
        # Update direction after turn
        self.update_direction_after_turn(action)

        # Update current_node and current_path
        if self.current_node == self.current_path[0]:
            self.current_path.pop(0)
            if self.current_path:
                self.current_node = self.current_path[0]
        else:
            self.current_node = self.current_path[0]
            self.current_path.pop(0)

        print(f" Now at node: {self.current_node} | Direction: {self.current_direction}")
        print(f" Remaining path: {' - '.join(self.current_path) if self.current_path else 'none'}")

        # Check if destination reached
        if self.current_mission and self.current_mission['target_node'] == self.current_node:
            self.handle_destination_reached()

    def calculate_turn_action(self, current, next_node):
        current_data = self.database.nodes[current]
        next_data = self.database.nodes[next_node]
        dx = next_data['x'] - current_data['x']
        dy = next_data['y'] - current_data['y']
        
        print(f"Movement: dx={dx}, dy={dy}")
        
        # Determine required movement direction based on coordinate changes
        if abs(dx) > abs(dy):
            # Horizontal movement (X-axis dominant)
            if dx > 0:
                required_direction = 'NORTH'  # +X
            else:
                required_direction = 'SOUTH'  # -X
        else:
            # Vertical movement (Y-axis dominant)
            if dy > 0:
                required_direction = 'WEST'   # +Y
            else:
                required_direction = 'EAST'   # -Y
        
        # Calculate turn based on current direction and required direction
        return self.get_turn_from_directions(self.current_direction, required_direction)

    def get_turn_from_directions(self, current_dir, required_dir):
        """Calculate turn action based on current and required direction"""
        direction_map = {
            'NORTH': 0,
            'EAST': 1, 
            'SOUTH': 2,
            'WEST': 3
        }
        
        current_idx = direction_map[current_dir]
        required_idx = direction_map[required_dir]
        
        diff = (required_idx - current_idx) % 4
        
        if diff == 0:
            return 'STRAIGHT'
        elif diff == 1:
            return 'RIGHT'
        elif diff == 2:
            return 'BACK'
        elif diff == 3:
            return 'LEFT'

    def update_direction_after_turn(self, turn_action):
        """Update current direction after executing a turn"""
        direction_cycle = ['NORTH', 'EAST', 'SOUTH', 'WEST']
        current_idx = direction_cycle.index(self.current_direction)
        
        if turn_action == 'STRAIGHT':
            # Direction remains the same
            pass
        elif turn_action == 'RIGHT':
            self.current_direction = direction_cycle[(current_idx + 1) % 4]
        elif turn_action == 'BACK':
            self.current_direction = direction_cycle[(current_idx + 2) % 4]
        elif turn_action == 'LEFT':
            self.current_direction = direction_cycle[(current_idx + 3) % 4]

    def handle_destination_reached(self):
        if not self.current_mission:
            return
            
        if self.current_mission['state'] == 'GOING_TO_ITEM':
            print(f"Reached item node {self.current_node}")
            self.wait_for_item_confirmation()
        elif self.current_mission['state'] == 'GOING_TO_DROP':
            print(" Reached drop zone N26")
            self.complete_mission()

    def wait_for_item_confirmation(self):
        print(" Waiting for camera confirmation...")
        detected = self.sensors.detect_item_color(timeout=5.0)
        if detected == 'red':
            print(" Item color RED detected - proceeding to drop zone N26")
            self.proceed_to_drop_zone()
        else:
            print(" Item color not detected within timeout. Returning to start.")
            self.return_to_start()

    def proceed_to_drop_zone(self):
        path_to_drop = self.database.get_dropoff_path(self.current_node)
        if path_to_drop:
            self.current_path = path_to_drop[:]
            self.current_mission['state'] = 'GOING_TO_DROP'
            print(f" Path to drop: {' - '.join(self.current_path)}")
        else:
            print(" Cannot find path to drop zone. Returning home.")
            self.return_to_start()

    def return_to_start(self):
        path_to_start = self.database.get_return_path(self.current_node)
        if path_to_start:
            self.current_path = path_to_start[:]
            self.current_mission = None
            self.state = 'RETURNING_HOME'
            print(f" Returning home: {'-'.join(self.current_path)}")
        else:
            print(" Cannot find path back to start.")

    def complete_mission(self):
        item_code = self.current_mission['item_code']
        self.database.items[item_code]['picked'] = True
        print(f" Mission completed: {item_code} delivered!")
        self.return_to_start()

# ---------------------------
# Main Controller
# ---------------------------
class MainController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.database = WarehouseDatabase()
        self.sensors = SensorManager(self.robot, self.timestep)
        self.motion = MotionController(self.robot, self.timestep, self.sensors)
        self.navigation = NavigationSystem(self.database, self.sensors, self.motion, self.robot, self.timestep)

        # Test sequence
        self.test_items = ['ITEM_A','ITEM_B','ITEM_C','ITEM_D','ITEM_E','ITEM_F','ITEM_G','ITEM_H','ITEM_I']
        self.current_test_index = 0

        print(" Fabtino Warehouse Robot Initialized")
        print(" Ready for missions")

    def run(self):
        # Get user input for item
        try:
            user_input = 'ITEM_F'
        except Exception:
            user_input = 'ITEM_A'  # Default for testing

        if user_input and user_input in self.database.items:
            print(f"Starting single mission for {user_input}")
            if self.navigation.start_mission(user_input):
                print("Mission started successfully")
            else:
                print("Failed to start mission")
                return
        elif user_input:
            print(f"Unknown item code: {user_input}. Using auto sequence.")
            user_input = ''

        # Main simulation loop
        while self.robot.step(self.timestep) != -1:
            # Auto sequence if no user input
            if not user_input and self.navigation.state in ['IDLE', 'RETURNING_HOME']:
                if self.navigation.current_mission is None:
                    # Find next unpicked item
                    while (self.current_test_index < len(self.test_items) and 
                           self.database.items[self.test_items[self.current_test_index]]['picked']):
                        self.current_test_index += 1
                    
                    if self.current_test_index < len(self.test_items):
                        next_item = self.test_items[self.current_test_index]
                        started = self.navigation.start_mission(next_item)
                        if started:
                            print(f"Auto-started mission for {next_item}")
                            self.current_test_index += 1
                        else:
                            print(f"Failed to start mission for {next_item}")
                            self.current_test_index += 1
                    else:
                        print('All test missions completed!')
                        break

            # Update navigation
            self.navigation.update_navigation()
            
            # Check if returned home after mission
            if (self.navigation.state == 'RETURNING_HOME' and 
                self.navigation.current_node == 'N1' and 
                not self.navigation.current_path):
                print("Successfully returned to N1")
                self.navigation.state = 'IDLE'

# ---------------------------
# Entry point
# ---------------------------
if __name__ == '__main__':
    controller = MainController()
    controller.run()