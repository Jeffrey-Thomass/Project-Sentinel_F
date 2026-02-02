# import numpy as np
# from flask import Flask, request, jsonify
# from flask_cors import CORS
# from ortools.sat.python import cp_model

# # --- Initialize Flask App ---
# app = Flask(__name__)
# # Enable CORS to allow your React frontend to call this API
# CORS(app)

# # --- Helper Function: Generate Targets ---
# # This is from your original script, but adapted for API input
# def generate_sub_targets(priority_areas, density):
#     """Generates a set of discrete points within the given priority areas."""
#     sub_targets = set()
#     for area in priority_areas:
#         # We read 'center' as a list [x, y] from the JSON
#         center_x, center_y = area['center'][0], area['center'][1]
#         radius = area['radius']
        
#         # Create a grid of points around the circle's bounding box
#         step = density
#         for x in np.arange(center_x - radius, center_x + radius + step, step):
#             for y in np.arange(center_y - radius, center_y + radius + step, step):
#                 # Check if the point is inside the circle
#                 if np.sqrt((x - center_x)**2 + (y - center_y)**2) <= radius:
#                     # Round to 2 decimal places to avoid floating point issues
#                     sub_targets.add((round(x, 2), round(y, 2)))
    
#     if not sub_targets:
#          print("\nWarning: Could not generate any sub-targets.")
#          return []
         
#     return list(sub_targets)

# # --- Core Optimization Function ---
# # This combines your 'main' logic into a single function
# def solve_deployment(params):
#     """
#     Runs the full optimization based on parameters from the web UI.
#     """
#     print("Starting optimization...")
    
#     # 1. Generate Sub-Targets
#     print(f"Generating sub-targets with density {params['density']}...")
#     sub_targets = generate_sub_targets(params['priority_areas'], params['density'])
#     print(f"Generated {len(sub_targets)} unique sub-target points.")

#     if not sub_targets:
#         return {"error": "No sub-targets generated. Try a smaller density value or larger radius."}

#     # 2. Generate Potential Sensor Locations
#     locations = []
#     grid_step = 1.0 # Hardcoding this for simplicity. You could make it a param.
#     for x in np.arange(0, params['width'] + grid_step/2, grid_step):
#         for y in np.arange(0, params['height'] + grid_step/2, grid_step):
#             locations.append((x,y))

#     num_locations = len(locations)
#     num_targets = len(sub_targets)
#     print(f"Checking {num_locations} potential locations against {num_targets} targets.")

#     # 3. Pre-calculate Coverage
#     covers = {}
#     for i in range(num_locations):
#         for j in range(num_targets):
#             dist = np.sqrt((locations[i][0] - sub_targets[j][0])**2 + (locations[i][1] - sub_targets[j][1])**2)
#             if dist <= params['sensor_range']:
#                 covers[(i, j)] = 1
#             else:
#                 covers[(i, j)] = 0
                
#     # 4. ILP Model
#     model = cp_model.CpModel()
#     x = [model.NewBoolVar(f'x_{i}') for i in range(num_locations)] # Sensor at location i
#     y = [model.NewBoolVar(f'y_{j}') for j in range(num_targets)]   # Target j is covered
    
#     # Track covers per target
#     num_covers = [model.NewIntVar(0, params['max_sensors'], f'num_covers_{j}') for j in range(num_targets)]

#     for j in range(num_targets):
#         sensors_covering_target_j = [x[i] for i in range(num_locations) if covers.get((i, j), 0) == 1]
#         model.Add(cp_model.LinearExpr.Sum(sensors_covering_target_j) == num_covers[j])

#     # Link y[j] to num_covers[j]
#     for j in range(num_targets):
#         model.Add(num_covers[j] > 0).OnlyEnforceIf(y[j])
#         model.Add(num_covers[j] == 0).OnlyEnforceIf(y[j].Not())

#     # Constraint: Max number of sensors
#     model.Add(cp_model.LinearExpr.Sum(x) <= params['max_sensors'])

#     # 5. Objective Function
#     total_objective_expression = []
    
#     # a) Maximize Coverage
#     for j in range(num_targets):
#         total_objective_expression.append(y[j] * params['w_coverage'])
        
#     # b) Minimize Cost
#     for i in range(num_locations):
#         total_objective_expression.append(x[i] * -params['w_cost'])
        
#     # c) Minimize Overlap
#     for j in range(num_targets):
#         # We only penalize overlap if the target is covered (avoids negative penalties)
#         actual_overlap_count = model.NewIntVar(0, params['max_sensors'], f'actual_overlap_{j}')
#         model.Add(actual_overlap_count == num_covers[j] - 1).OnlyEnforceIf(y[j])
#         model.Add(actual_overlap_count == 0).OnlyEnforceIf(y[j].Not())
        
#         total_objective_expression.append(actual_overlap_count * -params['w_overlap'])

#     model.Maximize(cp_model.LinearExpr.Sum(total_objective_expression))

#     # 6. Solve
#     solver = cp_model.CpSolver()
#     solver.parameters.log_search_progress = True
#     solver.parameters.max_time_in_seconds = 30.0 # 30-second time limit for the API
#     status = solver.Solve(model)

#     # 7. Process and Return Results
#     if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
#         print("Solution found!")
#         placed_sensors_coords = [locations[i] for i in range(num_locations) if solver.Value(x[i]) == 1]
        
#         covered_sub_targets = [sub_targets[j] for j in range(num_targets) if solver.Value(y[j]) == 1]
        
#         # Get all UNCOVERED targets for visualization
#         uncovered_sub_targets = [sub_targets[j] for j in range(num_targets) if solver.Value(y[j]) == 0]
        
#         num_deployed = len(placed_sensors_coords)
#         num_covered = len(covered_sub_targets)
#         total_targets = num_targets

#         return {
#             "status": solver.StatusName(status),
#             "placedSensors": placed_sensors_coords,
#             "coveredSubTargets": covered_sub_targets,
#             "uncoveredSubTargets": uncovered_sub_targets,
#             "summary": f"Deployed {num_deployed} sensors to cover {num_covered} / {total_targets} targets."
#         }
#     else:
#         print("No solution found.")
#         return {"error": f"Solver status: {solver.StatusName(status)}. No solution found."}

# # --- API Endpoint ---
# @app.route('/api/optimize', methods=['POST'])
# def optimize_api():
#     try:
#         # Get parameters from the React frontend
#         data = request.json
        
#         # --- Parameter Validation ---
#         params = {
#             'width': float(data['width']),
#             'height': float(data['height']),
#             'priority_areas': data['priority_areas'], # List of {'center': [x,y], 'radius': r}
#             'density': float(data['density']),
#             'max_sensors': int(data['max_sensors']),
#             'sensor_range': float(data['sensor_range']),
#             # Fixed weights
#             'w_coverage': 1000,
#             'w_overlap': 50,
#             'w_cost': 10
#         }
        
#         # Run the optimization
#         results = solve_deployment(params)
        
#         if "error" in results:
#             return jsonify(results), 400
            
#         return jsonify(results)

#     except Exception as e:
#         print(f"An error occurred: {e}")
#         return jsonify({"error": str(e)}), 500

# # --- Run the App ---
# if __name__ == '__main__':
#     # Runs on http://127.0.0.1:5000
#     app.run(debug=True, port=5000)


import numpy as np
import os
import traceback
from flask import Flask, request, jsonify, send_from_directory
from flask.json.provider import DefaultJSONProvider
from flask_cors import CORS
from ortools.sat.python import cp_model

# --- 1. Fix for Numpy JSON Serialization ---
class NumpyJSONProvider(DefaultJSONProvider):
    def default(self, obj):
        if isinstance(obj, (np.integer, np.floating)):
            return float(obj) if isinstance(obj, np.floating) else int(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return super().default(obj)

# --- Initialize Flask App ---
# REMOVED: static_url_path='' (This was causing the 405 error)
app = Flask(__name__)
app.json = NumpyJSONProvider(app) 
CORS(app)

# --- Logger Class ---
class OptimizationLogger:
    def __init__(self):
        self.logs = []
    def log(self, message):
        print(message) 
        self.logs.append(message)

# --- Helper Function: Generate Targets ---
def generate_sub_targets(priority_areas, density, logger):
    sub_targets = set()
    for area in priority_areas:
        center_x = float(area['center'][0])
        center_y = float(area['center'][1])
        radius = float(area['radius'])
        step = float(density)
        
        # Use numpy for range, then explicit cast
        x_range = np.arange(center_x - radius, center_x + radius + step, step)
        y_range = np.arange(center_y - radius, center_y + radius + step, step)
        
        for x in x_range:
            for y in y_range:
                xf, yf = float(x), float(y)
                if np.sqrt((xf - center_x)**2 + (yf - center_y)**2) <= radius:
                    sub_targets.add((round(xf, 2), round(yf, 2)))
    
    if not sub_targets:
         logger.log("Warning: Could not generate sub-targets. Check density/radius.")
         return []
         
    return list(sub_targets)

# --- Core Optimization Function ---
def solve_deployment(params):
    logger = OptimizationLogger()
    logger.log("--- INITIALIZING MISSION ---")
    
    # 1. Generate Sub-Targets
    logger.log(f"Generating sub-targets (Density={params['density']})...")
    sub_targets = generate_sub_targets(params['priority_areas'], params['density'], logger)
    logger.log(f"Target Acquisition: {len(sub_targets)} unique points identified.")

    if not sub_targets:
        return {"error": "No sub-targets generated.", "logs": logger.logs}

    # 2. Generate Potential Locations
    locations = []
    grid_step = 1.0 
    for x in np.arange(0, params['width'] + grid_step/2, grid_step):
        for y in np.arange(0, params['height'] + grid_step/2, grid_step):
            locations.append((float(x), float(y)))

    num_locations = len(locations)
    num_targets = len(sub_targets)
    logger.log(f"Grid Analysis: {num_locations} potential sites vs {num_targets} targets.")

    if num_targets > 500:
        logger.log("WARNING: High target count. Optimization may take longer.")

    # 3. Pre-calculate Coverage
    logger.log("Calculating Line-of-Sight matrix...")
    covers = {}
    range_sq = params['sensor_range'] ** 2 
    
    for i in range(num_locations):
        lx, ly = locations[i]
        for j in range(num_targets):
            tx, ty = sub_targets[j]
            dist_sq = (lx - tx)**2 + (ly - ty)**2
            if dist_sq <= range_sq:
                covers[(i, j)] = 1
                
    # 4. ILP Model
    logger.log("Building Constraint Programming Model...")
    model = cp_model.CpModel()
    x = [model.NewBoolVar(f'x_{i}') for i in range(num_locations)] 
    y = [model.NewBoolVar(f'y_{j}') for j in range(num_targets)]   
    
    num_covers = [model.NewIntVar(0, params['max_sensors'], f'nc_{j}') for j in range(num_targets)]

    for j in range(num_targets):
        sensors_covering_j = [x[i] for i in range(num_locations) if covers.get((i, j), 0) == 1]
        if sensors_covering_j:
            model.Add(cp_model.LinearExpr.Sum(sensors_covering_j) == num_covers[j])
        else:
            model.Add(num_covers[j] == 0)

    for j in range(num_targets):
        model.Add(num_covers[j] > 0).OnlyEnforceIf(y[j])
        model.Add(num_covers[j] == 0).OnlyEnforceIf(y[j].Not())

    model.Add(cp_model.LinearExpr.Sum(x) <= params['max_sensors'])

    # Objective
    total_obj = []
    for j in range(num_targets):
        total_obj.append(y[j] * params['w_coverage'])
        
    for i in range(num_locations):
        total_obj.append(x[i] * -params['w_cost'])
        
    for j in range(num_targets):
        actual_overlap = model.NewIntVar(0, params['max_sensors'], f'o_{j}')
        model.Add(actual_overlap == num_covers[j] - 1).OnlyEnforceIf(y[j])
        model.Add(actual_overlap == 0).OnlyEnforceIf(y[j].Not())
        total_obj.append(actual_overlap * -params['w_overlap'])

    model.Maximize(cp_model.LinearExpr.Sum(total_obj))

    # 5. Solve
    logger.log("Engaging Solver...")
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 30.0
    status = solver.Solve(model)

    # 6. Process Results
    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        status_name = solver.StatusName(status)
        logger.log(f"Optimization Complete: {status_name}")
        
        placed = [locations[i] for i in range(num_locations) if solver.Value(x[i]) == 1]
        covered = [sub_targets[j] for j in range(num_targets) if solver.Value(y[j]) == 1]
        uncovered = [sub_targets[j] for j in range(num_targets) if solver.Value(y[j]) == 0]
        
        logger.log(f"DEPLOYMENT SUMMARY: {len(placed)} sensors placed.")
        
        return {
            "status": status_name,
            "placedSensors": placed,
            "coveredSubTargets": covered,
            "uncoveredSubTargets": uncovered,
            "summary": f"Deployed {len(placed)} sensors to cover {len(covered)} targets.",
            "logs": logger.logs
        }
    else:
        logger.log(f"Optimization Failed: {solver.StatusName(status)}")
        return {"error": "No solution found.", "logs": logger.logs}

# --- ROUTES ---

@app.route('/')
def index():
    # Explicitly serve the index.html file from the current directory
    return send_from_directory('.', 'index.html')

@app.route('/api/optimize', methods=['POST'])
def optimize_api():
    try:
        data = request.json
        params = {
            'width': float(data.get('width', 50)),
            'height': float(data.get('height', 50)),
            'priority_areas': data.get('priority_areas', []),
            'density': float(data.get('density', 2.0)),
            'max_sensors': int(data.get('max_sensors', 10)),
            'sensor_range': float(data.get('sensor_range', 7)),
            'w_coverage': 1000, 
            'w_overlap': 50, 
            'w_cost': 10
        }
        
        results = solve_deployment(params)
        
        if "error" in results:
            return jsonify(results), 400
            
        return jsonify(results)
        
    except Exception as e:
        # This prints the actual error to your terminal for debugging
        print("--- SERVER ERROR ---")
        traceback.print_exc() 
        return jsonify({"error": str(e), "logs": ["Critical Server Error. Check Terminal."]}), 500

if __name__ == '__main__':
    # Runs on http://127.0.0.1:5000
    app.run(debug=True, port=5000)