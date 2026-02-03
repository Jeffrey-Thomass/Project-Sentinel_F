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