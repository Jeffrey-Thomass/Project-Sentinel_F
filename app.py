import numpy as np
from flask import Flask, request, jsonify
from flask_cors import CORS
from ortools.sat.python import cp_model

# --- Initialize Flask App ---
app = Flask(__name__)
# Enable CORS to allow your React frontend to call this API
CORS(app)

# --- Helper Function: Generate Targets ---
# This is from your original script, but adapted for API input
def generate_sub_targets(priority_areas, density):
    """Generates a set of discrete points within the given priority areas."""
    sub_targets = set()
    for area in priority_areas:
        # We read 'center' as a list [x, y] from the JSON
        center_x, center_y = area['center'][0], area['center'][1]
        radius = area['radius']
        
        # Create a grid of points around the circle's bounding box
        step = density
        for x in np.arange(center_x - radius, center_x + radius + step, step):
            for y in np.arange(center_y - radius, center_y + radius + step, step):
                # Check if the point is inside the circle
                if np.sqrt((x - center_x)**2 + (y - center_y)**2) <= radius:
                    # Round to 2 decimal places to avoid floating point issues
                    sub_targets.add((round(x, 2), round(y, 2)))
    
    if not sub_targets:
         print("\nWarning: Could not generate any sub-targets.")
         return []
         
    return list(sub_targets)

# --- Core Optimization Function ---
# This combines your 'main' logic into a single function
def solve_deployment(params):
    """
    Runs the full optimization based on parameters from the web UI.
    """
    print("Starting optimization...")
    
    # 1. Generate Sub-Targets
    print(f"Generating sub-targets with density {params['density']}...")
    sub_targets = generate_sub_targets(params['priority_areas'], params['density'])
    print(f"Generated {len(sub_targets)} unique sub-target points.")

    if not sub_targets:
        return {"error": "No sub-targets generated. Try a smaller density value or larger radius."}

    # 2. Generate Potential Sensor Locations
    locations = []
    grid_step = 1.0 # Hardcoding this for simplicity. You could make it a param.
    for x in np.arange(0, params['width'] + grid_step/2, grid_step):
        for y in np.arange(0, params['height'] + grid_step/2, grid_step):
            locations.append((x,y))

    num_locations = len(locations)
    num_targets = len(sub_targets)
    print(f"Checking {num_locations} potential locations against {num_targets} targets.")

    # 3. Pre-calculate Coverage
    covers = {}
    for i in range(num_locations):
        for j in range(num_targets):
            dist = np.sqrt((locations[i][0] - sub_targets[j][0])**2 + (locations[i][1] - sub_targets[j][1])**2)
            if dist <= params['sensor_range']:
                covers[(i, j)] = 1
            else:
                covers[(i, j)] = 0
                
    # 4. ILP Model
    model = cp_model.CpModel()
    x = [model.NewBoolVar(f'x_{i}') for i in range(num_locations)] # Sensor at location i
    y = [model.NewBoolVar(f'y_{j}') for j in range(num_targets)]   # Target j is covered
    
    # Track covers per target
    num_covers = [model.NewIntVar(0, params['max_sensors'], f'num_covers_{j}') for j in range(num_targets)]

    for j in range(num_targets):
        sensors_covering_target_j = [x[i] for i in range(num_locations) if covers.get((i, j), 0) == 1]
        model.Add(cp_model.LinearExpr.Sum(sensors_covering_target_j) == num_covers[j])

    # Link y[j] to num_covers[j]
    for j in range(num_targets):
        model.Add(num_covers[j] > 0).OnlyEnforceIf(y[j])
        model.Add(num_covers[j] == 0).OnlyEnforceIf(y[j].Not())

    # Constraint: Max number of sensors
    model.Add(cp_model.LinearExpr.Sum(x) <= params['max_sensors'])

    # 5. Objective Function
    total_objective_expression = []
    
    # a) Maximize Coverage
    for j in range(num_targets):
        total_objective_expression.append(y[j] * params['w_coverage'])
        
    # b) Minimize Cost
    for i in range(num_locations):
        total_objective_expression.append(x[i] * -params['w_cost'])
        
    # c) Minimize Overlap
    for j in range(num_targets):
        # We only penalize overlap if the target is covered (avoids negative penalties)
        actual_overlap_count = model.NewIntVar(0, params['max_sensors'], f'actual_overlap_{j}')
        model.Add(actual_overlap_count == num_covers[j] - 1).OnlyEnforceIf(y[j])
        model.Add(actual_overlap_count == 0).OnlyEnforceIf(y[j].Not())
        
        total_objective_expression.append(actual_overlap_count * -params['w_overlap'])

    model.Maximize(cp_model.LinearExpr.Sum(total_objective_expression))

    # 6. Solve
    solver = cp_model.CpSolver()
    solver.parameters.log_search_progress = True
    solver.parameters.max_time_in_seconds = 30.0 # 30-second time limit for the API
    status = solver.Solve(model)

    # 7. Process and Return Results
    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        print("Solution found!")
        placed_sensors_coords = [locations[i] for i in range(num_locations) if solver.Value(x[i]) == 1]
        
        covered_sub_targets = [sub_targets[j] for j in range(num_targets) if solver.Value(y[j]) == 1]
        
        # Get all UNCOVERED targets for visualization
        uncovered_sub_targets = [sub_targets[j] for j in range(num_targets) if solver.Value(y[j]) == 0]
        
        num_deployed = len(placed_sensors_coords)
        num_covered = len(covered_sub_targets)
        total_targets = num_targets

        return {
            "status": solver.StatusName(status),
            "placedSensors": placed_sensors_coords,
            "coveredSubTargets": covered_sub_targets,
            "uncoveredSubTargets": uncovered_sub_targets,
            "summary": f"Deployed {num_deployed} sensors to cover {num_covered} / {total_targets} targets."
        }
    else:
        print("No solution found.")
        return {"error": f"Solver status: {solver.StatusName(status)}. No solution found."}

# --- API Endpoint ---
@app.route('/api/optimize', methods=['POST'])
def optimize_api():
    try:
        # Get parameters from the React frontend
        data = request.json
        
        # --- Parameter Validation ---
        params = {
            'width': float(data['width']),
            'height': float(data['height']),
            'priority_areas': data['priority_areas'], # List of {'center': [x,y], 'radius': r}
            'density': float(data['density']),
            'max_sensors': int(data['max_sensors']),
            'sensor_range': float(data['sensor_range']),
            # Fixed weights
            'w_coverage': 1000,
            'w_overlap': 50,
            'w_cost': 10
        }
        
        # Run the optimization
        results = solve_deployment(params)
        
        if "error" in results:
            return jsonify(results), 400
            
        return jsonify(results)

    except Exception as e:
        print(f"An error occurred: {e}")
        return jsonify({"error": str(e)}), 500

# --- Run the App ---
if __name__ == '__main__':
    # Runs on http://127.0.0.1:5000
    app.run(debug=True, port=5000)