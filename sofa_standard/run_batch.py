import Sofa
import Sofa.Simulation
import SofaRuntime
import os
import sys
import importlib.util

# Set SOFA_ROOT if not set
if "SOFA_ROOT" not in os.environ:
    os.environ["SOFA_ROOT"] = "/home/wen-zheng/miniconda3/envs/sofanew"

def run_scene(scene_script, mode, vessel, trial_id):
    # Set environment variables for the scene script
    os.environ["MODE"] = str(mode)
    os.environ["VESSEL"] = vessel
    os.environ["TRIALS"] = str(1) # We run one trial per process
    
    # Import the scene script
    spec = importlib.util.spec_from_file_location("scene", scene_script)
    scene_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(scene_module)
    
    # Create the root node
    root = Sofa.Core.Node("root")
    scene_module.createScene(root)
    
    # Initialize the simulation
    Sofa.Simulation.init(root)
    
    # Manually run the simulation loop
    print(f"--- Starting Batch Simulation: MODE={mode}, VESSEL={vessel}, Trial={trial_id} ---")
    
    steps = 0
    try:
        while root.animate:
            Sofa.Simulation.animate(root, root.dt.value)
            steps += 1
            if steps > 200000: # Safety timeout
                print("Force quitting after 200,000 steps")
                break
    except SystemExit:
        # sys.exit(0) from controller will be caught here
        pass
    except Exception as e:
        print(f"Error during simulation: {e}")
            
    print(f"--- Batch Simulation Finished in {steps} steps ---")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python run_batch.py <mode> <vessel> [trial_id]")
        sys.exit(1)
        
    mode = int(sys.argv[1])
    vessel = sys.argv[2]
    trial_id = int(sys.argv[3]) if len(sys.argv) > 3 else 1
    
    run_scene("run_navigation_test.py", mode, vessel, trial_id)
