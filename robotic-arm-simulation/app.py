from flask import Flask, request, jsonify, render_template
import spacy
import pybullet as p 
from simulation.pybullet_env import sim_env
from simulation.robot_control import (
    run_pick_and_place, pick_object, place_object, gesture_move,move_arm_direction
)
from simulation.camera_control import get_object_position_from_camera

app = Flask(__name__, static_folder="static", template_folder="templates")

# --- Initialize environment ---
env = sim_env(gui=True)

robot_id = env["robot"]
objects = {
    "bottle": env["bottle"],
    "ball": env["ball"],
    "table": env["table"],
    "red_box": env.get("red_box"),
    "blue_box": env.get("blue_box")
}
arm_joints = env["arm_joints"]
gripper_joints = env["gripper_joints"]
ee_link_index = env["ee_link"]

# --- Known place locations ---
known_locations = {
    "red_box": [1.1, 0.4, 0.72],
    "blue_box": [1.1, -0.4, 0.72]
}

# --- NLP Setup ---
try:
    nlp = spacy.load("en_core_web_sm")
except OSError:
    import spacy.cli
    spacy.cli.download("en_core_web_sm")
    nlp = spacy.load("en_core_web_sm")

action_keywords = ["pick", "grab", "put", "place"]

# Direction commands
direction_keywords = ["upar", "nichay", "daye", "baye"]


def parse_command(command):
    """
    Parses both:
      - pick/place commands
      - direction commands (upar/nichay/daye/baye)
    """
    doc = nlp(command.lower())
    actions = []
    last_obj = None

    for token in doc:

        # ------------------------------------------------
        # direction commands
        # ------------------------------------------------
        if token.text in direction_keywords:
            actions.append(("move_dir", token.text, None))
            continue

        # ------------------------------------------------
        # pick/place parser 
        # ------------------------------------------------
        if token.lemma_ in action_keywords:
            action = token.lemma_
            obj = None
            dest = None

            # Object from children
            for child in token.children:
                if child.text in objects:
                    obj = child.text
                    last_obj = obj

            # Object from later tokens
            if obj is None:
                for next_token in doc[token.i + 1:]:
                    if next_token.text in objects:
                        obj = next_token.text
                        last_obj = obj
                        break

            # Destination lookup
            for next_token in doc[token.i + 1:]:
                if next_token.text in known_locations:
                    dest = next_token.text
                    break

            # If object missing use previously mentioned
            if obj is None and last_obj is not None:
                obj = last_obj

            if obj:
                actions.append((action, obj, dest))

    return actions


@app.route("/")
def index():
    return render_template("index.html")


# --- Robot state ---
robot_state = {"holding": None, "constraint_id": None}


@app.route("/execute_command", methods=["POST"])
def execute_command():
    data = request.json
    command = data.get("command", "").strip()
    print(f"Received: {command}")

    p.stepSimulation()

    actions = parse_command(command)
    if not actions:
        return jsonify({"error": "No valid action/object detected."})

    results = []

    for act, obj_name, dest_name in actions:

        # ------------------------------------------------------------
        # HANDLE SIMPLE MOVEMENT COMMANDS
        # ------------------------------------------------------------
        if act == "move_dir":
            print(f"Performing movement: {obj_name}")
            try:
                ok = move_arm_direction(robot_id, ee_link_index, arm_joints, obj_name)
            except Exception as e:
                import traceback
                traceback.print_exc()
                ok = False

            if ok:
                results.append(f"Arm moved: {obj_name}")
            else:
                results.append(f"Movement failed: {obj_name}")
            continue


        # ------------------------------------------------------------
        # pick/place logic below
        # ------------------------------------------------------------
        obj_id = objects.get(obj_name)
        if obj_id is None:
            results.append(f"Object '{obj_name}' not found.")
            continue

        try:
            # ---------------- PICK ----------------
            if act in ["pick", "grab"]:
                if robot_state["holding"] is not None:
                    results.append(
                        f"Cannot pick '{obj_name}': already holding '{robot_state['holding']}'"
                    )
                    continue

                object_pos = get_object_position_from_camera(obj_id, env)
                print("object_pos:", object_pos)

                if object_pos is None:
                    results.append(f"Object '{obj_name}' not visible in camera.")
                    continue

                print(f"Executing Pick: {obj_name}")
                cid = pick_object(
                    robot_id, obj_id, ee_link_index,
                    arm_joints, gripper_joints,
                    object_pos.tolist()
                )

                robot_state["holding"] = obj_name
                robot_state["constraint_id"] = cid

                results.append(f"Picked '{obj_name}' successfully.")

            # ---------------- PLACE ----------------
            elif act in ["put", "place"]:
                if robot_state["holding"] != obj_name:
                    results.append(
                        f"Cannot place '{obj_name}': robot is holding '{robot_state['holding']}'"
                    )
                    continue

                if dest_name is None:
                    results.append(f"Destination for '{obj_name}' not specified.")
                    continue

                target_coords = known_locations.get(dest_name)
                if target_coords is None:
                    results.append(f"Destination '{dest_name}' not found.")
                    continue

                print(f"Executing Place: {obj_name} -> {dest_name}")

                place_object(
                    robot_id, ee_link_index, arm_joints, gripper_joints,
                    target_coords, constraint_id=robot_state["constraint_id"]
                )

                robot_state["holding"] = None
                robot_state["constraint_id"] = None

                results.append(f"Placed '{obj_name}' at '{dest_name}'.")

            else:
                results.append(f"Action '{act}' could not be executed.")

        except Exception as e:
            import traceback
            traceback.print_exc()
            results.append(f"Robot execution failed for '{obj_name}': {e}")

    return jsonify({"result": results})


if __name__ == "__main__":
    import atexit
    atexit.register(p.disconnect)
    app.run(host="0.0.0.0", port=5000, debug=True, use_reloader=False)
