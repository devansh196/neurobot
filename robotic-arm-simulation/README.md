# 🤖🧠 NLP-Controlled Robotic Arm Simulator



## 🚀 Overview

This project provides a **web-based interface** to control a **PyBullet robotic arm** using **natural language commands**. It supports:

* Pick-and-place actions
* Directional movements (upar, nichay, daye, baye)
* Real-time physics simulation
* Integration with EEG/BCI systems



## ✨ Features

### 🔹 Natural Language Command Parsing

* Understands English + Hinglish commands
* Detects actions (pick, place, grab)
* Detects objects (bottle, ball)
* Detects destinations (red box, blue box)
* Supports directional gestures

### 🔹 Realistic Robot Motion

* PyBullet-based simulation
* Inverse kinematics for smooth arm control
* Human-like shoulder‑based movement for direction commands

### 🔹 Object Pick & Place

* Pick objects (bottle/ball)
* Place into red or blue box
* Automatic object coordinate detection via virtual camera

### 🔹 Web Interface

* Flask backend
* Live command execution
* Simple and responsive UI



## 📦 Requirements

```
pip install Flask spacy pybullet
python -m spacy download en_core_web_sm
```

> `app.py` auto-downloads the spaCy model if missing.

---

## ▶️ How to Run

1. Install dependencies
2. Run the Flask server:

```
python app.py
```

3. Open:

```
http://127.0.0.1:5000/
```

---

## 🗣️ Supported Commands

### **Pick & Place**

* "pick bottle"
* "place bottle on red box"

### **Directional Commands**

| Command    | Meaning | Axis |
| ---------- | ------- | ---- |
| **upar**   | Up      | +Z   |
| **nichay** | Down    | -Z   |
| **daye**   | Right   | +Y   |
| **baye**   | Left    | -Y   |

---

## 📂 Project Structure

```
.
├── app.py
├── simulation/
│   ├── pybullet_env.py
│   ├── robot_control.py
│   └── camera_control.py
└── templates/
    └── index.html
```

---

## 🧠 EEG Integration

Designed for EEG-based control:

```
EEG Signal → Decoder Model → Text Command → Flask → Robot Action
```

---

