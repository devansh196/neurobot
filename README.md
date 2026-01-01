# NeuroBot: EEG-Driven Brain-Controlled Robot 🧠🤖

NeuroBot is a project by **Group 18** for the **Design Practicum (IC 201P)** at the **Indian Institute of Technology Mandi**, aimed at building a robotic system controlled by brain signals using **Electroencephalography (EEG)**.

This project demonstrates a **proof-of-concept Brain-Computer Interface (BCI)** that can decode a user's *imagined words* from non-invasive EEG signals and use them to either command a robotic system or display the decoded words on a screen.

---

## ✨ Key Features

- **Non-Invasive Control**  
  Uses EEG signals captured from a headset placed on the scalp.

- **Imagined Word Decoding**  
  Trains a deep learning model to classify specific imagined words from EEG data.

- **Robotic Command System**  
  Decoded words are classified as:
  - **Commands** (e.g., *"आगे"* / ahead, *"पीछे"* / back) to move a robotic arm  
  - **Non-commands** to be displayed on a screen

- **Assistive Communication Potential**  
  Focuses on language-based BCI, including the under-represented **Hindi language**, to assist users with paralysis or communication impairments.

- **Hybrid Deep Learning Model**  
  Uses a specialized **3D-CNN + LSTM** hybrid architecture to capture both spatial and temporal EEG patterns effectively.

---

## 🎯 Project Goal

To demonstrate that **low-cost EEG technology** can be used to decode imagined words from brain signals and translate them into **real-world physical actions**, such as controlling a robotic system.

---

## 🛠️ Technologies Used

- **Data Collection**  
  High-density **64-channel ANT Neuro EEG headset** (used to create the dataset)

- **Experiment Setup**  
  **PsychoPy** for visual stimulus presentation using  
  *Rapid Serial Visual Presentation (RSVP)*

- **Model**  
  Hybrid Deep Learning architecture combining **CNN** and **LSTM**

- **Hardware (Showcase)**  
  Microcontroller-controlled object or a simple robotic system

---

## 📂 Dataset

The EEG dataset used for this project can be downloaded from the link below:

🔗 **Google Drive:**  
https://drive.google.com/drive/folders/1scRpbIvDKUIgq61dU386H4zd_zc1WAH1?usp=sharing
