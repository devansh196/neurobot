NeuroBot is a project by Group 18 for the Design Practicum (IC 201P) at Indian Institute of Technology Mandi, aimed at building a robotic system controlled by brain signals (EEG).

This project demonstrates a proof-of-concept for a Brain-Computer Interface (BCI) that can decode a user's imagined words from non-invasive EEG signals and use them to command a robotic arm or display them on a screen.

Key Features ✨
Non-Invasive Control: Uses Electroencephalography (EEG) signals captured from a headset on the scalp.
Imagined Word Decoding: Trains a Deep Learning model to classify specific thoughts (imagined words) from the EEG data.
Robotic Command System: Classifies the decoded words as either commands (e.g., "आगे" / ahead, "पीछे" / back) to move a robotic arm or as non-commands to be displayed on a screen.
Assistive Communication Potential: Focuses on language-based BCI, including the under-represented Hindi language, to aid users with paralysis or communication difficulties.
Hybrid Deep Learning Model: Utilizes a specialized 3D-CNN + LSTM hybrid architecture to effectively process the spatial and temporal patterns in the EEG signal.
Project Goal 🎯
To demonstrate that using low-cost EEG technology to decode words from the brain and translate them into a physical impact (like moving a robotic arm) is possible.

Technologies Used 🛠️
Data Collection: High-density 64-channel ANT Neuro EEG headset (for creating the dataset).
Experiment Setup: PsychoPy for presenting visual stimuli (Rapid Serial Visual Presentation).
Model: Hybrid Deep Learning (CNN and LSTM).
Hardware (Showcase): Microcontroller-controlled object or a simple robotic system.
Download the Data from here

https://drive.google.com/drive/folders/1scRpbIvDKUIgq61dU386H4zd_zc1WAH1?usp=sharing
