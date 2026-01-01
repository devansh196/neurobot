The file EEG_DataAnalysis+ML_Model.ipynb is the final scripts file used for this version of our code


📌 Overview

This project implements a complete EEG decoding pipeline using BrainVision .vhdr data.
The notebook performs:

EEG loading & preprocessing

Montage application (standard_1005)

Filtering & epoching

Artifact removal / channel cleaning

Creation of pseudo-trials

Feature extraction

ML model training & evaluation

Temporal decoding

Confusion matrix visualization

Topomap animation generation

The final goal is to classify EEG responses to different word/command categories using machine learning, and analyze temporal patterns of discriminability across the trial.



⚙️ Pipeline Summary
1. Load raw EEG

The notebook loads BrainVision .vhdr recordings, sets their sampling rate, metadata, and channel info.

2. Apply Standard 10–05 montage

All EEG channels are mapped to standard scalp positions using:

raw.set_montage("standard_1005", on_missing="ignore")

3. Preprocessing

Includes:

Band-pass filtering (e.g., 0.1–40 Hz)

Optional notch filtering

Bad channel detection

Re-referencing (average or linked mastoids)

Epoching around event markers

Baseline correction

4. Pseudo-trials

The notebook implements:

Trial mixing

Averaging limited random subsets

Separate pseudo-train and pseudo-test sets

No leakage between real/raw sets

These pseudo-trials increase SNR and stabilize decoding performance.

5. Feature Extraction

Features computed from epochs:

Sensor-level time series

Time-averaged windows

Flatted spatiotemporal matrices

Optional PCA-based dimensionality reduction

6. ML Model

A variety of ML approaches are used depending on analysis:

Logistic Regression

XGBoost


Time-resolved decoders (SlidingEstimator)

The pipeline supports:

Train on pseudo → test on pseudo

Train on pseudo → test on raw

Pure single-trial decoding

7. Temporal Decoding

Time-resolved decoding is performed using:

sliding window classifiers

cross-validated accuracy curves

SEM shading

peak accuracy reporting

8. Confusion Matrix

A seaborn-based confusion matrix is generated with clean formatting and blue colormaps.


🧪 Results

The pipeline outputs:

Classification accuracy per timepoint

Overall train/test accuracy

Confusion matrices

Smooth topomap animations

Plots of averaged ERPs





