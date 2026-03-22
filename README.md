# EEG-EMG Limb Pose Estimation

Estimate limb position from multimodal biosignals using EEG and EMG, with Kinect skeletal tracking as ground-truth supervision.

## Overview
This project explores whether limb pose can be predicted from electrophysiological signals recorded from a human subject.

The model takes as input synchronized EEG and EMG data and learns to estimate limb position over time.  
Kinect skeletal tracking is used to provide ground-truth joint positions during training and evaluation.

## Motivation
Inferring human movement directly from biosignals is an important problem in human-computer interaction, assistive technology, rehabilitation, and brain-computer interfaces.

EEG contains information related to motor planning and cortical activity, while EMG provides a more direct measurement of muscle activation. Combining both modalities may improve pose estimation compared with using a single signal source alone. Kinect provides a practical way to obtain approximate body joint trajectories for supervised learning. 

## Problem Formulation
Input:
- EEG time series
- EMG time series

Target:
- limb or joint position from Kinect skeletal tracking
- optionally joint velocity or joint angle

Task:
- supervised regression from biosignal windows to pose variables

## Pipeline
1. Record synchronized EEG, EMG, and Kinect data
2. Preprocess signals
   - EEG filtering / artifact reduction
   - EMG rectification / envelope extraction
   - Kinect smoothing / interpolation
3. Align all modalities in time
4. Segment into fixed-length windows
5. Train a model to map biosignal windows to joint positions
6. Evaluate prediction quality on held-out recordings

## Example Targets
Depending on the experiment, the model can predict:
- wrist position
- elbow position
- hand trajectory
- joint angles
- full upper-limb pose vector

## Model Ideas
Baseline models:
- linear regression
- ridge regression
- random forest
- MLP

Sequence models:
- 1D CNN
- LSTM / GRU
- temporal convolution network
- transformer encoder

Multimodal fusion options:
- early fusion of EEG + EMG features
- late fusion with separate encoders
- modality ablation to compare EEG-only, EMG-only, and EEG+EMG

## Evaluation
Possible metrics:
- mean absolute error (MAE)
- root mean squared error (RMSE)
- Pearson correlation
- trajectory error over time

Recommended comparisons:
- EEG only
- EMG only
- EEG + EMG
- previous-frame / constant-position baseline

## Project Structure
src/            core code
data/           dataset metadata or sample files
notebooks/      exploration and visualization
models/         saved models and configs
scripts/        preprocessing and training scripts
tests/          unit tests
docs/           experiment notes and diagrams

## Current Status
This project is currently focused on:
- building a synchronized EEG/EMG/Kinect dataset
- defining a clean preprocessing pipeline
- implementing baseline regression models
- comparing single-modal vs multimodal decoding

## Future Work
- predict full-body pose instead of selected joints
- improve temporal synchronization
- test subject-specific vs cross-subject models
- investigate real-time decoding

## Author
Edo
