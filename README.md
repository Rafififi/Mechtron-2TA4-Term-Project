# MECHTRON 2TA4 Term Project: Braitenberg Vehicles with Reinforcement Learning

Authors: Marco Tan, Rafael Toameh

# Project Overview

For our MECHTRON 2TA4 term project, we designed and built two Braitenberg vehicles with the objective of hiding in dark places. These vehicles simulate adaptive behavior using reinforcement learning, allowing them to "learn" from their environment by adjusting state transition probabilities based on light intensity. Additionally, the vehicles are capable of wireless communication, enabling them to influence each other's behavior in real time.
Key Features

## Mobility:
Each vehicle is fully mobile and capable of navigating its environment autonomously.

## State-Based Behavior:
Vehicles operate using a finite state machine consisting of the four states introduced in the "CyberBrat" lecture, along with an additional idle state. These states define how the vehicle responds to stimuli (e.g., light sensors).

## Reinforcement Learning Algorithm:

  - State transitions are governed by probabilities.
  - These probabilities are dynamically updated based on environmental feedback.
  - The system rewards behavior that results in a decrease in light intensity (i.e., moving toward darkness) and punishes behavior that leads to an increase in light intensity.

## Wireless Communication:
Vehicles can exchange messages using a wireless module. These messages are used to:
  
  - Share information about their state or environment.
  - Temporarily influence the probability of transitioning from one state to another in a peer vehicle.

# Objectives

The main objective was to create autonomous vehicles that:

  - Learn from their environment through a reward system.
  - Adjust their behavior in response to light intensity.
  - Collaborate or influence one another using wireless communication.
