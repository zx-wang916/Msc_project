# Project Plan Submission (Version 0.0.1)
This is the initial plan written at 30th April. There might be some parts where I'm not very clear of.

## Please enter the current planned title for the project.

Using Bayesian Optimization in Pose Graphs Based SLAM

## A one paragraph summary of what the project is about.

The problem we are interested in is being able to choose the information matrix in a factor graph. This problem appears to have not been widely tackled. However, a related problem is in Kalman filtering. The problem there is called tuning and is about picking the Q and R matrices (process and observation noise covariances). In this project, we try to use Bayesian Optimisation to estimate the global properties of the cost function to better measure the state estimator.

## Write down the sequence of actions you expect to take when working on your project.
1. Weekly meeting:
   - What I've done
   - What I'm going to do
   - Problems
2. Read the given two important literatures for the initial step
3. Explore more by searching unclear points 

---

Draft introduction in early June
Start working on the report in mid-July


## Project Introduction given by Prof. Simon Julier

Pose graphs are a powerful way to estimate the state of a nonlinear system. However, most algorithms use variants of gradient descent and can get stuck in local minima. This project will explore how Bayesian Optimization (BO) can be used. BO attempts to estimate the global properties of the cost function and quantifies the uncertainty in this estimate. The project will investigate the use of BO in several challenge cases including mulit-modal distributions


## Key words

SLAM; nonlinear estimation; optimization; posegraphs
