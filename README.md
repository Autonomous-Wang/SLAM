# self-driving
this folder mainly includes some python code on SLAM simulation of a bicycle model robot from a starting point to a defined destination. The file SLAM.py combine particle filter, path search, PID controller and bicycle model robot together. Other python files containes some individual problem, such kalman filter, particle filter and etc.

1. path search:
the path search is based on BFS and A* algorithm. We assume we know the position of obstacles in the map and then based on the algorithm we can find a path to the destinaton. The next step is to smooth the path by setting weighting parameters (data weight and smooth weight), so that the path is smooth rather than straight anywhere.

2. particle filter
particle filter is uesed to localized the robot based on the noisy measurement. Due to the whole thing is a simulation, gaussian noise is added to the measurement, and particle filter is used to localization. Resampling wheel method is used to update the location of robot, resampling wheel is mainly based on calculated the probabilities of all the particles. And resample the particle based on these probability value.

3. PD controller
The parameters for the PD controller is tuned mainly on the twiddle algorithm, a brute force method to find the optimal set of controller parameters. The error is calculated by the measuring the deviation from the predefined path, and change the steeling angle, so that the robot can stick to the path.
