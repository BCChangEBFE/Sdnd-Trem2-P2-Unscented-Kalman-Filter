# Sdnd-Trem2-P2-Unscented-Kalman-Filter

This is an implementation of the solution to the Unscented Kalman Filter for Self Driving Car project hosted at
https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project/blob/master/readme.txt

This readme mostly serves as a self-note documenting problems encountered whie working on the project and specific details that I feel are worth noting,
1. If first measurement is laser, it is possible to calculate phi using atan2()
2. When ever reading in phi or x, check that they are not too small (zero), if they are then I set them to a small number.
3. Take care to initialize the starting covariance matrix and standard deviation values. Make sure they make sense physically too.
4. There are some variable that are used more than once, we should try to optimize, as long as it does not sacrifice code readability. Specifically
     - z__diff is used in both Radar Measurement Prediction and UKF update
     - the loop through ( 2 * n_aug_ + 1) is everywhere. Should try to combine them, but be careful not to use variable that are not finalized yet
5. I have tried to implement updating of laser measurement by going through calculation of sigma points and all just like the Radar UKF case. It works but doesn't really provide any value, better off to just go with simple Kalman Filter approach as in project 1.
