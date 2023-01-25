# COMP-417-
Demonstration of various Robotics &amp; Machine Learning applications learned in COMP 417


Balance a pole on a cart using OpenCV + PID Controller + Kalman Filter

# Overview: 
The "Cart-Pole Problem" is one of the most classic control problems in the field of Robotics. In this implimentation the cart and pole are objects in a PyGame environment, developed my Dr. Faraz Lofti. The environment leverages relatively realistic physics, such that the pole will naturally fall due to gravity, if the cart does not adjust its position.
    
# Goal: 
Use tools from the OpenCV library to identify the pole and calculate the angle between it and the cart for every frame of the game. The angle is then processed by a Kalman Filter in order to smooth out any discrepancies between the actual angle, and the angle calculated from the best-fit rectangle in OpenCV. Finally, the filtered angle is passed into a PID controller, which outputs a force to move the cart with.

# Results: 
Here's a video of the simulation running.
![alt text](https://drive.google.com/file/d/1mG1vOfe_MUsUBLFtzYooBxZO5K3qt7U3/view?usp=share_link)
    
# Part 1) Visual Tracking

To estimate the angle (radians) between the pole and the cart at each time step I used a
combination of methods available in OpenCV. Specifically, my program uses color masking and
image thresholding to convert all but the pixel values of the red pole into white while the rest of
the image is converted into black. The pixels are then converted to greyscale, at which point the
program attempts to find the rectangular contours in the image, which is just the pole. Finally,
the angle of the pole with the cart can easily be obtained using the cv2.minAreaRect() method, at
which point the program merely converts the angle to negative or positive depending on the
quadrant the pole is in (ie, negative if it’s leaning left, positive if it’s leaning right). _To view the
results of the object detection live, un-comment line 45 in cart_pole_angle_detector.py._

Results
![alt text](https://drive.google.com/file/d/1bov-jit8NnrjjB5Rqyn2SyBqHspPqiAj/view?usp=share_link?raw=true)


# Part 2 ) Kalman Filter

Using the following equations, I was able to obtain highly-optimal results with a Kalman filter.
These equations are implemented in PID_Controller.py, with a custom KalmanFilter class.

![alt text](https://drive.google.com/file/d/19jtaiw2K5FNxObOQsfd7Zo6UzLU0Ej_x/view?usp=share_link?raw=true)

I found that setting the values of my Q and R matrices such that Q > R, was the most effective
tuning mechanism. As shown in Figure b), the estimated values of theta were accurate to within
~+/-0.01 radians, well within a range required to balance the pole. Figures c) and d) clearly
demonstrate that there’s a strong covariance between the estimated values of theta and the actual
values of theta, as well as between the estimated values of theta and theta dot. Overall, using the
filter yielded much more stable results.

Results

![alt text](https://drive.google.com/file/d/1oDVocr8-N74Od326a0B_F8qPDU5Dl-ps/view?usp=share_link?raw=true)

![alt text](https://drive.google.com/file/d/1hBU9sLSZ8x7H6YqYDOvXq0ZiUmB0JATt/view?usp=share_link?raw=true)

# Part 3) PID Controller

As the results in a) show, using the internal state values of theta and theta dot, I was able to
obtain a highly-stable pole. To tweak the gains I used the Ziegler-Nichols method to tune my
PID. Theta dot was calculated using the time delta of 0.005, as is used in the game.

![alt text](https://drive.google.com/file/d/1RiJgnYfvUhfZ1I4dMGC_9b7b_p3eYSo8/view?usp=share_link?raw=true)



    
