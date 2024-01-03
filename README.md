# Looking Glass
Using OpenCV and Unreal Engine 5, LookingGlass provides a low cost AR solution on a monitor with a web camera 
![Sample](https://github.com/hseddis321/looking-glass/assets/20444716/f6e7c765-45f3-430a-9f39-720938cf9496)

The idea for this project came from this video  
[![Real-Time In-Camera VFX for Next-Gen Filmmaking | Project Spotlight | Unreal Engine](https://img.youtube.com/vi/bErPsq5kPzE/0.jpg)](https://www.youtube.com/watch?v=bErPsq5kPzE)

To do this we need to get the position of the viewer and simulate what this camera would see in a virtual reality world

## Position Of Viewer
We track the viewer using a Web Camera with OpenCV. We will use ArUco tags to get the pose of the viewer. In our case, we are using two ArUco tags around a phone camera. We also use a Kalman Filter to track the pose of our viewer when the webcam loses sight of one or both of the ArUco tags. Even though this filter is very poorly tuned, it provides data for about half a second to bridge between missed frames. We also input some constants into our python script such as the webcam angle relative to the monitor and the distance between the webcam and the center of the monitor to more accurately track the viewer. We pass all this data to Unreal Engine 5 using UDP over a local port  

![Tracking Pose](https://github.com/hseddis321/looking-glass/assets/20444716/e0a2fa6e-6151-4549-a678-0ac07ed6d77e)

## Virtual Reality   
In UE5 we create a custom pawn component which consists of just a camera. We will move this camera relative to the centerpoint of the pawn.  
  
We take the values we get over the UDP port and transform the position of the camera and save it to a local variable. 
![Screenshot 2023-12-08 184149](https://github.com/hseddis321/looking-glass/assets/20444716/9bf81ffb-a18b-493c-82c7-db3cc59b97a3)  

We then calculate the rotation necessary for the camera to continue pointing at the centerpoint of the pawn and smoothly interpolate the camera to the necessary position and rotation
![Tick](https://github.com/hseddis321/looking-glass/assets/20444716/0b93cab8-0bec-455c-ae23-4d1b3588c69b)  


The demo uses the following environment as a sample map: https://www.unrealengine.com/marketplace/en-US/product/low-poly-town
## Future Work
Here are a few possible additions to improve this project:
 + Eye Tracking to remove the necessity of ArUco tags
 + Improve Kalman filter to better predict position
 + Filter out erroneous [Z flipping](https://answers.opencv.org/question/123375/aruco-z-axis-flipping-perspective/) of ArUco tags to remove the necessity of using 2 tags

