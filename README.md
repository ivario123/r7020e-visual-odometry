# r7020e-visual-odometry
This visual odometry pipeline was implemented as part of the [R7020E: Computer Vision and Image Processing at LTU](https://www.ltu.se/edu/course/R70/R7020E/R7020E-Datorseende-och-bildbehandling-1.204996?l=en) it is based on [SIFT feature detection and tracking](https://en.wikipedia.org/wiki/Scale-invariant_feature_transform) and pose estimation using the P3P algorithm [implemented in matlab](https://se.mathworks.com/help/vision/ref/estworldpose.html). 


## What is needed to run the code?
A calibrated stereo pair of cameras, a dataset of images from the stereo pair and timestamps for each image.

To make the implementation run in real-time instead of on a dataset you would just need to adapt the for loop to keep a state variable and refactor it to a function call and call it for each frame. 
## How it works
The pipeline is divided into four parts,
- feature detection
- feature tracking
- pose estimation
- landmark insertion.

### Feature detection
The feature detection is done using the [SIFT algorithm implemented in matlab](https://se.mathworks.com/help/vision/ref/detectsiftfeatures.html?s_tid=doc_ta)

## Feature tracking
### Feature matching
The feature matching is done using the built-in Matlab function](https://se.mathworks.com/help/vision/ref/matchfeatures.html?s_tid=doc_ta)](https://se.mathworks.com/help/vision/ref/matchfeatures.html?s_tid=doc_ta) which simply finds the most features with the most similar feature descriptors.
### Feature tracking
Let each time step in the feed contain a stereo pair of images. 
For a given time step $i > 1$ match the features in the $leftFrame_{i-1}$ with the features in the $leftFrame_{i}$ and the features in the $rightFrame_{i-1}$ with the features in the $rightFrame_{i}$. After this, we have a set of features that still exist in the left frame and a set of features that still exist in the right frame. Match the features in the left frame with the features in the right frame and vice versa. This will give us a set of features that still exist in both the left and right frames.

Remove all features from $leftFreame_{i-1}$ and $rightFrame_{i-1}$ that are not in the set of features that still exist in both the left and right frames. And remove all features from $leftFrame_{i}$ and $rightFrame_{i}$ that are not in the set of features that still exist in both the left and right frames.

Now we can triangulate the $[x,y,z]$ coordinates in both of the coordinate frames, this is done using the [triangulate function](https://se.mathworks.com/help/vision/ref/triangulate.html?s_tid=doc_ta) in Matlab. 
### Pose estimation
The pose estimation is done using the [P3P algorithm](https://en.wikipedia.org/wiki/Perspective-n-Point) implemented in Matlab. Which returns the pose $P$ estimated in the previous time steps coordinate frame. To translate this to the world coordinate frame we need calculate $P_{world} = P_{i-1}\cdotP_{i}$. This yields the pose of the camera in the world coordinate frame.

### Landmark insertion
The landmarks are triangulated from feature matches between the left and right frames. The landmarks are inserted into the world coordinate frame. The landmarks are inserted into the world coordinate frame by calculating the pose of the camera in the world coordinate frame and then transforming the landmarks from the camera coordinate frame to the world coordinate frame.


## Visualisation
The visualisation is done in multiple steps,
- Feature translation and new feature visualization in the left frame.
- 2d mapping of the path of the camera in the left frame and the ground truth path.
- mapping of the 2d pose error of the camera in the left frame and the ground truth pose.
- mapping of the 3d pose of the camera and the landmarks in the world coordinate frame.

## Improvements

1. We could look at previous poses and use them to predict the next pose and use this to improve the pose estimation.
2. We could use the right camera and do the same process there and then transform that pose relative to the left camera to the world coordinate frame and use this to improve the pose estimation.

## References
- [SIFT](https://en.wikipedia.org/wiki/Scale-invariant_feature_transform)
- [PnP](https://en.wikipedia.org/wiki/Perspective-n-Point)
- [Visual Odometry](https://en.wikipedia.org/wiki/Visual_odometry)
