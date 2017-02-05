# Vehicle Detection
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Project Outline
The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector. 
* Note: for those first two steps don't forget to normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

Here are links to the labeled data for [vehicle](https://s3.amazonaws.com/udacity-sdc/Vehicle_Tracking/vehicles.zip) and [non-vehicle](https://s3.amazonaws.com/udacity-sdc/Vehicle_Tracking/non-vehicles.zip) examples to train your classifier.  These example images come from a combination of the [GTI vehicle image database](http://www.gti.ssr.upm.es/data/Vehicle_database.html), the [KITTI vision benchmark suite](http://www.cvlibs.net/datasets/kitti/), and examples extracted from the project video itself.   You are welcome and encouraged to take advantage of the recently released [Udacity labeled dataset](https://github.com/udacity/self-driving-car/tree/master/annotations) to augment your training data.  

Some example images for testing your pipeline on single frames are located in the `test_images` folder.  To help the reviewer examine your work, please save examples of the output from each stage of your pipeline in the folder called `ouput_images`, and include them in your writeup for the project by describing what each image shows.    The video called `project_video.mp4` is the video your pipeline should work well on.  

**As an optional challenge** Once you have a working pipeline for vehicle detection, add in your lane-finding algorithm from the last project to do simultaneous lane-finding and vehicle detection!

**If you're feeling ambitious** (also totally optional though), don't stop there!  We encourage you to go out and take video of your own, and show us how you would implement this project on a new video!


## Project Process
### Histogram of Oriented Gradients (HOG)

#### 1. Extract HOG features from the training images.


1. Read in all `vehicle` and `non-vehicle` images.
Here are two example images, the first from the `vehicle` class and the second from the `non-vehicle` class:

![Vehicle](./readme_images/vehicle-example9.png)
![Non-vehicle](./readme_images/non-vehicle-example1.png)

2. Use `skimage.feature.hog(training_image, [parameters=parameter_values])` to extract HOG features and HOG visualisation.
    * Wrapped in function `get_hog_features`.
    
Code in second cell in Section 1.1 in `p5.ipynb`. Relevant functions:  `get_hog_features` and `extract_features`.

#### 2. Choose HOG parameters.

* I wanted to optimise for HOG parameters systematically, so I wrote a script `hog_experiment.py` that enables me to easily run through different HOG parameters and save the classifier accuracy, the HOG visualisation (image) and bounding boxes overlaid on the video frame (image).
* I then picked the HOG parameters based on classifier accuracies and looking at the output images. I would like to make this process more rigorous instead of sort of basing it on intuition. It was difficult to do this because the classifier accuracy was usually above 99% and was often shown as 1.0 even if the classifier later drew many false positive bounding boxes.

Code in second cell in Section 1.2.

#### 3. Train a classifier using selected HOG features and colour features.

1. Format features using `np.vstack` and `StandardScaler()`.
2. Split data into shuffled training and test sets
3. Train linear SVM using `sklearn.svm.LinearSVC()`.

Data preprocessing Code in Section 1.2, classifier trained in Section 1.3.

### Sliding Window Search

#### 1. Implement a sliding window search.

1. Define windows to search using helper function `slide_window`.
    * Restricted search space to lower half of the image (altered value of variable `y_start_stop`) because cars only appear on the road and not in the sky.
    * Scales to search: TODO:
    * How much to overlap windows: TODO
2.  Implement sliding window search using helper function `search_windows`.
    * For each window, 
        * extract features for that window, 
        * scale extracted features to be fed to the classifier, 
        * predict whether the window contains a car using our trained Linear SVM classifier, 
        * and save the window if the classifier predicts there is a car in that window.

Code in Section 2.

#### 2. Show some examples of test images to demonstrate how your pipeline is working.  What did you do to try to minimize false positives and reliably detect cars?

* Measures to reliably detect cars in single images: I restricted the search space to only the lower portion of the image, i.e. the non-sky and mostly non-tree portion.

Sample image:

![Sample image of bounding boxes around classified windows](./readme_images/1.2.png)
---

### Video Implementation

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)

[Will link to video result](project_video.mp4)


#### 2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.

1. Record positions of positive detections in each (video) frame.
2. Create a heatmap of these positions (past 20 frames).
3. Threshold the heatmap to identify vehicle positions.
4. Identify blobs in the heatmap and determine their x and y-wise lengths.
    * Detect blobs using [`skimage.feature.blob_doh()`](http://scikit-image.org/docs/dev/auto_examples/plot_blob.html)
    * Determine x and y-wise lengths using [`skimage.morphology.watershed()`](http://scikit-image.org/docs/dev/auto_examples/plot_watershed.html)
5. Construct bounding boxes to cover the area of each blob detected.
    * Assumed each bounding box corresponds to a vehicle.

I recorded the positions of positive detections in each frame of the video.  From the positive detections I created a heatmap and then thresholded that map to identify vehicle positions.  I then used blob detection in Sci-kit Image (Determinant of a Hessian  worked best for me) to identify individual blobs in the heatmap and then determined the extent of each blob using . I then assumed each blob corresponded to a vehicle.  I constructed bounding boxes to cover the area of each blob detected.  

Sample image of heatmap and bounding boxes overlaid on a video frame:
![To include]()

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Problems:
* When building the video pipeline, my functions failed to detect any vehicles for one of the frames. This returned `AttributeError: 'NoneType' object has no attribute 'shape' `. 
* I can't assess the strength of the parameter combination using only one instance of bounding boxes detected in an image. The bounding boxes detected by the same combination of parameters varies.
