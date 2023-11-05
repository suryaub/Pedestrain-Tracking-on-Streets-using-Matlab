## Pedestrain-Tracking-on-Streets-using-Matlab

#### Objective
With the implementation of this project, we are aiming at the below mentioned objectives: -
•	To improve the road safety by providing more information about the human objects (pedestrians) to the people travelling on the street.
•	To help the drivers with poor eye sight for recognizing the pedestrians. 
•	To analyze the traffic data by discriminating two wheelers and four wheelers on the road.

#### The major steps can be summarized as follows
•	Auxiliary input and global parameters of the tracking system.
•	Create System Objects for the Tracking System Initialization.
•	The initialize Tracks function creates an array of tracks, where each track is a structure representing a moving object in the video.
•	Read a Video Frame.
•	The detect People function returns the centroids, the bounding boxes, and the classification scores of the detected people. 
•	Predict New Locations of Existing Tracks using the Kalman Filter.
•	Assigning object detections in the current frame to the existing tracks.
•	The updateAssignedTracks function updates each assigned track with the corresponding detection. 
•	The deleteLostTracks function deletes tracks that have been invisible for too many consecutive frames. 
•	Create new tracks from unassigned detections. 
•	The displayTrackingResults function draws a colored bounding box for each track on the video frame. The level of transparency of the box together with the displayed score indicate the confidence of the detections and tracks.

#### Conclusion
•	“Pedestrain Tracking on Streets using Matlab” technique that we propose will overcome most of the limitations of the earlier and in use techniques. 
•	This project involves techniques like Aggregate Channel Feature , Kalman Filter etc. that are implemented in Matlab for detecting and tracking human objects .
•	If we use live video by attaching the camera to the car, the pedestrians are highlighted and it helps the drivers for recognizing them in dim lights and for having an enhanced view of the street. 
•	As the equipment needed for the implementation of the project is very minimal, the project can be used on a large scale.

We can develop a GUI using GUIDE in MATLAB, we can load a recorded video into the interface and then it detects the human objects and highlights them using the tracker.
