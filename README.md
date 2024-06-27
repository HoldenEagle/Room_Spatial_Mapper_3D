# Room_Spatial_Mapper_3D
Aimed for assisting Urban planner , construction companies, and housing planners with an easy way to visualize 3D spaces on your computer. Will allow to do walk through the space, compute distance and measurements, and perform building standard evaluations.

Right now the visualizer has been created with the PointCloudWindow class in the file attached. A text file has also been attached that provides the functionalities to move around the space , rotate the cloud, reset , etc. Currently, the only functionality other than moving around the space is computing the distance of points in the cloud, which is computed using the Euclidean distance. Adding the other functionalities currently. 

The room is currently being mapped out using a point cloud reconstruction of the room using LiDar technology. You can use any ipad or iphone above the 12th version to download technology that will allow you take a point cloud representation of this room. I have attached some demo point clouds to allow you to see them in the visualizer, but these point clouds were not created from a LiDar camera, and due to that, these demo point clouds are not full images of a room.

The necessary libraries needed to run this visualizer are listed in requirements.txt. To install the necessary libraries, use the command pip install -r requirements.txt.


A visual taken of the space of one of the demo point clouds is shown in the image in this branch.



