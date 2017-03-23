#teo_model
The initial design of the model of this robot has been done in a urdf.xacro format. To run this model in Gazebo the .sdf format is needed, therefore the following conversions were performed.


 **urdf.xacro -> urdf**

```
	rosrun xacro xacro.py -o teo.urdf teo_humanoid.urdf.xacro
```

**urdf -> sdf**

```
	gzsdf print teo.urdf > teo.sdf
```