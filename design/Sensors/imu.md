@ahussein @rtreiber
I'd like to hear your feedback about the following.

In the following I mostly refer to IMUs equiped with magnetometer and accelerometer, which I will call _fully equiped_. If one of the both sensors would miss, the orientations presented by IMU would have as reference some power-on initial coordinate system which we may or may not make use of, as the orientation would then be estimated only from the angular velocities which we (could) fuse directly.

As I understand, _fully equiped_ IMUs give out orientations referenced to a world fixed frame. In most scenarious these world fixed frames are ENU(x-east, y-north and z-up) or NED( x-north, y-east, z-down). Both coordinate frames are formed from a plane tangent to the Earth's surface. As shown in the photo below taken from [wikipedia](https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates).

![ECEF_ENU_Longitude_Latitude_relationships.svg](../images/ENU.svg)


We care only for the orientation of the ENU(NED) coordinate system not for its origin as we deal only with rotations.

The situation is the following:
1. We get orientations in the world fixed frame (roll_ENU, pitch_ENU, yaw_ENU) of the IMU sensor -> **R_enu_imu_meas**
2. We know how the IMU sensor is oriented in our vehicle and we have an estimate of it from our map frame ->  **R_map_imu = R_map_bl * R_bl_imu**
3. We want to know how to transform these ENU referenced orientation measurements to our map frame -> **R_map_imu_meas**
4. Afterwards we just need to make up for the orientation of the IMU on the vehicle -> **R_map_bl_new = R_map_imu_meas * R_bl_imu^-1**


![](../images/imu_tf.svg)


The only difficulty in all of this is how to get **R_map_enu** transformation which would allow as to express the measurements in map frame as: **R_map_imu_meas = R_map_enu * R_enu_imu_meas**.

There are 2 possible solutions for that:
1. We provide the Power-On Orientation of the Vehicle in ENU coordinates so -> **R_map_imu = R_enu_imu**
2. We make use of the first(or average of N first) IMU measurement to calculate **R_map_enu = R_map_bl * R_bl_imu * R_enu_imu^-1**. And than use that to transform the future measurements. This method has the other advantage that it can be used for arbitrary world fixed reference too. Maybe even for the case where the IMU is not _fully equiped_. But it comes with a bias depending on how bad our orientation estimation **R_map_bl** is at that time.
3. Finally we fuse **R_map_bl_meas = R_map_enu * R_enu_imu_meas * R_bl_imu^-1**

----------------------------------------------------------------------------------------------------------------
The above logic seems to work in practise. There are still 2 problems that I cannot find a solution.

**1.Transformation of Covariances:**

While transforming the orientations is straight forward: **R_map_bl_meas = R_map_enu * _R_enu_imu_meas_ * R_bl_imu^-1**, transforming the covariances is not so obvious because our orientation **R_enu_imu_meas** is once rotated and once used to rotate **R_bl_imu^-1**. The rotation with R_map_enu corresponds to a covariance **C_new =  R_map_enu * C * R_map_enu^T**.

**_What about the second rotation_?**


**2.Euler are not continous:**

The usage of _Euler Angles_ comes with a Gimbal Lock which causes lose of DOF if pitch is pi/2.

Is Gimbal Lock the cause for the following two consecutive measurements?
```
RPY1: 0.000338291   0.0204947  0.00262584
QUATERNION1: 0.00018259  0.0102469 0.00131458 0.999947

RPY2: 3.14138  3.1207 3.14134
QUATERNION2:  -0.00010735    0.0104444 -0.000125598 0.999945
```

I know too that choosing the order of euler angles could avoid this discontinuity for specific scenarious. I am using the ordering as given by Eigen but nevertheless the mounting orientation of the IMU could be arbitrary, so can we really avoid the gimbal lock?..

The question would then be:    **_Is there any way that we could have the quaternion as part of the state instead of RPY or should I let the Mahalanobi Gate ignore those abrupt changes?_**