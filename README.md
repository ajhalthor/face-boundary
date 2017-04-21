# Face-Boundary

## About 

This program takes an image as input and plots:
1. Tip of Nose
2. Boundary of the Face
3. plots points on the boundary at intervals of 2 degrees along the right side (from our perspective) of face. This can easily be changed in the code for the left side.
4. Connects the 2 points with a line

## The Concept

We start with the detection of nose and eyes using the `CascadeObjectDetector`. I extended these regions to include the mouth and eyebrows, respectively. These regions are forced to be part of the final face region.

The detection of the face boundary is done by thresholding the second order derivatives of the gray scale image. Especially the chin is hard to detect correctly. The border between the chin and the neck is highlighted by applying a large erosion step. Because this step is sensitive to noise in the face region, the image is first filtered non linearly by applying a small dilation and erosion step.

The face region is determined using the eroded image, otherwise the boundary between the chin and the neck may not be clear. Afterwards, the large erosion is undone by applying the (inverse) dilation step.

The results are quite good for the simple approach, but not perfect. You may obtain better results by iteratively changing the threshold used for the second order derivatives. If you start from a large threshold the neck will be included in the face region. You can detect it by assuming some maximum distance between the nose region and the bottom of the face. Then, you can decrease the threshold until the neck is not included anymore. Another alternative to make this method more robust may be to normalise the `LoG`.

Once the boundary is plotted, we find points at intervals of 2 degrees about the tip of the nose onto the boundary of the face. The tip of nose is determined as the midpoint of the Nose boundary plotted by the `CascadeObjectDetector` towards the beginning `[nx, ny]`.

## Plotting 90 lines at 2 degrees from each

Consider a line making an angle `theta` degrees with the X-axis. This line should pass though the tip of nose `[nx, ny]`.

The general equation of a line is :

```
y = mx + c1 
```
In the equation above, we know that `y = ny`, `x = nx`. Furthermore, we know the angle `theta` the line makes with the X-axis. Hence we know the slope of the line `m1` as `m1 = tan(theta)`. Using these values in the equation above, we compute the constant `c1` as follows:

```
c1 = ny - m1 * nx
```
For a second line that is 2 degrees more than the first line, we have the slope `m2 = tan(theta + 2)` which can be computed as we already know `theta`. This line also passes through the tip of nose `[nx, ny]` and hence the constant `c2` for this line is:

```
c2 = ny - m2 * nx
```

Thus we have `m2` and `c2` where the equation of the line is :
```
y2 = m2 * x2 + c2
```
By iterating a constant `xr` from the tip of nose to the boundary, the corrusponding `yr` for every point can be determined with:
```
yr = m2 * xr + c2
```
where `xr` and `yr` are equal dimensional vectors and each `[xi, yi]` pair corrusponds to a point on the line. The code segment that does everything discussed in this section is shown below.


```
%% From the bottom point to the top point, plot 90 points with a 2 degree spacing along B{1}

step = 2; step_rad = deg2rad(step);

for theta = -90:step:90
    
    theta_rad = deg2rad(theta);
    
    m2 = ( tan(theta_rad) + tan(step_rad) ) / (1 - tan(theta_rad)*tan(step_rad) );
    c2 = ny - m2 * nx;
    
    %find points on line in the box for now
    xr = nx: size(I,1); 
    yr = m2 * xr + c2;
    
    % Find point of intersection between line and curve
    [px, py] = intersections(xr,yr,x_curve,y_curve);
    
    % plot point on face boundary
    plot(px, py, 'ro','Color','red'); 
    
    % In case of multiple intersections
    px = px(1,:); py = py(1,:);
    
    % plot line between those nose tip and face boundary
    line([nx, px],[ny, py],'Color','green');

end;

```

## Sample Images

![Sample 1](/images/marked_photo_1.jpg)

![Sample 3](/images/marked_photo_3.jpg)

## The Files

1. `face_boundary.m` : Main MATLAB file that performs the steps dicussed in the about section.
2. `intersections.m` : To find the points of intersection between a line and a curve. This is used while plotting points on the facial boundary every 2 degrees.

Execute the program by running `face_boundary.m`
