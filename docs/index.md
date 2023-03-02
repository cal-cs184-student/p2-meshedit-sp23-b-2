# CS 184: Computer Graphics and Imaging, Spring 2023

## Project 2: MeshEdit
## Michael Lin, Rachel Lee
 *** 
### Overview
### Task 1: Bezier Curves with 1D de Casteljau Subdivision
- **Briefly explain de Casteljau's algorithm and how you implemented it in order to evaluate Bezier curves.**
    - De Casteljau’s algorithm is useful for evaluating Bezier curves provided a set of control points. The algorithm essentially uses linear interpolation to create subdivisions of the curve by adding a new point along each line segment edge and dividing it into new line segments. We calculate the new point at a given parameter t by using the formula: 
    - In evaluateStep, we iteratively calculate a new point from the set of control (or intermediate) points provided in the input using the above formula  points.size() - 1 times. Then, we add the new point to the 2D vector result and return the updated vector array containing the interpolated points.
- **Take a look at the provided .bzc files and create your own Bezier curve with 6 control points of your choosing.**
- **Show screenshots of each step / level of the evaluation from the original control points down to the final evaluated point. Press E to step through. Toggle C to show the completed Bezier curve as well.**
- **Show a screenshot of a slightly different Bezier curve by moving the original control points around and modifying the parameter**


