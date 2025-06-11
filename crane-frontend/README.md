# Prerequisite
To install all the required packages a modern enough version of `npm` is required.

# How to start
To install all dependencies run the following command within this folder
```
npm install
```

To run the fronetend in the debug mode use the command
```
npm run dev
```
It will start the `VITE` development server with the project available on some of the local port. Then one can open a web browser with the corresponding address `localhost:<port_number>`, for instance `http://localhost:5173/` 

# How to use
1. To connect to the backend server put a proper port and ip adress of the server (by default it is `localhost:8081`) and press `Connect` button. If evrything is alright then one should see the 3d model of the robot on the canvas.
2. By default the control is set into `joint control` control mode selected via a dropdown. In this mode the robot will follow the individual joints setpoints. The values for the desired joints angles can be modified using sliders in `Robot Control` GUI panel. To execute the command press `Send Command` button.
3. Another option of the dropdown is `pose control` control mode in which the robot is going to stabilize the desired 4DoF state of the TCP(*Tool Control Point*) in the world frame.  To execute the command press `Send Command` button.
4. The `pose control` has two options:
   - `stabilization` checkbox enables/disables the pose active stabilization logic and activates robot's based movement.
   - The `stabilization algorithm` dropdown has two options which algorithm to use for active stabilization either `ik` method based on the iterative solveing of the robot's inverse kinematics or `jacobian` use a Cartesian control method based on inversion of the Jacobian matrix. 
