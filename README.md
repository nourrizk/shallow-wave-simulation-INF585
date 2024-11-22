# Shallow Wave Simulation
Implementation of a Shallow Wave Simulator as part of the course INF585 on Computer Animation - Ecole Polytechnique

### Objective and Roadmap :
The goal of this project is to implement a **breaking wave visualisation** based on a shallow water representation coupled with a particle based approach for the breaking waves modeling. We based our work on N. Thürey, et al. paper _Real-time Breaking Waves for Shallow Water Simulations_ and implemented the shallow water representation for the surface of the fluid, as well as a wave front detection and wave line generation. Since the project is computation heavy to run in real-time on our machine, and as a first phase, we chose to leave the surrogate particle-based sheet representation of the actual breaking waves as a future step. However, we have developed the necessary code to detect the potential regions of overturning waves based on their height field, to compute their advection over time and to generate points and particles necessary to visualize the breaking waves.

### Code Setup :
To run the shallow wave code, you need to download the cgp library available on the github page of Prof. Damien Rohmer
(link: https://github.com/drohmer/cgp). 

Furthermore, the project directory should be structured as follows:

```bash
root dir
  │
  ├── project-code-INF585
  │      ├── assets
  │      ├── scripts
  │      ├── shaders 
  │      └── src 
  │
  └── cgp
       ├── library
       └── ...
```

### Results :
The _figure_ bellow shows our shallow wave simulation
![Wave simulated after the user applying a force on the water surface at rest](<pics/wave plus.png>)

The initial flat plane is animated by the user by gently moving the surface of the water with his mouse. Since the grid’s boundary conditions are reflective, we can observe interference between different wave fronts generating a dynamic and interesting water surface. However, since the energy impulsed into the plane is conserved, we can see waves interfere destructively at a certain point, which would physically lead to the water going still. However, once this point absorbs the waves, it generates back a circle shaped wave, as if the water ripples. This issue could be solved by addressing the energy conservation and the physical equations behind the shallow water equations.

We have implemented the detection of wave fronts, their enlargement, segmentation and the construction of wave lines. After taking into consideration the computation heaviness of the simultaneous wave lines, we decided to allowe a unique segment and a unique line for observing a single breaking wave per simulation. Furthermore, we left the implement of the particle based visualisation of the breaking waves and the collision with the shallow water framework as a future step.


### Conclusion
This paper effectively simulates breaking waves by using a combination of steepness detection, adaptive line tracking, and particle-based representation of fluid dynamics. The method strikes a balance between computational
efficiency and visual realism, making it particularly suitable for real-time applications where both factors are critical. Finally, an interesting aspect of the authors’ simulation is the two-way coupling of rigid bodies with the fluid, allowing for interactions between objects and the water surface. This addition opens up possibilities for more dynamic and interactive simulations, such as objects interacting with waves. Moreover, potential extensions of the paper might encompass improving the handling of chaotic wave conditions and enhancing the simulation of small-scale splashes, drops and ripples as mentioned in the result section.

### Main Bibliographic References : 
N. Thürey, M. Müller-Fischer, S. Schirm, & M. Gross (2007). _Real-time breaking waves for shallow water simulations_. In Proceedings - The Pacific Conference on Computer Graphics and Applications Pacific Graphics 2007, PG (pp. 39-46). Article 4392714 (Proceedings - Pacific Conference on Computer Graphics and Applications). https://doi.org/10.1109/PG.2007.54

A. Fournier, W. Reeves (1986). _A Simple Model of Ocean Waves_. SIGGRAPH 1986
