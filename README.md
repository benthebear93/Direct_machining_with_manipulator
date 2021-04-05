# Direct machining on 3D constructed freeform surface using point cloud with manipulator(staubli tx90)

This repository is for simulation of direct machining on unknown freeform surface.

Line scanner and depth camera that attached to manipulator used to reconstruct 3d surface to estimate better normal vector for drilling point. To reconstruct 3d surface, depth camera is being used to roughly estimate the ROI of product that need to be drilled. With this rough data, optimal scanning path for manipulator with line scanner is calculated. 