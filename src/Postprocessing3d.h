#pragma once
#include "Model.h"

/*  Morphologically close a voxel mesh by applying Dilution followed by Erosion with a specified size
*
*	model : the voxel grid to perform morphological closing on
*   target : the resulting voxel grid
*	kernelSize: the size of the neighborhood used for Dilution/Erosion, has to be an odd number (1 => no effect)
*/
int applyClosure(Model* model, int kernelSize);
