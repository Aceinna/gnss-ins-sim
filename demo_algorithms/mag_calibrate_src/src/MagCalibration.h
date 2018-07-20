#include<math.h>
#include<ctype.h>
//#include <direct.h>
#include <string.h>
#include <stdlib.h>
//#include <io.h>

//===================================================================
// soft iron and hard iron calibration
// input:
//      dArrayX: mag data when rotating about the x axis,nx-by-3
//      dArrayY: mag data when rotating about the y axis,ny-by-3
//      dArrayZ: mag data when rotating about the z axis,nz-by-3
//      iRowNum: 3x1,data counts of the mag data,[nx,ny,nz]
// output:
//      dSiMtx: soft iron matrix，mag×dSoftIronMatrix
//      dHi: hard iron, [x,y,z,r], r is estimated magnetic field amplitude.
// return:
void MagCalibrate(double *dSiMtx,double *dHi,
                  double *dArrayX,double *dArrayY, double *dArrayZ,
                  int *iRowNum);

