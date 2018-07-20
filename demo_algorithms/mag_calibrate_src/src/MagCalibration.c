/******************************************************************************
 * Soft iron and hard iron calibration.
 * Authro: Dong Xiaoguang
 * Date: 2018-07-09
 *****************************************************************************/
#include <stdio.h>
#include "MagCalibration.h"
#include "mymath.h"

//=============================================================================
// Calculate the normal of a points cloud using BLS.
// input:
//      mag: mag data, nx3
//      n: number of mag samples.
// output:
//      rotAxis: normal of the samples in mag, can be considered as the rotation axis.
int GetPointsNormal(double *mag,int n, double *rotAxis);

// Calculate the center and radius of points cloud using sphere fitting.
// The input mag data should be soft-iron-calibrated.
// input:
//      dArrayX: mag data when rotating about the x axis,nx-by-3
//      dArrayY: mag data when rotating about the y axis,ny-by-3
//      dArrayZ: mag data when rotating about the z axis,nz-by-3
//      iRowNum: 3x1,data counts of the mag data,[nx,ny,nz]
// output:
//      dHi: hard iron, [x,y,z,r], r is estimated magnetic field amplitude.
//      The above dArrayX/Y/Z are also calibrated by dHi in this functino.
void GetMagOffset(double *dHi,
                  double *dArrayX,double *dArrayY, double *dArrayZ,
                  int *iRowNum);

//=============================================================================
void MagCalibrate(double *dSiMtx,double *dHi,
                  double *dArrayX,double *dArrayY, double *dArrayZ,
                  int *iRowNum)
{
    //-------------------------------------------------------------------------
    // soft iron, misalighment
    // calculate the normal (or rotation axis) of mag data collected when rotaitng
    // aobut its x, y and z axis.
    double vx[3];
    double vy[3];
    double vz[3];
    GetPointsNormal(dArrayX,iRowNum[0],vx);
    GetPointsNormal(dArrayY,iRowNum[1],vy);
    GetPointsNormal(dArrayZ,iRowNum[2],vz);
    // Force the rotation axis to be close to the positive direction of the coordinate axis
    int idx;
    double absV[3];
    vecAbs(vx,3,absV);
    idx = vecMax(absV,3);
    if(vx[idx]<0.0)
    {
        vecMultiplyConst(vx,-1.0,vx,3);
    }
    vecNormalize(vx,vx,3);
    vecAbs(vy,3,absV);
    idx = vecMax(absV,3);
    if(vy[idx]<0.0)
    {
        vecMultiplyConst(vy,-1.0,vy,3);
    }
    vecNormalize(vy,vy,3);
    vecAbs(vz,3,absV);
    idx = vecMax(absV,3);
    if(vz[idx]<0.0)
    {
        vecMultiplyConst(vz,-1.0,vz,3);
    }
    vecNormalize(vz,vz,3);
    double orthMtx[3][3];   // soft iron and misalignment(non-orthogonality) are inclued
    orthMtx[0][0] = vx[0];
    orthMtx[0][1] = vx[1];
    orthMtx[0][2] = vx[2];
    orthMtx[1][0] = vy[0];
    orthMtx[1][1] = vy[1];
    orthMtx[1][2] = vy[2];
    orthMtx[2][0] = vz[0];
    orthMtx[2][1] = vz[1];
    orthMtx[2][2] = vz[2];
    //-------------------------------------------------------------------------
    //sensitivity
    // Apply soft iron and misalignment correciton before calculating sensitivity
    double tmpRawMag[3];
    double tmpCalMag[3];
    int i;
    for(i=0;i<iRowNum[0];i++)
    {
        tmpRawMag[0] = dArrayX[i*3+0];
        tmpRawMag[1] = dArrayX[i*3+1];
        tmpRawMag[2] = dArrayX[i*3+2];
        mtxMultiplyVec((double*)orthMtx,tmpRawMag,tmpCalMag,3,3);
        dArrayX[i*3+0] = tmpCalMag[0];
        dArrayX[i*3+1] = tmpCalMag[1];
        dArrayX[i*3+2] = tmpCalMag[2];
    }
    for(i=0;i<iRowNum[1];i++)
    {
        tmpRawMag[0] = dArrayY[i*3+0];
        tmpRawMag[1] = dArrayY[i*3+1];
        tmpRawMag[2] = dArrayY[i*3+2];
        mtxMultiplyVec((double*)orthMtx,tmpRawMag,tmpCalMag,3,3);
        dArrayY[i*3+0] = tmpCalMag[0];
        dArrayY[i*3+1] = tmpCalMag[1];
        dArrayY[i*3+2] = tmpCalMag[2];
    }
    for(i=0;i<iRowNum[2];i++)
    {
        tmpRawMag[0] = dArrayZ[i*3+0];
        tmpRawMag[1] = dArrayZ[i*3+1];
        tmpRawMag[2] = dArrayZ[i*3+2];
        mtxMultiplyVec((double*)orthMtx,tmpRawMag,tmpCalMag,3,3);
        dArrayZ[i*3+0] = tmpCalMag[0];
        dArrayZ[i*3+1] = tmpCalMag[1];
        dArrayZ[i*3+2] = tmpCalMag[2];
    }
    // Calculate the relative sensitivity
    double sZ2Y;
    double sZ2X;
    double sY2X;
    double *col1;
    double *col2;
    col1 = malloc(iRowNum[0]*sizeof(double));
    col2 = malloc(iRowNum[0]*sizeof(double));
    mtxCol(dArrayX,iRowNum[0],3,2,col1);
    mtxCol(dArrayX,iRowNum[0],3,1,col2);
    sZ2Y = ( col1[vecMax(col1,iRowNum[0])] - col1[vecMin(col1,iRowNum[0])] ) /
            ( col2[vecMax(col2,iRowNum[0])] - col2[vecMin(col2,iRowNum[0])] );
    free(col1);
    free(col2);
    col1 = malloc(iRowNum[1]*sizeof(double));
    col2 = malloc(iRowNum[1]*sizeof(double));
    mtxCol(dArrayY,iRowNum[1],3,2,col1);
    mtxCol(dArrayY,iRowNum[1],3,0,col2);
    sZ2X = ( col1[vecMax(col1,iRowNum[1])] - col1[vecMin(col1,iRowNum[1])] ) /
            ( col2[vecMax(col2,iRowNum[1])] - col2[vecMin(col2,iRowNum[1])] );
    free(col1);
    free(col2);
    col1 = malloc(iRowNum[2]*sizeof(double));
    col2 = malloc(iRowNum[2]*sizeof(double));
    mtxCol(dArrayZ,iRowNum[2],3,1,col1);
    mtxCol(dArrayZ,iRowNum[2],3,0,col2);
    sY2X = ( col1[vecMax(col1,iRowNum[2])] - col1[vecMin(col1,iRowNum[2])] ) /
            ( col2[vecMax(col2,iRowNum[2])] - col2[vecMin(col2,iRowNum[2])] );
    free(col1);
    free(col2);
    double sensVec[3] = {1.0, 1.0/sY2X, (1.0+sY2X*sY2X)/(sY2X*sY2X*sZ2X+sY2X*sZ2Y)};
    double sensMtx[3][3];
    diag(sensVec,(double*)sensMtx,3);
    // dSiMtx now contains soft iron, misalignment and sensitivity.
    mtxMultiply((double*)sensMtx,(double*)orthMtx,dSiMtx,3,3,3);

    //-------------------------------------------------------------------------
    //hard iron
    // Apply dsiMtx before calculating hard iron. Notice dArrayX/Y/Z are already
    // calibrated by soft irion and misalignment. Only sensitivity correction is
    // needed here.
    for(i=0;i<iRowNum[0];i++)
    {
        tmpRawMag[0] = dArrayX[i*3+0];
        tmpRawMag[1] = dArrayX[i*3+1];
        tmpRawMag[2] = dArrayX[i*3+2];
        mtxMultiplyVec((double*)sensMtx,tmpRawMag,tmpCalMag,3,3);
        dArrayX[i*3+0] = tmpCalMag[0];
        dArrayX[i*3+1] = tmpCalMag[1];
        dArrayX[i*3+2] = tmpCalMag[2];
    }
    for(i=0;i<iRowNum[1];i++)
    {
        tmpRawMag[0] = dArrayY[i*3+0];
        tmpRawMag[1] = dArrayY[i*3+1];
        tmpRawMag[2] = dArrayY[i*3+2];
        mtxMultiplyVec((double*)sensMtx,tmpRawMag,tmpCalMag,3,3);
        dArrayY[i*3+0] = tmpCalMag[0];
        dArrayY[i*3+1] = tmpCalMag[1];
        dArrayY[i*3+2] = tmpCalMag[2];
    }
    for(i=0;i<iRowNum[2];i++)
    {
        tmpRawMag[0] = dArrayZ[i*3+0];
        tmpRawMag[1] = dArrayZ[i*3+1];
        tmpRawMag[2] = dArrayZ[i*3+2];
        mtxMultiplyVec((double*)sensMtx,tmpRawMag,tmpCalMag,3,3);
        dArrayZ[i*3+0] = tmpCalMag[0];
        dArrayZ[i*3+1] = tmpCalMag[1];
        dArrayZ[i*3+2] = tmpCalMag[2];
    }
    // Calculate hard iron parameters using soft-iron calibrated mag data
    GetMagOffset(dHi, dArrayX, dArrayY, dArrayZ, iRowNum);

    //-------------------------------------------------------------------------
    // return results. NOTICE: the dSiMtx is transposed.
    // **Remove transpose**
    // double tmpMtx[3][3];
    // mtxTranspose(dSiMtx,(double*)tmpMtx,3,3);
    // mtxDuplicate((double*)tmpMtx,dSiMtx,3,3);
}

int GetPointsNormal(double *mag,int n, double *rotAxis)
{
    double *magT = malloc(n*3*sizeof(double));
    mtxTranspose((double*)mag,magT,n,3);
    double mTm[3][3];
    mtxMultiply(magT,mag,(double*)mTm,3,n,3);
    double invmTm[3][3];
    mtxInverse((double*)mTm,(double*)invmTm,3);
    double *b = malloc(n*sizeof(double));
    int i;
    for(i=0;i<n;i++)
    {
        b[i] = 1.0;
    }
    double mTb[3];
    mtxMultiplyVec(magT,b,mTb,3,n);
    mtxMultiplyVec((double*)invmTm,mTb,rotAxis,3,3);
    free(magT);
    free(b);
    //mT = m';// m--3xn
    //b = ones(size(m,2),1);
    //v = (m*mT)\(m*b);
}

void GetMagOffset(double *dHi,double *dArrayX,double *dArrayY,
               double *dArrayZ,int *iRowNum)
{
    int totalNum;
    totalNum = iRowNum[0] + iRowNum[1] + iRowNum[2];
    int i;
    // Calculate hard iron parameters using BLS.
    double *H = malloc(4*totalNum*sizeof(double));
    double *B = malloc(totalNum*sizeof(double));
    double tmpmag[3];
    for(i=0;i<totalNum;i++)
    {
        if(i<iRowNum[0])
        {
            tmpmag[0] = dArrayX[i*3+0];
            tmpmag[1] = dArrayX[i*3+1];
            tmpmag[2] = dArrayX[i*3+2];
            H[i*4+0] = 2*tmpmag[0];
            H[i*4+1] = 2*tmpmag[1];
            H[i*4+2] = 2*tmpmag[2];
            H[i*4+3] = 1;
            B[i] = dot(tmpmag,tmpmag,3);
        }
        else if(i<(iRowNum[0]+iRowNum[1]))
        {
            tmpmag[0] = dArrayY[(i-iRowNum[0])*3+0];
            tmpmag[1] = dArrayY[(i-iRowNum[0])*3+1];
            tmpmag[2] = dArrayY[(i-iRowNum[0])*3+2];
            H[i*4+0] = 2*tmpmag[0];
            H[i*4+1] = 2*tmpmag[1];
            H[i*4+2] = 2*tmpmag[2];
            H[i*4+3] = 1;
            B[i] = dot(tmpmag,tmpmag,3);
        }
        else
        {
            tmpmag[0] = dArrayZ[(i-iRowNum[0]-iRowNum[1])*3+0];
            tmpmag[1] = dArrayZ[(i-iRowNum[0]-iRowNum[1])*3+1];
            tmpmag[2] = dArrayZ[(i-iRowNum[0]-iRowNum[1])*3+2];
            H[i*4+0] = 2*tmpmag[0];
            H[i*4+1] = 2*tmpmag[1];
            H[i*4+2] = 2*tmpmag[2];
            H[i*4+3] = 1;
            B[i] = dot(tmpmag,tmpmag,3);
        }
    }
    double *HT = malloc(4*totalNum*sizeof(double));
    mtxTranspose(H,HT,totalNum,4);
    double HTH[4][4];
    double invHTH[4][4];
    mtxMultiply(HT,H,(double*)HTH,4,totalNum,4);
    mtxInverse((double*)HTH,(double*)invHTH,4);
    double HTB[4];
    mtxMultiplyVec(HT,B,HTB,4,totalNum);
    double p[4];
    mtxMultiplyVec((double*)invHTH,HTB,p,4,4);
    dHi[0] = p[0];
    dHi[1] = p[1];
    dHi[2] = p[2];
    dHi[3] = sqrt(p[3] + dot(p,p,3));
    // Apply the hard iron parameters to input mag data
    for(i=0;i<iRowNum[0];i++)
    {
        dArrayX[i*3+0] -= dHi[0];
        dArrayX[i*3+1] -= dHi[1];
        dArrayX[i*3+2] -= dHi[2];
    }
    for(i=0;i<iRowNum[1];i++)
    {
        dArrayY[i*3+0] -= dHi[0];
        dArrayY[i*3+1] -= dHi[1];
        dArrayY[i*3+2] -= dHi[2];
    }
    for(i=0;i<iRowNum[2];i++)
    {
        dArrayZ[i*3+0] -= dHi[0];
        dArrayZ[i*3+1] -= dHi[1];
        dArrayZ[i*3+2] -= dHi[2];
    }
    free(H);
    free(B);
    free(HT);
}




