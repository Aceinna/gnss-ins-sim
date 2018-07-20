/*****************************************************************************
 * 矩阵运算\矢量运算\缓冲区操作\均值方差运算\四元数运算\均值滤波\中值滤波\一般滤波
 * Author: Dong Xiaoguang
 * Created on 2015/11/24
 *****************************************************************************/

#ifndef MY_MATH_H_INCLUDED
#define MY_MATH_H_INCLUDED

//===========================REAL or float=========================
#define PRECISION_REAL 1
#if PRECISION_REAL
	#define REAL double
#else
	#define REAL float
#endif

//===========================some constants==========================
#if PRECISION_REAL
	#define PI 3.14159265358979323846
	#define R2D 180.0/PI
#else
	#define PI 3.14159265358979323846f
	#define R2D 180.0f/PI
#endif
//======================data struct definitions======================
typedef struct _buffer
{
    REAL *d;//数据，每列是一组完整数据
    int m;//总行数
    int n;//总列数
    int i;//当前写入数据索引, -1表示没有数据
    int full;//是否存满buffer, 0表示没满，1表示满;
    int num;//当前缓冲区内数据量
}Buffer;

//===================================================================
#define	max(a,b)	((a) > (b) ? (a) : (b))
#define	min(a,b)	((a) > (b) ? (b) : (a))

//===========================math operations=========================
//-------------------------------------------------------------------
// 打印矢量的全部元素
// input:   v--矢量
//          n--矢量长度
// output:
// return:
void printVec(REAL *v, int n);

//-------------------------------------------------------------------
// 打印矩阵的全部元素
// input:   a--矢量
//          m--行数
//          n--列数
// output:
// return:
void printMtx(REAL *a, int m, int n);

//-------------------------------------------------------------------
// vector add
// a, b and c are vectors with n elements
// c = a+b
void vecAdd(REAL* a, REAL* b, REAL* c, int n);

//-------------------------------------------------------------------
// vector subtraction
// a, b and c are vectors with n elements
// c = a-b
void vecSub(REAL* a, REAL* b, REAL* c, int n);

//-------------------------------------------------------------------
// dot product
// a and b are vectors with n elements
// return a.b
REAL dot(REAL* a, REAL* b, int len);

//-------------------------------------------------------------------
// vector 2-norm
// return the 2-norm of a
REAL norm2(REAL* a, int n);

//-------------------------------------------------------------------
// 矢量归一化，1e-15还需进一步优化
// input:   a--vector to be normalized
//          len--vector length
// output:  aN--normalized vector
// return:
void vecNormalize(REAL* a, REAL* aN, int len);

//-------------------------------------------------------------------
// 求数组均值
// input:   a--vector
//          n--length of the vector
// output:
// return: mean value of the first n elements of vector
REAL vecMean(REAL *a, int n);

//-------------------------------------------------------------------
// 数组方差
// input:   a--vector
//          am--数组的均值
//          n--length of the vector
// output:
// return: variance of the first n elements of vector
REAL vecVar(REAL *a, REAL am, int n);

//-------------------------------------------------------------------
// 数组排序，升序
// input:   a--vector
//          n--length of the vector
// output:  a--sorted vector
// return:
void vecSort(REAL *a, int n);

//-------------------------------------------------------------------
// 数组中最大元素的下标
// input:   a--vector
//          n--length of the vector
// output:
// return:  index of the max element
int vecMax(REAL *a, int n);

//-------------------------------------------------------------------
// 数组中最小元素的下标
// input:   a--vector
//          n--length of the vector
// output:
// return:  index of the min element
int vecMin(REAL *a, int n);

//-------------------------------------------------------------------
// 数组中所有元素取绝对值
// input:   a--vector
//          n--length of the vector
// output:  b--abs(a)
// return:
void vecAbs(REAL *a, int n, REAL *b);

//-------------------------------------------------------------------
// 数组中所有元素求和
// input:   a--vector
//          n--length of the vector
// output:
// return:  sum of all the elements
REAL vecSum(REAL *a, int n);

//-------------------------------------------------------------------
// 矢量叉乘
// input:   a--vector
//          b--vector
// output:   c--axb
// return:
void cross(REAL* a, REAL* b, REAL* axb);

//-------------------------------------------------------------------
// vector multiplied by a constant
// intput:  a--n vector
//          b--constant scalar
//          n--vector length
// output:  c--b*a,n vector
void vecMultiplyConst(REAL* a,REAL b, REAL* c, int n);

//-------------------------------------------------------------------
// vector duplication
// input:   vSrc--源矢量
//          n--矢量长度
// output:  vDst--目标矢量
// return:
void vecDuplicate(REAL* vSrc, REAL* vDst, int n);

//-------------------------------------------------------------------
// cross product matrix of a vector
// input:   a--3-dim vector
// output:  b--3x3 matrix
// return:
void crossMtx(REAL* a, REAL *b);

//-------------------------------------------------------------------
// max absolute elements in a matrix and its corresponding index
// input:   a--a matrix
//          m--rows
//          n--columns
// output:  mMax--row number of the max absolute element
//          nMax--column number of the max absolute element
// return:  max absolute value
REAL mtxMaxAbsIdx(REAL *a, int m, int n, int *mMax, int *nMax);

//-------------------------------------------------------------------
// max absolute elements in a matrix
// input:   a--a matrix
//          m--rows
//          n--columns
// output:
// return:  max absolute value
REAL mtxMaxAbs(REAL *a, int m, int n);


//-------------------------------------------------------------------
// one row of a matrix
// input:   a--matrix
//          m--row size
//          n--col size
//          r--index of the row to be extracted
// output:  b--the r row of a
void mtxRow(REAL *a, int m, int n, int r, REAL *b);

//-------------------------------------------------------------------
// one column of a matrix
// input:   a--matrix
//          m--row size
//          n--col size
//          r--index of the column to be extracted
// output:  b--the r column of a
void mtxCol(REAL *a, int m, int n, int r, REAL *b);

//-------------------------------------------------------------------
// lower triangular part of a matrix
// input:   a--matrix
//          m--row size
//          n--col size
// output:  l--tril(a)
// return:
void mtxTril(REAL *a, int m, int n, REAL *l);

//-------------------------------------------------------------------
// upper triangular part of a matrix
// input:   a--matrix
//          m--row size
//          n--col size
// output:  u--triu(a)
// return:
void mtxTriu(REAL *a, int m, int n, REAL *u);

//-------------------------------------------------------------------
// matrix duplication
// input:   aSrc--源矩阵
//          m--矩阵行数
//          n--矩阵列数
// output:  aDst--目标矢量
// return:
void mtxDuplicate(REAL* aSrc, REAL* aDst, int m, int n);

//-------------------------------------------------------------------
// matrix multiplication
// input: a--matrix(aRow x aCol), b--matrix(aCol x bCol)
// output: c--matrix(aRow x bCol)
void mtxMultiply(REAL* a, REAL* b, REAL* c, int aRow, int aCol,int bCol);

//-------------------------------------------------------------------
// matrix multiplied by a constant
// input: a--matrix(row x col), b--constant
// output: c--b*a;
void mtxMultiplyConst(REAL* a, REAL b, REAL* c, int row, int col);

//-------------------------------------------------------------------
// matrix add
// input: a and b are matrices(row x col)
// output: c = a+b
void mtxAdd(REAL* a,REAL *b,REAL *c, int row, int col);

//-------------------------------------------------------------------
// matrix substraction
// input: a and b are matrices(row x col)
// output: c = a-b
void mtxSub(REAL* a,REAL* b,REAL* c, int row, int col);

//-------------------------------------------------------------------
// merge two matrices along the row
// input: a--mxn, b--mxq
// output: c--mx(n+q)
void mtxMergeRow(REAL* a, REAL* b, REAL* c, int m, int n, int q);

//-------------------------------------------------------------------
// merge two matrices along col
// input: a--mxn, b--pxn
// output: c--(m+p)xn
void mtxMergeCol(REAL* a, REAL* b, REAL* c, int m, int n, int p);

//-------------------------------------------------------------------
// matrix tranpose
// input: a--matrix(row x col)
// output: aT = a'
void mtxTranspose(REAL* a, REAL* aT, int row, int col);

//-------------------------------------------------------------------
// matrix multiplied by a vector
// input: a--mxn matrix, b--nx1 vector
// return: c--a*b,mx1 vector
void mtxMultiplyVec(REAL* a, REAL* b, REAL* c, int m, int n);

//-------------------------------------------------------------------
// 2x2 matrix inverse
// input: a--matrix(2 x 2)
// output: x--inv(x)
void mtxInverse2(REAL* a, REAL* x);

//-------------------------------------------------------------------
// 3x3 matrix inverse
// input: a--matrix(3 x 3)
// output: x--inv(x)
void mtxInverse3(REAL* a, REAL* x);

//-------------------------------------------------------------------
// 4x4 matrix inverse
// input: a--matrix(4 x 4)
// output: x--inv(x)
void mtxInverse4(REAL* a, REAL* x);

//-------------------------------------------------------------------
// matrix inverse
// input: a--matrix(n x n)
// output: x--inv(x)
void mtxInverse(REAL* a, REAL* x, int n);

//-------------------------------------------------------------------
// eigenvalue and eigenvector of a real symmetric matrix
// input:   a--nxn real symmetric matrix
//          n--size of matrix a
//          eps--tolerance
//          jt--max iterations
// output:  a--eigenvalue will be stored as diagonal elements in matrix a
//          u--nxn matrix containing engenvector, stored as column
// return:  1 means OK, less than 0 means tolerance is not met after jt ierations
int realSymmetricMtxEig(double *a,int n,double *v,double eps,int jt);

//-------------------------------------------------------------------
// generate diagonal matrix
// input:   a--nx1 vector containing the diagonal elements
//          n--length of the vector
// output:  b = diag(a), nxn
// return:
void diag(REAL* a, REAL* b, int n);

//-------------------------------------------------------------------
// generate identity matrix
// input:   a--matrix
//          n--size of the identity matrix
// output:  a = eye(n)
// return:
void eye(REAL *a, int n);

//-------------------------------------------------------------------
// generate zeros matrix
// input:   a--matrix
//          m--rows
//          n--columns
// output:  a = zeros(n)
// return:
void zeros(REAL *a, int m, int n);

//-------------------------------------------------------------------
// generate ones matrix
// input:   a--matrix
//          m--rows
//          n--columns
// output:  a = ones(n)
// return:
void ones(REAL *a, int m, int n);

//-------------------------------------------------------------------
// exchange two row of a matrix
// input:   a--mxn
//          m--rows
//          n--columns
//          r1,r2--rows to be exchanged, zero based
// output:
// return:  1 means success, 0 error
int mtxExchangeRow(REAL* a, int m, int n, int r1, int r2);

//-------------------------------------------------------------------
// exchange two columns of a matrix
// input:   a--mxn
//          m--rows
//          n--columns
//          c1,c2--columns to be exchanged, zero based
// output:
// return:  1 means success, 0 error
int mtxExchangeCol(REAL* a, int m, int n, int c1, int c2);

//-------------------------------------------------------------------
// 新建缓冲区
// input:   bf--缓冲区指针
//			d--缓冲区数据地址
//          m--行数
//          n--列数
// output:
// return:
void bfNew(Buffer *bf, REAL*d, int m, int n);

//-------------------------------------------------------------------
// input:   bf--buffer指针
//          d--数据指针
// output:
// return:
void bfPut(Buffer *bf, REAL* d);

//===================================================================
//-------------------------------------------------------------------
// 读取缓冲区数据
// input:   bf--buffer指针
//          idx--数据的序号，idx=0表示当前数据，idx=1表示当前数据的前1一个数据，以此类推
// output:
// return: 1表示成功，0表示idx超出范围
int bfGet(Buffer *bf,REAL *d, int idx);

//-------------------------------------------------------------------
// 清空缓冲区
// input:   bf--buffer指针
// output:
// return:
void bfClear(Buffer *bf);

//-------------------------------------------------------------------
// 对缓冲区内数据进行中值滤波
// input:   u--输入缓冲区
//          y--滤波结果
//          n--中值滤波点数
// output:
// return: 1正确，0出错
int medFilter(Buffer *u, REAL *y, int n);

//-------------------------------------------------------------------
// 均值滤波
// input:   u--输入缓冲区
//          y--滤波结果
//          n--均值滤波点数
// output:
// return: 1正确，0出错
int meanFilter(Buffer *u, REAL *y, int n);

//===================================================================
// 注意：所有四元数皆是标量在前
//-------------------------------------------------------------------
// 四元数归一化(模太小的四元数的处理方法需要更新)
// input:   q--输入四元数
// output:  qN--归一化四元数
// return:
void quatNormalize(REAL* q, REAL* qN);

//-------------------------------------------------------------------
// 四元数转方向余弦矩阵
// input:   q--输入四元数
// output:  dcm--方向余弦矩阵
// return:
void quat2DCM(REAL *q, REAL *dcm);

//-------------------------------------------------------------------
// 方向余弦矩阵转四元数
// input:   a--方向余弦矩阵
// output:  q--四元数
// return:
void dcm2Quat(REAL *a, REAL *q);

//-------------------------------------------------------------------
// 四元数共轭
// input:   q--输入四元数
// output： qconj--共轭四元数
// return:
void quatConj(REAL *q, REAL *qconj);

//-------------------------------------------------------------------
// 四元数相乘
// input:   q1--输入四元数
//          q2--输入四元数
// output:  q--q1xq2
// return:
void quatMultiply(REAL* q1, REAL* q2, REAL* q);

//-------------------------------------------------------------------
// 四元数积分
// input:   q--待积分四元数
//          w--角速度
//          t--积分步长
// output:  q--积分后四元数
// return:
void quatIntegrate(REAL* q, REAL* w, REAL t);
//void quatIntegrate2(REAL* q, REAL* w, REAL t);

//-------------------------------------------------------------------
// 线性拟合
// input:   data--存储数据的数组，data[0]最老
//          n--数组长度
// output:  a--2维数组，y = a[0]x + a[1]
// return;
void PolyFit1(REAL *data, REAL *a, int n);

//-------------------------------------------------------------------
// modulus
// input:   x--
//          y--
// output:
// return:  x - y*floor(x/y)
REAL mod(REAL x, REAL y);



#endif // MY_MATH_H_INCLUDED
