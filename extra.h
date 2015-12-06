#include <windows.h>
#include <mmsystem.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include "glut.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <assert.h>

#define TRUE	1
#define FALSE	0

#define MIN(a, b)   ( ((a) < (b))? (a):(b) )

#define WIDTH 640
#define HEIGHT 480
#define RAD2DEG(x)  ((x)*180.0/3.1415926384)    
#define DEG2RAD(x)  ((x)*3.1415926384/180)    

void w32_ReadPixels(void);
void w32_DrawPixels(void);
unsigned char *w32_init(char *window_name);

typedef struct
{
    float x, y, z;
    float nx, ny, nz;
} XVERTEX;

typedef struct
{
    float nx, ny, nz;
    float r, g, b;
    int nvertex;
    XVERTEX **xvertex;
    int face_light;
    int face_eye;
} XPOLYGON;

typedef struct
{
    XVERTEX *a, *b;
    XPOLYGON *l, *r;
    XVERTEX shadowVertex[4];
    int has_shadow;
    int face_eye;
    float nx, ny, nz;
} XEDGE;

typedef struct tagXOBJ
{
    float x, y, z;
    float rx, ry, rz;
    int npolygon;
    XPOLYGON *xpolygon;
    int nvertex;
    XVERTEX *xvertex;
    int nedge;
    XEDGE *xedge;

    struct tagXOBJ *next;
} XOBJ;

void xAddQueue(XOBJ *obj);
void xDrawObj(XOBJ *obj, int shadow_mode);
void xDrawObjs(int shadow_mode);
void xDrawShadowVolume(XOBJ *obj, int face_eye);
void xDrawShadowVolumes(int face_eye);
XOBJ *xCreateSphere(float size, float r, float g, float b);
XOBJ *xCreateCube(float size, float r, float g, float b);
XOBJ *xCreateBox(float xsize, float ysize, float zsize, float r, float g, float b);
void xCalcShadowVolume(XOBJ *obj, float lx, float ly, float lz, float ex, float ey, float ez);

