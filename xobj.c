#include "extra.h"

static XOBJ *global_draw_queue = NULL;

/*******************************************************
 *                                                     *
 * xTransformVector                                    *
 *                                                     *
 * project a vector from a matrix                      *
 *                                                     *
 *******************************************************/
static void xTransformVector(float *x, float *y, float *z, float matrix[])
{
    float nx = (*x) * matrix[0] + 
	       (*y) * matrix[4] +
	       (*z) * matrix[8] +
	       matrix[12];

    float ny = (*x) * matrix[1] + 
	       (*y) * matrix[5] +
	       (*z) * matrix[9] +
	       matrix[13];

    float nz = (*x) * matrix[2] + 
	       (*y) * matrix[6] +
	       (*z) * matrix[10] +
	       matrix[14];

    *x = nx;
    *y = ny;
    *z = nz;
}

/*******************************************************
 *                                                     *
 * xCanPolygonSeePoint                                 *
 *                                                     *
 * does the polygon face the point                     *
 *                                                     *
 *******************************************************/
static int xCanPolygonSeePoint(XPOLYGON *poly, float matrix[], float lx, float ly, float lz)
{
    float x = poly->xvertex[0]->x;
    float y = poly->xvertex[0]->y;
    float z = poly->xvertex[0]->z;

    float vpx = x+poly->nx;
    float vpy = y+poly->ny;
    float vpz = z+poly->nz;

    xTransformVector(&vpx, &vpy, &vpz, matrix);
    xTransformVector(&x, &y, &z, matrix);

    vpx -= x;
    vpy -= y;
    vpz -= z;

    lx -= x;
    ly -= y;
    lz -= z;

    return ((vpx*lx + vpy*ly + vpz*lz) > 0)?TRUE:FALSE;
}

/*******************************************************
 *                                                     *
 * xCanEdgeSeePoint                                    *
 *                                                     *
 * does the edge face the point                        *
 *                                                     *
 *******************************************************/
static int xCanEdgeSeePoint(XEDGE *edge, float matrix[], float lx, float ly, float lz)
{
    float x = edge->a->x;
    float y = edge->a->y;
    float z = edge->a->z;

    float vpx = x+edge->nx;
    float vpy = y+edge->ny;
    float vpz = z+edge->nz;

    xTransformVector(&vpx, &vpy, &vpz, matrix);
    xTransformVector(&x, &y, &z, matrix);

    vpx -= x;
    vpy -= y;
    vpz -= z;

    lx -= x;
    ly -= y;
    lz -= z;

    return ((vpx*lx + vpy*ly + vpz*lz) > 0)?TRUE:FALSE;
}

/*******************************************************
 *                                                     *
 * xCalcShadowVolume                                   *
 *                                                     *
 * calculate the shadow volume for the object          *
 *                                                     *
 *******************************************************/
void xCalcShadowVolume(XOBJ *obj, float lx, float ly, float lz, float ex, float ey, float ez)
{
    float mvmatrix[16];
    int p, e;

    glPushMatrix();
    glLoadIdentity();
    glTranslatef(obj->x, obj->y, obj->z);
    glRotatef(obj->rx, 1, 0, 0);
    glRotatef(obj->ry, 0, 1, 0);
    glRotatef(obj->rz, 0, 0, 1);
    glGetFloatv(GL_MODELVIEW_MATRIX, mvmatrix);

    for(p=0; p<obj->npolygon; ++p)
    {
	obj->xpolygon[p].face_light = 
	    xCanPolygonSeePoint(&obj->xpolygon[p], mvmatrix, lx, ly, lz);
    }

    glPopMatrix();

    for(e=0; e<obj->nedge; ++e)
    {
	XEDGE *edge = &obj->xedge[e];

	edge->has_shadow = FALSE;

	if(!edge->l->face_light && 
	    (edge->r == NULL || !edge->r->face_light) )
	    continue;
	    
	if(edge->l->face_light &&
	    (edge->r != NULL && edge->r->face_light) )
	    continue;

	edge->face_eye = xCanEdgeSeePoint(edge, mvmatrix, ex, ey, ez);

	edge->has_shadow = TRUE;

	edge->shadowVertex[0].x = edge->a->x;
	edge->shadowVertex[0].y = edge->a->y;
	edge->shadowVertex[0].z = edge->a->z;
	xTransformVector(&edge->shadowVertex[0].x,
			&edge->shadowVertex[0].y,
			&edge->shadowVertex[0].z, mvmatrix);
	edge->shadowVertex[1].x = edge->b->x;
	edge->shadowVertex[1].y = edge->b->y;
	edge->shadowVertex[1].z = edge->b->z;
	xTransformVector(&edge->shadowVertex[1].x,
			&edge->shadowVertex[1].y,
			&edge->shadowVertex[1].z, mvmatrix);

	edge->shadowVertex[2].x =
	    100*(edge->shadowVertex[1].x - lx) + lx;
	edge->shadowVertex[2].y =
	    100*(edge->shadowVertex[1].y - ly) + ly;
	edge->shadowVertex[2].z =
	    100*(edge->shadowVertex[1].z - lz) + lz;

	edge->shadowVertex[3].x =
	    100*(edge->shadowVertex[0].x - lx) + lx;
	edge->shadowVertex[3].y =
	    100*(edge->shadowVertex[0].y - ly) + ly;
	edge->shadowVertex[3].z =
	    100*(edge->shadowVertex[0].z - lz) + lz;
    }
}

/*******************************************************
 *                                                     *
 * xDrawObj                                            *
 *                                                     *
 * draw the object                                     *
 *                                                     *
 *******************************************************/
void xDrawObj(XOBJ *obj, int shadow_mode)
{
    int p;
    XVERTEX **q;

    glPushMatrix();
    glTranslatef(obj->x, obj->y, obj->z);
    glRotatef(obj->rx, 1, 0, 0);
    glRotatef(obj->ry, 0, 1, 0);
    glRotatef(obj->rz, 0, 0, 1);
    for(p=0; p<obj->npolygon; ++p)
    {
	XPOLYGON *poly = &obj->xpolygon[p];

	if(shadow_mode)
	    glColor3f(poly->r/2, poly->g/2, poly->b/2);
	else
	    glColor3f(poly->r, poly->g, poly->b);

	glBegin(GL_POLYGON);
	for(q = obj->xpolygon[p].xvertex; *q != NULL; ++q)
	{
	    glNormal3f((*q)->nx, (*q)->ny, (*q)->nz);
	    glVertex3f((*q)->x, (*q)->y, (*q)->z);
	}
	glEnd();
    }
    glPopMatrix();
}

/*******************************************************
 *                                                     *
 * xDrawShadowVolume                                   *
 *                                                     *
 * draw the object's shadow volume                     *
 *                                                     *
 *******************************************************/
void xDrawShadowVolume(XOBJ *obj, int face_eye)
{
    int e, i;

    glBegin(GL_QUADS);
    for(e=0; e<obj->nedge; ++e)
	if(obj->xedge[e].has_shadow && obj->xedge[e].face_eye == face_eye)
	{
	    for(i=0; i<4; ++i)
		glVertex3f(obj->xedge[e].shadowVertex[i].x,
		    obj->xedge[e].shadowVertex[i].y,
		    obj->xedge[e].shadowVertex[i].z);
	}
    glEnd();
}

/*******************************************************
 *                                                     *
 * xNewObj                                             *
 *                                                     *
 * create a new object                                 *
 *                                                     *
 *******************************************************/
XOBJ *xNewObj(int nvertex, int npolygon)
{
    XOBJ *obj = malloc(sizeof(XOBJ));

    obj->x = obj->y = obj->z =
    obj->rx = obj->ry = obj->rz = 0;

    obj->nvertex = nvertex;
    obj->xvertex = malloc(sizeof(XVERTEX)*nvertex);
    obj->npolygon = npolygon;
    obj->xpolygon = malloc(sizeof(XPOLYGON)*npolygon);

    return obj;
}
 
/*******************************************************
 *                                                     *
 * xSetVertex                                          *
 *                                                     *
 * set the value of a vertex                           *
 *                                                     *
 *******************************************************/
void xSetVertex(XOBJ *obj, int vertex, float x, float y, float z)
{
    assert(vertex >= 0 && vertex < obj->nvertex);

    obj->xvertex[vertex].x = x;
    obj->xvertex[vertex].y = y;
    obj->xvertex[vertex].z = z;
}

/*******************************************************
 *                                                     *
 * xSetPolygon                                         *
 *                                                     *
 * set the value of a polygon                          *
 *                                                     *
 *******************************************************/
void xSetPolygon(XPOLYGON *poly, int nvertex, float nx, float ny, float nz,
		 float r, float g, float b)
{
    poly->nx = nx;
    poly->ny = ny;
    poly->nz = nz;
    poly->r = r;
    poly->g = g;
    poly->b = b;
    poly->nvertex = nvertex;
    poly->xvertex = malloc(sizeof(XVERTEX *)*(nvertex+1));
    poly->xvertex[nvertex] = NULL;
}

/*******************************************************
 *                                                     *
 * xCalcNormal                                         *
 *                                                     *
 * calculate the normals for the polygon               *
 *                                                     *
 *******************************************************/
void xCalcNormal(XOBJ *obj, XPOLYGON *poly)
{
    XVERTEX *v0 = poly->xvertex[0];
    XVERTEX v1 = *(poly->xvertex[1]);
    XVERTEX v2 = *(poly->xvertex[2]);

    float x, y, z, dist;

    assert(poly->nvertex >= 3);

    v1.x -= v0->x;
    v1.y -= v0->y;
    v1.z -= v0->z;

    v2.x -= v0->x;
    v2.y -= v0->y;
    v2.z -= v0->z;

    x = v1.y * v2.z - v1.z * v2.y;
    y = v1.z * v2.x - v1.x * v2.z;
    z = v1.x * v2.y - v1.y * v2.x;

    dist = sqrt(x*x + y*y + z*z);

    assert(dist > 0);

    x /= dist;
    y /= dist;
    z /= dist;

    if( (poly->nx > 0 && x < 0) ||
	(poly->nx < 0 && x > 0) ||
	(poly->ny > 0 && y < 0) ||
	(poly->ny < 0 && y > 0) ||
	(poly->nz > 0 && z < 0) ||
	(poly->nz < 0 && z > 0) )
    {
	x = -x;
	y = -y;
	z = -z;
    }

    poly->nx = x;
    poly->ny = y;
    poly->nz = z;
}

/*******************************************************
 *                                                     *
 * xSetPolygonTri                                      *
 *                                                     *
 * set the value of a triangle polygon                 *
 *                                                     *
 *******************************************************/
void xSetPolygonTri(XOBJ *obj, int polygon, float nx, float ny, float nz,
    int v0, int v1, int v2, float r, float g, float b)
{
    XPOLYGON *poly = &obj->xpolygon[polygon];

    assert(polygon >= 0 && polygon < obj->npolygon);

    xSetPolygon(poly, 3, nx, ny, nz, r, g, b);
    poly->xvertex[0] = &obj->xvertex[v0];
    poly->xvertex[1] = &obj->xvertex[v1];
    poly->xvertex[2] = &obj->xvertex[v2];
    xCalcNormal(obj, poly);
}

/*******************************************************
 *                                                     *
 * xSetPolygonQuad                                     *
 *                                                     *
 * set the value of a quad polygon                     *
 *                                                     *
 *******************************************************/
void xSetPolygonQuad(XOBJ *obj, int polygon, float nx, float ny, float nz,
    int v0, int v1, int v2, int v3, float r, float g, float b)
{
    XPOLYGON *poly = &obj->xpolygon[polygon];

    assert(polygon >= 0 && polygon < obj->npolygon);

    xSetPolygon(poly, 4, nx, ny, nz, r, g, b);
    poly->xvertex[0] = &obj->xvertex[v0];
    poly->xvertex[1] = &obj->xvertex[v1];
    poly->xvertex[2] = &obj->xvertex[v2];
    poly->xvertex[3] = &obj->xvertex[v3];
    xCalcNormal(obj, poly);
}


/*******************************************************
 *                                                     *
 * xCalcEdge                                           *
 *                                                     *
 * calculate object edges                              *
 *                                                     *
 *******************************************************/
void xCalcEdge(XOBJ *obj, int nedge)
{
    int e, i, p;
    XVERTEX **q, *v1, *v2;
    XEDGE *edge;

    obj->nedge = nedge;
    obj->xedge = malloc(sizeof(XEDGE)*nedge);

    for(e=0, p=0; p<obj->npolygon; ++p)
    {
        for(q = obj->xpolygon[p].xvertex; *q != NULL; ++q)
	{
	    v1 = *q;
	    v2 = *(q+1);

	    if(v2 == NULL)
		v2 = *(obj->xpolygon[p].xvertex);

	    for(i=0; i<e; ++i)
		if((obj->xedge[i].a == v1 && obj->xedge[i].b == v2)||
		   (obj->xedge[i].a == v2 && obj->xedge[i].b == v1))
		   break;

	    if(i<e)
	    {
		// old edge
		assert(obj->xedge[i].r == NULL);
		obj->xedge[i].r = &obj->xpolygon[p];
	    }
	    else
	    {
		// new edge
		assert(e < nedge);
		edge = &obj->xedge[e++];
		edge->a = v1;
		edge->b = v2;
		edge->l = &obj->xpolygon[p];
		edge->r = NULL;

		obj->xedge[i].nx = v1->nx + v2->nx;
		obj->xedge[i].ny = v1->ny + v2->ny;
		obj->xedge[i].nz = v1->nz + v2->nz;
	    }
	}
    }
    assert(e == nedge);	
}

/*******************************************************
 *                                                     *
 * xCalcVertexNormal                                   *
 *                                                     *
 * calculate the object's verticies' normals           *
 *                                                     *
 *******************************************************/
XOBJ *xCalcVertexNormal(XOBJ *obj)
{
    int v, p;
    float nx, ny, nz, d;
    XVERTEX **q;

    for(v=0; v<obj->nvertex; ++v)
    {
        XVERTEX *this_v = &obj->xvertex[v]; 

	nx = ny = nz = 0;

	for(p=0; p<obj->npolygon; ++p)
	    for(q = obj->xpolygon[p].xvertex; *q != NULL; ++q)
		if(*q == this_v)
		{
		    nx += obj->xpolygon[p].nx;
		    ny += obj->xpolygon[p].ny;
		    nz += obj->xpolygon[p].nz;
		}

	d = sqrt(nx*nx + ny*ny + nz*nz);

	assert(d > 0);

	nx /= d;
	ny /= d;
	nz /= d;

	this_v->nx = nx;
	this_v->ny = ny;
	this_v->nz = nz;
    }
    return obj;
}

/*******************************************************
 *                                                     *
 * xCreateBox                                          *
 *                                                     *
 * create a box object                                 *
 *                                                     *
 *******************************************************/
XOBJ *xCreateBox(float xsize, float ysize, float zsize, float r, float g, float b)
{
    XOBJ *obj = xNewObj(8, 6);

    xSetVertex(obj, 0,  xsize,  ysize,  zsize);
    xSetVertex(obj, 1, -xsize,  ysize,  zsize);
    xSetVertex(obj, 2, -xsize,  ysize, -zsize);
    xSetVertex(obj, 3,  xsize,  ysize, -zsize);
    xSetVertex(obj, 4,  xsize, -ysize,  zsize);
    xSetVertex(obj, 5, -xsize, -ysize,  zsize);
    xSetVertex(obj, 6, -xsize, -ysize, -zsize);
    xSetVertex(obj, 7,  xsize, -ysize, -zsize);

    xSetPolygonQuad(obj, 0,  0,  1,  0, 0, 1, 2, 3, r, g, b);
    xSetPolygonQuad(obj, 1,  0, -1,  0, 4, 5, 6, 7, r, g, b);
    xSetPolygonQuad(obj, 2,  0,  0,  1, 0, 1, 5, 4, r, g, b);
    xSetPolygonQuad(obj, 3,  0,  0, -1, 3, 2, 6, 7, r, g, b);
    xSetPolygonQuad(obj, 4,  1,  0,  0, 0, 3, 7, 4, r, g, b);
    xSetPolygonQuad(obj, 5, -1,  0,  0, 1, 2, 6, 5, r, g, b);

    xCalcVertexNormal(obj);
    xCalcEdge(obj, 12);

    return obj;
}

/*******************************************************
 *                                                     *
 * xCreateCube                                         *
 *                                                     *
 * create a cube object                                *
 *                                                     *
 *******************************************************/
XOBJ *xCreateCube(float size, float r, float g, float b)
{
    return xCreateBox(size, size, size, r, g, b);
}

#define RING_PTS 16

/*******************************************************
 *                                                     *
 * xCreateSphere                                       *
 *                                                     *
 * create a sphere object                              *
 *                                                     *
 *******************************************************/
XOBJ *xCreateSphere(float radius, float r, float g, float b)
{
    XOBJ *obj = xNewObj(RING_PTS*3+2, RING_PTS*4);

    float subradius = 1.0/sqrt(2)*radius;
    float two_pi_eigth = 3.1415928f*2/RING_PTS;

    int i, v = 0, top_v, bottom_v;

    // rings
    for(i = 0; i < RING_PTS; ++i)
    {
	float angle = two_pi_eigth*i;
        xSetVertex(obj, v, sin(angle)*radius, 0, 
	    cos(angle)*radius);
        xSetVertex(obj, v+RING_PTS, sin(angle)*subradius, 
	    subradius, cos(angle)*subradius);
        xSetVertex(obj, v+RING_PTS*2, sin(angle)*subradius, 
	    -subradius, cos(angle)*subradius);

	v++;
    }

    v += RING_PTS*2;

    // top/bottom
    xSetVertex(obj, v, 0, radius, 0); 
    top_v = v++;

    xSetVertex(obj, v, 0, -radius, 0); 
    bottom_v = v++;

    assert(v == obj->nvertex);
    v = 0;

    // generate quads
    for(i = 0; i < RING_PTS; ++i)
    {
	int inext = (i+1)%RING_PTS;
        xSetPolygonQuad(obj, v, 0,1,0, i, i+RING_PTS, 
	    inext+RING_PTS, inext, r, g, b);
        xSetPolygonQuad(obj, v+RING_PTS, 0,-1,0, i, i+RING_PTS*2, 
	    inext+RING_PTS*2, inext, r, g, b);
	++v;
    }

    v += RING_PTS;

    // generate triangles
    for(i = 0; i < RING_PTS; ++i)
    {
	int inext = (i+1)%RING_PTS;
        xSetPolygonTri(obj, v, 0,1,0, i+RING_PTS, 
	    inext+RING_PTS, top_v, r, g, b);
        xSetPolygonTri(obj, v+RING_PTS, 0,-1,0, i+RING_PTS*2, 
	    inext+RING_PTS*2, bottom_v, r, g, b);

	++v;
    }

    v += RING_PTS;

    assert(v == obj->npolygon);

    xCalcVertexNormal(obj);
    xCalcEdge(obj, RING_PTS*3+RING_PTS*4);

    return obj;
}


/*******************************************************
 *                                                     *
 * xAddQueue                                           *
 *                                                     *
 * add to rendering queue                              *
 *                                                     *
 *******************************************************/
void xAddQueue(XOBJ *obj)
{
    obj->next = global_draw_queue;
    global_draw_queue = obj;
}

/*******************************************************
 *                                                     *
 * xDrawObjs                                           *
 *                                                     *
 * draw objects in rendering queue                     *
 *                                                     *
 *******************************************************/
void xDrawObjs(int face_eye)
{
    XOBJ *obj;
    for(obj = global_draw_queue; obj != NULL; obj = obj->next)
	xDrawObj(obj, face_eye);
}

/*******************************************************
 *                                                     *
 * xDrawShadowVolumes                                  *
 *                                                     *
 * draw objects's shadow volumes in rendering queue    *
 *                                                     *
 *******************************************************/
void xDrawShadowVolumes(int has_shadow)
{
    XOBJ *obj;
    for(obj = global_draw_queue; obj != NULL; obj = obj->next)
	xDrawShadowVolume(obj, has_shadow);
}
