#include <GL/freeglut.h>
#include <iostream>
#include <jpeglib.h>
#include <jerror.h>
#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/Vehicle/btRaycastVehicle.h>
#include <iostream>
#include <SOIL/SOIL.h>
#include <png.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "obj.c"





#define TRUE 1
#define FALSE 0

#define NUMERO 100
#define MAXCAJAS 1000
#define MAXFICHAS 9000
#define NUMEROMUROS 900
#define NUMEROSUELO 3000
#define GREY	0
#define RED		1
#define GREEN	2
#define BLUE	3
#define CYAN	4
#define MAGENTA	5
#define YELLOW	6
#define BLACK	7

#define RENDER_WIDTH 800.0
#define RENDER_HEIGHT 800.0

#define RADIOMAXSOMBRA 30;
#define NPI 3.14159265358979323846
#define ESCALA 10
float rotacioncielo=0.0;
int girogradoscuerpo=0;
int cruzx=RENDER_WIDTH/2;
int cruzy=RENDER_HEIGHT/2;
int texturascajas[MAXCAJAS];
btRigidBody* chassisRigidBody;
float vara=0.0;
bool pintacircuito=false,fisica=true;
bool pintamuros=false;
bool freelook=true;
struct obj_model_t objfilelow, cuerpoobj, cabezaobj,beetleobj,ruedaobj,cieloobj,dunasobj,dominoobj;
int gradosgirocuerpo=0;
btVector3 *posver;
btVector3 *rotver;
float *alfaver;
bool vistavala=false;
inline double agrados(double radianes) {
    return radianes*(180.0/NPI);
}

GLfloat distancia(GLfloat *p1, GLfloat *p2){
	return (GLfloat) sqrt((p2[0]-p1[0])*(p2[0]-p1[0])+(p2[1]-p1[1])*(p2[1]-p1[1])+(p2[2]-p1[2])*(p2[2]-p1[2]));
}

GLuint texturecaja[4]; //the array for our texture
GLuint texturemuro,texturatecho,texturaventana;
GLuint texturacabeza,texturaobjeto,texturarueda;
GLuint texturacuerpo;
GLuint nubes,dunas;
GLubyte *datostexturaobj;
int anchotextura,altotextura;
GLuint textureesfe; //the array for our texture
GLuint texturadomino;
GLuint skybox[6]; 
GLuint texturasuelo;
int nobolas=0;
int moving, beginx, beginy;
float alfax=0.0,alfay=0.0;
bool is_first_time=true;
const int TEXDIM = 64;
float anchomuro[NUMEROMUROS];
float angulomuro[NUMEROMUROS];

float anchosuelo[NUMEROMUROS];
float angulosuelo[NUMEROMUROS];
//btVector3 pospersonaje=btVector3(13, altura+28.5, -96.5);
btVector3 pospersonaje=btVector3(13, 1.0, -80.5);
btVector3 posnave=btVector3(30.0,.5,-85.0);
//camara circuito
btVector3 vectorvision =btVector3(-1.000000, 0.052336, 0.00);
btVector3 eyePos=btVector3(17.526489, 1.093270, -1.14161);
/////vehiculo
btVector3 origenvehiculo=btVector3(0,0,1);
btVehicleRaycaster* raycaster;
btRaycastVehicle* vehicle;


float   acel=2.0f;
float	gEngineForce = 0.f;
float	gBreakingForce = 0.f;

float	maxEngineForce = 150.f;//this should be engine/velocity dependent
float	maxBreakingForce = 75.f;

float	gVehicleSteering = 0.f;
float	steeringIncrement = 0.03f;
float	steeringClamp = 0.05f;

btVector3 wheelDirection(0.0f, -1.0f, 0.0f);
btVector3 wheelAxis(-1.0f, 0.0f, 0.0f);

btScalar wheelRadius(.5f);


float	wheelWidth = 0.4f;
float	wheelFriction = 1000;//BT_LARGE_FLOAT;
float	suspensionStiffness = 10.f;
float	suspensionDamping = 2.3f;
float	suspensionCompression = 0.6f;
float	rollInfluence = 0.1f;//1.0f;
int rightIndex = 0; 
int upIndex = 2; 
int forwardIndex = 1;
btScalar suspensionRestLength(0.075f);


btDefaultMotionState *vehiculomotionstate;
btCompoundShape* mallacompuesta;



//camara tablero
//GLfloat eyePos[3] = {0.0, 10.0, 20.0};
//GLfloat vectorvision[3]={0.0,-1.0,-2.0};

GLfloat objectPos[4] = { 0.0, 0.0, 0.0, 1.0 };
float r=2.5f; //ancho pista
float d=10;
float alfa=0.001;
float dif=NPI/d;
int altura=0;
int nocajas=0,nofichas=0;
enum {
	X=0, Y, Z
};

int barrotes=0;
/* Display list names. */


btBroadphaseInterface* broadphase;
btDefaultCollisionConfiguration* collisionConfiguration;

btCollisionDispatcher* dispatcher;
btSequentialImpulseConstraintSolver* solver;
btTriangleMesh *mTriMesh;
btDiscreteDynamicsWorld* dynamicsWorld;
btConvexTriangleMeshShape *mayamodelocircuito;
btCollisionShape* suelocoll;
btBoxShape* pruebashape;
btConvexShape* circuitocolshape;
btCollisionShape* fallShapecoll[NUMERO],*bloque;
btCollisionShape* muroshapecol[NUMEROMUROS];
btCollisionShape* sueloshapecol[NUMEROSUELO];
btCollisionShape* tubomallacol;
btCollisionShape* paredcol[4];
btCollisionShape* cuboShapecoll[MAXCAJAS];
btCollisionShape* fichashapecol[MAXFICHAS];
btCollisionShape* personajecolshape;
btCollisionShape* navecolshape[7];
btDefaultMotionState* personajeestado;
btDefaultMotionState* navemotionstate[7];
btDefaultMotionState* groundMotionState;
btDefaultMotionState* paredMotionState[4];
btDefaultMotionState* muroMotionState[NUMEROMUROS];
btDefaultMotionState* sueloMotionState[NUMEROSUELO];
btDefaultMotionState* fichasMotionState[MAXFICHAS];
btDefaultMotionState* tubosestado;
btDefaultMotionState* circuitoMotionState,pruebamotionstate;
btDefaultMotionState* ds;
btIndexedMesh im;
btTriangleMesh* triangulos ;
btBvhTriangleMeshShape *trianguloss;

btTriangleIndexVertexArray* indvtriangulos;

btRigidBody* groundRigidBody;
btRigidBody* circuitoRB,*pruebarigidbody;
btDefaultMotionState* fallMotionState;
btRigidBody* fallRigidBody[NUMERO];
btRigidBody* murorb[NUMEROMUROS];
btRigidBody* suelorb[NUMEROSUELO];
btRigidBody* tubosrb;
btRigidBody* cuborb[MAXCAJAS];
btRigidBody* paredrb[4];
btRigidBody* personajerb;
btRigidBody* naverb[7];
btTriangleMesh* btojb;
btTransform t1, tcaja, trans[NUMERO], pt[4], tc[MAXCAJAS],tm[NUMEROMUROS],ts[NUMEROSUELO],tp,tn,tf[MAXFICHAS];
btTransform vt;
btRigidBody* ficharb[MAXFICHAS];

btCollisionShape* cajacs;

btTriangleIndexVertexArray *m_indexVertexArrays;

float desp = -1.5;

btRigidBody* objrb;
btCollisionShape* csobj;
float diffuse[] = { 1, 1, 1, 1 };
float position[] = { 1, 7, 1, 1 };

float angulo = 0.0f;
//texturas

unsigned long x;
unsigned long y;
unsigned short int bpp; //bits per pixels   unsigned short int
unsigned char* data;             //the data of the image
unsigned int ID;                //the id ogl gives it
unsigned long sf;     //length of the file
int channels;      //the channels of the image 3 = RGA 4 = RGBA
GLuint type;

/* Normals for the 6 faces of a cube. */

GLfloat n[6][3] = { { -1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 1.0, 0.0, 0.0 }, {
		0.0, -1.0, 0.0 }, { 0.0, 0.0, 1.0 }, { 0.0, 0.0, -1.0 } };
GLint faces[6][4] = { /* Vertex indices for the 6 faces of a cube. */
{ 0, 1, 2, 3 }, { 3, 2, 6, 7 }, { 7, 6, 5, 4 }, { 4, 5, 1, 0 }, { 5, 6, 2, 1 },
		{ 7, 4, 0, 3 } };
GLfloat v[8][3]; /* Will be filled in with X,Y,Z vertexes. */

typedef struct {
	int X;
	int Y;
	int Z;
	double U;
	double V;
} VERTICES;

typedef struct{
GLuint 	vindices[3];
GLuint 	nindices[3];
GLuint 	tindices[3];
GLuint 	findex;

} GLMtriangle;

const double PI = 3.1415926535897;
const int space = 10;
const int VertexCount = (90 / space) * (360 / space) * 4;
VERTICES VERTEX[NUMERO][VertexCount];

//animacion md2

float camino = -9.0;
int tick = 1;

//luces 

GLfloat LightAmbient[] = { 0.6f, 0.6f, 0.6f, 1.0f }; // Ambient Light Values ( NEW )
GLfloat LightDiffuse[] = { .7f, .7f, .7f, 1.0f }; // Diffuse Light Values ( NEW )
GLfloat LightPosition[] = { 0.0f, 0.0f, 15.0f, 1.0f }; // Light Position ( NEW )
GLfloat spotDir[] = { 0.0f, 0.0f, 0.0f };
static float lightSpec[4] = { 0.4, 0.4, 0.4, 1.0 };

//punto de luz
GLfloat lightPos[3] = { 0.0f, 0.0f, 15.0f };
GLfloat posobj[3] = { 0.0f, 0.0f, 0.0f };

static float lightSPos[4] = { 0.0, 10.0, 0.0, 1.0 };    //{5.0, 15.0, 5.0, 1.0};
//static float lightSDir[4] ={0.0,-1.0, 0.0, 1.0};
GLfloat exponent = 30.0f;
GLfloat cutoff = 60.0f;

GLfloat att = 0.6f;



static float materialColor[8][4] = { { 0.8, 0.8, 0.8, 1.0 }, { 0.8, 0.0, 0.0,
		1.0 }, { 0.0, 0.8, 0.0, 1.0 }, { 0.0, 0.0, 0.8, 1.0 }, { 0.0, 0.8, 0.8,
		1.0 }, { 0.8, 0.0, 0.8, 1.0 }, { 0.8, 0.8, 0.0, 1.0 }, { 0.0, 0.0, 0.0,
		0.6 }, };


#include "funciones.cpp"
#include "renderScene.cpp"
#include "specialkeyboard.cpp"



int main(int argc, char **argv) {

	// init GLUT and create window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STENCIL);
	glutInitWindowPosition(150, 100);
	glutInitWindowSize(RENDER_WIDTH, RENDER_HEIGHT);
	glutCreateWindow("Proyecto");

	// register callbacks

	glutDisplayFunc(renderScene);
	glutReshapeFunc(changeSize);
	glutIdleFunc(renderScene);
	glutMotionFunc(motion);
    glutMouseFunc(mouse); 
	glutSpecialFunc(specialkeyboard);
	glutKeyboardFunc(specialkeyboard);

	//glutSpecialFunc(keyboard);

	LoadTextureplano("texturacemento.jpg", 256, 256,&texturemuro); //load the texture
	LoadTextureplano("crate difuse.jpg", 894, 894,&texturecaja[0]); //load the texture
	LoadTextureplano("texturacaja.jpg", 512, 512,&texturecaja[1]); //load the texture
	LoadTextureplano("crate.jpg", 512, 512,&texturecaja[2]); //load the texture
	LoadTextureplano("crate_1.jpg", 512, 512,&texturecaja[3]); //load the texture
	//LoadTextureplano("char.jpg", 64, 32,&texturacuerpo); //load the texture
	//LoadTextureplano("char.jpg", 64, 32,&texturacabeza); //load the texture
//for(int z=0;z<6;z++){
//	LoadTextureplano("skyboxcielo.jpg", 1024, 1024,&skybox[z]); //load the texture
//}
	
	anchotextura=64;
	altotextura=32;
	bool alpha=true;
	texturacabeza = SOIL_load_OGL_texture("char.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);

	nubes = SOIL_load_OGL_texture("cieloesfera.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	dunas = SOIL_load_OGL_texture("dunasfondo.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	skybox[0] = SOIL_load_OGL_texture("Side5.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	skybox[1] = SOIL_load_OGL_texture("Side2.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	skybox[2] = SOIL_load_OGL_texture("Side3.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	skybox[3] = SOIL_load_OGL_texture("Side4.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	skybox[4] = SOIL_load_OGL_texture("Side1.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	skybox[5] = SOIL_load_OGL_texture("Side6.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);

	
	texturasuelo = SOIL_load_OGL_texture("terrain.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);



	
	//loadPngImage("char.png", anchotextura, altotextura, alpha, &datostexturaobj);
	/*
	LoadTextureplano("skyback.jpg", 256, 256,&skybox[0]); //load the texture
	LoadTextureplano("skyright.jpg", 256, 256,&skybox[1]); //load the texture
	LoadTextureplano("skyfront.jpg", 256, 256,&skybox[2]); //load the texture
	LoadTextureplano("skyleft.jpg", 256, 256,&skybox[3]); //load the texture
	LoadTextureplano("skydown.jpg", 256, 256,&skybox[4]); //load the texture
	LoadTextureplano("skyup.jpg", 256, 256,&skybox[5]); //load the texture
*/

	texturatecho = SOIL_load_OGL_texture("texturacarretera.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaventana = SOIL_load_OGL_texture("angry.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturaobjeto =  SOIL_load_OGL_texture("beatle.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturadomino =  SOIL_load_OGL_texture("domino.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	texturarueda =  SOIL_load_OGL_texture("rueda.png",SOIL_LOAD_AUTO,	SOIL_CREATE_NEW_ID,	SOIL_FLAG_MIPMAPS | SOIL_FLAG_INVERT_Y | SOIL_FLAG_NTSC_SAFE_RGB | SOIL_FLAG_COMPRESS_TO_DXT);
	ReadOBJModel ("carretera.obj", &objfilelow);
	ReadOBJModel ("angry.obj", &objfile);
	ReadOBJModel ("coche.obj", &beetleobj);
	ReadOBJModel ("cielo.obj", &cieloobj);
	ReadOBJModel ("rueda.obj", &ruedaobj);
	ReadOBJModel ("dunas.obj", &dunasobj);
	ReadOBJModel ("domino.obj", &dominoobj);
	
//inicia la fisica y pos objetos
	inicio();
	
	//init("circuitotubohd.obj"); //carga el .obj en la estructura
	ReadOBJModel ("MinecraftPlayerbody.obj", &cuerpoobj);
	ReadOBJModel ("MinecraftPlayercabeza.obj", &cabezaobj);
	barrotes=objfilelow.num_faces;
	glEnable(GL_LIGHTING);
	glEnable(GL_CULL_FACE);
	glEnable(GL_COLOR_MATERIAL);
	glCullFace(GL_BACK);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glShadeModel(GL_SMOOTH);							// Enable Smooth Shading
	glClearDepth(1.0f);								// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
	glDepthFunc(GL_LEQUAL);					// The Type Of Depth Testing To Do
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);// Really Nice Perspective Calculations

	glClearColor(0.15, .15, 0.15, 1.0); //selecciona color de fondo

	glutMainLoop();
	fin();
	return 1;
}
