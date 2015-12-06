#define TRUE 1
#define FALSE 0
#include <GL/freeglut.h>
#include <boost/asio.hpp>
#define MAXPISO 10
#define NUMERO 50
#define MAXCAJAS 50
#define MAXPUERTAS 20
#define NUMEROMUROS 1
#define NUMEROSUELO 1
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

#define RADIOMAXSOMBRA 30
#define NPI 3.14159265358979323846

#define ANCHOMAPA 1024

#define CAMPOVISION 96

#define CAMBIOCUADRO 12

int oct=6;


float frecuencia=ANCHOMAPA/24;

int suavizado=5;

int inii,inij,finni,finnj;
float ESCALA=8;

float viento=0.0;


std::vector <grass> cesped;
std::vector <grass> cespedi;

std::vector <palmera> arbol;
//palmera arbol(0.0,0.0,0.0);
//niebla
std::vector<grass>::iterator it;
 
btTriangleMesh* Cuadro;

bool   niebla=true;                      // G Pressed? ( New )
GLuint filter;                      // Which Filter To Use
GLuint fogMode[]= { GL_EXP, GL_EXP2, GL_LINEAR };   // Storage For Three Types Of Fog
GLuint fogfilter= 2;                    // Which Fog To Use
GLfloat fogColor[4]= {0.5f, 0.5f, 0.5f, 1.0f};      // Fog Color


GLfloat matenemigo[16];
bool servidor=true;
short puerto=4444;
const int max_length = 1024;
bool Serv=false;
GLfloat matrizene[16];


int contador=0;
int nume=0;
//tcp::socket s;
int ne=0;
int activob[NUMERO];
//std::vector <btRigidBody*> fallRigidBody;
//std::vector<btCollisionShape*> fallShapecoll;
//torpedo explosiones[NUMERO];
torpedo explosion[NUMERO];
GLfloat napa[ANCHOMAPA*ANCHOMAPA];

//grass cesped(0,0,0);


//std::vector <btDefaultMotionState*> fallMotionState;

struct obj_model_t *objetos;
float mmapa[ANCHOMAPA][ANCHOMAPA];
float colores[ANCHOMAPA][ANCHOMAPA][3];

float anchobola=ESCALA/16;

time_t oldseed=time(&oldseed);

int anchod=10;
int altod=10;
int nohab=7;




//btHeightfieldTerrainShape *heightFieldShape;
btBvhTriangleMeshShape *gTriMeshShape;
btCollisionShape* chassisShape;


int is = 0,js = 0;
int ie = 0,je = 0;   


int mapwindow,mainwindow;


//sobras

// The framebuffer, which regroups 0, 1, or more textures, and 0 or 1 depth buffer.
 //GLuint FramebufferName = 0;
 //GLuint depthTexture;





unsigned char* circuitobmp;
int anchoc=ANCHOMAPA;
//Machango machango(2.0,2.0,1.1,26);
struct obj_model_t hojas[12];
int puertas[MAXPUERTAS];
int puertas1[MAXPUERTAS];
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
struct obj_model_t setaobj, skysphere,circuitoobj,tile,puerta,muro,puerta1,muro1,piso,esq,cruz,bloque,modelo[27],escenario,tanqueobj,quadobj, tanquetobj, tanquetuobj,ruedaobj,tronco,cabeza,pina;
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
GLuint texturemuro,texturatecho,texturaventana,texturacolumna,texturapuerta,texturaagua,texturacaja[3],texturamuro,texturacubo,texturapiso,texturatanque,texturaesquina,texturacruz,texturabloque,texturamodelo,texturaaracnido,texturaskysphere;
GLuint texturacabeza,texturaobjeto,texturarueda,circuito,texturahoja,texturatronco,texturapalmera,texturacircuito;
GLuint texturacuerpo;
GLuint nubes,dunas;
GLubyte *datostexturaobj;
int anchotextura,altotextura;
GLuint textureesfe; //the array for our texture
GLuint texturadomino;
GLuint skybox[6]; 
GLuint texturasuelo;

int moving, beginx, beginy;
float alfax=0.0,alfay=0.0;
bool is_first_time=true;
const int TEXDIM = 64;

//btVector3 pospersonaje=btVector3(13, altura+28.5, -96.5);
btVector3 pospersonaje=btVector3(13, 1.0, -80.5);
btVector3 posnave=btVector3(30.0,.5,-85.0);
//camara circuito
btVector3 vectorvision =btVector3(
0.998630, -0.207912, 0.052336

	);
btVector3 eyePos=btVector3(
  
138.606964, 34.914505, 105.828735
	);


//btVector3 vectorvision =btVector3(0.0, 1.0, 0.0);
//btVector3 eyePos=btVector3(25, 25, 25);
float alfatorreta=180.0,atorreta=0;
/////vehiculo
btVector3 origenvehiculo=btVector3(0,0,1);
btVehicleRaycaster* raycaster;
btRaycastVehicle* vehicle;


float   acel=150.0f;
float	gEngineForce = 0.f;
float	gEngineForced = 0.f;
float	gBreakingForce = 0.f;

float	maxEngineForce = 7000.f;//this should be engine/velocity dependent
float	maxBreakingForce = 500.f;

float	gVehicleSteering = 0.f;
float	steeringIncrement = 0.03f;
float	steeringClamp = 0.05f;

btVector3 wheelDirection(0.0f, -1.0f, 0.0f);
btVector3 wheelAxis(-1.0f, 0.0f, 0.0f);

btScalar wheelRadius(1.0f);


float	wheelWidth = 0.4f;
float	wheelFriction = 10;//BT_LARGE_FLOAT;
float	suspensionStiffness = 5.f;
float	suspensionDamping = 7.3f;
float	suspensionCompression = .01f; //compresion de amortiguación menor mas rigida
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
int nocajas=0,nobolas=0,nopuertas1=0,nocajas1=0,nopiso=0,noes1=0,noes2=0,noes3=0,noes4=0,nocruz=0,nobloques=0;

enum {
	X=0, Y, Z
};

int barrotes=0;
/* Display list names. */


btBroadphaseInterface* broadphase;
btDefaultCollisionConfiguration* collisionConfiguration;

btCollisionDispatcher* dispatcher;
btSequentialImpulseConstraintSolver* solver;
btTriangleMesh *mTriMesh,*mTriMesh2;
btDiscreteDynamicsWorld* dynamicsWorld;
btConvexTriangleMeshShape *mayamodelocircuito,*mTriMeshShape2;
btCollisionShape* suelocoll;
btBoxShape* pruebashape;
btConvexShape* circuitocolshape;

btSoftBody* setasoftBody;


btRigidBody* fallRigidBody[NUMERO];

btRigidBody* fallrb[NUMERO];
btCollisionShape* fallShapecoll[NUMERO];
btDefaultMotionState* fallMotionState[NUMERO];
btCollisionShape* fallsc[NUMERO];
btDefaultMotionState* fallms[NUMERO];

btCollisionShape* cuboShapecoll[MAXCAJAS];

btCompoundShape * tanqueshape;
/*
btCollisionShape* tuboShape;
btCollisionShape* paredcol[MAXCAJAS];

btCollisionShape* es1Shapecoll[MAXCAJAS];
btCollisionShape* es2Shapecoll[MAXCAJAS];
btCollisionShape* es3Shapecoll[MAXCAJAS];
btCollisionShape* es4Shapecoll[MAXCAJAS];
btCollisionShape* cubo1Shapecoll[MAXCAJAS];
btCollisionShape* cubo2Shapecoll[MAXCAJAS];
btCollisionShape* pisoShapecoll[MAXPISO];
btCollisionShape* puertasShapecol[MAXPUERTAS];
btCollisionShape* puertas1Shapecol[MAXPUERTAS];
*/
btCollisionShape* personajecolshape;
btCollisionShape* navecolshape[7];
btDefaultMotionState* personajeestado;
btDefaultMotionState* navemotionstate[7];
btDefaultMotionState* groundMotionState;
btDefaultMotionState* escenarioMotionState,* escenarioMotionState2 ;
btDefaultMotionState* cuboMotionState[MAXCAJAS];
/*
btDefaultMotionState* paredMotionState[MAXCAJAS];
btDefaultMotionState* pisoMotionState[MAXPISO];
btDefaultMotionState* puertasMotionState[MAXPUERTAS];
btDefaultMotionState* puertas1MotionState[MAXPUERTAS];*/
btDefaultMotionState* tuboMotionState;
btDefaultMotionState* circuitoMotionState,pruebamotionstate;
btDefaultMotionState* ds;
btIndexedMesh im;
btTriangleMesh* triangulos ;
btBvhTriangleMeshShape *trianguloss;

btTriangleIndexVertexArray* indvtriangulos;
btRigidBody* cuborb[MAXCAJAS];
btRigidBody* groundRigidBody,*escenarioRigidBody,*escenarioRigidBody2;
btRigidBody* circuitoRB,*pruebarigidbody;
//btDefaultMotionState* fallMotionState;
//btRigidBody* fallRigidBody[NUMERO];
/*btRigidBody* murorb[NUMEROMUROS];
btRigidBody* suelorb[NUMEROSUELO];
btRigidBody* tuborb;
btRigidBody* puertasrb[MAXPUERTAS];
btRigidBody* puertas1rb[MAXPUERTAS];
btRigidBody* pisorb[MAXPISO];
btRigidBody* cubo1rb[MAXCAJAS];
btRigidBody* cubo2rb[MAXCAJAS];

btRigidBody* es1rb[MAXCAJAS];
btRigidBody* es2rb[MAXCAJAS];
btRigidBody* es3rb[MAXCAJAS];
btRigidBody* es4rb[MAXCAJAS];
btRigidBody* paredrb[MAXCAJAS];*/
btRigidBody* personajerb;
btRigidBody* naverb[7];
btTriangleMesh* btojb;
btTransform t1, tcaja, trans, pt[4], tc,tp,tn,tf[MAXPUERTAS],tpi,tb;
btTransform vt;
//btRigidBody* ficharb[MAXPUERTAS];

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

//sombras


//GLuint FramebufferName = 0;
// GLuint depthTexture;

//punto de luz
GLfloat lightPos[3] = { 0.0f, 0.0f, 15.0f };
GLfloat posobj[3] = { 0.0f, 0.0f, 0.0f };

static float lightSPos[4] = { 0.0, 10.0, 0.0, 1.0 };    //{5.0, 15.0, 5.0, 1.0};
//static float lightSDir[4] ={0.0,-1.0, 0.0, 1.0};
GLfloat exponent = 30.0f;
GLfloat cutoff = 60.0f;

GLfloat att = 0.6f;
struct obj_model_t exploobj;

static float materialColor[8][4] = { { 0.8, 0.8, 0.8, 1.0 }, { 0.8, 0.0, 0.0,
		1.0 }, { 0.0, 0.8, 0.0, 1.0 }, { 0.0, 0.0, 0.8, 1.0 }, { 0.0, 0.8, 0.8,
		1.0 }, { 0.8, 0.0, 0.8, 1.0 }, { 0.8, 0.8, 0.0, 1.0 }, { 0.0, 0.0, 0.0,
		0.6 }, };

struct ContactSensorCallback : public btCollisionWorld::ContactResultCallback {
    //! Constructor, pass whatever context you want to have available when processing contacts
    /*! You may also want to set m_collisionFilterGroup and m_collisionFilterMask
     * (supplied by the superclass) for needsCollision() */
	 btRigidBody &body;
	 int num;
    ContactSensorCallback(btRigidBody& tgtBody  /*, ... */)
        : btCollisionWorld::ContactResultCallback(), body(tgtBody) {
        	
        	body=tgtBody;
        	num=99;

        	
         }
	
        ContactSensorCallback(btRigidBody& tgtBody ,int &nu)
        : btCollisionWorld::ContactResultCallback(), body(tgtBody) {
        	
          body=tgtBody;
        	num=nu;

        	
         }
   //=fallRigidBody[nocajas-1]; //!< The body the sensor is monitoring
    //YourContext& ctxt; //!< External information for contact processing

   

    //! Called with each contact for your own processing (e.g. test if contacts fall in within sensor parameters)
    virtual btScalar addSingleResult(btManifoldPoint& cp,
        const btCollisionObjectWrapper* colObj0,int partId0,int index0,
        const btCollisionObjectWrapper* colObj1,int partId1,int index1)
    {
       // explosiones[num].desactiva();
       // will be set to point of collision relative to body
      // btTransform transex;
       /* glPushMatrix();
        			btScalar m[16];
       				glColor3f(0.0,1.0,0.0);
					transex.setIdentity();
					body.getMotionState()->getWorldTransform(transex);
					transex.getOpenGLMatrix(m);
					glMultMatrixf((GLfloat*) m);
					glutSolidSphere(1,12,12);
		glPopMatrix();*/

			   
		
			//if(explosiones==NULL){
				std::cout << "colision:" << num <<std::endl;
              //  for(int i=ne;i<ne+20;i++){
              // 
               explosion[ne].explota(num,btVector3(cp.getPositionWorldOnA().getX(),cp.getPositionWorldOnA().getY(),cp.getPositionWorldOnA().getZ()),true, exploobj.vertices, exploobj.num_verts);
                   ne++;
                //   explosion.llama.clear();
/*
                for(int x=(int) cp.getPositionWorldOnA().getX()-2;x<(int) cp.getPositionWorldOnA().getX()+2;x++){
                   for(int y=(int) cp.getPositionWorldOnA().getY()-2;y<(int) cp.getPositionWorldOnA().getY()+2;y++){
                        colores[x][y][0]=colores[x][y][0]-.3;
                        colores[x][y][1]=colores[x][y][1]-.3;
                        colores[x][y][2]=colores[x][y][2]-.3;
                   }
                }*/
            //  for (int z=1;z<25;z++){
            //       explosion[0].adde(btVector3(cp.getPositionWorldOnA().getX(),cp.getPositionWorldOnA().getY()+z/5,cp.getPositionWorldOnA().getZ()+z/5));
                    // std::cout << ".";
            //  }
              //  dibujaexplosion(num);

//explosion[ne].adde( btVector3((btScalar) cp.getPositionWorldOnA().getX()+((rand() % 90)-45)/10 ,(btScalar)cp.getPositionWorldOnA().getY()+(  rand() % 45 )/10,(btScalar)cp.getPositionWorldOnA().getZ()-((rand() % 90)-45)/10));
        


          ///  btVector3(eyePos[X],eyePos[Y],eyePos[Z]));
			//	explosiones=new torpedo( btVector3((btScalar) cp.getPositionWorldOnA().getX(),(btScalar)cp.getPositionWorldOnA().getY(),(btScalar)cp.getPositionWorldOnA().getZ()));
			//std::cout << "." << i ;
			//} //else explosiones->add( btVector3((btScalar) cp.getPositionWorldOnA().getX(),(btScalar)cp.getPositionWorldOnA().getY(),(btScalar)cp.getPositionWorldOnA().getZ()));
	
                // int j=ne;
             /*    btVector3 fallInertia=btVector3(0,0,0);
                fallsc[j] = new btSphereShape(.1);
                 btDefaultMotionState *fallMotionState;
                  fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3((btScalar) cp.getPositionWorldOnA().getX()+((rand() % 20)-10)/10 ,(btScalar)cp.getPositionWorldOnA().getY()+(  rand() % 10 )/10,(btScalar)cp.getPositionWorldOnA().getZ()+((rand() % 20)-10)/10 )));

                 fallsc[j]->calculateLocalInertia(.1, fallInertia);
                 btRigidBody::btRigidBodyConstructionInfo fallrbCI(2.0,fallMotionState, fallsc[j], fallInertia);
                 fallrb[j] = new btRigidBody(fallrbCI);
                 dynamicsWorld->addRigidBody(fallrb[j]);
                
                 ne++;*/
                
              //  fallrb[j]->setLinearVelocity(fallrb[j]->getLinearVelocity()+btVector3((btScalar) cp.getPositionWorldOnA().getX()+((rand() % 20)-10)/10 ,(btScalar)cp.getPositionWorldOnA().getY()+(  rand() % 10 )/10,(btScalar)cp.getPositionWorldOnA().getZ()+((rand() % 20)-10)/10 ));
              //   }
                //  }
		
		//explosiones.push_back(new btVector4((btScalar) cp.getPositionWorldOnA().getX(),(btScalar)cp.getPositionWorldOnA().getY(),(btScalar)cp.getPositionWorldOnA().getZ(),(btScalar)15));
            // dynamicsWorld->removeRigidBody(fallRigidBody[num]);
		     activob[num]=1;
        return 0; // not actually sure if return value is used for anything...?
    }
};


