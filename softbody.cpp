#include <iostream>
#include <map>
#include <vector>
#include <list>

#include <btBulletDynamicsCommon.h>
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftBody.h"

#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btConvexHull.h"

#include <irrlicht/irrlicht.h>

#pragma comment (lib, "libbulletdynamics.lib")
#pragma comment (lib, "libbulletcollision.lib")
#pragma comment (lib, "libbulletmath.lib")
#pragma comment (lib, "libbulletsoftbody.lib")
#pragma comment (lib, "Irrlicht.lib")

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

#define CUBE_HALF_EXTENTS 15

struct MeshData
{
    u16              *irr_indices;
    S3DVertex        *irr_vertices;
    int              *temp_indices;
    btScalar         *temp_vertices;
    int              *soft_indices;
    btSoftBody::Node **soft_nodes;
    int indexCount;
    int irr_vertexCount;
    int temp_vertexCount;
    int soft_nodeCount;
};

int main (void)
{

    // Irrlicht???
    IrrlichtDevice *device = createDevice(EDT_OPENGL, dimension2d<s32>(640, 480), 16, false, true, false,dimension2d <s32>(0.0));
    device->setWindowCaption(L"Irrlicht + Bullet : SoftBody Demo");
    IVideoDriver* driver = device->getVideoDriver();
    ISceneManager *smgr = device->getSceneManager();

    // ??????
    ICameraSceneNode *camera = smgr->addCameraSceneNodeFPS(0, 150, 500, -1, 0, 0, false);
    camera->setPosition(core::vector3df(0,400,-300));
    camera->setFarValue(10000);
    camera->setTarget(core::vector3df(0, 300, 0));

    // SoftBody????????????????????Irrlicht??
    IAnimatedMesh *cubeMesh = smgr->getMesh("../media/earth.obj");
    IAnimatedMeshSceneNode *cubeNode = smgr->addAnimatedMeshSceneNode(cubeMesh, 0, -1, vector3df(0, 0, 0), vector3df(0,0,0), vector3df(1,1,1), false);
    cubeNode->setMaterialFlag(video::EMF_LIGHTING, false);

    // ??????Irrlicht??
    IAnimatedMesh *planemesh = smgr->addHillPlaneMesh("myHill", dimension2d<f32>(24, 24), dimension2d<u32>(100, 100));
    ISceneNode *q3sn = smgr->addOctTreeSceneNode(planemesh);
    q3sn->setMaterialFlag(video::EMF_LIGHTING, false);
    q3sn->setMaterialTexture(0, driver->getTexture("../media/wall.jpg"));

    // SoftBody???????
    btSoftBody::btSoftBodyWorldInfo   m_softBodyWorldInfo;

    // ???????
    btVector3 worldAabbMin(-10000,-10000,-10000);
    btVector3 worldAabbMax(10000,10000,10000);
    // ????????????????????
    int maxProxies = 1024;
    // broadphase????SAP??
    btAxisSweep3* broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
    m_softBodyWorldInfo.m_broadphase = broadphase;

    // ?????????????????????
    btDefaultCollisionConfiguration* collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
    m_softBodyWorldInfo.m_dispatcher = dispatcher;

    // ???????
    btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    // Soft-Rigit???????
    btSoftRigidDynamicsWorld* dynamicsWorld = new btSoftRigidDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);

    // ??????????
    btCollisionShape* groundShape = new btBoxShape (btVector3(2000,CUBE_HALF_EXTENTS,2000));
    // ???MotionState???
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0,0,0,1),btVector3(0,-CUBE_HALF_EXTENTS/2.0,0)));
    // ??????????
    btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState,groundShape,btVector3(0,0,0));
    // ????????
    btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
    groundRigidBody->setCollisionFlags( groundRigidBody->getCollisionFlags() |  btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK );
    // ?????????????
    dynamicsWorld->addRigidBody(groundRigidBody);

    // ?????
    dynamicsWorld->setGravity(btVector3(0,-10.0,0));
    // softBodyWorldInfo???????
    m_softBodyWorldInfo.m_sparsesdf.Initialize();
    m_softBodyWorldInfo.m_gravity.setValue(0,-10.0,0);
    m_softBodyWorldInfo.air_density      =   (btScalar)1.2;
    m_softBodyWorldInfo.water_density   =   0;
    m_softBodyWorldInfo.water_offset   =   0;
    m_softBodyWorldInfo.water_normal   =   btVector3(0,0,0);

    // MeshData struct
    MeshData testMesh;

    int cMeshBuffer, j;
    // MeshBuffer
    IMeshBuffer *mb;

    std::cout << "get Mesh Buffer" << std::endl;
    // getMesh??????????????
    for (cMeshBuffer=0; cMeshBuffer<cubeMesh->getMesh(0)->getMeshBufferCount(); cMeshBuffer++)
    {
        // ???????????
        mb = cubeMesh->getMesh(0)->getMeshBuffer(cMeshBuffer);

        // ??????????????????????????????????
        testMesh.irr_vertices = (irr::video::S3DVertex*)mb->getVertices();
        testMesh.irr_vertexCount = mb->getVertexCount();
        // ??????????????????????????????????????????
        testMesh.irr_indices  = mb->getIndices();
        testMesh.indexCount = mb->getIndexCount();
    }

    // ????????????????
    // 3ds?????.obj????Irrlicht????????Index??Vertex??????Index?????Vertex??????
    // ???????SoftBody????????????????????????Vertex????????
    // ?????????????Index??????Index???????
    std::map<int, int> index_map;
    std::map<int, int> index2_map;
    std::map<int, S3DVertex> vertex_map;
    int count = 0;
    for (int i=0; i<testMesh.irr_vertexCount; i++)
    {
        int iIndex = testMesh.irr_indices[i];
        vector3df iVertex = testMesh.irr_vertices[iIndex].Pos;
        bool isFirst = true;
        for (int j=0; j<i; j++)
        {
            int jIndex = testMesh.irr_indices[j];
            vector3df jVertex = testMesh.irr_vertices[jIndex].Pos;
            if (iVertex == jVertex)
            {
                index_map.insert(std::make_pair(i, j));
                isFirst = false;
                break;
            }
        }
        // ???????Bullet??Index??????
        if (isFirst)
        {
            // Irrlicht?Index??????Index
            index_map.insert(std::make_pair(i, i));
            // ?????Index????Index
            index2_map.insert(std::make_pair(i, count));
            // ??Index?????????
            vertex_map.insert(std::make_pair(count, testMesh.irr_vertices[iIndex]));
            count++;
        }
    }

    // Temporary index-vertex data for Softbody
    testMesh.temp_indices = new int[testMesh.indexCount];
    testMesh.temp_vertexCount = vertex_map.size();
    testMesh.temp_vertices = new btScalar[testMesh.temp_vertexCount*3];

    std::cout << "IndexCount=" << testMesh.indexCount << ", IrrVertexCount=" << testMesh.irr_vertexCount << std::endl;
    std::cout << "IndexCount=" << testMesh.indexCount << ", TempVertexCount=" << testMesh.temp_vertexCount << std::endl;
    // ?????????????????????
    for (j=0; j<testMesh.indexCount; j++)
    {
        // ?????Index???????????Index
        int index1 = index_map.find(j)->second;
        // ?????????Index?????????Index
        int index2 = index2_map.find(index1)->second;
        testMesh.temp_indices[j]   = index2;
    }
    // SoftBody???????????????
    for (j=0; j<testMesh.temp_vertexCount; j++)
    {
        testMesh.temp_vertices[3*j]   =  vertex_map[j].Pos.X;
        testMesh.temp_vertices[3*j+1] =  vertex_map[j].Pos.Y;
        testMesh.temp_vertices[3*j+2] = -vertex_map[j].Pos.Z;
    }

    std::cout << "create softbody" << std::endl;
    // ????????SoftBody????btSoftBodyHelpers????
    btSoftBody* cubeSoftBody = btSoftBodyHelpers::CreateFromTriMesh(
        m_softBodyWorldInfo, testMesh.temp_vertices, testMesh.temp_indices, testMesh.indexCount/3);

    std::cout << "create map" << std::endl;
    std::cout << "cubeSoftBody->m_faces.size()=" << cubeSoftBody->m_faces.size() << std::endl;

    // softbody?index-node???
    testMesh.soft_indices = new int[testMesh.indexCount];
    testMesh.soft_nodeCount = cubeSoftBody->m_faces.size()*3;
    testMesh.soft_nodes = new btSoftBody::Node*[testMesh.soft_nodeCount];

    // index-node data
    count = 0;
    for (int i=0; i<cubeSoftBody->m_faces.size(); i++)
    {
        // A face has three nodes
        btSoftBody::Face face = cubeSoftBody->m_faces[i];
        for (int j=0; j<3; j++)
        {
            testMesh.soft_nodes[count++] = face.m_n[j];
        }
    }

    // Irrlicht?Vertex?Bullet?Vertex???????????
    std::cout << "create softbody indices"  << std::endl;

    // ???????Irrlicht?index?Bullet?index?????
    for (int i=0; i<testMesh.indexCount; i++)
    {
        int irr_index = testMesh.irr_indices[i];
        for (int j=0; j<testMesh.indexCount; j++)
        {
            btSoftBody::Node* node = testMesh.soft_nodes[j];
            if ((node->m_x.x() ==   testMesh.irr_vertices[irr_index].Pos.X) &&
                (node->m_x.y() ==   testMesh.irr_vertices[irr_index].Pos.Y) &&
                (node->m_x.z() ==  -testMesh.irr_vertices[irr_index].Pos.Z))
            {
                // Irrlicht?Bullet???????
                testMesh.soft_indices[i] = j;
//                std::cout << "i=" << i << ", irr_index=" << irr_index << ", soft_index=" << j << std::endl;
                break;
            }
        }
    }

    // SoftBody????????
    std::cout << "addSoftBody" << std::endl;
    cubeSoftBody->m_cfg.kDP = 0.0;// Damping coefficient [0,1]
    cubeSoftBody->m_cfg.kDF = 0.2;// Dynamic friction coefficient [0,1]
    cubeSoftBody->m_cfg.kMT = 0.02;// Pose matching coefficient [0,1]
    cubeSoftBody->m_cfg.kCHR = 1.0;// Rigid contacts hardness [0,1]
    cubeSoftBody->m_cfg.kKHR = 0.8;// Kinetic contacts hardness [0,1]
    cubeSoftBody->m_cfg.kSHR = 1.0;// Soft contacts hardness [0,1]
    cubeSoftBody->m_cfg.piterations=2;
    cubeSoftBody->m_materials[0]->m_kLST = 0.8;
    cubeSoftBody->m_materials[0]->m_kAST = 0.8;
    cubeSoftBody->m_materials[0]->m_kVST = 0.8;
    cubeSoftBody->scale(btVector3(1,1,1));
    cubeSoftBody->setPose(true, true);
    cubeSoftBody->generateBendingConstraints(2);
    cubeSoftBody->randomizeConstraints();

    // ???????Bullet??SoftBody???
    btMatrix3x3 m;
    m.setIdentity();
    cubeSoftBody->transform(btTransform(m,btVector3(0, 400, 0)));
    dynamicsWorld->addSoftBody(cubeSoftBody);

    std::cout << "start simulation" << std::endl;
    // ????????????
    while(device->run())
    {
        // ????????????????60Hz
        dynamicsWorld->stepSimulation(1/60.0f, 1);

        // Irrlicht????????
        for (int i=0; i<testMesh.indexCount; i++)
        {
            // Irrlict??????Bullet?????
            int irr_index = testMesh.irr_indices[i];
            int soft_index = testMesh.soft_indices[i];
            // Bullet??????????????,???????
            btSoftBody::Node* node = testMesh.soft_nodes[soft_index];
            // Irrlicht??mb_vertices[i]?OK
            testMesh.irr_vertices[irr_index].Pos.X =  node->m_x.x();
            testMesh.irr_vertices[irr_index].Pos.Y =  node->m_x.y();
            testMesh.irr_vertices[irr_index].Pos.Z = -node->m_x.z();
            testMesh.irr_vertices[irr_index].Normal.X =  node->m_n.x();
            testMesh.irr_vertices[irr_index].Normal.Y =  node->m_n.y();
            testMesh.irr_vertices[irr_index].Normal.Z = -node->m_n.z();
        }

        if (GetAsyncKeyState(VK_SPACE))
        {
            // 0????????????
            cubeSoftBody->addForce(btVector3(0, 10, 0), 0);
        }
        else if (GetAsyncKeyState(VK_ESCAPE))
        {
            break;
        }

        driver->beginScene(true, true, SColor(0,200,200,200));
        smgr->drawAll();
        driver->endScene();
    }
    device->drop();

    /* Clean up   */
    for(int i=dynamicsWorld->getNumCollisionObjects()-1;i>0;i--)
    {
        btCollisionObject*   obj=dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody*      body=btRigidBody::upcast(obj);
        if(body&&body->getMotionState())
        {
            delete body->getMotionState();
        }
        while(dynamicsWorld->getNumConstraints())
        {
            btTypedConstraint*   pc=dynamicsWorld->getConstraint(0);
            dynamicsWorld->removeConstraint(pc);
            delete pc;
        }
        btSoftBody* softBody = btSoftBody::upcast(obj);
        if (softBody)
        {
            dynamicsWorld->removeSoftBody(softBody);
        } else
        {
            dynamicsWorld->removeCollisionObject(obj);
        }
        delete obj;
    }

    delete [] testMesh.temp_indices;
    delete [] testMesh.temp_vertices;
    delete [] testMesh.soft_indices;
    delete [] testMesh.soft_nodes;

    // ???????????????????
    delete dynamicsWorld;
    delete solver;
    delete collisionConfiguration;
    delete dispatcher;
    delete broadphase;

    return 0;
}