
/* Create a single component checkboard texture map. */
GLfloat *makeTexture(int maxs, int maxt) {
	int s, t;
	static GLfloat *texture;

	texture = (GLfloat *) malloc(maxs * maxt * sizeof(GLfloat));
	for (t = 0; t < maxt; t++) {
		for (s = 0; s < maxs; s++) {
			texture[s + maxs * t] = ((s >> 4) & 0x1) ^ ((t >> 4) & 0x1);
		}
	}
	return texture;
}
void renderizaobj (struct obj_model_t *mdl, bool dentro)
{
  int i, j;
 bool bandera=true;

	
  for (i = 0; i < mdl->num_faces; ++i)
    {
   glBegin (mdl->faces[i].type);
   if(dentro){		
	//  printf("\ncaras %d,",mdl->faces[i].num_elems);
	for (j = mdl->faces[i].num_elems-1; j >=0; --j)
	  {
		
				  
		
			  glColor3f(1.0,1.0,1.0);
		
	
	  
		
	    if (mdl->has_texCoords)
			
			  
	      glTexCoord3fv (mdl->texCoords[mdl->faces[i].uvw_indices[j]].uvw);
		  
	    if (mdl->has_normals){
			  
	      glNormal3fv (mdl->normals[mdl->faces[i].norm_indices[j]].ijk);
		  glVertex4fv (mdl->vertices [mdl->faces[i].vert_indices[j]].xyzw);
		}
		  
	  }
   }else{
	for (j = 0; j <mdl->faces[i].num_elems; ++j)
	  {
		
			  glColor3f(1.0,1.0,1.0);
			 
	    if (mdl->has_texCoords)
	      glTexCoord3fv (mdl->texCoords[mdl->faces[i].uvw_indices[j]].uvw);
		  
	    if (mdl->has_normals){
	      glNormal3fv (mdl->normals[mdl->faces[i].norm_indices[j]].ijk);

	    glVertex4fv (mdl->vertices [mdl->faces[i].vert_indices[j]].xyzw);
		}
	  }
	
	
	   
   }
	glEnd();
		
    }
}
void dibujaskybox(){
glPushMatrix();
	glPushAttrib(GL_ENABLE_BIT);
   glEnable(GL_TEXTURE_2D);
   glDisable(GL_DEPTH_TEST);
   glDisable(GL_LIGHTING);
   glDisable(GL_BLEND);

   // Just in case we set all vertices to white.
   glColor4f(1,1,1,1);
	float tam=-1.0f;

	glTranslatef(eyePos[0],eyePos[1],eyePos[2]);
  
 // Render the bottom quad
  /* 
	   
      // Render the front quad
   glBindTexture(GL_TEXTURE_2D, skybox[0]);
   glBegin(GL_QUADS);
	   glTexCoord2f(0, 0); glVertex3f(  tam, tam,-tam );
       glTexCoord2f(0, 1); glVertex3f(  tam,-tam,-tam );
       glTexCoord2f(1, 1); glVertex3f( -tam,-tam,-tam );
       glTexCoord2f(1, 0); glVertex3f( -tam, tam,-tam );
   
   glEnd();

   // Render the left quad
   glBindTexture(GL_TEXTURE_2D, skybox[1]);
   glBegin(GL_QUADS);
       glTexCoord2f(1, 1); glVertex3f( tam,-tam, tam );
       glTexCoord2f(0, 1); glVertex3f( tam,-tam,-tam );
       glTexCoord2f(0, 0); glVertex3f( tam, tam,-tam );
       glTexCoord2f(1, 0); glVertex3f( tam, tam, tam );
   glEnd();

   // Render the back quad
   glBindTexture(GL_TEXTURE_2D, skybox[2]);
   glBegin(GL_QUADS);
       glTexCoord2f(1, 1); glVertex3f(-tam,-tam, tam );
       glTexCoord2f(0, 1); glVertex3f( tam,-tam, tam );
       glTexCoord2f(0, 0); glVertex3f( tam, tam, tam );
       glTexCoord2f(1, 0); glVertex3f(-tam, tam, tam );

   glEnd();

   // Render the right quad
   glBindTexture(GL_TEXTURE_2D, skybox[3]);
   glBegin(GL_QUADS);
       glTexCoord2f(1, 1); glVertex3f(-tam,-tam,-tam );
       glTexCoord2f(0, 1); glVertex3f(-tam,-tam, tam );
       glTexCoord2f(0, 0); glVertex3f(-tam, tam, tam );
       glTexCoord2f(1, 0); glVertex3f(-tam, tam,-tam );
   glEnd();

   // Render the top quad suelo
   glBindTexture(GL_TEXTURE_2D, skybox[4]);
   glBegin(GL_QUADS);
       glTexCoord2f(1, 0); glVertex3f(-tam, tam,-tam );
       glTexCoord2f(1, 1); glVertex3f(-tam, tam, tam );
       glTexCoord2f(0, 1); glVertex3f( tam, tam, tam );
       glTexCoord2f(0, 0); glVertex3f( tam, tam,-tam );
   glEnd();
	
   glBindTexture(GL_TEXTURE_2D, skybox[5]);
   glBegin(GL_QUADS);
       glTexCoord2f(1, 0); glVertex3f(-tam, -tam, tam );
       glTexCoord2f(1, 1); glVertex3f(-tam, -tam, -tam );
       glTexCoord2f(0, 1); glVertex3f(tam, -tam, -tam );
       glTexCoord2f(0, 0); glVertex3f(tam, -tam, tam );
   glEnd();*/

	
	glRotatef(rotacioncielo,0.0,1.0,0.0);
	glBindTexture(GL_TEXTURE_2D, nubes);
	renderizaobj(&cieloobj,true);
	glRotatef(-rotacioncielo,0.0,1.0,0.0);
	rotacioncielo+=0.01;
	
	glEnable(GL_BLEND);
	glBindTexture(GL_TEXTURE_2D, dunas);
	
	renderizaobj(&dunasobj,true);
	
	
   // Restore enable bits and matrix
   glPopAttrib();
   glPopMatrix();
	

}
static void setColor(int c) {
	bool useLighting = true;
	bool useRGB = true;
	if (useLighting) {
		if (useRGB) {
			glMaterialfv(GL_FRONT_AND_BACK,
			GL_AMBIENT_AND_DIFFUSE, &materialColor[c][0]);
		} else {
			glMaterialfv(GL_FRONT_AND_BACK,
			GL_COLOR_INDEXES, &materialColor[c][0]);
		}
	} else {
		if (useRGB) {
			glColor4fv(&materialColor[c][0]);
		} else {
			glIndexf(materialColor[c][1]);
		}
	}
}
static void 
text(GLuint x, GLuint y, GLfloat scale, char* format, ...)
{
  va_list args;
  char buffer[255], *p;
  GLfloat font_scale = 119.05 + 33.33;

  va_start(args, format);
  vsprintf(buffer, format, args);
  va_end(args);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, glutGet(GLUT_WINDOW_WIDTH), 0, glutGet(GLUT_WINDOW_HEIGHT));

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glPushAttrib(GL_ENABLE_BIT);
  glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
  glDisable(GL_DEPTH_TEST);
  glTranslatef(x, y, 0.0);

  glScalef(scale/font_scale, scale/font_scale, scale/font_scale);

  for(p = buffer; *p; p++)
    glutStrokeCharacter(GLUT_STROKE_ROMAN, *p);
  
  glPopAttrib();

  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
}

static void drawCheck(int w, int h, int evenColor, int oddColor) {
	static int initialized = 0;
	static int usedLighting = 0;
	static GLuint checklist = 0;
	bool useQuads = true;
	if (!initialized) {
		static float square_normal[4] = { 0.0, 0.0, 1.0, 0.0 };
		static float square[4][4];
		int i, j;

		if (!checklist) {
			checklist = glGenLists(1);
		}
		glNewList(checklist, GL_COMPILE_AND_EXECUTE);

		if (useQuads) {
			glNormal3fv(square_normal);
			glBegin(GL_QUADS);
		}
		for (j = 0; j < h; ++j) {
			for (i = 0; i < w; ++i) {
				square[0][0] = -1.0 + 2.0 / w * i;
				square[0][1] = 0.0;//-1.0 + 2.0 / h * (j + 1);
				square[0][2] = -1.0 + 2.0 / h * (j + 1);//0.0;
				square[0][3] = 1.0;

				square[1][0] = -1.0 + 2.0 / w * i;
				square[1][1] = 0.0;//-1.0 + 2.0 / h * j;
				square[1][2] = -1.0 + 2.0 / h * j;//0.0;
				square[1][3] = 1.0;

				square[2][0] = -1.0 + 2.0 / w * (i + 1);
				square[2][1] = 0.0;//-1.0 + 2.0 / h * j;
				square[2][2] = -1.0 + 2.0 / h * j;//0.0;
				square[2][3] = 1.0;

				square[3][0] = -1.0 + 2.0 / w * (i + 1);
				square[3][1] = 0.0;//-1.0 + 2.0 / h * (j + 1);
				square[3][2] = -1.0 + 2.0 / h * (j + 1);//0.0;
				square[3][3] = 1.0;

				if ((i & 1) ^ (j & 1)) {
					setColor(oddColor);
				} else {
					setColor(evenColor);
				}

				if (!useQuads) {
					glBegin(GL_POLYGON);
				}
				glVertex4fv(&square[3][0]);
				glVertex4fv(&square[2][0]);
				glVertex4fv(&square[1][0]);
				glVertex4fv(&square[0][0]);
				
				
				
				if (!useQuads) {
					glEnd();
				}
			}
		}

		if (useQuads) {
			glEnd();
		}
		glEndList();

		initialized = 1;
		//usedLighting = useLighting;
	} else {
		glCallList(checklist);
	}
}

void creafichav(float x,float y,float alt){
	btScalar mass = 1.0;
	btVector3 fallInertia(0, 0, 0);
	btQuaternion rotacion=btQuaternion(1, 0, 0,1 );
	//rotacion.setRotation(btVector3(0.0f,1.0f,0.0f), NPI);
	
	fichashapecol[nofichas] = new btBoxShape(btVector3(.15,1.0, .5));
	fichasMotionState[nofichas] = new btDefaultMotionState(btTransform(rotacion, btVector3(x, alt+.5, y)));
	fichashapecol[nofichas]->calculateLocalInertia(mass, fallInertia);
	btRigidBody::btRigidBodyConstructionInfo fichaCI(mass, fichasMotionState[nofichas], fichashapecol[nofichas], fallInertia);
	ficharb[nofichas] = new btRigidBody(fichaCI);
	dynamicsWorld->addRigidBody(ficharb[nofichas]);
	nofichas++;

}
void creafichah(float x,float y,float alt){
	btScalar mass = 1.0;
	btVector3 fallInertia(0, 0, 0);
	
	
	fichashapecol[nofichas] = new btBoxShape(btVector3(.15,1.0, .5));
	fichasMotionState[nofichas] = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 1, 1), btVector3(x, alt+3.0, y)));
	fichashapecol[nofichas]->calculateLocalInertia(mass, fallInertia);
	btRigidBody::btRigidBodyConstructionInfo fichaCI(mass, fichasMotionState[nofichas], fichashapecol[nofichas], fallInertia);
	ficharb[nofichas] = new btRigidBody(fichaCI);
	dynamicsWorld->addRigidBody(ficharb[nofichas]);
	nofichas++;

}

int piramide(int ancho, int x, int y) {
	btScalar mass = 1;
	btVector3 fallInertia(0, 0, 0);
	
	if (ancho > 0) {

		for (int i = 0; i < ancho; i++) {
			for (int j = 0; j < ancho; j++) {
				cuboShapecoll[nocajas] = new btBoxShape(btVector3(1.0,1.0, 1.0));
				fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i + desp+x, altura*2 + 1.01, j + desp+y)));
				cuboShapecoll[nocajas]->calculateLocalInertia(mass, fallInertia);
				btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,
						fallMotionState, cuboShapecoll[nocajas], fallInertia);
				cuborb[nocajas] = new btRigidBody(fallRigidBodyCI);
				dynamicsWorld->addRigidBody(cuborb[nocajas]);
				texturascajas[nocajas]=rand()%4;
				nocajas++;
			}
		}
		altura++;
		desp = desp + .5;
		return piramide(ancho - 1,x,y);
	} else {

		return altura;

	}

}

int tonga(int ancho, int alto, float x, float y) {
	btScalar mass = 1;
	btVector3 fallInertia(0, 0, 0);
	for (int z = 0; z < alto; z++) {
		for (int i = 0; i < ancho; i++) {
			for (int j = 0; j < ancho; j++) {
				cuboShapecoll[nocajas] = new btBoxShape(btVector3(1.0,1.0, 1.0));
				fallMotionState = new btDefaultMotionState(
						btTransform(btQuaternion(0, 0, 0, 1),
								btVector3(i*2 + desp+x, z*2 + 1.001+altura, j*2 + desp+y)));
				cuboShapecoll[nocajas]->calculateLocalInertia(mass*2, fallInertia);
				btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,
						fallMotionState, cuboShapecoll[nocajas], fallInertia);
				cuborb[nocajas] = new btRigidBody(fallRigidBodyCI);
				dynamicsWorld->addRigidBody(cuborb[nocajas]);
				texturascajas[nocajas]=rand()%4;
				nocajas++;
			}
		}
	}
	return nocajas;
}

int muro(int ancho, int alto, int profundo, float x, float y){
btScalar mass = 1;
btVector3 fallInertia(0, 0, 0);
for(int i=0;i<ancho;i++){
	for(int j=0;j<alto;j++){
		for(int z=0;z<profundo;z++){
			cuboShapecoll[nocajas] = new btBoxShape(btVector3(1.0,1.0, 1.0));
				fallMotionState = new btDefaultMotionState(
				btTransform(btQuaternion(0, 0, 0, 1), btVector3(i*2 + x, j*2 + 1.001, z*2+y)));
				cuboShapecoll[nocajas]->calculateLocalInertia(mass*2, fallInertia);
				btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass,
						fallMotionState, cuboShapecoll[nocajas], fallInertia);
				cuborb[nocajas] = new btRigidBody(fallRigidBodyCI);
				dynamicsWorld->addRigidBody(cuborb[nocajas]);
				texturascajas[nocajas]=rand()%4;
				nocajas++;
		}
	}
}

	
}


bool LoadJPEG(const char* FileName) {
	bool Fast = TRUE;
	FILE* file = fopen(FileName, "rb");  //open the file
	struct jpeg_decompress_struct info;  //the jpeg decompress info
	struct jpeg_error_mgr err;           //the error handler

	info.err = jpeg_std_error(&err); //tell the jpeg decompression handler to send the errors to err
	jpeg_create_decompress(&info);       //sets info to all the default stuff

	//if the jpeg file didnt load exit
	if (!file) {
		fprintf(stderr, "Error reading JPEG file %s!!!", FileName);
		// LoadBlackWhiteBorder();
		return false;
	}

	jpeg_stdio_src(&info, file);    //tell the jpeg lib the file we'er reading

	jpeg_read_header(&info, TRUE);   //tell it to start reading it

	//if it wants to be read fast or not
	if (Fast) {
		info.do_fancy_upsampling = FALSE;
	}

	jpeg_start_decompress(&info);    //decompress the file

	//set the x and y
	x = info.output_width;
	y = info.output_height;
	channels = info.num_components;

	type = GL_RGB;

	if (channels == 4) {
		type = GL_RGBA;
	}

	bpp = channels * 8;

	int Sizef = x * y * 3;

	//read turn the uncompressed data into something ogl can read
	data = new unsigned char[Sizef]; //setup data for the data its going to be handling

	unsigned char* p1 = data;
	unsigned char** p2 = &p1;
	int numlines = 0;

	while (info.output_scanline < info.output_height) {
		numlines = jpeg_read_scanlines(&info, p2, 1);
		*p2 += numlines * 3 * info.output_width;
	}

	jpeg_finish_decompress(&info);   //finish decompressing this file

	fclose(file);                    //close the file

	return true;
}

bool LoadTextureplano(const char * filename, int width, int height, GLuint *textura) {

//    GLuint texture;

	LoadJPEG(filename);

	glGenTextures(1, textura);
	glBindTexture( GL_TEXTURE_2D, *textura);
	glPixelStorei( GL_UNPACK_ALIGNMENT, 1);
	glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

//    // when texture area is small, bilinear filter the closest mipmap
//    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
//                    GL_LINEAR_MIPMAP_NEAREST );
//    // when texture area is large, bilinear filter the original
//    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
//    
//    // the texture wraps over at the edges (repeat)
//    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
//    glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
//    
//    //Generate the texture
//    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 2, 2, 0,GL_RGB, GL_UNSIGNED_BYTE, data);

// select modulate to mix texture with color for shading
	glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

// when texture area is small, bilinear filter the closest mipmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
	GL_LINEAR_MIPMAP_NEAREST);
// when texture area is large, bilinear filter the first mipmap
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

//    // the texture wraps over at the edges (repeat)
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

// build our texture mipmaps
	gluBuild2DMipmaps( GL_TEXTURE_2D, 3, width, height,
	GL_RGB, GL_UNSIGNED_BYTE, data);

	free(data);

	return TRUE; //return whether it was successful

}


btVector3 *gVertices,*p2,*p3,*ver,*aux;
int *gindice;
int indice,indicesuelo;
int *gin;

void creamayacircuito (struct obj_model_t *mdl)
{
btVector3 aux;	
gVertices = new btVector3[mdl->num_faces*3];
gindice=new int[mdl->num_faces*3];
	
  for (int  i = 0; i < mdl->num_faces; i++)
    {
		
		
			
			gVertices[i*3+2]= btVector3(mdl->vertices [mdl->faces[i].vert_indices[0]].xyzw[X],
			          				mdl->vertices [mdl->faces[i].vert_indices[0]].xyzw[Y],
			                		mdl->vertices [mdl->faces[i].vert_indices[0]].xyzw[Z]);
			
			gVertices[i*3+1]=btVector3(mdl->vertices [mdl->faces[i].vert_indices[1]].xyzw[X],
			                		mdl->vertices [mdl->faces[i].vert_indices[1]].xyzw[Y],
			                		mdl->vertices [mdl->faces[i].vert_indices[1]].xyzw[Z]);
			
			gVertices[i*3]= btVector3(mdl->vertices [mdl->faces[i].vert_indices[2]].xyzw[X],
			                			mdl->vertices [mdl->faces[i].vert_indices[2]].xyzw[Y],
			                			mdl->vertices [mdl->faces[i].vert_indices[2]].xyzw[Z]);
		
			gindice[i*3]=i*3+2;
			gindice[i*3+1]=i*3+1;
			gindice[i*3+2]=i*3;

			//gIndices[i] = sc.mod[0].triangles[i];
		/*
		aux=gVertices[i*3+2];
		btVector3 p0= btVector3(aux.getX(),aux.getY(),aux.getZ());
		aux=gVertices[i*3+1];
		btVector3 p1= btVector3(aux.getX(),aux.getY(),aux.getZ());
		aux=gVertices[i*3];
		btVector3 p2= btVector3(aux.getX(),aux.getY(),aux.getZ());
		*/
		
		//triangulos->addTriangle(p0,p1,p2);
		//	printf("No de triangulos=%d",triangulos->getNumTriangles());
	  //printf("\np1 %f, %f, %f",p[X],p[Y],p[Z]);
	  //printf("\np2 %f, %f, %f",p[X],p[Y],p[Z]);
	  //printf("\np3 %f, %f, %f",p[X],p[Y],p[Z]);
		
	//}
	}
	
}


void iniciacol(struct obj_model_t *mdl)
{
  int i, j;

	indice=0;
	indicesuelo=0;
	float a=0.0f,aa=0.0f,noant[3];
	for (i = 0; i < mdl->num_faces; i++)
	{

		

		


			float no[3]=	 { mdl->normals[mdl->faces[i].norm_indices[1]].ijk[0],
					           mdl->normals[mdl->faces[i].norm_indices[1]].ijk[1],
						       mdl->normals[mdl->faces[i].norm_indices[1]].ijk[2]};
				if(no[Y]<=0.1 && no[Y]>=-0.1){//|| i%8==7){ //MUROS izquierda y derecha
		if(i>0){
			 noant[0]=	  mdl->normals[mdl->faces[i-1].norm_indices[2]].ijk[0];
			 noant[1]=    mdl->normals[mdl->faces[i-1].norm_indices[2]].ijk[1];
			 noant[2]=    mdl->normals[mdl->faces[i-1].norm_indices[2]].ijk[2];
		}else{
			 noant[0]=	  mdl->normals[mdl->faces[mdl->num_faces-1].norm_indices[2]].ijk[0];
			 noant[1]=    mdl->normals[mdl->faces[mdl->num_faces-1].norm_indices[2]].ijk[1];
			 noant[2]=    mdl->normals[mdl->faces[mdl->num_faces-1].norm_indices[2]].ijk[2];
		}
	
			float *p0=mdl->vertices[mdl->faces[i].vert_indices[0]].xyzw;
			float *p1=mdl->vertices[mdl->faces[i].vert_indices[1]].xyzw;
			float *p2=mdl->vertices[mdl->faces[i].vert_indices[2]].xyzw;
		
			float d0=distancia(p0,p1);
			float d1=distancia(p1,p2);
			float d2=distancia(p2,p0);

			if(d0 < d1 && d0< d2){
				d=d0;
			}
			if(d1 < d0 && d1< d2){
				d=d1;
			}
			if(d2 < d0 && d2 <d1) d=d2;
			anchomuro[indice]=d/1.8;
			//glScalef(anchomuro[j]*8.0,20.0,1.0);
			muroshapecol[indice] = new btBoxShape(btVector3(anchomuro[indice],2.15f,.05f));		

				
		//angulo giro respecto bloque actual
	
			if(no[X]>=0.0){
				if(no[Z]<=0.0){
					a=-asin(no[Z]);
				}
				else a=-acos(no[X]);
				
			}
			else{
				if(no[Z]<=0.0)
					a=acos(no[X]);
				else a=asin(no[Z]);
			}
			a=a+NPI/2;
			
			angulomuro[indice]=a;

			btQuaternion rotacion=btQuaternion(0, 1, 0,1 );
			rotacion.setRotation(btVector3(0.0f,1.0f,0.0f), a);
	
			float alt=0.0; //nocajas del bloque segun 1 cara
		
			if(i%8==3){
				float *pa0=mdl->vertices[mdl->faces[i-2].vert_indices[0]].xyzw;
				float *pa1=mdl->vertices[mdl->faces[i-2].vert_indices[1]].xyzw;
				float *pa2=mdl->vertices[mdl->faces[i-2].vert_indices[2]].xyzw;
				alt=(pa0[1]+pa1[1]+pa2[1])/3;
			}
			else {
				alt=(p0[1]+p1[1]+p2[1])/3;
			}
			
			
	
			muroMotionState[indice] = new btDefaultMotionState(btTransform(rotacion, btVector3((p0[0]+p1[0]+p2[0])/3, +3.0, (p0[2]+p1[2]+p2[2])/3)));
			//muroMotionState[indice] = new btDefaultMotionState(btTransform(btQuaternion(0, 1, 0, 1), btVector3(p0[0], 1.5, p0[2])));
			btRigidBody::btRigidBodyConstructionInfo muroRigidBodyCI(0,muroMotionState[indice], muroshapecol[indice], btVector3(0, 0, 0));
			murorb[indice] = new btRigidBody(muroRigidBodyCI);
			dynamicsWorld->addRigidBody(murorb[indice]);


			indice++;
		 }

		
	}
	
}



#define CUBE_HALF_EXTENTS 1
void initvehiculo(){

	// The vehicle
btScalar chassisMass(20.0f);
btVector3 chassisInertia(0.0f, 0.0f, 0.0f);
btCollisionShape* chassisShape = new btBoxShape(btVector3(2.0f, 0.5f, 2.0f));
btQuaternion rotacion=btQuaternion(0, 1, 0,1 );
rotacion.setRotation(btVector3(0.0f,1.0f,0.0f), NPI);
	
btDefaultMotionState* chassisMotionState = new btDefaultMotionState(btTransform(rotacion, btVector3(15.0f, .5f, 0.0f)));
chassisShape->calculateLocalInertia(chassisMass, chassisInertia);
btRigidBody::btRigidBodyConstructionInfo chassisRigidBodyCI(chassisMass, chassisMotionState, chassisShape, chassisInertia);
chassisRigidBody = new btRigidBody(chassisRigidBodyCI);
chassisRigidBody->setActivationState(DISABLE_DEACTIVATION);
// Be sure to add the chassis of the vehicle into the world as a rigid body
dynamicsWorld->addRigidBody(chassisRigidBody);

btRaycastVehicle::btVehicleTuning tuning;
raycaster = new btDefaultVehicleRaycaster(dynamicsWorld);
vehicle = new btRaycastVehicle(tuning, chassisRigidBody, raycaster);
vehicle->setCoordinateSystem(0, 1, 2);


// Be sure to attach the wheels not higher than the upper bounds of the hull of the vehicle chassis
vehicle->addWheel(btVector3(-0.85f, 0.0f, 1.40f), wheelDirection, wheelAxis, suspensionRestLength, wheelRadius, tuning, true);
vehicle->addWheel(btVector3(0.85f, 0.0f, 1.40f), wheelDirection, wheelAxis, suspensionRestLength, wheelRadius, tuning, true);
vehicle->addWheel(btVector3(-0.85f, 0.0f, -1.5f), wheelDirection, wheelAxis, suspensionRestLength, wheelRadius, tuning, false);
vehicle->addWheel(btVector3(0.85f, 0.0f, -1.5f), wheelDirection, wheelAxis, suspensionRestLength, wheelRadius, tuning, false);

for (int i=0;i<vehicle->getNumWheels();i++)
		{
			btWheelInfo& wheel = vehicle->getWheelInfo(i);
			wheel.m_suspensionStiffness = suspensionStiffness;
			wheel.m_wheelsDampingRelaxation = suspensionDamping;
			wheel.m_wheelsDampingCompression = suspensionCompression;
			wheel.m_frictionSlip = wheelFriction;
			wheel.m_rollInfluence = rollInfluence;
		}

	
dynamicsWorld->addAction(vehicle);

}




void initnave(){
	
	//posnave=btVector3(13, altura+28.5, -96.5);

	//girogradoscuerpo=-90;
//tanque

	
/*//feisar(nave wipeout)	
		navecolshape = new btBoxShape(btVector3(1.5,.25  , 2.0));
		btVector3 fallInertia(0, 0, 0);
		navemotionstate = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), posnave));
		personajecolshape->calculateLocalInertia(.5, fallInertia);
		btRigidBody::btRigidBodyConstructionInfo naveRigidBodyCI(.5,navemotionstate, navecolshape, fallInertia);
		naverb = new btRigidBody(naveRigidBodyCI);
		dynamicsWorld->addRigidBody(naverb);
*/
}
btRaycastVehicle *coche;
btRigidBody* cocherb;
btCompoundShape* compuesto;
void inittrasto(){
	//	compuesto= new btCompoundShape();
		//cocherb = localCreateRigidBody(800,tn,compuesto);//chassisShape);
		//btDefaultVehicleRaycaster *cocheRayCaster = new btDefaultVehicleRaycaster(dynamicsWorld);
		//coche = new btRaycastVehicle(m_tuning,cocherb,cocheRayCaster);




	//rueda
	navecolshape[0]=new btCylinderShape(btVector3(1.0,1.1,.5));
	btVector3 fallInertia(0, 0, 0);
	navemotionstate[0] = new btDefaultMotionState(btTransform(btQuaternion(1, 0, 0, 1), posnave));
	navecolshape[0]->calculateLocalInertia(2.0, fallInertia);
	btRigidBody::btRigidBodyConstructionInfo naveRigidBodyCI(2.0,navemotionstate[0], navecolshape[0], fallInertia);
	naverb[0] = new btRigidBody(naveRigidBodyCI);
	dynamicsWorld->addRigidBody(naverb[0]);


	navecolshape[1]=new btCylinderShape(btVector3(1.0,1.1,.5));
	//btVector3 fallInertia(0, 0, 0);
	navemotionstate[1] = new btDefaultMotionState(btTransform(btQuaternion(1, 0, 0, 1), posnave+btVector3(3.0,0.0,0.0)));
	navecolshape[1]->calculateLocalInertia(2.0, fallInertia);
	btRigidBody::btRigidBodyConstructionInfo naveRigidBodyCI1(2.0,navemotionstate[1], navecolshape[1], fallInertia);
	naverb[1] = new btRigidBody(naveRigidBodyCI1);
	dynamicsWorld->addRigidBody(naverb[1]);


}


void initcastillodomino(float x,float y,float ancho,float alto,float h){
float profundo=ancho/2;

	
for(;h<alto+1;h=h+1.20){
	for(float c=0;c<profundo-1;c=c+.70) creafichav(x+1, y+ c*3.0, h);


	for(float z=-ancho;z<0;z=z+2){
		
		for(float c=0;c<profundo*2-3;c=c+.70) creafichah(x+1+z, y+c*1.5, h-1.85);
		
		for(float c=0;c<profundo-1;c=c+.70) creafichav(x+z, y+c*3.0, h);
		
		//for(float c=0;c<4;c=c+.70) creafichav(5.9+z,  c*3.0, h);
		  
	  }
		//for(float c=0;c<2;c=c+.70) creafichav(6.9,  c*1.5, h);
		//for(float c=0;c<2;c=c+.70) creafichav(8.9,  c*1.5, h);
		//for(float c=0;c<2;c=c+.70) creafichah(6.9,  c*1.5, h);

    }
}




void initpersonaje(){
	
	

	girogradoscuerpo=-90;
	personajecolshape = new btBoxShape(btVector3(.5,1.5  , .5));
		btVector3 fallInertia(0, .01, 0);
		personajeestado = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), pospersonaje));
		personajecolshape->calculateLocalInertia(.5, fallInertia);
		btRigidBody::btRigidBodyConstructionInfo personajeRigidBodyCI(.5,personajeestado, personajecolshape, fallInertia);
		personajerb = new btRigidBody(personajeRigidBodyCI);
		dynamicsWorld->addRigidBody(personajerb);

}


void inicio() {

	glEnable(GL_TEXTURE_2D);

	/* Setup cube vertex data. */
	v[0][0] = v[1][0] = v[2][0] = v[3][0] = -.5;
	v[4][0] = v[5][0] = v[6][0] = v[7][0] = .5;
	v[0][1] = v[1][1] = v[4][1] = v[5][1] = -.5;
	v[2][1] = v[3][1] = v[6][1] = v[7][1] = .5;
	v[0][2] = v[3][2] = v[4][2] = v[7][2] = .5;
	v[1][2] = v[2][2] = v[5][2] = v[6][2] = -.5;

//inicio
	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	broadphase = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver;
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver,
			collisionConfiguration);

	dynamicsWorld->setGravity(btVector3(0, -10, 0));

	
	suelocoll = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

	groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));

	btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(0,groundMotionState, suelocoll, btVector3(0, 0, 0));
	groundRigidBody = new btRigidBody(groundRigidBodyCI);
	dynamicsWorld->addRigidBody(groundRigidBody);


	
	//iniciacol(&objfilelow);
	
	//seleccionar masa e inercia
	btScalar mass = 1;
	btVector3 fallInertia(0, 0, 0);

	//cuerpos rigidos
	

	altura=0;
	tonga(4, 15,10.0,-100.0);
	altura=30;
	tonga(3,10,11.0,-99.0);
	altura=50;
	tonga(2,5,12.0,-98.0);
	altura=60;
	tonga(1,3,13.0,-97.0);
	altura=66;
/*


	altura=0;
	tonga(5,10,9.0,-71.0);
	altura=20;
	tonga(4, 15,10.0,-70.0);
	altura=50;
	tonga(3,10,11.0,-69.0);
	altura=70;
	tonga(2,5,12.0,-68.0);
	altura=80;
	tonga(1,3,13.0,-647.0);
	altura=86;
*/

	
    
	//altura=10;
	//tonga(2,3,11.0,-101.0);
	//altura=12;
	//piramide(2,13.5,-97.5);
	
	//nocajas = nocajas + tonga(1, 10,60.0,-100.0);
	//muro(100,10,1,60.0,-100.0);
	
//initcastillodomino(0.0,0.0,9.0,9.0,1.0);
/*
	altura=11;
	tonga(4,10,-5.5,3.0);
	altura=31;
	tonga(3, 15,-4.5,4.0);
	altura=61;
	tonga(2,10,-3.5,5.0);
	altura=81;
	tonga(1,3,-2.5,6.0);
	*/
//initcastillodomino(0.0,0.0,9.0,9.0,1.0);
/*
for(int a=2;a<16;a=a+2){
	
	for(float  t=-10-a;t>-25-a;t=t-2.0){
		for(float  c=-10-a;c>-25-a;c=c-1.0){
			creafichah(t, c, a+1.0);	
		}
	}
}*/


pospersonaje=btVector3(-3.75, 3.0, 4.5);


initpersonaje();
initvehiculo();
inittrasto();
}






void rendercoche (struct obj_model_t *mdl)
{
  int i, j;
 bool bandera=true;
 float *v3t;
 float *v3n;
 float *v4;
	
  for (i = 0; i < mdl->num_faces; ++i)
   {
   glBegin (mdl->faces[i].type);
   
	for (j = 0; j <mdl->faces[i].num_elems; ++j)
	{
		
		  glColor3f(.5,.5,.5);
		
	  
		
	    if (mdl->has_texCoords){			  
	      glTexCoord3fv (mdl->texCoords[mdl->faces[i].uvw_indices[j]].uvw);
		}  
	    if (mdl->has_normals){
	      glNormal3fv (mdl->normals[mdl->faces[i].norm_indices[j]].ijk);

	    glVertex4fv (mdl->vertices [mdl->faces[i].vert_indices[j]].xyzw);
		}
	  }
/*
	for (j = mdl->faces[i].num_elems-1; j>=0; --j)
	{
		
		  glColor3f(.7,.7,.7);
		
	  
		
	    if (mdl->has_texCoords){
		v3t=mdl->texCoords[mdl->faces[i].uvw_indices[j]].uvw;
	      glTexCoord3f (v3t[0],v3t[1],-v3t[2]);
		}  
	    if (mdl->has_normals){
			v3n=mdl->normals[mdl->faces[i].norm_indices[j]].ijk;
	      glNormal3f (v3n[0],v3n[1],-v3n[2]);
			v4=mdl->vertices [mdl->faces[i].vert_indices[j]].xyzw;
		  glVertex4f (v4[0],v4[1],-v4[2],v4[3]);
		}
	  }
	*/   
   
	glEnd();
		
    }
}



void fin() {
	for (int j = 0; j < nobolas; j++) {
		dynamicsWorld->removeRigidBody(fallRigidBody[j]);
		delete fallRigidBody[j]->getMotionState();
		delete fallRigidBody[j];
	}
	for (int j = 0; j < nocajas; j++) {
		dynamicsWorld->removeRigidBody(cuborb[j]);
		delete cuborb[j]->getMotionState();
		delete cuborb[j];
	}
	for (int j = 0; j < indice; j++) {
		dynamicsWorld->removeRigidBody(murorb[j]);
		delete murorb[j]->getMotionState();
		delete murorb[j];
	}
	dynamicsWorld->removeRigidBody(groundRigidBody);
	delete groundRigidBody->getMotionState();
	delete groundRigidBody;

	for (int j = 0; j < nobolas; j++) {
		delete fallShapecoll[j];
	}
	for (int j = 0; j < MAXCAJAS; j++) {
		delete cuboShapecoll[j];
	}
	for (int j = 0; j < indice; j++) {
		delete muroshapecol[j];
	}
	delete suelocoll;

	delete dynamicsWorld;
	delete solver;
	delete collisionConfiguration;
	delete dispatcher;
	delete broadphase;

}
double paso() {

	return 0.0;
}

void changeSize(int w, int h) {

	// Prevent a divide by zero, when window is too short
	// (you cant make a window of zero width).
	if (h == 0)
		h = 1;

	float ratio = w * 1.0 / h;

	// Use the Projection Matrix
	glMatrixMode(GL_PROJECTION);

	// Reset Matrix
	glLoadIdentity();

	// Set the viewport to be the entire window
	glViewport(0, 0, w, h);

	// Set the correct perspective.
	gluPerspective(60.0f, ratio, 0.1f, 1000.0f);

	// Get Back to the Modelview
	glMatrixMode(GL_MODELVIEW);
	cruzx=w/2;
	cruzy=h/2;
}

void dibujaplanoz0(double x1, double y1, double x2, double y2) {
	glBegin(GL_QUADS);
	glColor3f(1., 1., .1);
	glVertex3f(x1, -0.001, y1);
	glVertex3f(x2, -0.001, y1);
	glVertex3f(x2, -0.001, y2);
	glVertex3f(x1, -0.001, y2);
	glEnd();
}

void drawBox(int i) {
	glColor3f(1.0,1.0,1.0);
	glBindTexture( GL_TEXTURE_2D, texturecaja[texturascajas[i]]); //bind the texture
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
	GL_LINEAR_MIPMAP_NEAREST);
	glBegin(GL_QUADS);
	// Front Face
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f);  // Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(1.0f, -1.0f, 1.0f);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(1.0f, 1.0f, 1.0f);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, 1.0f);  // Top Left Of The Texture and Quad
	// Back Face
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(-1.0f, -1.0f, -1.0f);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(1.0f, 1.0f, -1.0f);  // Top Left Of The Texture and Quad
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(1.0f, -1.0f, -1.0f);  // Bottom Left Of The Texture and Quad
	// Top Face
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f);  // Top Left Of The Texture and Quad
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(-1.0f, 1.0f, 1.0f);  // Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(1.0f, 1.0f, 1.0f);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(1.0f, 1.0f, -1.0f);  // Top Right Of The Texture and Quad
	// Bottom Face
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(-1.0f, -1.0f, -1.0f);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(1.0f, -1.0f, -1.0f);  // Top Left Of The Texture and Quad
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(1.0f, -1.0f, 1.0f);  // Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f);  // Bottom Right Of The Texture and Quad
	// Right face
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(1.0f, -1.0f, -1.0f);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(1.0f, 1.0f, -1.0f);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(1.0f, 1.0f, 1.0f);  // Top Left Of The Texture and Quad
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(1.0f, -1.0f, 1.0f);  // Bottom Left Of The Texture and Quad
	// Left Face
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(-1.0f, -1.0f, -1.0f);  // Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, 1.0f);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f);  // Top Left Of The Texture and Quad
	glEnd();

}

void dibujamuro(float ancho,float alto,float profundo) {

	glBindTexture( GL_TEXTURE_2D, texturemuro); //bind the texture
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
	GL_LINEAR_MIPMAP_NEAREST);
	glBegin(GL_QUADS);
	// Front Face
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(-ancho/2, -alto/2, profundo/2);  // Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(ancho/2, -alto/2, profundo/2);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(ancho/2, alto/2, profundo/2);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(-ancho/2, alto/2, profundo/2);  // Top Left Of The Texture and Quad
	// Back Face
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(-ancho/2, -alto/2, -profundo/2);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(-ancho/2, alto/2, -profundo/2);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(ancho/2, alto/2, -profundo/2);  // Top Left Of The Texture and Quad
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(ancho/2, -alto/2, -profundo/2);  // Bottom Left Of The Texture and Quad
	// Top Face
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(-ancho/2, alto/2, -profundo/2);  // Top Left Of The Texture and Quad
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(-ancho/2, alto/2, profundo/2);  // Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(ancho/2, alto/2, profundo/2);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(ancho/2, alto/2, -profundo/2);  // Top Right Of The Texture and Quad
	// Bottom Face
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(-ancho/2, -alto/2, -profundo/2);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(ancho/2, -alto/2, -profundo/2);  // Top Left Of The Texture and Quad
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(ancho/2, -alto/2, profundo/2);  // Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(-1.0f, -alto/2, profundo/2);  // Bottom Right Of The Texture and Quad
	// Right face
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(ancho/2, -alto/2, -profundo/2);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(ancho/2, alto/2, -profundo/2);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(ancho/2, alto/2, profundo/2);  // Top Left Of The Texture and Quad
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(ancho/2, -alto/2, profundo/2);  // Bottom Left Of The Texture and Quad
	// Left Face
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(-ancho/2, -alto/2, -profundo/2);  // Bottom Left Of The Texture and Quad
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(-ancho/2, -alto/2, profundo/2);  // Bottom Right Of The Texture and Quad
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f(-ancho/2, alto/2, profundo/2);  // Top Right Of The Texture and Quad
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(-ancho/2, alto/2, -profundo/2);  // Top Left Of The Texture and Quad
	glEnd();

}




void initluces() {
//luces
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	//glMatrixMode(GL_MODELVIEW);
	//luces
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);
	glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpec);

	glLightfv(GL_LIGHT1, GL_POSITION, lightSPos);
	glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, spotDir);
	glLightf(GL_LIGHT1, GL_SPOT_EXPONENT, exponent);
	glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, cutoff);
}

void puntodevista() {

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	if(!freelook){
		
		vt.setIdentity();
		chassisRigidBody->getMotionState()->getWorldTransform(vt);
		eyePos[0]=vt.getOrigin().getX();
		eyePos[1]=vt.getOrigin().getY()+2.0;
		eyePos[2]=vt.getOrigin().getZ();
	/*	btQuaternion vector=vt.getRotation();
		btQuaternion vectorn=vector.normalized();
		btVector3 mu=btVector3(0.0,0.0,1.0);
		mu=vt*mu;
		vectorvision[X]=vectorn[X];
		vectorvision[Y]=vectorn[Y];
		vectorvision[Z]=vectorn[Z];*/
	}
	if(vistavala){
		tp.setIdentity();
		fallRigidBody[nobolas-1]->getMotionState()->getWorldTransform(tp);
		eyePos[0]=tp.getOrigin().getX()-vectorvision[X];
		eyePos[1]=tp.getOrigin().getY()-vectorvision[Y]+.75;
		eyePos[2]=tp.getOrigin().getZ()-vectorvision[Z];
	}
		//puntodevista();
	// Set the camera
	gluLookAt( eyePos[X], eyePos[Y], eyePos[Z],
	           eyePos[X]+vectorvision[X], eyePos[Y]+vectorvision[Y], eyePos[Z]+vectorvision[Z],
	           0.0, 1.0, 0.0);
}


void inicializa() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// set the GL clear color - use when the color buffer is cleared
	// glClearColor( 0.0f, 0.0f,0.0f, 1.0f );
	// set the shading model to 'smooth'
	glShadeModel( GL_SMOOTH);
	// enable depth
	glEnable( GL_DEPTH_TEST);
	// set the front faces of polygons
	glFrontFace( GL_CCW);
	// set fill mode
	glPolygonMode( GL_FRONT_AND_BACK, GL_FILL);
}

void
mouse(int button, int state, int x, int y)
{
  if (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN) {
    moving = 1;
    beginx = x;
	beginy = y;
	
  }
  if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
    moving = 0;
  }
	 if (button==3) {
	  	 eyePos[0]=eyePos[0]+vectorvision[0];
		 eyePos[1]=eyePos[1]+vectorvision[1];
	     eyePos[2]=eyePos[2]+vectorvision[2];
	}
	if (button==4){
		eyePos[0]=eyePos[0]-vectorvision[0];
		 eyePos[1]=eyePos[1]-vectorvision[1];
	     eyePos[2]=eyePos[2]-vectorvision[2];
	}
if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {
	if(freelook){
				
				 int j=nobolas;
				 btVector3 fallInertia=btVector3(0,0,0);
				 fallShapecoll[j] = new btSphereShape(0.5f);
				 fallMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(eyePos[X], eyePos[Y], eyePos[Z])));
				 fallShapecoll[j]->calculateLocalInertia(2.0, fallInertia);
				 btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(2.0,fallMotionState, fallShapecoll[j], fallInertia);
				 fallRigidBody[j] = new btRigidBody(fallRigidBodyCI);
				 dynamicsWorld->addRigidBody(fallRigidBody[j]);
				 nobolas++;
				
				fallRigidBody[j]->setLinearVelocity(fallRigidBody[j]->getLinearVelocity()+vectorvision*40);
			}
}
}
/* ARGSUSED1 */
void
motion(int x, int y)
{
if(moving){
if (is_first_time) {
        //is_first_time = false;
        // Check for zooming mode, but do not allow the model
        // jump off the screen when the mouse enters the window from an outside
        // region of screen. Thus, to activate the mouse, you need to click the
        // mouse to zoom the view AND move it a little:
        is_first_time = false;
        beginx = x;
        beginy = y;
        return;
    }
    GLint deltaX = beginx-x;
    GLint deltaY = beginy-y;
	
 
    if ( deltaX < -1.0 ) { deltaX = -1.0; }
    if ( deltaX >  1.0 ) { deltaX =  1.0; }
    if ( deltaY < -1.0 ) { deltaY = -1.0; }
    if ( deltaY >  1.0 ) { deltaY =  1.0; }
	alfax=alfax-deltaX*(NPI/180);
	alfay=alfay+deltaY*(NPI/180);

	if(alfay>NPI/2) alfay=NPI/2;
	if(alfay<-NPI/2) alfay=-NPI/2;
	vectorvision[X]=cos(alfax);
	vectorvision[Y]=sin(alfay);
	vectorvision[Z]=sin(alfax);

	

	

	beginx = x;
    beginy = y;

	glutPostRedisplay();
   
}


	/*
  if (moving) {
    viewAnglex = viewAnglex + (x - beginx);
	viewAngley = viewAngley + (y - beginy);
    eyePos[X] = sin(viewAnglex * NPI / 180.0) * 10.0;
    eyePos[Z] = cos(viewAnglex * NPI / 180.0) * 10.0;
    begin = x;
    glutPostRedisplay();
  }*/
}





void modelacircuito (struct obj_model_t *mdl)
{
  int i, j;
	float a[3],pn[3];
	//obj_vertex_t v=  mdl->vertices[0];
	
	for (i = 0; i < mdl->num_faces; i++)
    {

		for(int m=0;m<mdl->faces[i].num_elems-1;m++){
		//dibujacacho(mdl->vertices[i].xyzw,mdl->vertices[i+1].xyzw);
		//btScalar ma[16]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
		float fx=1;
			float *pi,*pf;
			float n2[3],nm[3],pr[3],pm[3];
				 pi=mdl->vertices[mdl->faces[i].norm_indices[m]].xyzw;
			if(m!=2){
				pf=mdl->vertices[mdl->faces[i].norm_indices[m+1]].xyzw;
				n2[0]=	  mdl->normals[mdl->faces[i].norm_indices[m+1]].ijk[0];
				n2[1]=    mdl->normals[mdl->faces[i].norm_indices[m+1]].ijk[1];
				n2[3]=    mdl->normals[mdl->faces[i].norm_indices[m+1]].ijk[2];

			}

			else	{
				pf=mdl->vertices[mdl->faces[i].norm_indices[0]].xyzw;
				n2[0]=	  mdl->normals[mdl->faces[i].norm_indices[0]].ijk[0];
				n2[1]=    mdl->normals[mdl->faces[i].norm_indices[0]].ijk[1];
				n2[3]=    mdl->normals[mdl->faces[i].norm_indices[0]].ijk[2];
			}

					
		float no[3]=	 { mdl->normals[mdl->faces[i].norm_indices[m]].ijk[0],
				           mdl->normals[mdl->faces[i].norm_indices[m]].ijk[1],
				           mdl->normals[mdl->faces[i].norm_indices[m]].ijk[2]};


			nm[0]=(no[X]+n2[X])/2;
			nm[1]=(no[Y]+n2[Y])/2;
			nm[2]=(no[Z]+n2[Z])/2; 
			
			pr[0]=pf[0]-pi[0];
			pr[1]=pf[1]-pi[1];
			pr[2]=pf[2]-pi[2];
			
			pm[0]=pi[X]+pr[X]/2;
			pm[1]=pi[Y]+pr[Y]/2;
			pm[2]=pi[Z]+pr[Z]/2;


			float dis=distancia(pm,pi);
			
					pn[X]=pr[X]/dis;
					pn[Y]=pr[Y]/dis;
					pn[Z]=pr[Z]/dis;
			
			
		float dife=distancia(pi,pf);
		if(no[Y]>-0.2 && no[Y]<0.2){
			glPushMatrix();
			
			glTranslatef(pi[X],pi[Y],pi[Z]);
			glTranslatef(no[0]/2.0,
				         no[1]/2.0,
				         no[2]/2.0);
			glLineWidth(2.0f);
			
			glBegin(GL_LINES);
				glColor3f(0.0,0.0,0.0);
				glVertex3d(0.0, 0.0, 0.0);
				glVertex3d(pr[X],pr[Y],pr[Z]);
			glEnd();
			
				glTranslatef(pr[X]/2,pr[Y]/2,pr[Z]/2);
				
				glColor3f(0.0,1.0,0.0);
			glLineWidth(2.0f);
			
			glBegin(GL_LINES);
				glVertex3d(0.0, 0.0, 0.0);
				glVertex3d(no[X],no[Y],no[Z]);
			glEnd();
			glColor3f(1.0,1.0,1.0);
			glBegin(GL_LINES);
				glVertex3d(0.0, 0.0, 0.0);
				glVertex3d(pn[X],pn[Y],pn[Z]);
			glEnd();
			a[0]=0.0;
			a[1]=0.0;
			a[2]=0.0;

			
			float di=distancia(pn,a);
			if(pn[X]>=0.0){
					if(pn[Y]>=0.0){ //angulos primer cuadrante
						glColor3f(1.0,5.0,1.0);
						a[0]=agrados(asin(pn[Y]/di));
						a[1]=agrados(acos(pn[Z]/di));
						a[2]=agrados(atan(no[Y]/no[Z]));	
					}
					else{//angulos segundo cuadrante
						glColor3f(0.5,0.5,0.5);
						a[0]=-agrados(asin(pn[Y]/di));
						a[1]=agrados(acos(pn[Z]/di));
						a[2]=-agrados(atan(no[Y]/no[Z]));
					}
			}else{
					if(pn[Y]>=0){//angulos tercer cuadrante
						glColor3f(0.1,0.1,0.1);
						a[0]=agrados(asin(pn[Y]/di));
						a[1]=-agrados(acos(pn[Z]/di));
						a[2]=agrados(atan(no[Y]/no[Z]));
					}
					else{//angulos cuarto cuadrante
						glColor3f(0.0,0.0,1.0);
						a[0]=-agrados(asin(pn[Y]/di));
						a[1]=-agrados(acos(pn[Z]/di));
						a[2]=-agrados(atan(pn[Y]/no[Z]));
					}
			}
					
					
					
					glLineWidth(3.0f);
					glRotatef(a[0],1.0,0.0,0.0);
					glRotatef(a[1]-180,0.0,1.0,0.0);
					glRotatef(vara+a[2],0.0,0.0,1.0);
					GLfloat co[3]={1.0f, 0.0f, 0.0f};
					
					
					//glScalef(1.5,1.5,dife*4.25);
					//glutSolidCube(.25);
					glutSolidCone(10.0,.3,5,3);

			
				
			glPopMatrix();

			



			


			
		}

		}
    }
	
}

void dibujavertices(struct obj_model_t *mdl)
{
	vec4_t v;
for (int i = 0; i < mdl->num_verts; i++)
    {
		glPushMatrix();
				glTranslatef(mdl->vertices[i].xyzw[0],mdl->vertices[i].xyzw[1],mdl->vertices[i].xyzw[2]);
			//	glTranslatef(kk[X],kk[Y],kk[Z]);
				//glTranslatef(mdl->vertices[i].xyzw[0],mdl->vertices[i].xyzw[1],mdl->vertices[i].xyzw[2]);
				glColor3f(0.0,0.0,0.0);
				//glColor3f(0.3*m+0.3,0.3*m+0.3,0.3*m+0.3);
				glutSolidSphere(.05,6,6);
		glPopMatrix();
/*
		for(int m=0;m<3;m++){
				 float *pi=mdl->vertices[i+m].xyzw;
				glPushMatrix();
				glTranslatef(pi[X],pi[Y],pi[Z]);
				glColor3f(0.3*m+0.3,0.3*m+0.3,0.3*m+0.3);
				glutSolidSphere(.05,6,6);
				glPopMatrix();
				
		}
*/

}
}

void pintacol(struct obj_model_t *mdl)
{
  int i, j;
	float a[3],pn[3], *pi,*pf,d;
	//obj_vertex_t v=  mdl->vertices[0];
	
	for (i = 0; i < mdl->num_faces/2; i++)
    {

		float *p0=mdl->vertices[mdl->faces[i].vert_indices[0]].xyzw;
		float *p1=mdl->vertices[mdl->faces[i].vert_indices[1]].xyzw;
		float *p2=mdl->vertices[mdl->faces[i].vert_indices[2]].xyzw;

		float *pf0=mdl->vertices[mdl->faces[i*2].vert_indices[0]].xyzw;
		float *pf1=mdl->vertices[mdl->faces[i*2].vert_indices[1]].xyzw;
		float *pf2=mdl->vertices[mdl->faces[i*2].vert_indices[2]].xyzw;

		
		float d0=distancia(p0,p1);
		float d1=distancia(p1,p2);
		float d2=distancia(p2,p0);

		float pm[3]={   (p0[0]+p1[0]+p2[0])/3,
						(p0[1]+p1[1]+p2[1])/3,
						(p0[2]+p1[2]+p2[2])/3 };
		
		float pfm[3]={  (pf0[0]+pf1[0]+pf2[0])/3,
						(pf0[1]+pf1[1]+pf2[1])/3,
						(pf0[2]+pf1[2]+pf2[2])/3 };

		float dr[3]={   (pfm[0]-pm[0])/2,
						(pfm[1]-pm[1])/2,
						(pfm[2]-pm[2])/2};
		
		if(d0 < d1 && d0< d2){
			d=d0;
		}
		if(d1 < d0 && d1< d2){
			d=d1;
		}
		if(d2 < d0 && d2 <d1) d=d2;
		float no[3]=	 { mdl->normals[mdl->faces[i].norm_indices[0]].ijk[0],
				           mdl->normals[mdl->faces[i].norm_indices[0]].ijk[1],
				           mdl->normals[mdl->faces[i].norm_indices[0]].ijk[2]};
		
		
		obj_normal_t normal=mdl->normals[i];
		
		
			glPushMatrix();
				
				if(i%7==3) glColor3f(1.0,0.0,0.0); //suelo rojo 
		
				if(i%14==1 || i%16==14 || i%16==2 || i%16==10){ //paredes izquierda y derecha
				float *pr,*pm,pn;
					
					
									
					if(i%16==6) glColor3f(1.0,1.0,0.0); //paredes interiores
					else glColor3f(1.0,.2,.2); //paredes interiores
	
					glTranslatef((p0[0]+p1[0]+p2[0])/3, (p0[1]+p1[1]+p2[1])/3, (p0[2]+p1[2]+p2[2])/3); //se translada al punto medio
							
				}
		
			if(no[X]>=0.0){
					if(no[Z]>=0.0){ 
						glColor3f(1.0,0.0,1.0);
						a[1]=180+agrados(acos(no[Z]));
					}
					else{
						glColor3f(0.0,1.0,1.0);
						a[1]=180+agrados(acos(no[Z]));
					}
			}else{
					if(no[Z]>=0){
						glColor3f(0.5,0.5,0.5);
						a[1]=180-agrados(acos(no[Z]));	
					}
					else{
						glColor3f(0.1,6.0,1.0);
						a[1]=180-agrados(acos(no[Z]));
					}
			}
			glRotatef(a[1],0.0,1.0,0.0);
			//glS//lef(1.0,11.0d);
			glutSolidCube(.25);
			glPopMatrix();

			 }
	
}

void
RenderOBJedificio (struct obj_model_t *mdl)
{
  int i, j;
  glColor3f(.7,.7,.7);
	
  for (i = 0; i < mdl->num_faces; ++i)
    {
		/*if (mdl->normals[mdl->faces[i].norm_indices[0]].ijk[1]>.85){
			glBindTexture( GL_TEXTURE_2D, texturatecho); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);			
		}else{
			glBindTexture( GL_TEXTURE_2D, texturaventana); //bind the texture
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST); // ( NEW )
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
		}*/
      glBegin (mdl->faces[i].type);
				
	for (j = 0; j < mdl->faces[i].num_elems; ++j)
	  {
		

		if (mdl->has_texCoords)
	      glTexCoord3fv (mdl->texCoords[mdl->faces[i].uvw_indices[j]].uvw);

	    if (mdl->has_normals)
	      glNormal3fv (mdl->normals[mdl->faces[i].norm_indices[j]].ijk);

	    glVertex4fv (mdl->vertices [mdl->faces[i].vert_indices[j]].xyzw);
	  }
	glEnd();
    }
}

