void specialkeyboard(int key, intf x, int y) {
	//printf("\ntecla %d", key);
//fallRigidBody[0]->applyTorqueImpulse(btVector3(-1.0,0.0,0.0));
	btVector3 vm=btVector3(0.0,0.0,0.0);
	if (key==100) //izquierda
	//	vm=vm+btVector3(1.0, 0.0, 0.0);
		
	if (key==103) {//abajo
		//vm=vm+btVector3(0.0, 0.0, 1.0);
		if(freelook){
			eyePos[0]=eyePos[0]-vectorvision[0]*2;
			eyePos[1]=eyePos[1]-vectorvision[1]*2;
			eyePos[2]=eyePos[2]-vectorvision[2]*2;
		}else{
			//personajerb->translate( btVector3(vectorvision[0]/8, 0.0f,vectorvision[2])/8 );
		}
	}
	if (key==101){ //arriba
	//	vm=vm+btVector3(0.0, 0.0, -1.0);
		if(freelook){
		 eyePos[0]=eyePos[0]+vectorvision[0]*2;
		 eyePos[1]=eyePos[1]+vectorvision[1]*2;
	     eyePos[2]=eyePos[2]+vectorvision[2]*2;
		}else{
			//personajerb->translate( btVector3(-vectorvision[0]/8, 0.0f,-vectorvision[2])/8 );
		}
	}
	if (key==102){ //derecha
	//	vm=vm+btVector3(-1.0, 0.0, 0.0);
	}
	if (key== 27){ //esc

		fin();
		exit(0);
	}
	
}



void specialkeyboard(unsigned char key, int x, int y) {
	bool mover=false;
	float fuerzai=0.0,fuerzad=0.0;
	//printf("\ntecla %uc", key);
//fallRigidBody[0]->applyTorqueImpulse(btVector3(0.0,-1.0,0.0));
	switch (key) {
	case 'e': //arrib	
			if(gEngineForce<maxEngineForce){
				gEngineForce += acel;
				gBreakingForce = 0.f;
				movervehiculo();
			}
		//naverb->setLinearVelocity(fallRigidBody[0]->getLinearVelocity()+vectorvision);
		//eeeenaverb[0]->applyCentralImpulse(vectorvision);
		//personajerb->translate(btVector3(vectorvision[X],0.0,vectorvision[Z]));
	break;
		case 'c': //atras
			if(gEngineForce>-maxEngineForce){
				gEngineForce -= acel;
				gBreakingForce = 0.f;
				movervehiculo();
			}
		
	break;
		case 's':
			if(gVehicleSteering<(NPI/5)){
				gVehicleSteering += steeringIncrement;
			}
			movervehiculo();	
		//fallRigidBody[0]->setLinearVelocity(fallRigidBody[0]->getLinearVelocity()+btVector3(+sin(vectorvision[2]), 0.0, +cos(vectorvision[2])));
		break;
	case 'f':
			if(gVehicleSteering>-(NPI/5)){
				gVehicleSteering -= steeringIncrement;
			}
			movervehiculo();
		//fallRigidBody[0]->setLinearVelocity(fallRigidBody[0]->getLinearVelocity()+btVector3(-sin(vectorvision[2]), 0.0,-cos(vectorvision[2])));
		break;
	case 'd':
			gBreakingForce = maxBreakingForce; 
			gEngineForce = 0.f;
		    movervehiculo();
		    break;
	case 'l':
		freelook=!freelook;
		break;
	case 'p':
		vara=vara+1.0;
		break;
	case 'o':
		/*	movervehiculo();*/
		vara=vara-1.0;
		break;

	case 'y':
		initpersonaje();
		break;
	
	
	case ' ':
		printf("\neyePos[]={%f, %f, %f}\t vectorvision[]={%f, %f, %f}",eyePos[X],eyePos[Y],eyePos[Z], vectorvision[X],vectorvision[Y],vectorvision[Z]);	
		break;	
	case 'v':
		pintacircuito=!pintacircuito;
		
		break;
	case 'x':
		pintamuros=!pintamuros;
		
		break;
		case 'z':
		fisica=!fisica;
		
		break;	
	case 'q':
			fin();
			exit(0);
			break;
	case '0':
			vistavala=!vistavala;
	default:
		break;
	}

	
	
}