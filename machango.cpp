#include <iostream>
#include <math.h>




class Machango
{
	public:
   int anima;
   int maxanim;
 
 
	

    float casx;
    float casy;
    float alfa;
    float altura;

	Machango(float inx, float iny, float alt, int sprlen){ 
		this->anima=0;
		this->maxanim=sprlen;
		this->casx=inx;
		this->casy=iny;
		this->altura=alt;
		this->alfa=0;
		
		

	}
	
	void update(bool adelante)
	{
		if(adelante){
			if(anima < maxanim-1) anima++;
			else anima = 0;
		}else{
			if(anima>0) anima--;
			else anima=maxanim-1;
		}
	}

	void avanza(void)
	{
		this->casx=this->casx+sin(this->alfa);
		this->casy=this->casy+cos(this->alfa);

	}


	
};

	
    
	