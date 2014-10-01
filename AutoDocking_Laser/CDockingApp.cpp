/* +---------------------------------------------------------------------------+
   |                 Open MORA (MObile Robot Arquitecture)                     |
   |                                                                           |
   |                        http://babel.isa.uma.es/mora/                      |
   |                                                                           |
   |   Copyright (C) 2010  University of Malaga                                |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics (MAPIR) Lab, University of Malaga (Spain).                  |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MORA project.                                   |
   |                                                                           |
   |     MORA is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MORA is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MORA.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

/**  @moos_module Module to automaticly dock the robot in the charging station using scan lasers and a predefined pattern over the charging station.
  *  This module implements the functionality of detecting a pattern based on three sticks of different width, and move towards it until
  *  the robot detects that its charging.
  */


/**  @moos_ToDo
  *  Complete description of parameteres and published variables.
  */

#include "CDockingApp.hpp"

#include <sstream>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/gui.h>

using namespace std;

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::gui;



CDockingApp::CDockingApp()
//	: var (init_val), ...
{
}

CDockingApp::~CDockingApp()
{
}

bool CDockingApp::OnStartUp()
{
  // Read parameters (if any) from the mission configuration file.
  
  //! @moos_param   distancia_corte
  m_MissionReader.GetConfigurationParam("distancia_corte",distancia_corte);
  //! @moos_param  longitud_total_patron
  m_MissionReader.GetConfigurationParam("longitud_total_patron",longitud_total_patron);
  //! @moos_param  distancia_comprobacion
  m_MissionReader.GetConfigurationParam("distancia_comprobacion",distancia_comprobacion);
  //! @moos_param  error_pendiente
  m_MissionReader.GetConfigurationParam("error_pendiente",error_pendiente);
  //! @moos_param  distancia_parada
  m_MissionReader.GetConfigurationParam("distancia_parada",distancia_parada);
  //! @moos_param  v_lineal_f2_rap
  m_MissionReader.GetConfigurationParam("v_lineal_f2_rap",v_lineal_f2_rap);
  //! @moos_param  v_lineal_f2_len
  m_MissionReader.GetConfigurationParam("v_lineal_f2_len",v_lineal_f2_len);
  //! @moos_param  agiro_f2_rap
  m_MissionReader.GetConfigurationParam("agiro_f2_rap",agiro_f2_rap);
  //! @moos_param  agiro_f2_len
  m_MissionReader.GetConfigurationParam("agiro_f2_len",agiro_f2_len);
  //! @moos_param  v_lineal_mover
  m_MissionReader.GetConfigurationParam("v_lineal_mover",v_lineal_mover);
  //! @moos_param  agiro_mover
  m_MissionReader.GetConfigurationParam("agiro_mover",agiro_mover);
  //! @moos_param  demasiado_cerca
  m_MissionReader.GetConfigurationParam("demasiado_cerca",demasiado_cerca);
  //! @moos_param  lon_sp1i
  m_MissionReader.GetConfigurationParam("lon_sp1i",lon_sp1i);
  //! @moos_param  lon_sp1s
  m_MissionReader.GetConfigurationParam("lon_sp1s",lon_sp1s);
  //! @moos_param  lon_sp2i
  m_MissionReader.GetConfigurationParam("lon_sp2i",lon_sp2i);
  //! @moos_param  lon_sp2s
  m_MissionReader.GetConfigurationParam("lon_sp2s",lon_sp2s);
  //! @moos_param  lon_sp3i
  m_MissionReader.GetConfigurationParam("lon_sp3i",lon_sp3i);
  //! @moos_param  lon_sp3s
  m_MissionReader.GetConfigurationParam("lon_sp3s",lon_sp3s);
  //! @moos_param  lon_aux1i
  m_MissionReader.GetConfigurationParam("lon_aux1i",lon_aux1i);
  //! @moos_param  lon_aux1s
  m_MissionReader.GetConfigurationParam("lon_aux1s",lon_aux1s);
  //! @moos_param  lon_aux2i
  m_MissionReader.GetConfigurationParam("lon_aux2i",lon_aux2i);
  //! @moos_param  lon_aux2s
  m_MissionReader.GetConfigurationParam("lon_aux2s",lon_aux2s);


  //cga:vble debug from the configuration file. If it is true, the wxwindow is displayed
  //! @moos_param  debug If it is true, a wxwindow is displayed
  m_MissionReader.GetConfigurationParam("debug",debug);
  if (debug)
    win=new CDisplayWindowPlots("Docking debug window");


  //-------------------------
  // Set initial state
  //-------------------------
  f2activada = false;				//Init the sistem as Far from docking!  
  contador_no_encontrados = 0;		//number of non-matches
  encontrados = 0;					//number of matches
  patronverdadero = false;			//only after 4 consecutive matches, we declare the pattern as the correct one
  angulovi = 0;
  angulovf = 0;
  disv = 0;  
  ok_anterior = false;				//Was the pattern found in the previous scan?
  punto_inicio = 0;
  punto_fin = 0;
  contfases2 = 0;

  //miparking=true;
  //robotenposicion = false;
  //pruebafich = 0;

  //Start doing nothing!!
  go_docking = false;
  charge = 1.0;			//is charging
  park = 0.0;			//is not parking


  return DoRegistrations();
}


//---------------------------------------------------------------------
// dibujar_puntos: Display Laser detected points in the "debug" window
//---------------------------------------------------------------------
void CDockingApp::dibujar_puntos(int p1i,int p1f,int p2i,int p2f,int p3i,int p3f)
{
  mrpt::slam::CSimplePointsMap TheMap,TheMap2;
  int i;
  mrpt::math::CVectorFloat xs,ys,zs,xs1,ys1,zs1;


  /*	for(i=0;i<p1i;i++){
                TheMap.insertPoint(posX[i]/100,posY[i]/100);
        } */

  for(i=p1i;i<=p1f;i++){
      TheMap.insertPoint(posX[i]/100,posY[i]/100,0,0.9,0,0);
    }

  /*	for(i=p1f+1;i<p2i;i++){
                TheMap.insertPoint(posX[i]/100,posY[i]/100);
        } */

  for(i=p2i;i<=p2f;i++){
      TheMap.insertPoint(posX[i]/100,posY[i]/100,0,0.9,0.9,0.9);
    }

  /*	for(i=p2f+1;i<p3i;i++){
                TheMap.insertPoint(posX[i]/100,posY[i]/100);
        } */

  for(i=p3i;i<=p3f;i++){
      TheMap.insertPoint(posX[i]/100,posY[i]/100,0,0.9,0.9,0.9);
    }

  /*	for(i=p3f+1;i<TAM;i++){
                TheMap.insertPoint(posX[i]/100,posY[i]/100);
        } */

  for(i=0;i<TAM;i++){
      TheMap2.insertPoint(posX[i]/100,posY[i]/100,0,0.9,0.9,0.9);
    }
  TheMap2.getAllPoints(xs1,ys1,zs1);
  win->plot(xs1,ys1,".b3");

  //sleep(150);

  TheMap.getAllPoints(xs,ys,zs);
  win->plot(xs,ys,".r3");

  //sleep(150);
}

//---------------------------------------------------------
// Check that found pattern matchs those previously found
//---------------------------------------------------------
void CDockingApp::comprobar_patron_verdadero(int pi,int pf)
{
	float ai=0;
	float af=0;
	float d=0;
	float aux=20;

	ai=pi*angulo_giro_hok;
	af=pf*angulo_giro_hok;
	d=obs->scan[pi]*100;
  
	ok=false;

	if((angulovi<(ai+aux))&&(angulovi>(ai-aux)))
	{//punto inicio correcto
		if((angulovf<(af+aux))&&(angulovf>(af-aux)))
		{ // punto fin correcto
			if((disv<(d+aux))&&(disv>(d-aux)))
			{ //distancia correcta
				ok = true;			//pattern found!!
				angulovi = ai;
				angulovf = af;
				disv = d;
			}
		}
	}
}



void CDockingApp::comprobar_pendientes_rectas(float r1b,float r2b,float r3b){

  float auxiliar;
  float limite=5;

  auxiliar=r1b-r2b;
  if(auxiliar<0){
      auxiliar=auxiliar*(-1);
    }
  if(auxiliar>limite){
      ok=false;
    }
  auxiliar=r2b-r3b;
  if(auxiliar<0){
      auxiliar=auxiliar*(-1);
    }
  if(auxiliar>limite){
      ok=false;
    }
  auxiliar=r1b-r3b;
  if(auxiliar<0){
      auxiliar=auxiliar*(-1);
    }
  if(auxiliar>limite){
      ok=false;
    }

  auxiliar=valorbfinal-r1b;
  if(auxiliar<0){
      auxiliar=auxiliar*(-1);
    }
  if(auxiliar>limite){
      ok=false;
    }

  auxiliar=valorbfinal-r2b;
  if(auxiliar<0){
      auxiliar=auxiliar*(-1);
    }
  if(auxiliar>limite){
      ok=false;
    }

  auxiliar=valorbfinal-r3b;
  if(auxiliar<0){
      auxiliar=auxiliar*(-1);
    }
  if(auxiliar>limite){
      ok=false;
    }
}



//--------------------------------------------------------------------------
//  mover_fase2 --> Mode of operation when we are close to the docking
//--------------------------------------------------------------------------
void CDockingApp::mover_fase2(float cp,float dis,float cory,float corx)
{
  float agiro_rad_rap;
  float agiro_rad_len;
  float volt=0;

  //CMOOSVariable *PfVoltaje= GetMOOSVar("fVoltaje");
  //volt=pfVoltaje->GetDoubleVal();

  agiro_rad_rap=DEG2RAD(agiro_f2_rap);
  agiro_rad_len=DEG2RAD(agiro_f2_len);

  cout << "[AutoDocking_Laser] Working in phase 2 " << endl;
  //cout << "angulo central: " << angulo_central << endl;
  //cout << "centro patron: " << cp << endl;

  contfases2++;
  if(contfases2=20)
  {
      if(!(((corx<10)&&(corx>-10))||((cory<10)&&(cory>-10))))
	  {
          f2activada = false;	//return to far mode operation
          contfases2=0;
      }
  }

  if((cp<=(angulo_central+3))&&(cp>=(angulo_central-3))){
      m_Comms.Notify("MOTION_CMD_W",0.0);
      //cga:if((dis<distancia_parada)||(volt>12.1)){
      //cga:robotenposicion=true;
      //cga://cout << "distancia parada recta: " << valorafinal << endl;
      //cga:cout << "distancia parada: " << dis << endl;
      //cga:m_Comms.Notify("MOTION_CMD_V",0);
      //cga:}
      //cga:	else{
      if(dis<distancia_parada+4){
          m_Comms.Notify("MOTION_CMD_V",v_lineal_f2_len);
        }
      else{
          m_Comms.Notify("MOTION_CMD_V",v_lineal_f2_rap);
        }
    }
  //cga:}
  else{
      m_Comms.Notify("MOTION_CMD_V",v_lineal_f2_len);
      if(cp>(angulo_central+10)){//giro derecha rapido
          m_Comms.Notify("MOTION_CMD_W",agiro_rad_rap);
        }
      else{
          if(cp>(angulo_central+2)){//giro derecha lento
              m_Comms.Notify("MOTION_CMD_W",agiro_rad_len);
            }
          else{
              if(cp<(angulo_central-10)){ //giro izquierda rapido
                  m_Comms.Notify("MOTION_CMD_W",agiro_rad_rap*(-1));
                }
              else{//giro izquierda lento
                  m_Comms.Notify("MOTION_CMD_W",agiro_rad_len*(-1));
                }
            }
        }
    }
}


//--------------------------------------------------
// MoverRobot -> Pattern has been found! Go for it
//--------------------------------------------------
void CDockingApp::MoverRobot()
{

  float agiro_rad;
  float centropatron=0;
  float poscpatron=0;
  bool terminar=false;
  float tl=10000; //para moverme unos 30cm atras
  float px=0;
  float py=0;
  float pen2=0;
  float aprima=0;
  float bprima=0;
  float disaux=0;
  unsigned int indice=0;
  float di,df,vla,vlb;
  int pi,pf;
  float distancia_a_cp;
  ofstream fich;
  float cortex=0;

  pi=p1ini;
  pf=p3fin;
  di=obs->scan[pi]*100;
  df=obs->scan[pf]*100;
  vla=valorafinal;
  vlb=valorbfinal;

  //cout << "valor a: " << vla << endl;
  //cout << "valor b: " << vlb << endl;


  agiro_rad=DEG2RAD(agiro_mover);

  //hallo el punto medio del patron donde deberia estar justo el frente del hokuyo
  poscpatron=pf-pi+1;
  poscpatron=poscpatron*(longitud_total_patron/2);
  poscpatron=poscpatron/longitud_total_patron;
  poscpatron=poscpatron+pi;
  centropatron=poscpatron*angulo_giro_hok;

  poscpatron=poscpatron+0.5;
  poscpatron=floor(poscpatron);
  indice=(unsigned int) poscpatron;

  //cout << "centropatron: " << centropatron << endl;
  //necesito recta perpendicular que pase por centro patron

  px=posX[indice];
  py=posY[indice];
  pen2=(-1)/vlb;
  bprima=pen2;
  aprima=pen2*px;
  aprima=py-aprima;
  cortex=(aprima*(-1))/bprima;


  //cout << "aprima: " << aprima << endl;
  //distancia entre el centro del patron y el punto corte eje y

  disaux=sqrt(((px-0)*(px-0))+((py-aprima)*(py-aprima)));



  //Check mode of operation (close or far from docking)
  if(f2activada)
  {
      //Phase 2 -> Close to the docking
      distancia_a_cp = (obs->scan[poscpatron])*100;
      mover_fase2(centropatron,distancia_a_cp,aprima,cortex);
  }
  else
  {
	  //Phase 1 -> Far from the docking
      if((aprima>-4)&&(aprima<-2))
	  {   
          f2activada = true; //Activate phase 2
          m_Comms.Notify("MOTION_CMD_V",0.0);
          m_Comms.Notify("MOTION_CMD_W",0.0);
      }
      else
	  {
          if(aprima<=-2){
              if(centropatron>angulo_central){
                  m_Comms.Notify("MOTION_CMD_W",agiro_rad);
                  m_Comms.Notify("MOTION_CMD_V",0.0);
                }
              else{
                  m_Comms.Notify("MOTION_CMD_W",agiro_rad*(-1));
                  m_Comms.Notify("MOTION_CMD_V",0.0);
                }
            }
          else{
              if(centropatron>(angulo_central+90)){//giro izquierda
                  //cout << "centro patron >angulocentral+90" << endl;
                  m_Comms.Notify("MOTION_CMD_W",agiro_rad);
                  m_Comms.Notify("MOTION_CMD_V",0.0);
                }
              else{//else4
                  if(centropatron<(angulo_central-90)){//giro derecha
                      //cout << "centro patron <angulocentral-90" << endl;
                      m_Comms.Notify("MOTION_CMD_W",agiro_rad*(-1));
                      m_Comms.Notify("MOTION_CMD_V",0.0);
                    }else{//else5
                      if(centropatron>angulo_central){
                          if(vlb<0){//giro izquierda
                              //cout << "centro patron >angulocentral vlb<0" << endl;
                              m_Comms.Notify("MOTION_CMD_W",agiro_rad);
                              m_Comms.Notify("MOTION_CMD_V",0.0);
                            }
                          else{//ando o giro derecha
                              if(disaux>(distancia_parada+20)){
                                  //cout << "centro patron >angulocentral vlb>0 disaux>dis" << endl;
                                  m_Comms.Notify("MOTION_CMD_V",v_lineal_mover);
                                  m_Comms.Notify("MOTION_CMD_W",0.0);
                                }
                              else{
                                  //cout << "centro patron >angulocentral vlb>0 disaux<dis" << endl;
                                  m_Comms.Notify("MOTION_CMD_V",0.0);
                                  m_Comms.Notify("MOTION_CMD_W",agiro_rad*(-1));
                                }
                            }
                        }
                      else{//else6
                          if(vlb>0){//giro derecha
                              //cout << "centro patron <angulocentral vlb>0 " << endl;
                              m_Comms.Notify("MOTION_CMD_W",agiro_rad*(-1));
                              m_Comms.Notify("MOTION_CMD_V",0.0);
                            }
                          else{//ando o giro izquierda
                              if(disaux>(distancia_parada+20)){
                                  //cout << "centro patron <angulocentral vlb<0 disaux>dis" << endl;
                                  m_Comms.Notify("MOTION_CMD_V",v_lineal_mover);
                                  m_Comms.Notify("MOTION_CMD_W",0.0);
                                }
                              else{
                                  //cout << "centro patron <angulocentral vlb<0 disaUX<DIS" << endl;
                                  m_Comms.Notify("MOTION_CMD_V",0.0);
                                  m_Comms.Notify("MOTION_CMD_W",agiro_rad);
                                }
                            }
                        }//else6
                    }//else5

                }//else4
            }
        }//else3
    }//else2
}

void CDockingApp::ComprobacionFinal()
{

  int indaux;
  int errores,npos;
  float sumaxy,sumax,sumay,va,vb,vd,sumax2;

  sumaxy=0;
  sumax=0;
  sumay=0;
  npos=0;
  sumax2=0;
  va=0;
  vb=0;
  //primero hayo la ecuacion de la recta de todos los puntos
  for(indaux=p1ini;indaux<=p1fin;indaux++){//for1

      sumaxy=sumaxy+(posX[indaux]*posY[indaux]);
      sumax=sumax+posX[indaux];
      sumay=sumay+posY[indaux];
      npos=npos+1;
      sumax2=sumax2+(posX[indaux]*posX[indaux]);
    }//for1
  for(indaux=p2ini;indaux<=p2fin;indaux++){//for1

      sumaxy=sumaxy+(posX[indaux]*posY[indaux]);
      sumax=sumax+posX[indaux];
      sumay=sumay+posY[indaux];
      npos=npos+1;
      sumax2=sumax2+(posX[indaux]*posX[indaux]);
    }//for1
  for(indaux=p3ini;indaux<=p3fin;indaux++){//for1

      sumaxy=sumaxy+(posX[indaux]*posY[indaux]);
      sumax=sumax+posX[indaux];
      sumay=sumay+posY[indaux];
      npos=npos+1;
      sumax2=sumax2+(posX[indaux]*posX[indaux]);
    }//for1

  vb=((npos*sumaxy)-(sumay*sumax))/((npos*sumax2)-(sumax*sumax));
  va=(sumay-(vb*sumax))/npos;

  //ahora compruebo que "casi" todos los puntos disten menos de 3cm de la recta
  errores=0;
  for(indaux=p1ini;indaux<=p1fin;indaux++){//for1
      vd=(vb*posX[indaux])-posY[indaux]+va;
      if(vd<0){//si es negativo lo pongo positivo por el valor absoluto
          vd=vd*-1;
        }
      vd=vd/(sqrt((vb*vb)+1));
      if(vd>distancia_comprobacion){//if2
          errores++;

        }//if2
    }//for1

  for(indaux=p2ini;indaux<=p2fin;indaux++){//for1
      vd=(vb*posX[indaux])-posY[indaux]+va;
      if(vd<0){//si es negativo lo pongo positivo por el valor absoluto
          vd=vd*-1;
        }
      vd=vd/(sqrt((vb*vb)+1));
      if(vd>distancia_comprobacion){//if2
          errores++;

        }//if2
    }//for1

  for(indaux=p3ini;indaux<=p3fin;indaux++){//for1
      vd=(vb*posX[indaux])-posY[indaux]+va;
      if(vd<0){//si es negativo lo pongo positivo por el valor absoluto
          vd=vd*-1;
        }
      vd=vd/(sqrt((vb*vb)+1));
      if(vd>distancia_comprobacion){//if2
          errores++;

        }//if2
    }//for1

  //compruebo que en los huecos no haya puntos a menos de distancia comprobacion
  /*
        for(indaux=p1fin+1;indaux<p2ini;indaux++){//for1
                vd=(vb*posX[indaux])-posY[indaux]+va;
                if(vd<0){//si es negativo lo pongo positivo por el valor absoluto
                   vd=vd*-1;
                }
                vd=vd/(sqrt((vb*vb)+1));
                if(vd<distancia_comprobacion){//if2
                        errores++;

                }//if2
        }//for1

        for(indaux=p2fin+1;indaux<p3ini;indaux++){//for1
                vd=(vb*posX[indaux])-posY[indaux]+va;
                if(vd<0){//si es negativo lo pongo positivo por el valor absoluto
                   vd=vd*-1;
                }
                vd=vd/(sqrt((vb*vb)+1));
                if(vd<distancia_comprobacion){//if2
                        errores++;

                }//if2
        }//for1
   */

  if(errores<2)
  {
      ok=true;
      valorafinal=va;
      valorbfinal=vb;
  }
  else
	ok=false;
}

//--------------------------------------
// Obtener_puntos_plano : Read Laser
//--------------------------------------
void CDockingApp::Obtener_puntos_plano()
{
	unsigned int ind;
	float dis,px,py;
	float ang_hokuyo = 0;
	float ang_calculo = 0;
	CSerializablePtr obj;
	float cua1,cua2,cua3,apertura;

	//Get laser readings
	CMOOSVariable *PLaser= GetMOOSVar("LASER1");
	mrpt::utils::RawStringToObject(PLaser->GetStringVal(),obj);
	if (obj && IS_CLASS(obj,CObservation2DRangeScan))
		obs = CObservation2DRangeScanPtr(obj);
	else
		printf("[CAutoDocking_Laser] ERROR: LASER1 is not a CObservation2DRangeScan \n");

	apertura=obs->aperture;
	apertura=RAD2DEG(apertura);
	TAM=obs->scan.size();

	angulo_giro_hok=apertura/TAM;
	angulo_central=apertura/2;

	cua1=(apertura/2)-90;
	cua2=apertura/2;
	cua3=(apertura/2)+90;

	//initialize empty vector
	for(ind=0;ind<TAM;ind++)
	{
		posX[ind]=0;
		posY[ind]=0;
	}

	//Check readings from laser
	for(ind=0;ind<obs->scan.size();ind++)
	{
		dis=obs->scan[ind]*100;
		//Ignore out of range values
		if((dis<1)||(dis>200))
		{
			px=0;
			py=0;
		}
		else
		{
			if(ang_hokuyo<=cua1)
			{
				ang_calculo=cua1-ang_hokuyo;
				ang_calculo=DEG2RAD(ang_calculo);
				py=sin(ang_calculo)*dis;
				px=cos(ang_calculo)*dis;
				py=py*(-1); //en esta zona la posicion "y" es negativa
			}
			else
			{
				if((ang_hokuyo>cua1)&&(ang_hokuyo<=cua2))
				{
					ang_calculo=ang_hokuyo-cua1;
					ang_calculo=DEG2RAD(ang_calculo);
					py=sin(ang_calculo)*dis;
					px=cos(ang_calculo)*dis;
				}
				else
				{
					if((ang_hokuyo>cua2)&&(ang_hokuyo<=cua3))
					{
						ang_calculo=ang_hokuyo-cua2;
						ang_calculo=DEG2RAD(ang_calculo);
						py=cos(ang_calculo)*dis;
						px=sin(ang_calculo)*dis;
						px=px*(-1);//en esta zona la x es negativa
					}
					else
					{
						ang_calculo=ang_hokuyo-cua3;
						ang_calculo=DEG2RAD(ang_calculo);
						py=sin(ang_calculo)*dis;
						px=cos(ang_calculo)*dis;
						px=px*(-1);//en esta zona la x es negativa
						py=py*(-1);//en esta zona la x es negativa
					}
				}
			}
		}
		posX[ind]=px;
		posY[ind]=py;
		ang_hokuyo=ang_hokuyo+angulo_giro_hok;
	}//end for
}


void CDockingApp::BuscaPatron(int ind,int &pfin,float &longitud, float &valora, float &valorb){


  unsigned int ind2,indaux;
  int npos;
  double sumaxy,sumax,sumay,va,vb,vd,sumax2;
  bool salir=false;

  ind2=ind+1;
  sumaxy=0;
  sumax=0;
  sumay=0;
  sumax2=0;
  npos=0;

  if((posX[ind2]==0)&&(posY[ind2]==0)){
      pfin=ind2;
      longitud=1;
      valora=1;
      valorb=1;
    }
  else{
      for(indaux=ind;indaux<=ind2;indaux++){//for1
          sumaxy=sumaxy+(posX[indaux]*posY[indaux]);
          sumax=sumax+posX[indaux];
          sumay=sumay+posY[indaux];
          npos=npos+1;
          sumax2=sumax2+(posX[indaux]*posX[indaux]);

        }//for1
      vb=((npos*sumaxy)-(sumay*sumax))/((npos*sumax2)-(sumax*sumax));
      va=(sumay-(vb*sumax))/npos;
      ind2++;
      while((ind2<TAM-2)&&(!salir)){
          vd=(vb*posX[ind2])-posY[ind2]+va;
          if(vd<0){
              vd=vd*-1;
            }
          vd=vd/(sqrt((vb*vb)+1));
          if(vd>distancia_corte){
              salir=true;
            }
          else{
              sumaxy=0;
              sumax=0;
              sumay=0;
              sumax2=0;
              npos=0;
              for(indaux=ind;indaux<=ind2;indaux++){//for1
                  sumaxy=sumaxy+(posX[indaux]*posY[indaux]);
                  sumax=sumax+posX[indaux];
                  sumay=sumay+posY[indaux];
                  npos=npos+1;
                  sumax2=sumax2+(posX[indaux]*posX[indaux]);
                }//for1
              vb=((npos*sumaxy)-(sumay*sumax))/((npos*sumax2)-(sumax*sumax));
              va=(sumay-(vb*sumax))/npos;
              ind2++;
            }
        }

      ind2=ind2-1;
      pfin=ind2;
      longitud=sqrt(((posX[ind2]-posX[ind])*(posX[ind2]- posX[ind]))
                    +((posY[ind2]- posY[ind])*(posY[ind2]-posY[ind])));
      valora=va;
      valorb=vb;
    }

}

bool CDockingApp::OnCommandMsg( CMOOSMsg Msg )
{

  return true;
}


void CDockingApp::Buscar_Patron_Completo(unsigned int pinicio,unsigned int pfin){

  unsigned int ind=0;
  unsigned int ind2=0;
  unsigned int ind3=0;
  int indaux=0;
  float lon1,lon2,lon3,lonaux1,lonaux2,vr1a,vr1b,vr2a,vr2b,vr3a,vr3b;


  int puntosp1=0;
  int puntosp2=0;
  int puntosp3=0;

  p1ini=0;
  p1fin=0;
  p2ini=0;
  p2fin=0;
  p3ini=0;
  p3fin=0;

  ind2=0;
  lon1=0;
  lon2=0;
  lon3=0;
  lonaux1=0;
  lonaux2=0;
  vr1a=0;
  vr2a=0;
  vr1b=0;
  vr2b=0;
  vr3a=0;
  vr3b=0;

  ok=false;
  ind=pinicio;
  while((ind<(pfin-10))&&(!ok)) //while1
  {
      if((posX[ind]==0)&&(posY[ind]==0))
	  {//if1
		//no hace nada, hueco
      }//if1
      else
	  {//else1
          BuscaPatron(ind,indaux,lon1,vr1a,vr1b);//busca primer subpatron
          if((lon1>lon_sp1i)&&(lon1<lon_sp1s)){//if1
              p1ini=ind;
              p1fin=indaux;
              ind2=indaux+4;
              while((ind2<(pfin-1))&&(!ok)){//while2
                  if((posX[ind2]==0)&&(posY[ind2]==0)){//if2
                      //no hace nada, hueco

                    }//if2
                  else{//else2
                      BuscaPatron(ind2,indaux,lon2,vr2a,vr2b);//busca el segundo subpatron
                      if((lon2>lon_sp2i)&&(lon2<lon_sp2s)){//if2
                          //si la distancia del sugundo subpatron es correcta compruebo que la
                          //distancia entre los dos subpatrones tambien es correcta

                          lonaux1=sqrt(((posX[ind2]-posX[p1fin])*(posX[ind2]-posX[p1fin]))+((posY[ind2]-posY[p1fin])*(posY[ind2]-posY[p1fin])));
                          if((lonaux1<lon_aux1s)&&(lonaux1>lon_aux1i)){//if3
                              //la distancia entre huecos tambien es valida, podemos continuar
                              //buscando el tercer subpatron
                              p2ini=ind2;
                              p2fin=indaux;
                              ind3=indaux+4;
                              while((ind3<(pfin-1))&&(!ok)){//while3
                                  if((posX[ind3]==0)&&(posY[ind3]==0)){//if4
                                      //no hace nada, hueco

                                    }//if4
                                  else{
                                      BuscaPatron(ind3,indaux,lon3,vr3a,vr3b);//buscamos subpatron3
                                      if((lon3>lon_sp3i)&&(lon3<lon_sp3s)){//if5
                                          //es correcto, vemos si la distancia con respecto al subpatron2 es correcta


                                          lonaux2=sqrt(((posX[ind3]-posX[p2fin])*(posX[ind3]-posX[p2fin]))+((posY[ind3]-posY[p2fin])*(posY[ind3]-posY[p2fin])));
                                          if((lonaux2>lon_aux1i)&&(lonaux2<lon_aux1s)){
                                              //comprobamos si todos los subpatrones proporcionan una linea recta
                                              p3ini=ind3;
                                              p3fin=indaux;
                                              ComprobacionFinal();
                                              puntosp1=p1fin-p1ini+1;
                                              puntosp2=p2fin-p2ini+1;
                                              puntosp3=p3fin-p3ini+1;


                                              if(ok){
                                                  //comprobar_pendientes_rectas(vr1b,vr2b,vr3b);
                                                  if(patronverdadero)
												  {
                                                      comprobar_patron_verdadero(p1ini,p3fin);
                                                  }
                                              }
                                          }

                                        }//if5
                                    }
                                  ind3++;
                                }//while3
                            }//if3
                        }//if2
                    }//else2
                  ind2++;
                }//while2
            }//if1

        }//else1
      ind++;

    };//while1

}


bool CDockingApp::Iterate()
{
	//Initialization  
	float angf1 = 0;
	float angf2 = 0;
	float dispatron = 0;  
	ok = false;
	p1ini = 0;
	p1fin = 0;
	p2ini = 0;
	p2fin = 0;
	p3ini = 0;
	p3fin = 0;
	valorafinal = 0;
	valorbfinal = 0;
  
	
	//Is module active?
	if( go_docking )
	{
		// 1- Get Laser readings
		//----------------------
		Obtener_puntos_plano();

		// 2- Search pattern
		//---------------------
		// Was the pattern found in the previous scan? 
		//If so, reduce the search range to improve computation speed
		if(ok_anterior)
		{
			if((punto_inicio>100)&&(punto_fin<(TAM-100)))			
				Buscar_Patron_Completo(punto_inicio-100,punto_fin+100);
			else
			{
				if(punto_inicio>100)
					Buscar_Patron_Completo(punto_inicio-100,TAM);				
				else
					Buscar_Patron_Completo(0,punto_fin+100);
			}
		}
		else
		{ //No luck- Search the pattern within all the Laser scan
			Buscar_Patron_Completo(0,TAM);
		}

		
		// 3- Check if pattern has been found
		//-------------------------------------
		if(ok)
		{
			encontrados++;					//increase the number of consecutive matches
			contador_no_encontrados = 0.0;	//rested non-matches

			if(encontrados==1.0)			//first time the pattern has been found!
			{
				//Save the angles and distance of the match for future matches.
				angf1 = p1ini*angulo_giro_hok;
				angf2 = p3fin*angulo_giro_hok;
				dispatron = obs->scan[p1ini]*100;
				angulovi = angf1;
				angulovf = angf2;
				disv = dispatron;
				printf("[AutoDocking_Laser] FIRST TIME - pattern found at angles (%.2f %.2f) at %.2f cm\n",angf1,angf2,dispatron);
			}
			else
			{
				//Check that the found pattern complies with previous matches!
				comprobar_patron_verdadero(p1ini,p3fin);
				if(ok)
				{
					printf("[AutoDocking_Laser] Pattern found at angles (%.2f %.2f) at %.2f cm\n",angulovi,angulovf,disv);
					contador_no_encontrados = 0.0;
					if (encontrados == 4.0)
						patronverdadero = true;		//we can start moving the robot!
				}
				else
				{					
					printf("[AutoDocking_Laser] ERROR: There was a problem with the found pattern. Restarting!\n");
					encontrados = 0.0;			//reset consecutive matches
					contador_no_encontrados++;
				}
			}
			
			// 4- Move robot
			//---------------
			if(patronverdadero)		//only after 4 consecutive matches, we declare the pattern as the correct one (patronverdadero)
				MoverRobot();
			
			// 5- Debug window?
			//-----------------
			if (debug) dibujar_puntos(p1ini,p1fin,p2ini,p2fin,p3ini,p3fin);

			//angf1 = p1ini*angulo_giro_hok;
			//angf2 = p3fin*angulo_giro_hok;			
			//dispatron = obs->scan[p1ini]*100;
			//if(encontrados==4)
			//{
			//	patronverdadero = true;
			//	angulovi = angf1;
			//	angulovf = angf2;
			//	disv = dispatron;
			//}
			//contador_no_encontrados=0;	//rested non-matches
			//printf("PATRON ENCONTRADO ENTRE LOS ANGULOS %f %f A %f CM\n",angf1,angf2,dispatron);
		}		
		else
		{	//Pattern NOT-found!
			encontrados = 0;			//reset consecutive matches
			contador_no_encontrados++;	//increase consecutive non-matches
			printf("[AutoDocking_Laser]: ERROR [%.1f] Pattern not found within the lasser scan. Retrying.\n", contador_no_encontrados);
			m_Comms.Notify("MOTION_CMD_V",0.0);
			m_Comms.Notify("MOTION_CMD_W",0.0);
			patronverdadero = false;
		}



		//Problems finding the pattern?
		//-------------------------------		
		if(contador_no_encontrados>20)
		{
			//Make the robot spin			
			m_Comms.Notify("MOTION_CMD_W",DEG2RAD(agiro_mover));
			m_Comms.Notify("MOTION_CMD_V",0.0);
			patronverdadero = false;
		}
		else if(contador_no_encontrados>2000)
		{
			cout<< "****NO SE ENCUENTRA EL PATRON POR NINGUN SITIO. Cancelando AutoDocking request****" << endl;
			m_Comms.Notify("PARKING",0.0);
		}	
	}// end if(go_docking)

	return true;
}


bool CDockingApp::OnConnectToServer()
{
  DoRegistrations();
  return true;
}


bool CDockingApp::DoRegistrations()
{
  //! @moos_subscribe LASER1
  //! @moos_var LASER1 the scan lassers of a laser rangefinder on the front of the robot
  AddMOOSVariable("LASER1","LASER1","LASER1",0);
 
  //! @moos_subscribe MOTION_CMD_V 
  //! @moos_var MOTION_CMD_V The current robot linear velocity  
  AddMOOSVariable( "MOTION_CMD_V", "MOTION_CMD_V", "MOTION_CMD_V", 0 );

  //! @moos_subscribe MOTION_CMD_W
  //! @moos_var MOTION_CMD_W The current robot angular velocity  
  AddMOOSVariable( "MOTION_CMD_W", "MOTION_CMD_W", "MOTION_CMD_W", 0 );
  
  //! @moos_subscribe Is_charging
  //! @moos_var Is_charging Variable that indicates when the robot is charging its battery
  AddMOOSVariable( "Is_Charging", "Is_Charging", "Is_Charging", 0 );
  
  //! @moos_subscribe PARKING
  //! @moos_var PARKING Variable that indicates when the robot should start the Docking process.
  AddMOOSVariable( "PARKING", "PARKING", "PARKING", 0 );

  //! @moos_subscribe SHUTDOWN
  AddMOOSVariable( "SHUTDOWN", "SHUTDOWN", "SHUTDOWN", 0 );


  RegisterMOOSVariables();
  return true;
}



bool CDockingApp::OnNewMail(MOOSMSG_LIST &NewMail)
{
  //cga:Standard for OpenMore modules
  //cga:This function will close the module when the "SHUTDOWN" message is received

	std::string cad;	
	for (MOOSMSG_LIST::const_iterator it=NewMail.begin();it!=NewMail.end();++it)
	{
		const CMOOSMsg &m = *it;

		if( MOOSStrCmp(m.GetKey(),"Is_Charging") )
		{
			//Update charging status
			charge = m.GetDouble();

			//Did we success in the Autodking?
			if( (go_docking)&&(charge==1.0) )
			{
				printf("[CAutoDocking_Laser] SUCCESS! Docking completed!\n");
				go_docking = false;

				//Stop robot
				m_Comms.Notify("MOTION_CMD_V",0.0);
				m_Comms.Notify("MOTION_CMD_W",0.0);
				//Cancel Autodocking request
				m_Comms.Notify("PARKING",0.0);
				
				//-------------------------
				// Set initial state
				//-------------------------
				f2activada = false;				//Init the sistem as Far from docking!  
				contador_no_encontrados = 0;
				encontrados = 0;
				patronverdadero = false;
				angulovi = 0;
				angulovf = 0;
				disv = 0;  
				ok_anterior = false;
				punto_inicio = 0;
				punto_fin = 0;
				contfases2 = 0;				
			}
		}
		
		if( MOOSStrCmp(m.GetKey(),"PARKING") )
		{
			//Update PARKING status
			park = m.GetDouble();
			
			if (park==1.0)
			{
				if(charge==0.0)
				{
					printf("[CAutoDocking_Laser] Starting Autodocking process...\n");
					go_docking = true;
				}
				else
				{
					printf("[CAutoDocking_Laser] Requested Autodocking process, but ALREADY CHARGING! Disabling request.\n");
					park = 0.0;
					//m_Comms.Notify("PARKING",0.0);
				}
			}
			else
			{
				printf("[CAutoDocking_Laser] Autodocking request CANCELED.\n");
				go_docking = false;
			}
		}

		if( (MOOSStrCmp(m.GetKey(),"SHUTDOWN")) && (MOOSStrCmp(m.GetString(),"true")) )
		{
			// Disconnect comms:
			MOOSTrace("Closing Module \n");
			this->RequestQuit();
		}

	}

  UpdateMOOSVariables(NewMail);
  return true;
}
