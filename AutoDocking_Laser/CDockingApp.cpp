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
  *  The dimensions of the pattern can be set via configuration params, as well as the movement speeds.
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
	// PATTERN DIMENSIONS
	//--------------------
	// it is composed of three sticks of known dimensions and separation
	//  PATTERN (front view) = __|__||__|||___
	//  Sticks                   3   2   1 
	//--------------------------------------------------------------------
	//! @moos_param  longitud_total_patron The total length of the pattern (cm), from first to third stick
	m_MissionReader.GetConfigurationParam("longitud_total_patron",longitud_total_patron);

	//! @moos_param  laserVar The name of the OpenMora variable from which to get the scans.
	laserVar = m_ini.read_string("","laserVar","LASER1",false);

	//! @moos_param   distancia_corte  Max depth difference (cm) between points to consider them as part of the same line (subpattern)
	m_MissionReader.GetConfigurationParam("distancia_corte",distancia_corte);
  
	//! @moos_param  distancia_comprobacion  //max error (cm) to consider a point as part of the line containing the three sub-patterns
	m_MissionReader.GetConfigurationParam("distancia_comprobacion",distancia_comprobacion);

	//! @moos_param  distancia_parada  Distance to the pattern (cm) to slow down the robot movement
	m_MissionReader.GetConfigurationParam("distancia_parada",distancia_parada);

	// Sticks
		//! @moos_param  lon_sp1i  Min width of the first stick
		m_MissionReader.GetConfigurationParam("lon_sp1i",lon_sp1i);
		//! @moos_param  lon_sp1s  Max width of the first stick
		m_MissionReader.GetConfigurationParam("lon_sp1s",lon_sp1s);
		//! @moos_param  lon_sp2i  Min width of the second stick
		m_MissionReader.GetConfigurationParam("lon_sp2i",lon_sp2i);
		//! @moos_param  lon_sp2s  Max width of the second stick
		m_MissionReader.GetConfigurationParam("lon_sp2s",lon_sp2s);
		//! @moos_param  lon_sp3i  Min width of the thrid stick
		m_MissionReader.GetConfigurationParam("lon_sp3i",lon_sp3i);
		//! @moos_param  lon_sp3s  Max width of the third stick
		m_MissionReader.GetConfigurationParam("lon_sp3s",lon_sp3s);

	// Separation between sticks
		//! @moos_param  lon_aux1i  min separation between patterns 1 and 2
		m_MissionReader.GetConfigurationParam("lon_aux1i",lon_aux1i);
		//! @moos_param  lon_aux1s  max separation between patterns 1 and 2
		m_MissionReader.GetConfigurationParam("lon_aux1s",lon_aux1s);
		//! @moos_param  lon_aux2i  min separation between patterns 2 and 3
		m_MissionReader.GetConfigurationParam("lon_aux2i",lon_aux2i);
		//! @moos_param  lon_aux2s  max separation between patterns 2 and 3
		m_MissionReader.GetConfigurationParam("lon_aux2s",lon_aux2s);

	// ROBOT SPEEDS
		//! @moos_param  v_lineal_mover  Linear speed (m/s) to approach the pattern
		m_MissionReader.GetConfigurationParam("v_lineal_mover",v_lineal_mover);
		//! @moos_param  agiro_mover  Angular speed (deg/s) to approach the pattern
		m_MissionReader.GetConfigurationParam("agiro_mover",agiro_mover);

		// PHASE 2 (Close to the pattern)
		//! @moos_param  v_lineal_f2_rap  Liner speed (m/s) to approach the pattern in phase 2
		m_MissionReader.GetConfigurationParam("v_lineal_f2_rap",v_lineal_f2_rap);
		//! @moos_param  v_lineal_f2_len  Linear speed (m/s) to approaach the pattern when it's too close (<distancia_parada)
		m_MissionReader.GetConfigurationParam("v_lineal_f2_len",v_lineal_f2_len);
		//! @moos_param  agiro_f2_rap  Angular speed (deg/s) to approach the pattern in phase 2
		m_MissionReader.GetConfigurationParam("agiro_f2_rap",agiro_f2_rap);
		//! @moos_param  agiro_f2_len  Angular speed (deg/s) to approach the pattern when it's too close (<distancia_parada)
		m_MissionReader.GetConfigurationParam("agiro_f2_len",agiro_f2_len);
  
  //cga:vble debug from the configuration file. If it is true, the wxwindow is displayed
  //! @moos_param  debug If it is true, a wxwindow is displayed with the location of the pattern
  m_MissionReader.GetConfigurationParam("debug",debug);
  if (debug)
    win=new CDisplayWindowPlots("Docking debug window");


  //-------------------------
  // Set initial state
  //-------------------------
  go_docking = false;
  charge = 1.0;								//is charging
  found_in_previous_iteration = false;		//Was the pattern found in the previous scan?

  f2activada = false;				//Init the sistem in phase1, far from docking!
  contador_no_encontrados = 0;		//number of non-matches
  encontrados = 0;					//number of matches
  patronverdadero = false;			//only after 4 consecutive matches, we declare the pattern as found
  angulovi = 0;
  angulovf = 0;
  disv = 0;  
  
  punto_inicio = 0;
  punto_fin = 0;
  contfases2 = 0;

  //Start doing nothing!!
  

  return DoRegistrations();
}


//---------------------------------------------------------------------
// dibujar_puntos: Display sub-pattern detected points in the "debug" window
//---------------------------------------------------------------------
void CDockingApp::dibujar_puntos(int p1i,int p1f,int p2i,int p2f,int p3i,int p3f)
{
	mrpt::slam::CSimplePointsMap TheMap,TheMap1,TheMap2,TheMap3;
	int i;
	mrpt::math::CVectorFloat xs,ys,zs,xs1,ys1,zs1;

	//1. All the scanned points!
	for(i=0;i<TAM;i++)
		TheMap.insertPoint(posX[i]/100,posY[i]/100,0,0.9,0.9,0.9);
	// Display all in black
	TheMap.getAllPoints(xs,ys,zs);
	win->plot(xs,ys,".k3","scan");

	// Insert points of sub-pattern1 (red)
	for( i=p1i;i<=p1f;i++ )
		TheMap1.insertPoint( posX[i]/100,posY[i]/100,0.0,0.9,0.0,0.0 );	
	TheMap1.getAllPoints(xs,ys,zs);
	win->plot(xs,ys,".r3","pattern1");

	// Insert points of sub-pattern2 (green)
	for( i=p2i;i<=p2f;i++ )
		TheMap2.insertPoint( posX[i]/100,posY[i]/100,0.0,0.0,0.9,0.0 );	
	TheMap2.getAllPoints(xs,ys,zs);
	win->plot(xs,ys,".g3","pattern2");

	// Insert points of sub-pattern3 (blue)
	for(i=p3i;i<=p3f;i++)
		TheMap3.insertPoint( posX[i]/100,posY[i]/100,0.0,0.0,0.0,0.9 );	
	TheMap3.getAllPoints(xs,ys,zs);
	win->plot(xs,ys,".b3","pattern3");	
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

	ai = pi*angulo_giro_hok;
	af = pf*angulo_giro_hok;
	d = obs->scan[pi]*100;
  
	pattern_found = false;

	if( (angulovi<(ai+aux))&&(angulovi>(ai-aux)) )
	{//punto inicio correcto
		if( (angulovf<(af+aux))&&(angulovf>(af-aux)) )
		{ // punto fin correcto
			if( (disv<(d+aux))&&(disv>(d-aux)) )
			{ //distancia correcta
				pattern_found = true;			//pattern found!!
				angulovi = ai;
				angulovf = af;
				disv = d;
			}
		}
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

  agiro_rad_rap = DEG2RAD(agiro_f2_rap);
  agiro_rad_len = DEG2RAD(agiro_f2_len);

  cout << "[AutoDocking_Laser] *************  Working in PHASE 2  ************* " << endl;
  //cout << "angulo central: " << angulo_central << endl;
  //cout << "centro patron: " << cp << endl;

  contfases2++;
  if(contfases2 = 20)
  {
      if( !(((corx<10)&&(corx>-10)) || ((cory<10)&&(cory>-10))) )
	  {
          f2activada = false;	//return to far mode operation
          contfases2=0;
      }
  }

  if( (cp<=(angulo_central+3))&&(cp>=(angulo_central-3)) )
  {
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


  agiro_rad = DEG2RAD(agiro_mover);

  //hallo el punto medio del patron donde deberia estar justo el frente del hokuyo
  poscpatron = pf-pi+1;
  poscpatron = poscpatron*(longitud_total_patron/2);
  poscpatron = poscpatron/longitud_total_patron;
  poscpatron = poscpatron+pi;
  centropatron = poscpatron*angulo_giro_hok;

  poscpatron = poscpatron+0.5;
  poscpatron = floor(poscpatron);
  indice = (unsigned int) poscpatron;

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


//--------------------------------------------------------------------------
// Check that the three sub-patters found belong to the same line in space
//--------------------------------------------------------------------------
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

	//Line equation of all points from subpattern 1, 2 and 3
	for(indaux=p1ini;indaux<=p1fin;indaux++)
	{
		sumaxy=sumaxy+(posX[indaux]*posY[indaux]);
		sumax=sumax+posX[indaux];
		sumay=sumay+posY[indaux];
		npos=npos+1;
		sumax2=sumax2+(posX[indaux]*posX[indaux]);
	}
	for(indaux=p2ini;indaux<=p2fin;indaux++)
	{
		sumaxy=sumaxy+(posX[indaux]*posY[indaux]);
		sumax=sumax+posX[indaux];
		sumay=sumay+posY[indaux];
		npos=npos+1;
		sumax2=sumax2+(posX[indaux]*posX[indaux]);
	}
	for(indaux=p3ini;indaux<=p3fin;indaux++)
	{
		sumaxy=sumaxy+(posX[indaux]*posY[indaux]);
		sumax=sumax+posX[indaux];
		sumay=sumay+posY[indaux];
		npos=npos+1;
		sumax2=sumax2+(posX[indaux]*posX[indaux]);
	}

	vb=((npos*sumaxy)-(sumay*sumax))/((npos*sumax2)-(sumax*sumax));
	va=(sumay-(vb*sumax))/npos;

	
	//check that "almost" every point fall within this line. Error < "distancia_comprobacion"	
	errores = 0;
	for( indaux=p1ini;indaux<=p1fin;indaux++ )
	{
		vd = (vb*posX[indaux])-posY[indaux]+va;
		if( vd<0 )	//si es negativo lo pongo positivo por el valor absoluto		
			vd=vd*-1;		
		vd = vd/(sqrt((vb*vb)+1));
		if( vd>distancia_comprobacion )
			errores++;
	}
	for( indaux=p2ini;indaux<=p2fin;indaux++ )
	{
		vd = (vb*posX[indaux])-posY[indaux]+va;
		if(vd<0) 
			vd=vd*-1;		
		vd = vd/(sqrt((vb*vb)+1));
		if( vd>distancia_comprobacion )
			errores++;
	}
	for( indaux=p3ini;indaux<=p3fin;indaux++ )
	{
		vd = (vb*posX[indaux])-posY[indaux]+va;
		if(vd<0)
			vd=vd*-1;
		vd = vd/(sqrt((vb*vb)+1));
		if( vd>distancia_comprobacion )
			errores++;
	}

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

	// If less than 2 errors, declare the pattern as found!
	if( errores<2 )
	{		
		pattern_found = true;
		valorafinal = va;
		valorbfinal = vb;
	}
	else
	{
		pattern_found = false;
		printf("[AutoDockingLaser]: ERROR Sub-patterns do not fall in a line, %u errors\n", errores);
	}
}

//-----------------------------------------------------------------------
// Obtener_puntos_plano : Read Laser Scan
// The Laser Scan provides an array with distances(m) to the obstacles
// This function transform the array of distances, to an array of coordinates (x,y)
//-----------------------------------------------------------------------
void CDockingApp::Obtener_puntos_plano()
{
	unsigned int ind;
	float dis,px,py;
	float ang_hokuyo = 0;		//Angle(deg) indicating the angle of one point of the scan (used as iterator)
	float ang_calculo = 0;
	CSerializablePtr obj;
	float cua1,cua2,cua3,apertura;

	// Get laser readings
	//--------------------
	CMOOSVariable *PLaser= GetMOOSVar(laserVar);
	if (PLaser)
	{
		mrpt::utils::RawStringToObject(PLaser->GetStringVal(),obj);
		if (obj && IS_CLASS(obj,CObservation2DRangeScan))
			obs = CObservation2DRangeScanPtr(obj);
		else
			printf( "[CAutoDocking_Laser] ERROR: %s is not a CObservation2DRangeScan \n",laserVar.c_str() );
	}
	else
	{
		printf( "[CAutoDocking_Laser] ERROR: Variable %s not found!\n",laserVar.c_str() );
		return;
	}

	// Read parameters
	//------------------
	apertura = obs->aperture;				//laser aperture in rad
	apertura = RAD2DEG(apertura);			//laser aperture in deg, typically 180º
	TAM = obs->scan.size();					//Number of points in the scan
	angulo_giro_hok = apertura/(TAM-1);		//angular resolution = deg/point
	mrpt::poses::CPose3D laser_pose;
	obs->getSensorPose(laser_pose);

	// Is laser rolled?
	if( (abs(laser_pose.roll()) - PI) < 0.01 )
	{		
		// Scans ar not from right to left!, but left to right!
		std::reverse(obs->scan.begin(),obs->scan.end());
	}



	//printf("[TEST]: Aperture=%.3f, size=%u, ang_res=%.3f\n",apertura,TAM,angulo_giro_hok);

	//Define angles of the working regions (cuadrants)    
	//							 cua2
	//                     cua3 __|__ cua1
	//                            | 
	angulo_central = apertura/2;		//the center of the scan
	cua1 = (apertura/2)-90;
	cua2 = apertura/2;
	cua3 = (apertura/2)+90;

	//reset vector of detected obstacles (x,y)
	for(ind=0;ind<TAM;ind++)
	{
		posX[ind] = 0.0;
		posY[ind] = 0.0;
	}


	// Transform scan laser to X,Y
	//----------------------------
	for( ind=0;ind<TAM;ind++ )
	{
		dis = obs->scan[ind]*100;	//distance (cm) to point [ind]
		//Ignore out of range values, set to origin
		if( (dis<1.0)||(dis>200.0) )
		{
			px = 0.0;
			py = 0.0;
		}
		else
		{
			//Get (x,y) according to working region (cua1,cua2,cua3)
			if( ang_hokuyo<=cua1 )
			{
				ang_calculo = cua1-ang_hokuyo;
				ang_calculo = DEG2RAD(ang_calculo);
				py = sin(ang_calculo)*dis;
				px = cos(ang_calculo)*dis;
				py = py*(-1); //en esta zona la posicion "y" es negativa
			}
			else if( (ang_hokuyo>cua1)&&(ang_hokuyo<=cua2) )
			{
				ang_calculo = ang_hokuyo-cua1;
				ang_calculo = DEG2RAD(ang_calculo);
				py = sin(ang_calculo)*dis;
				px = cos(ang_calculo)*dis;
			}
			else if( (ang_hokuyo>cua2)&&(ang_hokuyo<=cua3) )
			{
				ang_calculo = ang_hokuyo-cua2;
				ang_calculo = DEG2RAD(ang_calculo);
				py = cos(ang_calculo)*dis;
				px = sin(ang_calculo)*dis;
				px = px*(-1);//en esta zona la x es negativa
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

		//Save location (x,y) of the current scan point
		posX[ind] = px;
		posY[ind] = py;

		//increase the counter of the angle
		ang_hokuyo = ang_hokuyo + angulo_giro_hok;
	}//end for
}


//---------------------------------------------------------------------------------------------
// BuscaPatron: Search for a sub-pattern (plannar and continuous surface = line) in the laser scans
//				ind = point to start the search
//				pfin = point where the search ended
//				longitud = length of the sub-pattern found (planar continuous surface)
//				valor_a = 
//				valor_b = Slope of the line containing the sub-pattern
//---------------------------------------------------------------------------------------------
void CDockingApp::BuscaPatron(int ind,int &pfin,float &longitud, float &valora, float &valorb)
{
	unsigned int ind2,indaux;
	int npos;
	double sumaxy,sumax,sumay,va,vb,vd,sumax2;
	bool salir = false;

	ind2 = ind+1;	//the next point in the scan laser	
	sumax = 0;		//Aggregate value of X components
	sumay = 0;		//Aggregate value of Y components
	sumaxy = 0;		//Aggregate value of X*Y components
	sumax2 = 0;		//Aggregate value of X*X components
	npos = 0;

	// We need at least 2 consecutive points to check the pattern (a line)
	// Is the next point in the search an invalid point?
	if( (posX[ind2]==0) && (posY[ind2]==0) )
	{
		//invalid point
		pfin = ind2;		//search ended at point
		longitud = 1;		//distance of max line (planar surface) found
		valora = 1;			//Slope found
		valorb = 1;			
	}
	else
	{
		//next point is a valid point, lets check!
		for( indaux=ind;indaux<=ind2;indaux++ )
		{
			sumaxy += (posX[indaux]*posY[indaux]);
			sumax += posX[indaux];
			sumay += posY[indaux];
			npos += 1;
			sumax2 += (posX[indaux]*posX[indaux]);
		}

		//Some calculus here for estimating the line slope
		vb = ((npos*sumaxy)-(sumay*sumax))/((npos*sumax2)-(sumax*sumax));
		va = (sumay-(vb*sumax))/npos;
		
		ind2++;
		while( (ind2<TAM-2)&&(!salir) )
		{
			vd = (vb*posX[ind2])-posY[ind2]+va;
			if( vd<0 )			
				vd = vd*-1;
			
			vd = vd/(sqrt((vb*vb)+1));
			if( vd>distancia_corte )
				salir = true;		//Stop searching				
			else
			{
				sumaxy = 0;
				sumax = 0;
				sumay = 0;
				sumax2 = 0;
				npos = 0;
				for( indaux=ind;indaux<=ind2;indaux++ )
				{
					sumaxy=sumaxy+(posX[indaux]*posY[indaux]);
					sumax=sumax+posX[indaux];
					sumay=sumay+posY[indaux];
					npos=npos+1;
					sumax2=sumax2+(posX[indaux]*posX[indaux]);
				}

				vb = ((npos*sumaxy)-(sumay*sumax))/((npos*sumax2)-(sumax*sumax));
				va = (sumay-(vb*sumax))/npos;
				ind2++;
			}
		}//end-while

		// Return the parameters of the surface found
		ind2 = ind2-1;
		pfin = ind2;
		longitud = sqrt( ((posX[ind2]-posX[ind])*(posX[ind2]- posX[ind]))
					+ ((posY[ind2]- posY[ind])*(posY[ind2]-posY[ind])) );
		valora = va;
		valorb = vb;
	}//end if next-point is valid
}



bool CDockingApp::OnCommandMsg( CMOOSMsg Msg )
{
  return true;
}



//----------------------------------------------------------------------------------
// Search for the complete pattern in the scan lasser, from "init_point" to "end_point"
//----------------------------------------------------------------------------------
void CDockingApp::Buscar_Patron_Completo(unsigned int pinicio,unsigned int pfin)
{
	pattern_found = false;
	unsigned int ind = pinicio;
	unsigned int ind2 = 0;
	unsigned int ind3 = 0;
	
	int indaux = 0;
	float lon1,lon2,lon3,lonaux1,lonaux2,vr1a,vr1b,vr2a,vr2b,vr3a,vr3b;
	int puntosp1 = 0;
	int puntosp2 = 0;
	int puntosp3 = 0;

	//Reset position of the different pattern's Sticks in the scan (from previous searches)
	p1ini = 0;
	p1fin = 0;
	p2ini = 0;
	p2fin = 0;
	p3ini = 0;
	p3fin = 0;
	// Reset dimensions fonud of the sticks in the pattern, and their separation (from previous searches)
	lon1 = 0;
	lon2 = 0;
	lon3 = 0;
	lonaux1 = 0;
	lonaux2 = 0;

	// Reset slopes of the lines (sub-patterns) found
	vr1a=0;
	vr2a=0;
	vr1b=0;
	vr2b=0;
	vr3a=0;
	vr3b=0;

	// Start searching
	while( (ind<(pfin-10)) && (!pattern_found) )
	{
		//Discard points that are at the origin (0,0) ->comming from out of range scans (see Obtener_puntos_plano)
		if( (posX[ind]!=0.0) || (posY[ind]!=0.0) )
		{
			//Search for a sub-pattern (planar surface)
			BuscaPatron(ind,indaux,lon1,vr1a,vr1b);

			//Are dimensions correct for Sub-pattern 1?
			if( (lon1>lon_sp1i)&&(lon1<lon_sp1s) )
			{
				//YES, first sub-pattern found!
				p1ini = ind;		//start point of first sub-pattern
				p1fin = indaux;		//end point of first sub-pattern
				//if (debug) printf("[AutoDockingLaser]: First pattern found at points %u-%u, with long=%0.3fcm\n",p1ini,p1fin,lon1);

				//Keep searching
				ind2 = indaux+4;
				while( (ind2<(pfin-1)) && (!pattern_found) )
				{
					if( (posX[ind2]!=0.0) || (posY[ind2]!=0.0) )
					{
						//Search for another sub-pattern (planar surface)
						BuscaPatron(ind2,indaux,lon2,vr2a,vr2b);
						
						//Are dimensions correct for sub-pattern 2?
						if( (lon2>lon_sp2i)&&(lon2<lon_sp2s) )
						{
							//Check that distance between the two first sub-patterns is correct
							lonaux1 = sqrt(((posX[ind2]-posX[p1fin])*(posX[ind2]-posX[p1fin]))+((posY[ind2]-posY[p1fin])*(posY[ind2]-posY[p1fin])));
							if( (lonaux1<lon_aux1s)&&(lonaux1>lon_aux1i) )
							{	
								// YES, second pattern found!
								p2ini = ind2;
								p2fin = indaux;
								//if (debug) printf("[AutoDockingLaser]: Second pattern found at points %u-%u, with long=%0.3fcm\n",p2ini,p2fin,lon2);

								//Keep searching
								ind3=indaux+4;
								while( (ind3<(pfin-1))&&(!pattern_found) )
								{									
									if( (posX[ind3]!=0.0) || (posY[ind3]!=0.0) )
									{
										//Search for another sub-pattern (planar surface)
										BuscaPatron(ind3,indaux,lon3,vr3a,vr3b);

										//Are dimensions correct for sub-pattern 3?
										if( (lon3>lon_sp3i)&&(lon3<lon_sp3s) )
										{
											//Check that distance between sub-patterns is correct
											lonaux2 = sqrt(((posX[ind3]-posX[p2fin])*(posX[ind3]-posX[p2fin]))+((posY[ind3]-posY[p2fin])*(posY[ind3]-posY[p2fin])));
											if( (lonaux2>lon_aux2i)&&(lonaux2<lon_aux2s) )
											{
												// YES, third pattern found!
												p3ini = ind3;
												p3fin = indaux;
												//if (debug) printf("[AutoDockingLaser]: Third pattern found at points %u-%u, with long=%0.3fcm\n",p3ini,p3fin,lon3);
												printf("[AutoDockingLaser]: Pattern found-> [%.3f] %.3f [%.3f] %.3f [%.3f] \n",lon3,lonaux2,lon2,lonaux1,lon1);
												//Finally, check that all sub-patterns fall in a line
												// this method set the pattern_found to true/false
												ComprobacionFinal();

												puntosp1 = p1fin-p1ini+1;
												puntosp2 = p2fin-p2ini+1;
												puntosp3 = p3fin-p3ini+1;


												if( pattern_found )
												{
													//check if pattern has been repeteadly found (patronverdadero > 4 matches)
													if( patronverdadero )
													{
														comprobar_patron_verdadero(p1ini,p3fin);
													}
												}
											}

										}//if5
									}
									ind3++;
								}// end-while 3rd pattern not_found

							}//if distance between pattern 1 and 2
						}//if-second pattern found
					}// end if correct point
					ind2++;
                }// end-while 2nd pattern not_found

            }//if-first pattern found
        }// end if correct point
		ind++;
    };// end-while 1st pattern not_found
}


//----------------------------
// Main Loop Iterate
//---------------------------
bool CDockingApp::Iterate()
{
	//Initialization 
	pattern_found = false;

	float angf1 = 0;
	float angf2 = 0;
	float dispatron = 0;	
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
		// 1- Get Laser scan, and transform to array of points with X,Y coordinates in cm 
		//---------------------------------------------------------------------------------
		Obtener_puntos_plano();

		// 2- Search for pattern in the scan
		//------------------------------------
		// Was the pattern found in the previous scan? 
		//If so, reduce the search range to improve computation speed (only for laser with high resolution)
		if( found_in_previous_iteration && (TAM>1000) )
		{
			if( (punto_inicio>100)&&(punto_fin<(TAM-100)) )
				Buscar_Patron_Completo(punto_inicio-100,punto_fin+100);
			else if( punto_inicio>100 )
				Buscar_Patron_Completo(punto_inicio-100,TAM);				
			else
				Buscar_Patron_Completo(0,punto_fin+100);			
		}
		else
		{ //No luck- Search the pattern within all the Laser points
			Buscar_Patron_Completo(0,TAM);
		}

		
		// 3- Check if pattern has been found
		//-------------------------------------
		if( pattern_found )
		{
			encontrados++;					//increase the number of consecutive matches
			contador_no_encontrados = 0.0;	//rested non-matches

			if( encontrados==1.0 )			//first time the pattern has been found!
			{
				//Save the angles and distance of the match for future matches.
				angf1 = p1ini*angulo_giro_hok;
				angf2 = p3fin*angulo_giro_hok;
				dispatron = obs->scan[p1ini]*100;
				angulovi = angf1;
				angulovf = angf2;
				disv = dispatron;
				printf("[AutoDocking_Laser] FIRST TIME - pattern found\n");
			}
			else
			{
				//Check that the found pattern complies with previous matches!
				comprobar_patron_verdadero(p1ini,p3fin);
				if( pattern_found )
				{
					printf("[AutoDocking_Laser] Pattern found [%.1f] times. \n",encontrados);
					contador_no_encontrados = 0.0;
					if (encontrados == 4.0)
						patronverdadero = true;		//we can start moving the robot!
				}
				else
				{					
					printf("[AutoDocking_Laser] ERROR: Restarting!\n");
					encontrados = 0.0;			//reset consecutive matches
					contador_no_encontrados++;
				}
			}
			

			// 4- Move robot
			//---------------
			//only after 4 consecutive matches, we declare the pattern as the correct one (patronverdadero)
			if( patronverdadero )
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
			printf(".");
			//printf("[AutoDocking_Laser]: ERROR [%.1f] Pattern not found.\n", contador_no_encontrados);
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
			cout<< "**** PATTERN NOT FOUND ****" << endl;
			m_Comms.Notify("PARKING",0.0);
			go_docking = false;
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
  //! @moos_subscribe param_laserVar  
  AddMOOSVariable(laserVar,laserVar,laserVar,0);
 
  //! @moos_subscribe MOTION_CMD_V 
  //! @moos_var MOTION_CMD_V The current robot linear velocity  
  AddMOOSVariable( "MOTION_CMD_V", "MOTION_CMD_V", "MOTION_CMD_V", 0 );

  //! @moos_subscribe MOTION_CMD_W
  //! @moos_var MOTION_CMD_W The current robot angular velocity  
  AddMOOSVariable( "MOTION_CMD_W", "MOTION_CMD_W", "MOTION_CMD_W", 0 );
  
  //! @moos_subscribe BATTERY_IS_CHARGING
  //! @moos_var BATTERY_IS_CHARGING Variable that indicates when the robot is charging its battery
  AddMOOSVariable( "BATTERY_IS_CHARGING", "BATTERY_IS_CHARGING", "BATTERY_IS_CHARGING", 0 );
  
  //! @moos_subscribe BATTERY_MANAGER_IS_CHARGING
  AddMOOSVariable( "BATTERY_MANAGER_IS_CHARGING", "BATTERY_MANAGER_IS_CHARGING", "BATTERY_MANAGER_IS_CHARGING", 0);

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
	std::string cad;	
	for (MOOSMSG_LIST::const_iterator it=NewMail.begin();it!=NewMail.end();++it)
	{
		const CMOOSMsg &m = *it;

		// charging?
		if( MOOSStrCmp(m.GetKey(),"BATTERY_IS_CHARGING") || MOOSStrCmp(m.GetKey(),"BATTERY_MANAGER_IS_CHARGING") )
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
				
				//------------------------------------
				// Reset the system to initial state
				//------------------------------------
				f2activada = false;				//Init the sistem as Far from docking!  
				contador_no_encontrados = 0;
				encontrados = 0;
				patronverdadero = false;
				angulovi = 0;
				angulovf = 0;
				disv = 0;  
				found_in_previous_iteration = false;
				punto_inicio = 0;
				punto_fin = 0;
				contfases2 = 0;				
			}
		}
		

		// Start!
		if( MOOSStrCmp(m.GetKey(),"PARKING") )
		{	
			if( m.GetDouble()==1.0 )
			{
				if(charge==0.0)
				{
					printf("[CAutoDocking_Laser] Starting Autodocking process...\n");
					go_docking = true;
				}
				else
				{
					printf("[CAutoDocking_Laser] Requested Autodocking process, but ALREADY CHARGING! Disabling request.\n");
					m_Comms.Notify("PARKING",0.0);
				}
			}
			else
			{
				printf("[CAutoDocking_Laser] Autodocking request CANCELED.\n");
				go_docking = false;
			}
		}


		//Close module
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


//------------------------
// NOT USED !!
//------------------------
void CDockingApp::comprobar_pendientes_rectas(float r1b,float r2b,float r3b){

  float auxiliar;
  float limite=5;

  auxiliar=r1b-r2b;
  if(auxiliar<0){
      auxiliar=auxiliar*(-1);
    }
  if(auxiliar>limite){
      pattern_found=false;
    }
  auxiliar=r2b-r3b;
  if(auxiliar<0){
      auxiliar=auxiliar*(-1);
    }
  if(auxiliar>limite){
      pattern_found=false;
    }
  auxiliar=r1b-r3b;
  if(auxiliar<0){
      auxiliar=auxiliar*(-1);
    }
  if(auxiliar>limite){
      pattern_found=false;
    }

  auxiliar=valorbfinal-r1b;
  if(auxiliar<0){
      auxiliar=auxiliar*(-1);
    }
  if(auxiliar>limite){
      pattern_found=false;
    }

  auxiliar=valorbfinal-r2b;
  if(auxiliar<0){
      auxiliar=auxiliar*(-1);
    }
  if(auxiliar>limite){
      pattern_found=false;
    }

  auxiliar=valorbfinal-r3b;
  if(auxiliar<0){
      auxiliar=auxiliar*(-1);
    }
  if(auxiliar>limite){
      pattern_found=false;
    }
}