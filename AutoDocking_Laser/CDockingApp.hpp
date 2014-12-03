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

#ifndef CDockingApp_H
#define CDockingApp_H

//#include <MOOS/libMOOS/App/MOOSApp.h>
#include <COpenMORAMOOSApp.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <math.h>
// Any other includes..

class CDockingApp : public COpenMORAApp
{
public:
    CDockingApp();
    virtual ~CDockingApp();

protected:
	/** called at startup */
	virtual bool OnStartUp();
	/** called when new mail arrives */
	virtual bool OnNewMail(MOOSMSG_LIST & NewMail);


	void dibujar_puntos(int p1i,int p1f,int p2i,int p2f,int p3i,int p3f);

	void comprobar_patron_verdadero(int pi,int pf);

	void comprobar_pendientes_rectas(float r1b,float r2b,float r3b);

	void mover_fase2(float cp,float dis,float cory,float corx);

	void MoverRobot();

    void ComprobacionFinal();

	void Obtener_puntos_plano();

	void BuscaPatron(int ind,int &pfin,float &longitud, float &valora, float &valorb);

	void Buscar_Patron_Completo(unsigned int pinicio,unsigned int pfin);

	/** called when work is to be done */
	virtual bool Iterate();
	/** called when app connects to DB */
	virtual bool OnConnectToServer();

	bool OnCommandMsg( CMOOSMsg Msg );

	bool DoRegistrations();

	// DATA. Your local variables here...

	
	mrpt::slam::CObservation2DRangeScanPtr obs;			//The laser scans
	const static unsigned int TAMMAXIMO=1300;
    float posX[TAMMAXIMO];								//X coordinates of the obstacles in the scan (cm)
    float posY[TAMMAXIMO];								//Y coordinates of the obstacles in the scan (cm)
	unsigned int TAM;									//Size of laser scans (points)
	bool pattern_found;									//whether the pattern is found or not
	int p1ini,p1fin,p2ini,p2fin,p3ini,p3fin;
	float valorafinal,valorbfinal;
	std::string laserVar;								//The name of the laser variable from which get the scans.
    
	float distancia_corte;								//max distance in cm to consider a point as part of the same subpattern (planar continuous surface)
	float distancia_comprobacion;
	float longitud_total_patron;	
	float angulo_central;
	float distancia_parada;
	float v_lineal_f2_rap;
	float v_lineal_f2_len;
	float agiro_f2_rap;
	float agiro_f2_len;
	float angulo_giro_hok;								//angular resolution = deg/point
	float v_lineal_mover;
	float agiro_mover;
	float demasiado_cerca;
	float lon_sp1i;
	float lon_sp1s;
	float lon_sp2i;
	float lon_sp2s;
	float lon_sp3i;
	float lon_sp3s;
	float lon_aux1i;
	float lon_aux1s;
	float lon_aux2i;
	float lon_aux2s;
	float contador_no_encontrados;
	float encontrados;
	float angulovi;
	float angulovf;
	float disv;
	bool found_in_previous_iteration;
	float punto_inicio;
	float punto_fin;	
    //bool miparking;
	float charge;

    bool patronverdadero;
	bool f2activada;
	//bool robotenposicion;
    //float pruebafich;
	float contfases2;

	bool debug;		////cga:if true, visual information is shown.
	bool go_docking;		// Indicates if the module is active or waiting

	//cga:declaration of win as a pointer 
	mrpt::gui::CDisplayWindowPlots *win;
};
#endif