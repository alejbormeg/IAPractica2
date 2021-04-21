#include "../Comportamientos_Jugador/jugador.hpp"
#include "motorlib/util.h"

#include <iostream>
#include <cmath>
#include <set>
#include <stack>
#include <queue>
#include <utility>


// Este es el método principal que debe contener los 4 Comportamientos_Jugador
// que se piden en la práctica. Tiene como entrada la información de los
// sensores y devuelve la acción a realizar.
Action ComportamientoJugador::think(Sensores sensores) {
 Action accion;

  accion = actIDLE;
   // Estoy en el nivel 1
  
      
          actual.fila        = sensores.posF;
          actual.columna     = sensores.posC;
          actual.orientacion = sensores.sentido;

          cout << "Fila: " << actual.fila << endl;
          cout << "Col : " << actual.columna << endl;
          cout << "Ori : " << actual.orientacion << endl;

        destino.fila       = sensores.destinoF;
        destino.columna    = sensores.destinoC;

      if(!hayplan){
        hayplan = pathFinding (sensores.nivel, actual, destino, plan);
	  }
      

      if( hayplan and plan.size() > 0) {
		if(sensores.nivel == 4){ //nivel2
			ActualizarMapa(sensores);

			if (bateriaOk){ //Cuando vamos bien de batería buscamos objetivos
				cout << "Bateria OK" << endl;
				if (pasos==0){
					hayplan =  pathFinding(sensores.nivel, actual, destino, plan);  // Recalculamos el plan cada 10 pasos
					if (sensores.tiempo > 200)
						pasos=-1; //plan kamikaze por si vamos justos de tiempo Calcula una vez el plan y lo intenta seguir hasta el final (si conocemos gran parte del mapa es beneficioso)
					else 
						pasos=maxPasos;
				}

				if (mapaResultado[sensores.posF][sensores.posC]=='D' and zapatillas==false)
					zapatillas=true;

				if (mapaResultado[sensores.posF][sensores.posC]=='K' and bikini==false)
					bikini=true;

				if(HayObstaculoDelante(actual) and plan.front() == actFORWARD){
				cout << "\nRecalculando ruta... (1)" << endl;
				hayplan = pathFinding(sensores.nivel, actual, destino, plan);
				}

				if (sensores.superficie[2]=='a' and plan.front() == actFORWARD){
						return actIDLE;
				}

				if (mapaResultado[sensores.posF][sensores.posC] == 'X' and sensores.bateria < max(1600, int (sensores.tiempo)))
					return actIDLE;

				if (sensores.bateria < bateria_min){ //Si la batería por debajo del mínimo 
					if (bateria_encontrada){ //Miramos si hemos encontrado la casilla de recarga en algún momento
						bateriaOk=false; //Si es así vamos a recargar
						bateria_min = bateria_min/2; //Reducimos el mínimo de batería a la mitad para no perder mucho tiempo
						pasos=1;
					}
				}
				pasos --;
			}
			else{ //Cuando vamos mal de batería vamos a la casilla de Batería si la hemos enctontrado
				cout << "Bateria malamente" << endl;
				if (pasos==0){
					hayplan =  pathFinding(sensores.nivel, actual, bateria, plan);  // buscamos la batería
					pasos=maxPasos;
				}

				if(HayObstaculoDelante(actual) and plan.front() == actFORWARD){
				cout << "\nRecalculando ruta... (2)" << endl;
				hayplan = pathFinding(sensores.nivel, actual, bateria, plan);
				}

				if (sensores.superficie[2]=='a' and plan.front() == actFORWARD){
						return actIDLE;
				}
		
				if (mapaResultado[sensores.posF][sensores.posC] == 'X' and sensores.bateria < sensores.tiempo)
					return actIDLE;
				
				if (sensores.bateria>= bateria_max){
					
					bateriaOk=true; //Volvemos a buscar objetivos
					bateria_max=bateria_max/2; // como ya quedará menos tiempo intentaremos recargar la mitad la próxima vez
					pasos=maxPasos;
					return actIDLE;
				}	

				
							
				pasos --;
			}
			
		}
			accion = plan.front();
			plan.erase(plan.begin());
      }
      else {

        cerr << "\nRecalculando ruta ..." << endl;
		if (mapaResultado[sensores.posF][sensores.posC] != 'X' or sensores.bateria>=bateria_max)
        hayplan = pathFinding(sensores.nivel, actual, destino, plan);  // Recalculamos el plan

      }
       
  return accion;
}


// Llama al algoritmo de busqueda que se usará en cada comportamiento del agente
// Level representa el comportamiento en el que fue iniciado el agente.
bool ComportamientoJugador::pathFinding (int level, const estado &origen, const estado &destino, list<Action> &plan){
	switch (level){
		case 1: cout << "Busqueda en profundad\n";
			    return pathFinding_Profundidad(origen,destino,plan);
						break;
		case 2: cout << "Busqueda en Anchura\n";
				return pathFinding_Anchura(origen,destino,plan);
						break;
		case 3: cout << "Busqueda Costo Uniforme\n";
				return pathFinding_CostoUniforme(origen,destino,plan);
						break;
		case 4: cout << "Busqueda para el reto\n";
				return pathFinding_nivel2(origen,destino,plan);
						break;
	}
	cout << "Comportamiento sin implementar\n";
	return false;
}


//---------------------- Implementación de la busqueda en profundidad ---------------------------

// Dado el código en carácter de una casilla del mapa dice si se puede
// pasar por ella sin riegos de morir o chocar.
bool EsObstaculo(unsigned char casilla){
	if (casilla=='P' or casilla=='M')
		return true;
	else
	  return false;
}


// Comprueba si la casilla que hay delante es un obstaculo. Si es un
// obstaculo devuelve true. Si no es un obstaculo, devuelve false y
// modifica st con la posición de la casilla del avance.
bool ComportamientoJugador::HayObstaculoDelante(estado &st){
	int fil=st.fila, col=st.columna;

  // calculo cual es la casilla de delante del agente
	switch (st.orientacion) {
		case 0: fil--; break;
		case 1: col++; break;
		case 2: fil++; break;
		case 3: col--; break;
	}

	// Compruebo que no me salgo fuera del rango del mapa
	if (fil<0 or fil>=mapaResultado.size()) return true;
	if (col<0 or col>=mapaResultado[0].size()) return true;

	// Miro si en esa casilla hay un obstaculo infranqueable
	if (!EsObstaculo(mapaResultado[fil][col])){
		// No hay obstaculo, actualizo el parámetro st poniendo la casilla de delante.
    	st.fila = fil;
		st.columna = col;
		return false;
	}
	else{
	  return true;
	}
}




struct nodo{
	estado st;
	list<Action> secuencia;
	bool bikini=false; //CosteUniforme
	bool zapatillas=false; //Coste Uniforme
	int coste=0; //Coste Uniforme
};

bool operator< (const nodo &a, const nodo &b){
	if (a.coste > b.coste)
		return true;
	else
		return false;
		
}

struct nodoA_estrella{ //Struct para el algoritmo del nivel 2
	nodo n;
	int funcion=0;
	int distancia=0;

	void setFuncion (){
		funcion=distancia + n.coste;
	}

	void distManhattan ( nodoA_estrella &b){
		int distanciaManhattan=abs(n.st.fila - b.n.st.fila) + abs(n.st.columna - b.n.st.columna);
		distancia=distanciaManhattan;
	}
};

bool operator< (const nodoA_estrella &a, const nodoA_estrella &b){
		if (a.funcion>b.funcion)
			return true;
		else 
			return false;
	}

struct ComparaEstados{
	bool operator()(const estado &a, const estado &n) const{
		if ((a.fila > n.fila) or (a.fila == n.fila and a.columna > n.columna) or
	      (a.fila == n.fila and a.columna == n.columna and a.orientacion > n.orientacion))
			return true;
		else
			return false;
	}
};


// Implementación de la búsqueda en profundidad.
// Entran los puntos origen y destino y devuelve la
// secuencia de acciones en plan, una lista de acciones.
bool ComportamientoJugador::pathFinding_Profundidad(const estado &origen, const estado &destino, list<Action> &plan) {
	//Borro la lista
	cout << "Calculando plan\n";
	plan.clear();
	set<estado,ComparaEstados> generados; // Lista de Cerrados
	stack<nodo> pila;	// Lista de Abiertos

  	nodo current;
	current.st = origen;
	current.secuencia.empty();

	pila.push(current);

  while (!pila.empty() and (current.st.fila!=destino.fila or current.st.columna != destino.columna)){

		pila.pop();
		generados.insert(current.st);

		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion+1)%4;
		if (generados.find(hijoTurnR.st) == generados.end()){
			hijoTurnR.secuencia.push_back(actTURN_R);
			pila.push(hijoTurnR);

		}

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion+3)%4;
		if (generados.find(hijoTurnL.st) == generados.end()){
			hijoTurnL.secuencia.push_back(actTURN_L);
			pila.push(hijoTurnL);
		}

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st)){
			if (generados.find(hijoForward.st) == generados.end()){
				hijoForward.secuencia.push_back(actFORWARD);
				pila.push(hijoForward);
			}
		}

		// Tomo el siguiente valor de la pila
		if (!pila.empty()){
			current = pila.top();
		}
	}

  cout << "Terminada la busqueda\n";

	if (current.st.fila == destino.fila and current.st.columna == destino.columna){
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else {
		cout << "No encontrado plan\n";
	}


	return false;
}


// Implementación de la búsqueda en anchura.
// Entran los puntos origen y destino y devuelve la
// secuencia de acciones en plan, una lista de acciones.
bool ComportamientoJugador::pathFinding_Anchura(const estado &origen, const estado &destino, list<Action> &plan) {
	//Borro la lista
	cout << "Calculando plan\n";
	plan.clear();
	set<estado,ComparaEstados> generados; // Lista de Cerrados
	queue<nodo> cola;	// Lista de Abiertos, usamos una cola FIFO

  	nodo current;
	current.st = origen;
	current.secuencia.empty();

	cola.push(current);

  while (!cola.empty() and (current.st.fila!=destino.fila or current.st.columna != destino.columna)){

		cola.pop();
		generados.insert(current.st);

		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion+1)%4;
		if (generados.find(hijoTurnR.st) == generados.end()){
			hijoTurnR.secuencia.push_back(actTURN_R);
			cola.push(hijoTurnR);

		}

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion+3)%4;
		if (generados.find(hijoTurnL.st) == generados.end()){
			hijoTurnL.secuencia.push_back(actTURN_L);
			cola.push(hijoTurnL);
		}

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st)){
			if (generados.find(hijoForward.st) == generados.end()){
				hijoForward.secuencia.push_back(actFORWARD);
				cola.push(hijoForward);
			}
		}

		// Tomo el siguiente valor de la pila
		if (!cola.empty()){
			current = cola.front();
		}
	}

  cout << "Terminada la busqueda\n";

	if (current.st.fila == destino.fila and current.st.columna == destino.columna){
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else {
		cout << "No encontrado plan\n";
	}


	return false;
}

// Implementación dnivel 2
// Entran los puntos origen y destino y devuelve la
// secuencia de acciones en plan, una lista de acciones.
bool ComportamientoJugador::pathFinding_nivel2(const estado &origen, const estado &destino, list<Action> &plan) {
	//Borro la lista
	cout << "Calculando plan\n";
	plan.clear();
	set<estado,ComparaEstados> cerrados; // Lista de Cerrados
	priority_queue<nodoA_estrella,vector<nodoA_estrella>,less<vector<nodoA_estrella>::value_type> > abiertos;	// Lista de Abiertos
	int coste_total=0;

  	nodoA_estrella current, final;
	current.n.st = origen;
	current.n.secuencia.empty();
	final.n.st=destino;
	abiertos.push(current);


    while (!abiertos.empty() and (current.n.st.fila!=destino.fila or current.n.st.columna != destino.columna)){
		abiertos.pop();
		cerrados.insert(current.n.st);

		
		// Generar descendiente de girar a la derecha
		nodoA_estrella hijoTurnR = current;
		hijoTurnR.n.st.orientacion = (hijoTurnR.n.st.orientacion+1)%4;
		if (cerrados.find(hijoTurnR.n.st) == cerrados.end()){
			hijoTurnR.n.secuencia.push_back(actTURN_R);
			hijoTurnR.n.coste+= costeA_estrella(hijoTurnR.n.st);
			hijoTurnR.distManhattan(final);
			hijoTurnR.setFuncion();
			abiertos.push(hijoTurnR);
		}

		// Generar descendiente de girar a la izquierda
		nodoA_estrella hijoTurnL = current;
		hijoTurnL.n.st.orientacion = (hijoTurnL.n.st.orientacion+3)%4;		
		if (cerrados.find(hijoTurnL.n.st) == cerrados.end()){
			hijoTurnL.n.secuencia.push_back(actTURN_L);
			hijoTurnL.n.coste+= costeA_estrella(hijoTurnL.n.st);
			hijoTurnL.distManhattan(final);
			hijoTurnL.setFuncion();
			abiertos.push(hijoTurnL);
		}

		// Generar descendiente de avanzar
		nodoA_estrella hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.n.st)){
			if (cerrados.find(hijoForward.n.st) == cerrados.end()){
				hijoForward.n.secuencia.push_back(actFORWARD);
				hijoForward.n.coste+= costeA_estrella(hijoTurnL.n.st);
				hijoForward.distManhattan(final);
				hijoForward.setFuncion();
				abiertos.push(hijoForward);

			}
		}

		//Tomo el siguiente valor de la cola con prioridad
		if (!abiertos.empty()){
			current=abiertos.top();
		}
	
	}

  cout << "Terminada la busqueda\n";

	if (current.n.st.fila == destino.fila and current.n.st.columna == destino.columna){
		cout << "Cargando el plan\n";
		plan = current.n.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		cout << "Coste total del plan: " << current.n.coste<< endl;
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else {
		cout << "No encontrado plan\n";
	}


	return false;
}


bool ComportamientoJugador::pathFinding_CostoUniforme(const estado &origen, const estado &destino, list<Action> &plan) {
	//Borro la lista
	cout << "Calculando plan\n";
	plan.clear();
	set<estado,ComparaEstados> cerrados; // Lista de Cerrados
	priority_queue<nodo,vector<nodo>,less<vector<nodo>::value_type> > abiertos;	// Lista de Abiertos
	int coste_total=0;

  	nodo current;
	current.st = origen;
	current.secuencia.empty();
	abiertos.push(current);


    while (!abiertos.empty() and (current.st.fila!=destino.fila or current.st.columna != destino.columna)){
		abiertos.pop();
		cerrados.insert(current.st);

		
		// Generar descendiente de girar a la derecha
		nodo hijoTurnR = current;
		hijoTurnR.st.orientacion = (hijoTurnR.st.orientacion+1)%4;
		if (cerrados.find(hijoTurnR.st) == cerrados.end()){
			hijoTurnR.secuencia.push_back(actTURN_R);
			hijoTurnR.coste+= coste(hijoTurnR.st,hijoTurnR.bikini,hijoTurnR.zapatillas);
			abiertos.push(hijoTurnR);

		}

		// Generar descendiente de girar a la izquierda
		nodo hijoTurnL = current;
		hijoTurnL.st.orientacion = (hijoTurnL.st.orientacion+3)%4;		
		if (cerrados.find(hijoTurnL.st) == cerrados.end()){
			hijoTurnL.secuencia.push_back(actTURN_L);
			hijoTurnL.coste+= coste(hijoTurnL.st,hijoTurnL.bikini,hijoTurnL.zapatillas);
			abiertos.push(hijoTurnL);

		}

		// Generar descendiente de avanzar
		nodo hijoForward = current;
		if (!HayObstaculoDelante(hijoForward.st)){
			if (cerrados.find(hijoForward.st) == cerrados.end()){
				hijoForward.secuencia.push_back(actFORWARD);
				hijoForward.coste+= coste(hijoTurnL.st, hijoForward.bikini,hijoForward.zapatillas);
				abiertos.push(hijoForward);

			}
		}

		//Tomo el siguiente valor de la cola con prioridad
		if (!abiertos.empty()){
			current=abiertos.top();

			//comprobamos si el nodo actual es un bikini o unas zapatillas
					if (mapaResultado[current.st.fila][current.st.columna]=='K' && current.bikini==false)
						current.bikini=true;
	
					if (mapaResultado[current.st.fila][current.st.columna]=='D' && current.zapatillas==false)
						current.zapatillas=true;
		}
	
	}

  cout << "Terminada la busqueda\n";

	if (current.st.fila == destino.fila and current.st.columna == destino.columna){
		cout << "Cargando el plan\n";
		plan = current.secuencia;
		cout << "Longitud del plan: " << plan.size() << endl;
		PintaPlan(plan);
		cout << "Coste total del plan: " << current.coste<< endl;
		// ver el plan en el mapa
		VisualizaPlan(origen, plan);
		return true;
	}
	else {
		cout << "No encontrado plan\n";
	}


	return false;
}


// Sacar por la términal la secuencia del plan obtenido
void ComportamientoJugador::PintaPlan(list<Action> plan) {
	auto it = plan.begin();
	while (it!=plan.end()){
		if (*it == actFORWARD){
			cout << "A ";
		}
		else if (*it == actTURN_R){
			cout << "D ";
		}
		else if (*it == actTURN_L){
			cout << "I ";
		}
		else {
			cout << "- ";
		}
		it++;
	}
	cout << endl;
}



void AnularMatriz(vector<vector<unsigned char> > &m){
	for (int i=0; i<m[0].size(); i++){
		for (int j=0; j<m.size(); j++){
			m[i][j]=0;
		}
	}
}


// Pinta sobre el mapa del juego el plan obtenido
void ComportamientoJugador::VisualizaPlan(const estado &st, const list<Action> &plan){
  AnularMatriz(mapaConPlan);
	estado cst = st;

	auto it = plan.begin();
	while (it!=plan.end()){
		if (*it == actFORWARD){
			switch (cst.orientacion) {
				case 0: cst.fila--; break;
				case 1: cst.columna++; break;
				case 2: cst.fila++; break;
				case 3: cst.columna--; break;
			}
			mapaConPlan[cst.fila][cst.columna]=1;
		}
		else if (*it == actTURN_R){
			cst.orientacion = (cst.orientacion+1)%4;
		}
		else {
			cst.orientacion = (cst.orientacion+3)%4;
		}
		it++;
	}
}



int ComportamientoJugador::interact(Action accion, int valor){
  return false;
}

int ComportamientoJugador::coste (estado &st, bool bikini, bool zapatillas){
	unsigned char casilla=mapaResultado[st.fila][st.columna];

	switch (casilla){
		case 'A':
			if (bikini)
				return 10;
			else
				return 100;
		break;
		
		case 'B':
			if(zapatillas)
				return 5;
			else 
				return 50;
		break;

		case 'T':
			return 2;
		break;

		case 'X':       //Tengo en cuenta las casillas de recarga para el coste uniforme
			return -10;
		break;

		default:
			return 1;
		break;
	}

}

void ComportamientoJugador:: ActualizarMapa (Sensores &sensores){ //Función para ir rellenando el mapa con la información que vamos descubriendo 
	int fila_actual=sensores.posF;
	int columna_actual=sensores.posC;
	int pos=0; //contador para iterar sobre el vector de sensores.terreno

	switch (sensores.sentido){ //Switch para rellenar el mapa dependiendo de a donde mire el personaje
		case 0:  //Mirando al norte
			for (int i=0; i<=3; i++){
				for (int j=-i; j<=i; j++){
					mapaResultado[fila_actual-i][columna_actual+j]=sensores.terreno[pos];
					if (sensores.terreno[pos]=='X'){ //si encontramos la batería guardamos su fila y columna 
						bateria_encontrada=true;
						bateria.fila=fila_actual-i;
						bateria.columna=columna_actual+j;
					}
					pos++;
				}
			}	
		break;

		case 1: //Mirando al este
			for (int j=0; j<=3; j++){
				for (int i=-j; i<=j; i++){
					mapaResultado[fila_actual+i][columna_actual+j]=sensores.terreno[pos];
					if (sensores.terreno[pos]=='X'){ //si encontramos la batería guardamos su fila y columna 
						bateria_encontrada=true;
						bateria.fila=fila_actual+i;
						bateria.columna=columna_actual+j;
					}
					pos++;
				}
			}		
		break;

		case 2: //Mirando al sur
			for (int i=0; i<=3; i++){
				for (int j=-i; j<=i; j++){
					mapaResultado[fila_actual+i][columna_actual-j]=sensores.terreno[pos];
					if (sensores.terreno[pos]=='X'){ //si encontramos la batería guardamos su fila y columna 
						bateria_encontrada=true;
						bateria.fila=fila_actual+i;
						bateria.columna=columna_actual-j;
					}
					pos++;
				}
			}				
		break;

		case 3: //Mirando al oeste
			for (int i=0; i<=3; i++){
				for (int j=-i; j<=i; j++){
					mapaResultado[fila_actual-j][columna_actual-i]=sensores.terreno[pos];
					if (sensores.terreno[pos]=='X'){ //si encontramos la batería guardamos su fila y columna 
						bateria_encontrada=true;
						bateria.fila=fila_actual-j;
						bateria.columna=columna_actual-i;
					
					}
					pos++;
				}
			}		
		break;

	}

}


int ComportamientoJugador::costeA_estrella (estado &st){ //Funcion coste modificada para A* pues vamos a "engañar" a nuestro personaje en algunos casos
	unsigned char casilla=mapaResultado[st.fila][st.columna];

	switch (casilla){
		case 'A':
			if (bikini)
				return 5; //cambio su valor a 5 para que recorte por el agua si puede
			else
				return 100;
		break;
		
		case 'B':
			if(zapatillas)
				return 2; //Cambio su valor a 2 para que recorte por el bosque si puede
			else 
				return 50;
		break;

		case 'T':
			return 2;
		break;


		
		case 'X':       //Tengo en cuenta las casillas de recarga para el coste uniforme

			if (bateriaOk)
				return 1;
			else 
				return -10;
		break;

		case 'K':
			if (bikini)
				return 1;
			else 
				return -25; //Si puede ser, que lo coja cuanto antes
		break;

		case 'D':
			if (zapatillas)
				return 1;
			else 
				return -25;//si puede ser, que lo coja cuanto antes
		break;

		default: //El resto de casillas valen 1, esto implica que la casilla ? también, por eso nuestro personaje preferirá explorar a ir por sendero conocido en algunas ocasiones
			return 1;
		break;
	}

}
