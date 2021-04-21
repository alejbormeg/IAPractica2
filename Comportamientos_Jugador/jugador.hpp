#ifndef COMPORTAMIENTOJUGADOR_H
#define COMPORTAMIENTOJUGADOR_H

#include "comportamientos/comportamiento.hpp"

#include <list>

struct estado {
  int fila;
  int columna;
  int orientacion;
};

class ComportamientoJugador : public Comportamiento {
  public:
    ComportamientoJugador(unsigned int size) : Comportamiento(size) {
      // Inicializar Variables de Estado
      fil = col = 99;
      brujula = 0; // 0: Norte, 1:Este, 2:Sur, 3:Oeste
      destino.fila = -1;
      destino.columna = -1;
      destino.orientacion = -1;
      hayplan=false;
    }
    ComportamientoJugador(std::vector< std::vector< unsigned char> > mapaR) : Comportamiento(mapaR) {
      // Inicializar Variables de Estado
      fil = col = 99;
      brujula = 0; // 0: Norte, 1:Este, 2:Sur, 3:Oeste
      destino.fila = -1;
      destino.columna = -1;
      destino.orientacion = -1;
      hayplan=false;
    }
    ComportamientoJugador(const ComportamientoJugador & comport) : Comportamiento(comport){}
    ~ComportamientoJugador(){}

    Action think(Sensores sensores);
    int interact(Action accion, int valor);
    void VisualizaPlan(const estado &st, const list<Action> &plan);
    ComportamientoJugador * clone(){return new ComportamientoJugador(*this);}
    int coste (estado &st, bool bikini, bool zapatillas);
    int costeA_estrella (estado &st);
  private:
    // Declarar Variables de Estado
    int fil, col, brujula;
    estado actual, destino, bateria; //Para el nivel 2 añadimos el estado batería que guarda la fila y la columna para la bateria
    int bateria_min=500; //si la batería baja de este minimo va a recargar
    int bateria_max=2990; //si la bateria supera este maximo deja de recargarse
    list<Action> plan;
    bool hayplan;
    bool zapatillas=false, bikini=false, bateriaOk=true, bateria_encontrada=false; //variables para el nivel 2, una si ha encontrado las zapatillas, 
                                                                                  //otra para el bikini, otra para el estado de la batería 
                                                                                  //y otra por si encuentra la batería
    int pasos=2; //Numero de pasos que da sin recalcular la ruta
    int maxPasos=2;//El máximo de pasos sin recalcular la ruta es 2 así reducimos a la mitad el tiempo de ejecución
    

    // Métodos privados de la clase
    bool pathFinding(int level, const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_Profundidad(const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_Anchura(const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_CostoUniforme(const estado &origen, const estado &destino, list<Action> &plan);
    bool pathFinding_nivel2(const estado &origen, const estado &destino, list<Action> &plan);
    void ActualizarMapa(Sensores &sensores);

    void PintaPlan(list<Action> plan);
    bool HayObstaculoDelante(estado &st);

};

#endif
