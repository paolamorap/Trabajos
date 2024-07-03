//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "veins/modules/application/traci/TraCIDemo11p.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include <string>

using namespace veins;

Define_Module(veins::TraCIDemo11p);

//Variables para el protocolo de Enrutamiento

//const int ide = 17;  // ID del emisor del mensaje
//const int idr = 29;  // ID del emisor del mensaje

//COn RSU
const int ide = 47;  // ID del emisor del mensaje
const int idr = 41;  // ID del emisor del mensaje
static bool st = false; //Estado del mensaje Define Ida o Vuelta del mensaje
std::map<int, int> reenviosPorNodo;// //Diccionario para el conteo de los reenvios realizados
//por cada vehiculo
std::map<bool, bool> diccionarioVehiculos;

void TraCIDemo11p::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;

        // Initialize bounding box variables
        Coord pos = mobility->getPositionAt(simTime());
        minX = maxX = pos.x;
        minY = maxY = pos.y;
    }
}

void TraCIDemo11p::onWSA(DemoServiceAdvertisment* wsa)
{
    if (currentSubscribedServiceId == -1) {
        mac->changeServiceChannel(static_cast<Channel>(wsa->getTargetChannel()));
        currentSubscribedServiceId = wsa->getPsid();
        if (currentOfferedServiceId != wsa->getPsid()) {
            stopService();
            startService(static_cast<Channel>(wsa->getTargetChannel()), wsa->getPsid(), "Mirrored Traffic Service");
        }
    }
}

int warnedVehicles = 0;

//(PROGRAMAMOS EL CAMBIO DE RUTA EN UN TIEMPO FUTURO)
void TraCIDemo11p::onWSM(BaseFrame1609_4* frame) {
    // Variable estática para llevar un registro de cuántas veces se ha ejecutado una acción específica.
    static int actionCount = 0;
    TraCIDemo11pMessage* wsm = check_and_cast<TraCIDemo11pMessage*>(frame);

    //Nodos-Vehiculos que interactuan en la comunicación
    int nodeact = mobility->getId(); //Nodo Actual
    int noder  = wsm->getPidr();    //Nodo Receptor
    int nodee  = wsm->getPide();    //Nodo Emisor

    if (diccionarioVehiculos[nodeact] == false) {
        warnedVehicles++;
        diccionarioVehiculos[nodeact] = true;
    }

    //Recepción del destinario
    if (nodeact == noder) {

        //Solo Recibe el mensaje pero no realiza ninguna acción
        std::string destinationRoadId = wsm->getDestinationRoadId();
        findHost()->getDisplayString().setTagArg("i", 1, "green");
        std::cout << "Se Recibió el mensaje" << std::endl;
        std::cout << noder <<"ID Receptor" << std::endl;
        std::cout << nodee <<"ID Emisor" << std::endl;
        std::cout << nodeact <<"ID Actual" << std::endl;
        actionCount++;

        //Recibe el mensaje y realiza acción de reenvio
        if(!(wsm->getStatus())) {
            //Se invierten emisor-receptor
            wsm->setPidr(ide);
            wsm->setPide(idr);
            wsm->setTte(0); //Campo de tiempo del mensaje / no se ha usado
            wsm->setSenderAddress(myId);
            wsm->setSerial(3);
            reenviosPorNodo[nodeact]++;
            wsm->setNemisor(idr);     //No se ha utilizado
            wsm->setNreceptor(ide);     //No se ha utilizado
            //Verifica que el Nodo Receptor solo reenvie una vez el mensaje
            if(reenviosPorNodo[nodeact] <= 1) {
                wsm->setStatus(true);
                //std::cout << "Se Programo Renvio en el Receptor:" << nodeact << std::endl;
                wsm->setSpeed(mobility->getVehicleCommandInterface()->getSpeed()); //Se envia velocidad de receptor
                scheduleAt(simTime() + uniform(1.01, 1.2), wsm->dup());
            }
        }
        //En caso de que se necesite reenvio por nodos intermedios
     }else{

        //Reenvio hacia el Receptor
        if(reenviosPorNodo[nodeact] < 1 && nodeact !=ide && nodeact !=idr) {
                     reenviosPorNodo[nodeact]++;
                     //std::cout << "Se Programo Renvio de ida en::" << nodeact << std::endl;
                     wsm->setPide(nodeact);
                     wsm->setTte(0);
                     wsm->setSenderAddress(myId);
                     wsm->setSerial(3);
                     scheduleAt(simTime() + uniform(1.01, 1.2), wsm->dup());
         }

        //Reenvio hacia el Emisor
        if(reenviosPorNodo[nodeact] < 2 && (wsm->getStatus()) && nodeact !=ide && nodeact !=idr) {
                     reenviosPorNodo[nodeact]++;
                     //std::cout << "Se Programo Renvio de vuelta en:" << nodeact << std::endl;
                     wsm->setPide(nodeact);
                     wsm->setTte(0);
                     wsm->setSenderAddress(myId);
                     wsm->setSerial(3);
                     scheduleAt(simTime() + uniform(1.01, 1.2), wsm->dup());
         }

     }

}



// Maneja los mensajes automáticos en una simulación de red vehicular.
void TraCIDemo11p::handleSelfMsg(cMessage* msg) {
    TraCIDemo11pMessage* wsm = dynamic_cast<TraCIDemo11pMessage*>(msg);
    if (!wsm) {
        DemoBaseApplLayer::handleSelfMsg(msg);
        return;
    }

    int messageSerial = wsm->getSerial();
    int ne = wsm->getPide(); //NodeEmisor
    int nodeId = mobility->getId(); ///Nodo Actual

    if (messageSerial == 3) {
        wsm->setReroute(true);
        wsm->setFwd(wsm->getReroute());
    }

    //Previene que el nodo emisor reenvie el mensaje
    if (nodeId != ide){
        sendDown(wsm->dup());
    }
    messageSerial++;
    wsm->setSerial(messageSerial);
    wsm->setFwdno(wsm->getSerial());
    if (messageSerial >= 4){
        stopService();
        delete wsm;
    } else {
        // De lo contrario, programa este mensaje para ser enviado nuevamente después de 5 unidades de tiempo de simulación.
        scheduleAt(simTime() + 2, wsm);
    }
}
static int contadorSinNodoAct = 0;
void TraCIDemo11p::handlePositionUpdate(cObject* obj)
{
     // Contador de llamadas sin nodeact
    static std::set<int> nodosContados;  // Conjunto para almacenar los nodeact ya contados


    DemoBaseApplLayer::handlePositionUpdate(obj);
    // Nodos-Vehiculos que interactúan en la comunicación
    int nodeact = mobility->getId(); // Nodo Actual

    // Verificar si nodeact ya ha sido contado
    if (nodosContados.count(nodeact) == 0) {
        // Si nodeact no está en el conjunto de nodos contados, aumentar el contador
        nodosContados.insert(nodeact);
        contadorSinNodoAct++;
        std::cout << "Nodeact " << nodeact << " contado por primera vez." << std::endl;
    }

    if (simTime() - lastDroveAt >= 2) {
        if (reenviosPorNodo[nodeact] - mobility->getSpeed() >= 7){
            //std::cout << mobility->getId() <<"ID Receptor" << std::endl;
            //std::cout << mobility->getSpeed() <<"Velocidad del Vehiculo" << std::endl;
            //std::cout << reenviosPorNodo[nodeact] - mobility->getSpeed() <<"Diferencia de vel del Vehiculo" << std::endl;
            // Obtener la posición actual del nodo que va a enviar el mensaje
            Coord senderPosition = mobility->getPositionAt(simTime());
            findHost()->getDisplayString().setTagArg("i", 1, "red");
            sentMessage = true;
            TraCIDemo11pMessage* wsm = new TraCIDemo11pMessage();
            populateWSM(wsm);
            wsm -> setCurrentTime(simTime());
            wsm -> setAccidente(true);
            wsm->setDemoData(mobility->getRoadId().c_str());
            lastDroveAt = simTime();


            // host is standing still due to crash
            if (dataOnSch) {
                startService(Channel::sch2, 42, "Traffic Information Service");
                // started service and server advertising, schedule message to self to send later
                scheduleAt(computeAsynchronousSendingTime(1, ChannelType::service), wsm);
            }
            else {
                // send right away on CCH, because channel switching is disabled
                sendDown(wsm);
            }
        }else{
            reenviosPorNodo[nodeact] = mobility->getSpeed();
        }
    }
}

void TraCIDemo11p::updateBoundingBox()
{
    Coord pos = mobility->getPositionAt(simTime());
    if (pos.x < minX) minX = pos.x;
    if (pos.x > maxX) maxX = pos.x;
    if (pos.y < minY) minY = pos.y;
    if (pos.y > maxY) maxY = pos.y;
}


void TraCIDemo11p::finish()
{
    double warnedPercentage = contadorSinNodoAct > 0 ? (double)warnedVehicles / contadorSinNodoAct * 100 : 0;
        std::cout << "contadorSinNodoAct: " << contadorSinNodoAct<< std::endl;
        std::cout << "warnedVehicles: " << warnedVehicles << std::endl;
        std::cout << "Porcentage Vehiculos Avisados: " << warnedPercentage << " %" << std::endl;
        std::cout << "----------------------------------------------------------------------------- "  << std::endl;
        // Calculate the bounding box area and vehicle density
        double area = (maxX - minX) * (maxY - minY) / 1e6; // Area in km²
        double density = area > 0 ? contadorSinNodoAct / area : 0;
        std::cout << "Densidad " << density  << std::endl;
        std::cout << "----------------------------------------------------------------------------- "  << std::endl;

    DemoBaseApplLayer::finish();
}


