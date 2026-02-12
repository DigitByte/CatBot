//
// Created by andres on 14.10.20.
//

#include "polygon_support.h"


PredPolygonSupport::PredPolygonSupport(){
    this->weight_factor_fl = 0;
    this->weight_factor_fr = 0;
    this->weight_factor_bl = 0;
    this->weight_factor_br = 0;

    this->pos_feet_fl << 0,0,0;
    this->pos_feet_fr << 0,0,0;
    this->pos_feet_bl << 0,0,0;
    this->pos_feet_br << 0,0,0;


}

PredPolygonSupport::~PredPolygonSupport(){

}

void PredPolygonSupport::setLeg(std::string leg, const Eigen::Vector3f feet_pos, float weight_factor){
    if(leg == "fl"){
        this->pos_feet_fl.operator=(  feet_pos  );
        this->weight_factor_fl = weight_factor;
    }

    if(leg == "fr"){
        this->pos_feet_fr.operator=(  feet_pos  );
        this->weight_factor_fr = weight_factor;
    }

    if(leg == "bl"){
        this->pos_feet_bl.operator=(  feet_pos  );
        this->weight_factor_bl = weight_factor;
    }

    if(leg == "br"){
        this->pos_feet_br.operator=(  feet_pos  );
        this->weight_factor_br = weight_factor;
    }
}


void PredPolygonSupport::getVirtualVertice(std::string leg, Eigen::Vector3f* pos_vertice){
    Eigen::Vector3f pos, pos_adj1, pos_adj2;
    float weight, weight_adj1, weight_adj2;

    if(leg == "fl"){
        pos << this->pos_feet_fl.x(), this->pos_feet_fl.y(), 0;
        pos_adj1  << this->pos_feet_fr.x(), this->pos_feet_fr.y(), 0;
        pos_adj2  << this->pos_feet_bl.x(), this->pos_feet_bl.y(), 0;

        weight = this->weight_factor_fl;
        weight_adj1 = this->weight_factor_fr;
        weight_adj2 = this->weight_factor_bl;
    }

    if(leg == "fr"){
        pos << this->pos_feet_fr.x(), this->pos_feet_fr.y(), 0;
        pos_adj1 << this->pos_feet_fl.x(), this->pos_feet_fl.y(), 0;
        pos_adj2 << this->pos_feet_br.x(), this->pos_feet_br.y(), 0;

        weight = this->weight_factor_fr;
        weight_adj1 = this->weight_factor_fl;
        weight_adj2 = this->weight_factor_br;
    }

    if(leg == "bl"){
        pos << this->pos_feet_bl.x(), this->pos_feet_bl.y(), 0;
        pos_adj1 << this->pos_feet_fl.x(), this->pos_feet_fl.y(), 0;
        pos_adj2 << this->pos_feet_br.x(), this->pos_feet_br.y(), 0;

        weight = this->weight_factor_bl;
        weight_adj1 = this->weight_factor_fl;
        weight_adj2 = this->weight_factor_br;
    }

    if(leg == "br"){
        pos << this->pos_feet_br.x(), this->pos_feet_br.y(), 0;
        pos_adj1 << this->pos_feet_fr.x(), this->pos_feet_fr.y(), 0;
        pos_adj2 << this->pos_feet_bl.x(), this->pos_feet_bl.y(), 0;

        weight = this->weight_factor_br;
        weight_adj1 = this->weight_factor_fr;
        weight_adj2 = this->weight_factor_bl;
    }

    Eigen::Vector3f virtual1, virtual2;
    virtual1 = pos.operator*(weight) + pos_adj1.operator*(1 - weight);
    virtual2 = pos.operator*(weight) + pos_adj2.operator*(1 - weight);

    float factor = 1/(weight + weight_adj1 + weight_adj2);
    pos_vertice->operator=(   (pos.operator*(weight) + pos_adj1.operator*(weight_adj1) + pos_adj2.operator*(weight_adj2)).operator*(factor)  );
    // pos_vertice->operator=(pos.operator*(weight));
}


void PredPolygonSupport::getDesiredBodyPos(Eigen::Vector3f *body_pos){
    Eigen::Vector3f vertice_fl, vertice_fr, vertice_bl, vertice_br;
    this->getVirtualVertice("fl", &vertice_fl);
    this->getVirtualVertice("fr", &vertice_fr);
    this->getVirtualVertice("bl", &vertice_bl);
    this->getVirtualVertice("br", &vertice_br);

    body_pos->operator=( vertice_fl + vertice_fr + vertice_bl + vertice_br  );
    body_pos->operator*(0.25);
}
