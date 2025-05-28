/*********************************************************
* Fichier : embase3pap.h                                *
* Description : Prototypes des fonctions de gestion     *
* des mouvements de la base à trois moteurs             *
* (affectations, déplacement, vitesse, trimoteur, etc.) *
**********************************************************/

#ifndef EMBASE3PAP_H
#define EMBASE3PAP_H

void affectations_moteurs();
void ordre_trimoteur(int num_mvt);
void trimoteur_vit(int vx, int vy, int vc);
void trimoteur_dep(int vx, int vy, int vc, uint16_t vitesse_max);

#endif