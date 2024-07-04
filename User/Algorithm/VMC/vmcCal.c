#include "vmcCal.h"


#define pi 3.1415926f

void VMC_init(vmc_t *vmc){
    vmc->l5 = 0.15f;
    vmc->l1 = 0.12f;
    vmc->l2 = 0.24f;
    vmc->l3 = 0.24f;
    vmc->l4 = 0.12f;
}

void VMC_R_cal1(vmc_t *vmc, INS_t *ins, float dt){
    static float pitch = 0.0f;
    static float pitchGyro = 0.0f;
    pitch = ins->Pitch;
    pitchGyro = ins->Gyro[1];

    vmc->yD = vmc->l4 * arm_sin_f32(vmc->phi4);
    vmc->yB = vmc->l1 * arm_sin_f32(vmc->phi1);
    vmc->xD = vmc->l5 + vmc->l4 * arm_cos_f32(vmc->phi4);
    vmc->xB = vmc->l1 * arm_cos_f32(vmc->phi1);

    vmc->d_yD = vmc->l4 * vmc->d_phi4 * arm_cos_f32(vmc->phi4);
    vmc->d_yB = vmc->l1 * vmc->d_phi1 * arm_cos_f32(vmc->phi1);
    vmc->d_xD = -vmc->l4 * vmc->d_phi4 * arm_sin_f32(vmc->phi4);
    vmc->d_xB = -vmc->l1 * vmc->d_phi1 * arm_sin_f32(vmc->phi1);

    vmc->lBD = sqrtf((vmc->xD - vmc->xB) * (vmc->xD - vmc->xB) + (vmc->yD - vmc->yB) * (vmc->yD - vmc->yB));

    vmc->A0 = 2 * vmc->l2 * (vmc->xD - vmc->xB);
    vmc->B0 = 2 * vmc->l2 * (vmc->yD - vmc->yB);
    vmc->C0 = vmc->l2 * vmc->l2 - vmc->l3 * vmc->l3 + vmc->lBD * vmc->lBD;
    vmc->phi2 = 2 * atan2f((vmc->B0 + sqrt(vmc->A0 * vmc->A0 + vmc->B0 * vmc->B0 - vmc->C0 * vmc->C0)), vmc->A0 + vmc->C0);
    vmc->phi3 = atan2f(vmc->yB - vmc->yD + vmc->l2 * arm_sin_f32(vmc->phi2), vmc->xB - vmc->xD + vmc->l2 * arm_cos_f32(vmc->phi2));
    
    vmc->d_phi2 = ((vmc->d_xD - vmc->d_xB)*arm_cos_f32(vmc->phi3) + (vmc->d_yD - vmc->d_yB)*arm_sin_f32(vmc->phi3))/(vmc->l2*arm_sin_f32(vmc->phi3 - vmc->phi2));

    vmc->xC = vmc->l1 * arm_cos_f32(vmc->phi1) + vmc->l2 * arm_cos_f32(vmc->phi2);
    vmc->yC = vmc->l1 * arm_sin_f32(vmc->phi1) + vmc->l2 * arm_sin_f32(vmc->phi2);

    vmc->d_xc = -vmc->l1 * arm_sin_f32(vmc->phi1) * vmc->d_phi1 - vmc->l2 * arm_sin_f32(vmc->phi2) * vmc->d_phi2;
    vmc->d_yc = vmc->l1 * arm_cos_f32(vmc->phi1) * vmc->d_phi1 + vmc->l2 * arm_cos_f32(vmc->phi2) * vmc->d_phi2;

    vmc->L0 = sqrt((vmc->xC - vmc->l5/2.0f)*(vmc->xC - vmc->l5/2.0f) + vmc->yC*vmc->yC);
    vmc->phi0 = atan2f(vmc->yC, vmc->xC - vmc->l5/2.0f);

    if(vmc->first_flag == 0){
        vmc->last_phi0 = vmc->phi0;
        vmc->first_flag = 1;
    }

//    vmc->d_phi0 = (vmc->phi0 - vmc->last_phi0) / dt;
    vmc->d_phi0 = -(vmc->d_xc*vmc->yC - vmc->d_yc*(vmc->xC - vmc->l5/2.0f))/(vmc->L0*vmc->L0);
    vmc->d_alpha = 0.0f - vmc->d_phi0;

    vmc->theta = -pi/2.0f - pitch + vmc->phi0;
		//vmc->theta = pi/2.0f + pitch - vmc->phi0;
    vmc->d_theta = pitchGyro + vmc->d_phi0;
//		vmc->d_theta = pitchGyro - vmc->d_phi0;

    vmc->last_phi0 = vmc->phi0;

    vmc->d_L0 = (vmc->L0 - vmc->last_L0) / dt;
    vmc->d2_L0 = (vmc->d_L0 - vmc->last_d_L0) / dt;

    vmc->last_L0 = vmc->L0;
    vmc->last_d_L0 = vmc->d_L0;

    vmc->d2_theta = (vmc->d_theta - vmc->last_d_theta) / dt;
    vmc->last_d_theta = vmc->d_theta;
}

void VMC_L_cal1(vmc_t *vmc, INS_t *ins, float dt){
    static float pitch = 0.0f;
    static float pitchGyro = 0.0f;
    pitch = ins->Pitch;
    pitchGyro = ins->Gyro[1];

    vmc->yD = vmc->l4 * arm_sin_f32(vmc->phi4);
    vmc->yB = vmc->l1 * arm_sin_f32(vmc->phi1);
    vmc->xD = vmc->l5 + vmc->l4 * arm_cos_f32(vmc->phi4);
    vmc->xB = vmc->l1 * arm_cos_f32(vmc->phi1);

    vmc->d_yD = vmc->l4 * vmc->d_phi4 * arm_cos_f32(vmc->phi4);
    vmc->d_yB = vmc->l1 * vmc->d_phi1 * arm_cos_f32(vmc->phi1);
    vmc->d_xD = -vmc->l4 * vmc->d_phi4 * arm_sin_f32(vmc->phi4);
    vmc->d_xB = -vmc->l1 * vmc->d_phi1 * arm_sin_f32(vmc->phi1);

    vmc->lBD = sqrtf((vmc->xD - vmc->xB) * (vmc->xD - vmc->xB) + (vmc->yD - vmc->yB) * (vmc->yD - vmc->yB));

    vmc->A0 = 2 * vmc->l2 * (vmc->xD - vmc->xB);
    vmc->B0 = 2 * vmc->l2 * (vmc->yD - vmc->yB);
    vmc->C0 = vmc->l2 * vmc->l2 - vmc->l3 * vmc->l3 + vmc->lBD * vmc->lBD;
    vmc->phi2 = 2 * atan2f((vmc->B0 + sqrt(vmc->A0 * vmc->A0 + vmc->B0 * vmc->B0 - vmc->C0 * vmc->C0)), vmc->A0 + vmc->C0);
    vmc->phi3 = atan2f(vmc->yB - vmc->yD + vmc->l2 * arm_sin_f32(vmc->phi2), vmc->xB - vmc->xD + vmc->l2 * arm_cos_f32(vmc->phi2));
    
    vmc->d_phi2 = ((vmc->d_xD - vmc->d_xB)*arm_cos_f32(vmc->phi3) + (vmc->d_yD - vmc->d_yB)*arm_sin_f32(vmc->phi3))/(vmc->l2*arm_sin_f32(vmc->phi3 - vmc->phi2));

    vmc->xC = vmc->l1 * arm_cos_f32(vmc->phi1) + vmc->l2 * arm_cos_f32(vmc->phi2);
    vmc->yC = vmc->l1 * arm_sin_f32(vmc->phi1) + vmc->l2 * arm_sin_f32(vmc->phi2);

    vmc->d_xc = -vmc->l1 * arm_sin_f32(vmc->phi1) * vmc->d_phi1 - vmc->l2 * arm_sin_f32(vmc->phi2) * vmc->d_phi2;
    vmc->d_yc = vmc->l1 * arm_cos_f32(vmc->phi1) * vmc->d_phi1 + vmc->l2 * arm_cos_f32(vmc->phi2) * vmc->d_phi2;

    vmc->L0 = sqrt((vmc->xC - vmc->l5/2.0f)*(vmc->xC - vmc->l5/2.0f) + vmc->yC*vmc->yC);
    vmc->phi0 = atan2f(vmc->yC, vmc->xC - vmc->l5/2.0f);

    if(vmc->first_flag == 0){
        vmc->last_phi0 = vmc->phi0;
        vmc->first_flag = 1;
    }

    //vmc->d_phi0 = (vmc->phi0 - vmc->last_phi0) / dt;
    vmc->d_phi0 = -(vmc->d_xc*vmc->yC - vmc->d_yc*(vmc->xC - vmc->l5/2.0f))/(vmc->L0*vmc->L0);
    vmc->d_alpha = 0.0f - vmc->d_phi0;

    vmc->theta = -pi/2.0f - pitch + vmc->phi0;
		//vmc->theta = pi/2.0f + pitch - vmc->phi0;
    vmc->d_theta = pitchGyro + vmc->d_phi0;
//		vmc->d_theta = pitchGyro - vmc->d_phi0;

    vmc->last_phi0 = vmc->phi0;

    vmc->d_L0 = (vmc->L0 - vmc->last_L0) / dt;
    vmc->d2_L0 = (vmc->d_L0 - vmc->last_d_L0) / dt;

    vmc->last_L0 = vmc->L0;
    vmc->last_d_L0 = vmc->d_L0;

    vmc->d2_theta = (vmc->d_theta - vmc->last_d_theta) / dt;
    vmc->last_d_theta = vmc->d_theta;
}

void VMC_cal2(vmc_t *vmc){
    vmc->J11 = (vmc->l1*arm_sin_f32(vmc->phi0-vmc->phi3)*arm_sin_f32(vmc->phi1-vmc->phi2))/arm_sin_f32(vmc->phi3-vmc->phi2);
	vmc->J12 = (vmc->l1*arm_cos_f32(vmc->phi0-vmc->phi3)*arm_sin_f32(vmc->phi1-vmc->phi2))/(vmc->L0*arm_sin_f32(vmc->phi3-vmc->phi2));
	vmc->J21 = (vmc->l4*arm_sin_f32(vmc->phi0-vmc->phi2)*arm_sin_f32(vmc->phi3-vmc->phi4))/arm_sin_f32(vmc->phi3-vmc->phi2);
	vmc->J22 = (vmc->l4*arm_cos_f32(vmc->phi0-vmc->phi2)*arm_sin_f32(vmc->phi3-vmc->phi4))/(vmc->L0*arm_sin_f32(vmc->phi3-vmc->phi2));

    vmc->T_set[0] = vmc->J11 * vmc->F0 + vmc->J12 * vmc->Tp;
    vmc->T_set[1] = vmc->J21 * vmc->F0 + vmc->J22 * vmc->Tp;
}