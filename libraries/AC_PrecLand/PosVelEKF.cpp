#include "PosVelEKF.h"
#include <math.h>
#include <string.h>

#define POSVELEKF_POS_CALC_NIS(__P, __R, __X, __Z, __RET_NIS) \
__RET_NIS = ((-__X[0] + __Z)*(-__X[0] + __Z))/(__P[0] + __R);

#define POSVELEKF_POS_CALC_STATE(__P, __R, __X, __Z, __RET_STATE) \
__RET_STATE[0] = __P[0]*(-__X[0] + __Z)/(__P[0] + __R) + __X[0]; __RET_STATE[1] = __P[1]*(-__X[0] + \
__Z)/(__P[0] + __R) + __X[1];

#define POSVELEKF_POS_CALC_COV(__P, __R, __X, __Z, __RET_COV) \
__RET_COV[0] = ((__P[0])*(__P[0]))*__R/((__P[0] + __R)*(__P[0] + __R)) + __P[0]*((-__P[0]/(__P[0] + \
__R) + 1)*(-__P[0]/(__P[0] + __R) + 1)); __RET_COV[1] = __P[0]*__P[1]*__R/((__P[0] + __R)*(__P[0] + \
__R)) - __P[0]*__P[1]*(-__P[0]/(__P[0] + __R) + 1)/(__P[0] + __R) + __P[1]*(-__P[0]/(__P[0] + __R) + \
1); __RET_COV[2] = ((__P[1])*(__P[1]))*__R/((__P[0] + __R)*(__P[0] + __R)) - \
((__P[1])*(__P[1]))/(__P[0] + __R) - __P[1]*(-__P[0]*__P[1]/(__P[0] + __R) + __P[1])/(__P[0] + __R) + \
__P[2];

#define POSVELEKF_PREDICTION_CALC_STATE(__P, __DT, __DV, __DV_NOISE, __X, __RET_STATE) \
__RET_STATE[0] = __DT*__X[1] + __X[0]; __RET_STATE[1] = __DV + __X[1];

#define POSVELEKF_PREDICTION_CALC_COV(__P, __DT, __DV, __DV_NOISE, __X, __RET_COV) \
__RET_COV[0] = __DT*__P[1] + __DT*(__DT*__P[2] + __P[1]) + __P[0]; __RET_COV[1] = __DT*__P[2] + \
__P[1]; __RET_COV[2] = ((__DV_NOISE)*(__DV_NOISE)) + __P[2];

#define POSVELEKF_VEL_CALC_NIS(__P, __R, __X, __Z, __RET_NIS) \
__RET_NIS = ((-__X[1] + __Z)*(-__X[1] + __Z))/(__P[2] + __R);

#define POSVELEKF_VEL_CALC_STATE(__P, __R, __X, __Z, __RET_STATE) \
__RET_STATE[0] = __P[1]*(-__X[1] + __Z)/(__P[2] + __R) + __X[0]; __RET_STATE[1] = __P[2]*(-__X[1] + \
__Z)/(__P[2] + __R) + __X[1];

#define POSVELEKF_VEL_CALC_COV(__P, __R, __X, __Z, __RET_COV) \
__RET_COV[0] = __P[0] + ((__P[1])*(__P[1]))*__R/((__P[2] + __R)*(__P[2] + __R)) - \
((__P[1])*(__P[1]))/(__P[2] + __R) - __P[1]*(-__P[1]*__P[2]/(__P[2] + __R) + __P[1])/(__P[2] + __R); \
__RET_COV[1] = __P[1]*__P[2]*__R/((__P[2] + __R)*(__P[2] + __R)) + (-__P[2]/(__P[2] + __R) + \
1)*(-__P[1]*__P[2]/(__P[2] + __R) + __P[1]); __RET_COV[2] = ((__P[2])*(__P[2]))*__R/((__P[2] + \
__R)*(__P[2] + __R)) + __P[2]*((-__P[2]/(__P[2] + __R) + 1)*(-__P[2]/(__P[2] + __R) + 1));

void PosVelEKF::init(float pos, float posVar, float vel, float velVar)
{
    _state[0] = pos;
    _state[1] = vel;
    _cov[0] = posVar;
    _cov[1] = 0.0f;
    _cov[2] = velVar;
    _cov[3] = 0.0f;
}

// void PosVelEKF::predict(float dt, float dVel, float dVelNoise, float test0, float test1, float test2, float test3)
// {
//     float newState[2];
//     float newCov[4];

//     newState[0] = dt*(_state[1] + dVel) + _state[0];
//     newState[1] = dVel + _state[1];

//     float noise_factor = dt *dVelNoise*dVelNoise;

//     newCov[0] = dt*_cov[3] + dt*(dt*_cov[2] + _cov[1]) + _cov[0] + noise_factor*dt;// + test0;
//     newCov[1] = dt*_cov[2] + _cov[1] + noise_factor;// + test1;
//     newCov[2] = _cov[2] + dVelNoise*dVelNoise;// + test2;
//     newCov[3] = _cov[3] + dt*_cov[2] + noise_factor;// + test3;

//     memcpy(_state,newState,sizeof(_state));
//     memcpy(_cov,newCov,sizeof(_cov));
// }

// void PosVelEKF::fusePos(float pos, float posVar)
// {
//     float newState[2];
//     float newCov[4];

//     const float innovation = _cov[0] + posVar;
//     const float residual = -_state[0] + pos;

//     // newState[0] = _cov[0]*(-_state[0] + pos)/(_cov[0] + posVar) + _state[0];
//     newState[0] = _cov[0]*(residual)/(innovation) + _state[0];
//     newState[1] = _cov[3]*(residual)/(innovation) + _state[1];

//     // this is basically: (cov[0] * posVar)/(cov[0]+posVar) or (cov[0] * posVar)/invoation
//     // newCov[0] = ((_cov[0])*(_cov[0]))*posVar/((_cov[0] + posVar)*(_cov[0] + posVar)) + _cov[0]*((-_cov[0]/(_cov[0] + posVar) + 1)*(-_cov[0]/(_cov[0] + posVar) + 1));
//     newCov[0] = _cov[0] * posVar/innovation;


//     // Cov[1]*posVar/inovation
//     // newCov[1] = _cov[0]*_cov[1]*posVar/((_cov[0] + posVar)*(_cov[0] + posVar)) - _cov[0]*_cov[1]*(-_cov[0]/(_cov[0] + posVar) + 1)/(_cov[0] + posVar) + _cov[1]*(-_cov[0]/(_cov[0] + posVar) + 1);
//     newCov[1] = _cov[1] * posVar/innovation;

//     // -Cov[1]*Cov[1]/inovation + Cov[2]
//     // newCov[2] = ((_cov[1])*(_cov[1]))*posVar/((_cov[0] + posVar)*(_cov[0] + posVar)) - ((_cov[1])*(_cov[1]))/(_cov[0] + posVar) - _cov[1]*(-_cov[0]*_cov[1]/(_cov[0] + posVar) + _cov[1])/(_cov[0] + posVar) + _cov[2];
//     newCov[2] = (-_cov[1]*_cov[3]/innovation) + _cov[2];

//     newCov[3] = _cov[3] * posVar/innovation;

//     memcpy(_state,newState,sizeof(_state));
//     memcpy(_cov,newCov,sizeof(_cov));
// }



void PosVelEKF::predict(float dt, float dVel, float dVelNoise)
{
    float newState[2];
    float newCov[3];

    POSVELEKF_PREDICTION_CALC_STATE(_cov, dt, dVel, dVelNoise, _state, newState)
    newState[0] = dt*_state[1] + _state[0];
    newState[1] = dVel + _state[1];

    POSVELEKF_PREDICTION_CALC_COV(_cov, dt, dVel, dVelNoise, _state, newCov)

    newCov[0] = dt*_cov[1] + dt*(dt*_cov[2] + _cov[1]) + _cov[0]; 
    newCov[1] = dt*_cov[2] + _cov[1]; 
    newCov[2] = ((dVelNoise)*(dVelNoise)) + _cov[2];

    memcpy(_state,newState,sizeof(_state));
    memcpy(_cov,newCov,sizeof(_cov));
}

void PosVelEKF::fusePos(float pos, float posVar)
{
    float newState[2];
    float newCov[3];

    POSVELEKF_POS_CALC_STATE(_cov, posVar, _state, pos, newState)
    POSVELEKF_POS_CALC_COV(_cov, posVar, _state, pos, newCov)
    
    newCov[0] = ((_cov[0])*(_cov[0]))*posVar/((_cov[0] + posVar)*(_cov[0] + posVar)) + _cov[0]*((-_cov[0]/(_cov[0] + posVar) + 1)*(-_cov[0]/(_cov[0] + posVar) + 1)); 
    newCov[1] = _cov[0]*_cov[1]*posVar/((_cov[0] + posVar)*(_cov[0] + posVar)) - _cov[0]*_cov[1]*(-_cov[0]/(_cov[0] + posVar) + 1)/(_cov[0] + posVar) + _cov[1]*(-_cov[0]/(_cov[0] + posVar) + 1); 
    newCov[2] = ((_cov[1])*(_cov[1]))*posVar/((_cov[0] + posVar)*(_cov[0] + posVar)) - ((_cov[1])*(_cov[1]))/(_cov[0] + posVar) - _cov[1]*(-_cov[0]*_cov[1]/(_cov[0] + posVar) + _cov[1])/(_cov[0] + posVar) + _cov[2];
    
    memcpy(_state,newState,sizeof(_state));
    memcpy(_cov,newCov,sizeof(_cov));
}













void PosVelEKF::fuseVel(float vel, float velVar)
{
    float newState[2];
    float newCov[3];

    POSVELEKF_VEL_CALC_STATE(_cov, velVar, _state, vel, newState)
    newState[0] = _cov[1]*(-_state[1] + vel)/(_cov[2] + velVar) + _state[0];
    newState[1] = _cov[2]*(-_state[1] + vel)/(_cov[2] + velVar) + _state[1];

    POSVELEKF_VEL_CALC_COV(_cov, velVar, _state, vel, newCov)

    memcpy(_state,newState,sizeof(_state));
    memcpy(_cov,newCov,sizeof(_cov));
}

float PosVelEKF::getPosNIS(float pos, float posVar)
{
    float ret;

    POSVELEKF_POS_CALC_NIS(_cov, posVar, _state, pos, ret)

    return ret;
}
