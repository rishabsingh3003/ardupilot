#pragma once

class PosVelEKF {
public:
    void init(float pos, float posVar, float vel, float velVar);
    void predict(float dt, float dVel, float dVelNoise);
    void fusePos(float pos, float posVar);
    void fuseVel(float vel, float velVar);

    float getPos() const { return _state[0]; }
    float getVel() const { return _state[1]; }
    float getCovA() const { return _cov[0]; }
    float getCovB() const { return _cov[1]; }
    float getCovC() const { return _cov[2]; }

    float getPosNIS(float pos, float posVar);

private:
    float _state[2];
    float _cov[3];
};

class PosVelEKF_refactor {
public:
    // Initialize the covariance and state matrix
    // This is called when the landing target is located for the first time or it was lost, then relocated
    void init(float pos, float posVar, float vel, float velVar);

    // This functions runs the Prediction Step of the EKF
    // This is called at 400 hz
    void predict(float dt, float dVel, float dVelNoise);

    // fuse the new sensor measurement into the EKF calculations
    // This is called whenever we have a new measurement available
    void fusePos(float pos, float posVar);

    // Get the EKF state position
    float getPos() const { return _state[0]; }

    // Get the EKF state velocity
    float getVel() const { return _state[1]; }

    // get the normalized innovation squared
    float getPosNIS(float pos, float posVar);
    
    float getCovA() const { return _cov[0]; }
    float getCovB() const { return _cov[1]; }
    float getCovC() const { return _cov[2]; }


private:
    // stored covariance and state matrix

    /*
    _state[0] = position
    _state[1] = velocity
    */
    float _state[2];

    /*
    Covariance Matrix is ALWAYS symmetric, therefore the following matrix is assumed:
    P = Covariance Matrix = |_cov[0]  _cov[1]|
                            |_cov[1]  _cov[2]|
    */
    float _cov[3];
};

class PosVelEKF_AB {
public:
    // Initialize the covariance and state matrix
    // This is called when the landing target is located for the first time or it was lost, then relocated
    void init(float pos, float posVar, float vel, float velVar);

    // This functions runs the Prediction Step of the EKF
    // This is called at 400 hz
    void predict(float dt, float dVel, float dVelNoise);

    // fuse the new sensor measurement into the EKF calculations
    // This is called whenever we have a new measurement available
    void fusePos(float pos, float posVar);

    // Get the EKF state position
    float getPos() const { return _state[0]; }

    // Get the EKF state velocity
    float getVel() const { return _state[1]; }

    // get the normalized innovation squared
    float getPosNIS(float pos, float posVar);

    float getCovA() const { return _cov[0]; }
    float getCovB() const { return _cov[1]; }
    float getCovC() const { return _cov[2]; }
    float getCovD() const { return _cov[3]; }
    float getCovE() const { return _cov[4]; }
    float getCovF() const { return _cov[5]; }



private:
    // stored covariance and state matrix

    /*
    _state[0] = position
    _state[1] = velocity
    _state[2] = accel bias
    */
    float _state[3];

    /*
    Covariance Matrix is ALWAYS symmetric, therefore the following matrix is assumed:
    P = Covariance Matrix = |_cov[0]  _cov[1] _cov[2]|
                            |_cov[1]  _cov[3] _cov[4]|
                            |_cov[2]  _cov[4] _cov[5]|
    */
    float _cov[6];
};

