/*
 * Extended Kalman Filter for embedded processors
 *
 * Copyright (C) 2024 Simon D. Levy
 *
 * MIT License
 */

#include <math.h>
#include <stdbool.h>
#include <string.h>

#ifndef EKF_H_
#define EKF_H_

/**
  * Floating-point precision defaults to single but can be made double via
    <tt><b>#define _float_t double</b></tt> before <tt>#include <tinyekf.h></tt>
  */
#ifndef _float_t
#define _float_t float
#endif


typedef struct 
{
    /** State vector **/
    _float_t* x;

    /** Prediction error covariance **/
    _float_t* P;
} ekf_t;

class EKF {
public:
    // EKF();
    EKF(int n, int m);
    virtual ~EKF();

    void ekf_initialize(ekf_t* ekf, const _float_t* pdiag);
    void ekf_predict(ekf_t* ekf, const _float_t* fx, const _float_t* F, const _float_t* Q);
    void ekf_update_step3(ekf_t* ekf, _float_t* GH);
    bool ekf_update(ekf_t* ekf, const _float_t* z, const _float_t* hx, const _float_t* H, const _float_t* R);



private:
    int EKF_N;
    int EKF_M;

    void _mulmat(const _float_t * a, const _float_t * b, _float_t * c, const int arows, const int acols, const int bcols);
    void _mulvec(const _float_t * a, const _float_t * x, _float_t * y, const int m, const int n);
    void _transpose(const _float_t * a, _float_t * at, const int m, const int n);
    void _addmat(const _float_t * a, const _float_t * b, _float_t * c, const int m, const int n);
    void _negate(_float_t * a, const int m, const int n);
    void _addeye(_float_t * a, const int n);
    
    int _choldc1(_float_t * a, _float_t * p, const int n);
    int _choldcsl(const _float_t * A, _float_t * a, _float_t * p, const int n);
    int _cholsl(const _float_t * A, _float_t * a, _float_t * p, const int n);
    void _addvec(const _float_t * a, const _float_t * b, _float_t * c, const int n);
    void _sub(const _float_t * a, const _float_t * b, _float_t * c, const int n);
    bool invert(const _float_t * a, _float_t * ainv);
};

#endif /* EKF_H_ */