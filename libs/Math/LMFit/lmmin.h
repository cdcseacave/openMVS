/*
 * Project:  LevenbergMarquardtLeastSquaresFitting
 *
 * File:     lmmin.h
 */
 
#ifndef LMMIN_H
#define LMMIN_H


/** Compact high-level interface. **/

/* Collection of control (input) parameters. */
typedef struct {
    double ftol;      /* relative error desired in the sum of squares. */
    double xtol;      /* relative error between last two approximations. */
    double gtol;      /* orthogonality desired between fvec and its derivs. */
    double epsilon;   /* step used to calculate the jacobian, or 0 for user provided jacobian. */
    double stepbound; /* initial bound to steps in the outer loop. */
    int maxcall;      /* maximum number of iterations. */
    int scale_diag;   /* UNDOCUMENTED, TESTWISE automatical diag rescaling? */
    #ifdef LMFIT_PRINTOUT
    int printflags;   /* OR'ed to produce more noise */
    #endif
} lm_control_struct;

/* Collection of status (output) parameters. */
typedef struct {
    double fnorm;     /* norm of the residue vector fvec. */
    int nfev;         /* actual number of iterations. */
    int info;         /* status (index for lm_infmsg and lm_shortmsg). */
} lm_status_struct;

/* Recommended control parameter settings. */
extern const lm_control_struct lm_control_double;
extern const lm_control_struct lm_control_float;

#ifdef LMFIT_PRINTOUT
/* Standard monitoring routine. */
void lm_printout_std( int n_par, const double *par, int m_dat,
                      const void *data, const double *fvec,
                      int printflags, int iflag, int iter, int nfev );
#endif

/* Refined calculation of Eucledian norm, typically used in printout routine. */
double lm_enorm( int, const double * );

/* The actual minimization. */
void lmmin( int n_par, double *par, int m_dat, const void *data, 
            void (*evaluate) (const double *par, int m_dat, const void *data,
                              double *fvec, double *fjac, int *info),
            const lm_control_struct *control, lm_status_struct *status
            #ifdef LMFIT_PRINTOUT
            , void (*printout) (int n_par, const double *par, int m_dat,
                                const void *data, const double *fvec,
                                int printflags, int iflag, int iter, int nfev) = NULL
            #endif
          );


/** Legacy low-level interface. **/

/* Alternative to lm_minimize, allowing full control, and read-out
   of auxiliary arrays. For usage, see implementation of lmmin. */
void lm_lmdif( int m, int n, double *x, double *fvec, double ftol,
               double xtol, double gtol, int maxfev, double epsfcn,
               double *diag, int mode, double factor, int& info, int& nfev,
               double *fjac, int *ipvt, double *qtf, double *wa1,
               double *wa2, double *wa3, double *wa4,
               void (*evaluate) (const double *par, int m_dat, const void *data,
                                 double *fvec, double *fjac, int *info),
               const void *data
               #ifdef LMFIT_PRINTOUT
               , void (*printout) (int n_par, const double *par, int m_dat,
                                   const void *data, const double *fvec,
                                   int printflags, int iflag, int iter, int nfev),
               int printflags
               #endif
             );

extern const char *lm_infmsg[];
extern const char *lm_shortmsg[];


#endif /* LMMIN_H */
