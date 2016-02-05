// modified version of:
/*
 * Project:  LevenbergMarquardtLeastSquaresFitting
 *
 * File:     lmmin.c
 *
 * Contents: Levenberg-Marquardt core implementation,
 *           and simplified user interface.
 *
 * Author:   Joachim Wuttke <j.wuttke@fz-juelich.de>
 *
 * Acknowledgments:
 *           This software has been build on top of public domain work
 *           by Burton S. Garbow, Kenneth E. Hillstrom, Jorge J. Moré,
 *           Steve Moshier, and the authors of lapack.
 *           Over the years, it has been much improved thanks to
 *           feedback from numerous users. See ../CHANGELOG.
 *
 * Licence:  see ../COPYING (FreeBSD)
 *
 * Homepage: joachimwuttke.de/lmfit
 */

#include "Common.h"
#include "lmmin.h"

using namespace SEACAVE;


/*****************************************************************************/
/*  set numeric constants                                                    */
/*****************************************************************************/

/* machine-dependent constants from float.h */
#define LM_MACHEP     DBL_EPSILON   /* resolution of arithmetic */
#define LM_DWARF      DBL_MIN       /* smallest nonzero number */
#define LM_SQRT_DWARF sqrt(DBL_MIN) /* square should not underflow */
#define LM_SQRT_GIANT sqrt(DBL_MAX) /* square should not overflow */
#define LM_USERTOL    30*LM_MACHEP  /* users are recommended to require this */

/* If the above values do not work, the following seem good for an x86:
 LM_MACHEP     .555e-16
 LM_DWARF      9.9e-324
 LM_SQRT_DWARF 1.e-160
 LM_SQRT_GIANT 1.e150
 LM_USER_TOL   1.e-14
   The following values should work on any machine:
 LM_MACHEP     1.2e-16
 LM_DWARF      1.0e-38
 LM_SQRT_DWARF 3.834e-20
 LM_SQRT_GIANT 1.304e19
 LM_USER_TOL   1.e-14
*/

const lm_control_struct lm_control_double = {
    LM_USERTOL, LM_USERTOL, LM_USERTOL, LM_USERTOL, 100., 100, 1
    #ifdef LMFIT_PRINTOUT
    , 0
    #endif
};
const lm_control_struct lm_control_float = {
    1.e-7, 1.e-7, 1.e-7, 1.e-7, 100., 100, 0
    #ifdef LMFIT_PRINTOUT
    , 0
    #endif
};


/*****************************************************************************/
/*  set message texts (indexed by status.info)                               */
/*****************************************************************************/

const char *lm_infmsg[] = {
    "success (sum of squares below underflow limit)",
    "success (the relative error in the sum of squares is at most tol)",
    "success (the relative error between x and the solution is at most tol)",
    "success (both errors are at most tol)",
    "trapped by degeneracy (increasing epsilon might help)",
    "timeout (number of calls to fcn has reached maxcall*(n+1))",
    "failure (ftol<tol: cannot reduce sum of squares any further)",
    "failure (xtol<tol: cannot improve approximate solution any further)",
    "failure (gtol<tol: cannot improve approximate solution any further)",
    "exception (not enough memory)",
    "fatal coding error (improper input parameters)",
    "exception (break requested within function evaluation)"
};

const char *lm_shortmsg[] = {
    "success (0)",
    "success (f)",
    "success (p)",
    "success (f,p)",
    "degenerate",
    "call limit",
    "failed (f)",
    "failed (p)",
    "failed (o)",
    "no memory",
    "invalid input",
    "user break"
};


#ifdef LMFIT_PRINTOUT
/*****************************************************************************/
/*  lm_printout_std (default monitoring routine)                             */
/*****************************************************************************/

void lm_printout_std( int n_par, const double *par, int m_dat,
                      const void *data, const double *fvec,
                      int printflags, int iflag, int iter, int nfev)
/*
 *       data  : for soft control of printout behaviour, add control
 *                 variables to the data struct
 *       iflag : 0 (init) 1 (outer loop) 2(inner loop) -1(terminated)
 *       iter  : outer loop counter
 *       nfev  : number of calls to *evaluate
 */
{
    int i;

    if( !printflags )
        return;

    if( printflags & 1 ){
        /* location of printout call within lmdif */
        if (iflag == 2) {
            printf("trying step in gradient direction  ");
        } else if (iflag == 1) {
            printf("determining gradient (iteration %2d)", iter);
        } else if (iflag == 0) {
            printf("starting minimization              ");
        } else if (iflag == -1) {
            printf("terminated after %3d evaluations   ", nfev);
        }
    }

    if( printflags & 2 ){
        printf("  par: ");
        for (i = 0; i < n_par; ++i)
            printf(" %18.11g", par[i]);
        printf(" => norm: %18.11g", lm_enorm(m_dat, fvec));
    }

    if( printflags & 3 )
        printf( "\n" );

    if ( (printflags & 8) || ((printflags & 4) && iflag == -1) ) {
        printf("  residuals:\n");
        for (i = 0; i < m_dat; ++i)
            printf("    fvec[%2d]=%12g\n", i, fvec[i] );
    }
}
#endif


/*****************************************************************************/
/*  lm_minimize (intermediate-level interface)                               */
/*****************************************************************************/
void lmmin( int n_par, double *par, int m_dat, const void *data,
            void (*evaluate) (const double *par, int m_dat, const void *data,
                              double *fvec, double *fjac, int *info),
            const lm_control_struct *control, lm_status_struct *status
            #ifdef LMFIT_PRINTOUT
            , void (*printout) (int n_par, const double *par, int m_dat,
                              const void *data, const double *fvec,
                              int printflags, int iflag, int iter, int nfev)
            #endif
          )
{

/*** allocate work space. ***/

    double *fvec, *diag, *fjac, *qtf, *wa1, *wa2, *wa3, *wa4;
    int *ipvt;

    int n = n_par;
    int m = m_dat;

    /* One malloc call to allocate several arrays (Frank Polchow, 2013) */
    fvec = (double*)malloc((2*m+5*n+n*m)*sizeof(double) + n*sizeof(int));
    if (NULL==fvec) {//fail in allocation
        status->info = 9;
        return;
    }
    diag = (double *) &fvec[m];
    qtf  = (double *) &diag[n];
    fjac = (double *) &qtf[n];
    wa1  = (double *) &fjac[n*m];
    wa2  = (double *) &wa1[n];
    wa3  = (double *) &wa2[n];
    wa4  = (double *) &wa3[n];
    ipvt = (int    *) &wa4[m];

/*** perform fit. ***/

    status->info = 0;

    /* this goes through the modified legacy interface: */
    lm_lmdif(
        m, n, par, fvec, control->ftol, control->xtol, control->gtol,
        control->maxcall, control->epsilon, (control->scale_diag ? diag : NULL),
        (control->scale_diag ? 1 : 2), control->stepbound, status->info,
        status->nfev, fjac, ipvt, qtf, wa1, wa2, wa3, wa4,
        evaluate, data
        #ifdef LMFIT_PRINTOUT
        , printout, control->printflags
        #endif
    );

    #ifdef LMFIT_PRINTOUT
    if ( printout )
        (*printout)( n, par, m, data, fvec, control->printflags, -1, 0, status->nfev );
    #endif
    status->fnorm = lm_enorm(m, fvec);
    if ( status->info < 0 )
        status->info = 11;

/*** clean up. ***/
    free(fvec);

} /*** lmmin. ***/


/*****************************************************************************/
/*  lm_lmdif (low-level, modified legacy interface for full control)         */
/*****************************************************************************/

void lm_lmpar( int n, double *r, int ldr, int *ipvt, double *diag,
               double *qtb, double delta, double *par, double *x,
               double *sdiag, double *aux, double *xdi );
void lm_qrfac( int m, int n, double *a, int pivot, int *ipvt,
               double *rdiag, double *acnorm, double *wa );
void lm_qrsolv( int n, double *r, int ldr, int *ipvt, double *diag,
                double *qtb, double *x, double *sdiag, double *wa );

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
             )
{
/*
 *   The purpose of lmdif is to minimize the sum of the squares of
 *   m nonlinear functions in n variables by a modification of
 *   the levenberg-marquardt algorithm. The user must provide a
 *   subroutine evaluate which calculates the functions. The jacobian
 *   is then calculated by a forward-difference approximation.
 *
 *   The multi-parameter interface lm_lmdif is for users who want
 *   full control and flexibility. Most users will be better off using
 *   the simpler interface lmmin provided above.
 *
 *   Parameters:
 *
 *      m is a positive integer input variable set to the number
 *        of functions.
 *
 *      n is a positive integer input variable set to the number
 *        of variables; n must not exceed m.
 *
 *      x is an array of length n. On input x must contain an initial
 *        estimate of the solution vector. On OUTPUT x contains the
 *        final estimate of the solution vector.
 *
 *      fvec is an OUTPUT array of length m which contains
 *        the functions evaluated at the output x.
 *
 *      ftol is a nonnegative input variable. Termination occurs when
 *        both the actual and predicted relative reductions in the sum
 *        of squares are at most ftol. Therefore, ftol measures the
 *        relative error desired in the sum of squares.
 *
 *      xtol is a nonnegative input variable. Termination occurs when
 *        the relative error between two consecutive iterates is at
 *        most xtol. Therefore, xtol measures the relative error desired
 *        in the approximate solution.
 *
 *      gtol is a nonnegative input variable. Termination occurs when
 *        the cosine of the angle between fvec and any column of the
 *        jacobian is at most gtol in absolute value. Therefore, gtol
 *        measures the orthogonality desired between the function vector
 *        and the columns of the jacobian.
 *
 *      maxfev is a positive integer input variable. Termination
 *        occurs when the number of calls to lm_fcn is at least
 *        maxfev by the end of an iteration.
 *
 *      epsfcn is an input variable used in choosing a step length for
 *        the forward-difference approximation. The relative errors in
 *        the functions are assumed to be of the order of epsfcn.
 *
 *      diag is an array of length n. If mode = 1 (see below), diag is
 *        internally set. If mode = 2, diag must contain positive entries
 *        that serve as multiplicative scale factors for the variables.
 *
 *      mode is an integer input variable. If mode = 1, the
 *        variables will be scaled internally. If mode = 2,
 *        the scaling is specified by the input diag.
 *
 *      factor is a positive input variable used in determining the
 *        initial step bound. This bound is set to the product of
 *        factor and the euclidean norm of diag*x if nonzero, or else
 *        to factor itself. In most cases factor should lie in the
 *        interval (0.1,100.0). Generally, the value 100.0 is recommended.
 *
 *      info is an integer OUTPUT variable that indicates the termination
 *        status of lm_lmdif as follows:
 *
 *        info < 0  termination requested by user-supplied routine *evaluate;
 *
 *        info = 0  fnorm almost vanishing;
 *
 *        info = 1  both actual and predicted relative reductions
 *                  in the sum of squares are at most ftol;
 *
 *        info = 2  relative error between two consecutive iterates
 *                  is at most xtol;
 *
 *        info = 3  conditions for info = 1 and info = 2 both hold;
 *
 *        info = 4  the cosine of the angle between fvec and any
 *                  column of the jacobian is at most gtol in
 *                  absolute value;
 *
 *        info = 5  number of calls to lm_fcn has reached or
 *                  exceeded maxfev;
 *
 *        info = 6  ftol is too small: no further reduction in
 *                  the sum of squares is possible;
 *
 *        info = 7  xtol is too small: no further improvement in
 *                  the approximate solution x is possible;
 *
 *        info = 8  gtol is too small: fvec is orthogonal to the
 *                  columns of the jacobian to machine precision;
 *
 *        info =10  improper input parameters;
 *
 *      nfev is an OUTPUT variable set to the number of calls to the
 *        user-supplied routine *evaluate.
 *
 *      fjac is an OUTPUT m by n array. The upper n by n submatrix
 *        of fjac contains an upper triangular matrix r with
 *        diagonal elements of nonincreasing magnitude such that
 *
 *              pT*(jacT*jac)*p = rT*r
 *
 *              (NOTE: T stands for matrix transposition),
 *
 *        where p is a permutation matrix and jac is the final
 *        calculated jacobian. Column j of p is column ipvt(j)
 *        (see below) of the identity matrix. The lower trapezoidal
 *        part of fjac contains information generated during
 *        the computation of r.
 *
 *      ipvt is an integer OUTPUT array of length n. It defines a
 *        permutation matrix p such that jac*p = q*r, where jac is
 *        the final calculated jacobian, q is orthogonal (not stored),
 *        and r is upper triangular with diagonal elements of
 *        nonincreasing magnitude. Column j of p is column ipvt(j)
 *        of the identity matrix.
 *
 *      qtf is an OUTPUT array of length n which contains
 *        the first n elements of the vector (q transpose)*fvec.
 *
 *      wa1, wa2, and wa3 are work arrays of length n.
 *
 *      wa4 is a work array of length m, used among others to hold
 *        residuals from evaluate.
 *
 *      evaluate points to the subroutine which calculates the
 *        m nonlinear functions. Implementations should be written as follows:
 *
 *        void evaluate( double* par, int m_dat, void *data,
 *                       double* fvec, double *fjac, int *info )
 *        {
 *           // for ( i=0; i<m_dat; ++i )
 *           //     calculate fvec[i] for given parameters par;
 *           // to stop the minimization,
 *           //     set *info to a negative integer.
 *        }
 *
 *      printout points to the subroutine which informs about fit progress.
 *        Call with printout=0 if no printout is desired.
 *        Call with printout=lm_printout_std to use the default implementation.
 *
 *      printflags is passed to printout.
 *
 *      data is an input pointer to an arbitrary structure that is passed to
 *        evaluate. Typically, it contains experimental data to be fitted.
 *
 */
    int i, j;
    double actred, delta, dirder, eps, fnorm, fnorm1, gnorm, par, pnorm,
        prered, ratio, step, sum, temp1, temp2, temp3, xnorm;
    static const double p1 = 0.1;
    static const double p0001 = 1.0e-4;

    #ifdef LMFIT_PRINTOUT
    int iter = 0;/* outer loop counter */
    #endif
    nfev = 0;    /* function evaluation counter */
    par = 0;     /* Levenberg-Marquardt parameter */
    delta = 0;   /* to prevent a warning (initialization within if-clause) */
    xnorm = 0;   /* ditto */

/*** lmdif: check input parameters for errors. ***/

    if ((n <= 0) || (m < n) || (ftol < 0.) || (xtol < 0.) || (gtol < 0.) || (maxfev <= 0) || (factor <= 0.)) {
        info = 10; // invalid parameter
        return;
    }
    #ifdef LMFIT_DEBUG_MESSAGES
    printf("lmdif\n");
    #endif

/*** lmdif: the outer loop. ***/

    do {

        info = 0;

        if (epsfcn == 0) {

            if (nfev == 0) {

/*** outer: evaluate function at starting point, calculate norm, and get user provided Jacobian. ***/

                (*evaluate) (x, m, data, fvec, fjac, &info);
                fnorm = lm_enorm(m, fvec);
                if (fnorm <= LM_DWARF) {
                    info = 0;
                    return;
                }

            } else {

/*** outer: get user provided Jacobian. ***/

                (*evaluate) (x, m, data, NULL, fjac, &info);

            }

            #ifdef LMFIT_PRINTOUT
            if( printout )
                (*printout) (n, x, m, data, fvec, printflags, 1, iter, nfev);
            #endif
            if (info < 0)
                return; /* user requested break */

        } else {

            if (nfev == 0) {

/*** outer: evaluate function at starting point and calculate norm. ***/

                (*evaluate) (x, m, data, fvec, NULL, &info);
                #ifdef LMFIT_PRINTOUT
                if( printout )
                    (*printout) (n, x, m, data, fvec, printflags, 0, 0, nfev);
                #endif
                if (info < 0)
                    return;
                fnorm = lm_enorm(m, fvec);
                if( fnorm <= LM_DWARF ){
                    info = 0;
                    return;
                }

                eps = sqrt(MAXF(epsfcn,LM_MACHEP)); /* for calculating the Jacobian by forward differences */
            }

/*** outer: calculate the Jacobian. ***/

            for (j = 0; j < n; ++j) {
                const double temp = x[j];
                step = eps*MAXF(eps,ABS(temp));
                x[j] = temp + step; /* replace temporarily */
                info = 0;
                (*evaluate) (x, m, data, wa4, NULL, &info);
                #ifdef LMFIT_PRINTOUT
                if( printout )
                    (*printout) (n, x, m, data, wa4, printflags, 1, iter, nfev);
                #endif
                if (info < 0)
                    return; /* user requested break */
				step = x[j] - temp; /* due to float point arithmetic errors most of the time this is not equal to step */
				for (i = 0; i < m; ++i)
                    fjac[j*m+i] = (wa4[i] - fvec[i]) / step;
                x[j] = temp; /* restore */
            }

        }
        #ifdef LMFIT_DEBUG_MESSAGES
        printf("lmdif/ outer loop iter=%d nfev=%d fnorm=%.10e\n", iter, nfev, fnorm);
        #endif
        #ifdef LMFIT_DEBUG_MATRIX
        /* print the entire matrix */
        for (i = 0; i < m; i++) {
            for (j = 0; j < n; j++)
                printf("%.5e ", fjac[j*m+i]);
            printf("\n");
        }
        #endif

/*** outer: compute the qr factorization of the Jacobian. ***/

        lm_qrfac(m, n, fjac, 1, ipvt, wa1, wa2, wa3);
        /* return values are ipvt, wa1=rdiag, wa2=acnorm */

        if (nfev == 0) {
            /* first iteration only */
            if (mode != 2) {
                /* diag := norms of the columns of the initial Jacobian */
                for (j = 0; j < n; j++) {
                    diag[j] = wa2[j];
                    if (wa2[j] == 0.)
                        diag[j] = 1.;
                }
                /* use diag to scale x, then calculate the norm */
                for (j = 0; j < n; j++)
                    wa3[j] = diag[j] * x[j];
                xnorm = lm_enorm(n, wa3);
            } else {
                /* calculate the norm */
                xnorm = lm_enorm(n, x);
            }
            /* initialize the step bound delta. */
            delta = factor * xnorm;
            if (delta == 0.)
                delta = factor;
        } else {
            if (mode != 2) {
                for (j = 0; j < n; j++)
                    diag[j] = MAXF( diag[j], wa2[j] );
            }
        }

/*** outer: form (q transpose)*fvec and store first n components in qtf. ***/

        memcpy(wa4, fvec, sizeof(double)*m);

        for (j = 0; j < n; j++) {
            temp3 = fjac[j*m+j];
            if (temp3 != 0.) {
                sum = 0;
                for (i = j; i < m; i++)
                    sum += fjac[j*m+i] * wa4[i];
                double temp = -sum / temp3;
                for (i = j; i < m; i++)
                    wa4[i] += fjac[j*m+i] * temp;
            }
            fjac[j*m+j] = wa1[j];
            qtf[j] = wa4[j];
        }

/*** outer: compute norm of scaled gradient and test for convergence. ***/

        gnorm = 0;
        for (j = 0; j < n; j++) {
            if (wa2[ipvt[j]] == 0)
                continue;
            sum = 0.;
            for (i = 0; i <= j; i++)
                sum += fjac[j*m+i] * qtf[i];
            gnorm = MAXF( gnorm, fabs( sum / wa2[ipvt[j]] / fnorm ) );
        }

        if (gnorm <= gtol) {
            info = 4;
            return;
        }

/*** the inner loop. ***/
        do {
            #ifdef LMFIT_DEBUG_MESSAGES
            printf("lmdif/ inner loop iter=%d nfev=%d\n", iter, nfev);
            #endif

/*** inner: determine the levenberg-marquardt parameter. ***/

            lm_lmpar( n, fjac, m, ipvt, diag, qtf, delta, &par, wa1, wa2, wa4, wa3 );
            /* used return values are fjac (partly), par, wa1=x, wa3=diag*x */

            for (j = 0; j < n; j++)
                wa2[j] = x[j] - wa1[j]; /* new parameter vector ? */

            pnorm = lm_enorm(n, wa3);

            /* at first call, adjust the initial step bound. */

            if (nfev++ == 0)
                delta = MINF(delta, pnorm);

/*** inner: evaluate the function at x + p and calculate its norm. ***/

            info = 0;
            (*evaluate) (wa2, m, data, wa4, NULL, &info);
            #ifdef LMFIT_PRINTOUT
            if( printout )
                (*printout) (n, wa2, m, data, wa4, printflags, 2, iter, nfev);
            #endif
            if (info < 0)
                return; /* user requested break. */

            fnorm1 = lm_enorm(m, wa4);
            #ifdef LMFIT_DEBUG_MESSAGES
            printf("lmdif/ pnorm %.10e  fnorm1 %.10e  fnorm %.10e"
                   " delta=%.10e par=%.10e\n",
                   pnorm, fnorm1, fnorm, delta, par);
            #endif

/*** inner: compute the scaled actual reduction. ***/

            if (p1 * fnorm1 < fnorm)
                actred = 1 - SQUARE(fnorm1 / fnorm);
            else
                actred = -1;

/*** inner: compute the scaled predicted reduction and
     the scaled directional derivative. ***/

            for (j = 0; j < n; j++) {
                wa3[j] = 0;
                for (i = 0; i <= j; i++)
                    wa3[i] -= fjac[j*m+i] * wa1[ipvt[j]];
            }
            temp1 = lm_enorm(n, wa3) / fnorm;
            temp2 = sqrt(par) * pnorm / fnorm;
            prered = SQUARE(temp1) + 2 * SQUARE(temp2);
            dirder = -(SQUARE(temp1) + SQUARE(temp2));

/*** inner: compute the ratio of the actual to the predicted reduction. ***/

            ratio = prered != 0 ? actred / prered : 0;
            #ifdef LMFIT_DEBUG_MESSAGES
            printf("lmdif/ actred=%.10e prered=%.10e ratio=%.10e"
                   " sq(1)=%.10e sq(2)=%.10e dd=%.10e\n",
                   actred, prered, prered != 0 ? ratio : 0.,
                   SQUARE(temp1), SQUARE(temp2), dirder);
            #endif

/*** inner: update the step bound. ***/

            if (ratio <= 0.25) {
                double temp = (actred >= 0 ? 0.5 : 0.5 * dirder / (dirder + 0.5 * actred));
                if (p1 * fnorm1 >= fnorm || temp < p1)
                    temp = p1;
                delta = temp * MINF(delta, pnorm / p1);
                par /= temp;
            } else if (par == 0. || ratio >= 0.75) {
                delta = pnorm / 0.5;
                par *= 0.5;
            }

/*** inner: test for successful iteration. ***/

            if (ratio >= p0001) {
                /* yes, success: update x, fvec, and their norms. */
                if (mode != 2) {
                    for (j = 0; j < n; j++) {
                        x[j] = wa2[j];
                        wa2[j] = diag[j] * x[j];
                    }
                } else {
                    memcpy(x, wa2, sizeof(double)*n);
                }
                std::swap(fvec, wa4);
                xnorm = lm_enorm(n, wa2);
                fnorm = fnorm1;
                #ifdef LMFIT_PRINTOUT
                ++iter;
                #endif
            }
            #ifdef LMFIT_DEBUG_MESSAGES
            else {
                printf("ATTN: iteration considered unsuccessful\n");
            }
            #endif

/*** inner: test for convergence. ***/

            if( fnorm<=LM_DWARF ){
                info = 0;
                return;
            }

            if (fabs(actred) <= ftol && prered <= ftol && 0.5 * ratio <= 1)
                info = 1;
            if (delta <= xtol * xnorm)
                info += 2;
            if (info != 0)
                return;

/*** inner: tests for termination and stringent tolerances. ***/

            if (nfev >= maxfev){
                info = 5;
                return;
            }
            if (fabs(actred) <= LM_MACHEP &&
                prered <= LM_MACHEP && 0.5 * ratio <= 1){
                info = 6;
                return;
            }
            if (delta <= LM_MACHEP * xnorm){
                info = 7;
                return;
            }
            if (gnorm <= LM_MACHEP){
                info = 8;
                return;
            }

/*** inner: end of the loop. repeat if iteration unsuccessful. ***/

        } while (ratio < p0001);

/*** outer: end of the loop. ***/

    } while (1);

} /*** lm_lmdif. ***/


/*****************************************************************************/
/*  lm_lmpar (determine Levenberg-Marquardt parameter)                       */
/*****************************************************************************/

void lm_lmpar(int n, double *r, int ldr, int *ipvt, double *diag,
              double *qtb, double delta, double *par, double *x,
              double *sdiag, double *aux, double *xdi)
{
/*     Given an m by n matrix a, an n by n nonsingular diagonal
 *     matrix d, an m-vector b, and a positive number delta,
 *     the problem is to determine a value for the parameter
 *     par such that if x solves the system
 *
 *          a*x = b  and  sqrt(par)*d*x = 0
 *
 *     in the least squares sense, and dxnorm is the euclidean
 *     norm of d*x, then either par=0 and (dxnorm-delta) < 0.1*delta,
 *     or par>0 and abs(dxnorm-delta) < 0.1*delta.
 *
 *     Using lm_qrsolv, this subroutine completes the solution of the problem
 *     if it is provided with the necessary information from the
 *     qr factorization, with column pivoting, of a. That is, if
 *     a*p = q*r, where p is a permutation matrix, q has orthogonal
 *     columns, and r is an upper triangular matrix with diagonal
 *     elements of nonincreasing magnitude, then lmpar expects
 *     the full upper triangle of r, the permutation matrix p,
 *     and the first n components of qT*b. On output
 *     lmpar also provides an upper triangular matrix s such that
 *
 *          pT*(aT*a + par*d*d)*p = sT*s.
 *
 *     s is employed within lmpar and may be of separate interest.
 *
 *     Only a few iterations are generally needed for convergence
 *     of the algorithm. If, however, the limit of 10 iterations
 *     is reached, then the output par will contain the best
 *     value obtained so far.
 *
 *     parameters:
 *
 *      n is a positive integer input variable set to the order of r.
 *
 *      r is an n by n array. on input the full upper triangle
 *        must contain the full upper triangle of the matrix r.
 *        on OUTPUT the full upper triangle is unaltered, and the
 *        strict lower triangle contains the strict upper triangle
 *        (transposed) of the upper triangular matrix s.
 *
 *      ldr is a positive integer input variable not less than n
 *        which specifies the leading dimension of the array r.
 *
 *      ipvt is an integer input array of length n which defines the
 *        permutation matrix p such that a*p = q*r. column j of p
 *        is column ipvt(j) of the identity matrix.
 *
 *      diag is an input array of length n which must contain the
 *        diagonal elements of the matrix d.
 *
 *      qtb is an input array of length n which must contain the first
 *        n elements of the vector (q transpose)*b.
 *
 *      delta is a positive input variable which specifies an upper
 *        bound on the euclidean norm of d*x.
 *
 *      par is a nonnegative variable. on input par contains an
 *        initial estimate of the levenberg-marquardt parameter.
 *        on OUTPUT par contains the final estimate.
 *
 *      x is an OUTPUT array of length n which contains the least
 *        squares solution of the system a*x = b, sqrt(par)*d*x = 0,
 *        for the output par.
 *
 *      sdiag is an array of length n which contains the
 *        diagonal elements of the upper triangular matrix s.
 *
 *      aux is a multi-purpose work array of length n.
 *
 *      xdi is a work array of length n. On OUTPUT: diag[j] * x[j].
 *
 */
    int i, j, nsing;
    double dxnorm, fp, fp_old, gnorm, parc, parl, paru;
    static const double p1 = 0.1;

#ifdef LMFIT_DEBUG_MESSAGES
    printf("lmpar\n");
#endif

/*** lmpar: compute and store in x the gauss-newton direction. if the
     jacobian is rank-deficient, obtain a least squares solution. ***/

    nsing = n;
    for (j = 0; j < n; j++) {
        aux[j] = qtb[j];
        if (r[j * ldr + j] == 0 && nsing == n)
            nsing = j;
        if (nsing < n)
            aux[j] = 0;
    }
#ifdef LMFIT_DEBUG_MESSAGES
    printf("nsing %d ", nsing);
#endif
    for (j = nsing - 1; j >= 0; j--) {
        aux[j] = aux[j] / r[j + ldr * j];
        double temp = aux[j];
        for (i = 0; i < j; i++)
            aux[i] -= r[j * ldr + i] * temp;
    }

    for (j = 0; j < n; j++)
        x[ipvt[j]] = aux[j];

/*** lmpar: initialize the iteration counter, evaluate the function at the
     origin, and test for acceptance of the gauss-newton direction. ***/

    if (diag) {
        for (j = 0; j < n; j++)
            xdi[j] = diag[j] * x[j];
        dxnorm = lm_enorm(n, xdi);
    } else {
        dxnorm = lm_enorm(n, x);
    }
    fp = dxnorm - delta;
    if (fp <= p1 * delta) {
#ifdef LMFIT_DEBUG_MESSAGES
        printf("lmpar/ terminate (fp<p1*delta)\n");
#endif
        *par = 0;
        return;
    }

/*** lmpar: if the jacobian is not rank deficient, the newton
     step provides a lower bound, parl, for the 0. of
     the function. otherwise set this bound to 0.. ***/

    parl = 0;
    if (nsing >= n) {
        if (diag) {
            for (j = 0; j < n; j++)
                aux[j] = diag[ipvt[j]] * xdi[ipvt[j]] / dxnorm;
        } else {
            for (j = 0; j < n; j++)
                aux[j] = x[ipvt[j]] / dxnorm;
        }

        for (j = 0; j < n; j++) {
            double sum = 0;
            for (i = 0; i < j; i++)
                sum += r[j * ldr + i] * aux[i];
            aux[j] = (aux[j] - sum) / r[j + ldr * j];
        }
        double temp = lm_enorm(n, aux);
        parl = fp / delta / temp / temp;
    }

/*** lmpar: calculate an upper bound, paru, for the 0. of the function. ***/

    for (j = 0; j < n; j++) {
        double sum = 0;
        for (i = 0; i <= j; i++)
            sum += r[j * ldr + i] * qtb[i];
        if (diag) {
            aux[j] = sum / diag[ipvt[j]];
        } else {
            aux[j] = sum;
        }
    }
    gnorm = lm_enorm(n, aux);
    paru = gnorm / delta;
    if (paru == 0.)
        paru = LM_DWARF / MINF(delta, p1);

/*** lmpar: if the input par lies outside of the interval (parl,paru),
     set par to the closer endpoint. ***/

    *par = MAXF(*par, parl);
    *par = MINF(*par, paru);
    if (*par == 0.)
        *par = gnorm / dxnorm;
#ifdef LMFIT_DEBUG_MESSAGES
    printf("lmpar/ parl %.4e  par %.4e  paru %.4e\n", parl, *par, paru);
#endif

/*** lmpar: iterate. ***/

    for (int iter = 0; ; iter++) {

        /** evaluate the function at the current value of par. **/

        if (*par == 0.)
            *par = MAXF(LM_DWARF, 0.001 * paru);
        double temp = sqrt(*par);
        if (diag) {
            for (j = 0; j < n; j++)
                aux[j] = temp * diag[j];
        } else {
            for (j = 0; j < n; j++)
                aux[j] = temp;
        }

        lm_qrsolv( n, r, ldr, ipvt, aux, qtb, x, sdiag, xdi );
        /* return values are r, x, sdiag */

        if (diag) {
            for (j = 0; j < n; j++)
                xdi[j] = diag[j] * x[j]; /* used as output */
            dxnorm = lm_enorm(n, xdi);
        } else {
            dxnorm = lm_enorm(n, x);
        }
        fp_old = fp;
        fp = dxnorm - delta;

        /** if the function is small enough, accept the current value
            of par. Also test for the exceptional cases where parl
            is zero or the number of iterations has reached 10. **/

        if (fabs(fp) <= p1 * delta
            || (parl == 0. && fp <= fp_old && fp_old < 0.)
            || iter == 10)
            break; /* the only exit from the iteration. */

        /** compute the Newton correction. **/

        if (diag) {
            for (j = 0; j < n; j++)
                aux[j] = diag[ipvt[j]] * xdi[ipvt[j]] / dxnorm;
        } else {
            for (j = 0; j < n; j++)
                aux[j] = x[ipvt[j]] / dxnorm;
        }

        for (j = 0; j < n; j++) {
            aux[j] = aux[j] / sdiag[j];
            for (i = j + 1; i < n; i++)
                aux[i] -= r[j * ldr + i] * aux[j];
        }
        temp = lm_enorm(n, aux);
        parc = fp / delta / temp / temp;

        /** depending on the sign of the function, update parl or paru. **/

        if (fp > 0)
            parl = MAXF(parl, *par);
        else if (fp < 0)
            paru = MINF(paru, *par);
        /* the case fp==0 is precluded by the break condition  */

        /** compute an improved estimate for par. **/

        *par = MAXF(parl, *par + parc);

    }

} /*** lm_lmpar. ***/


/*****************************************************************************/
/*  lm_qrfac (QR factorisation, from lapack)                                 */
/*****************************************************************************/

void lm_qrfac(int m, int n, double *a, int pivot, int *ipvt,
              double *rdiag, double *acnorm, double *wa)
{
/*
 *     This subroutine uses householder transformations with column
 *     pivoting (optional) to compute a qr factorization of the
 *     m by n matrix a. That is, qrfac determines an orthogonal
 *     matrix q, a permutation matrix p, and an upper trapezoidal
 *     matrix r with diagonal elements of nonincreasing magnitude,
 *     such that a*p = q*r. The householder transformation for
 *     column k, k = 1,2,...,min(m,n), is of the form
 *
 *          i - (1/u(k))*u*uT
 *
 *     where u has zeroes in the first k-1 positions. The form of
 *     this transformation and the method of pivoting first
 *     appeared in the corresponding linpack subroutine.
 *
 *     Parameters:
 *
 *      m is a positive integer input variable set to the number
 *        of rows of a.
 *
 *      n is a positive integer input variable set to the number
 *        of columns of a.
 *
 *      a is an m by n array. On input a contains the matrix for
 *        which the qr factorization is to be computed. On OUTPUT
 *        the strict upper trapezoidal part of a contains the strict
 *        upper trapezoidal part of r, and the lower trapezoidal
 *        part of a contains a factored form of q (the non-trivial
 *        elements of the u vectors described above).
 *
 *      pivot is a logical input variable. If pivot is set true,
 *        then column pivoting is enforced. If pivot is set false,
 *        then no column pivoting is done.
 *
 *      ipvt is an integer OUTPUT array of length lipvt. This array
 *        defines the permutation matrix p such that a*p = q*r.
 *        Column j of p is column ipvt(j) of the identity matrix.
 *        If pivot is false, ipvt is not referenced.
 *
 *      rdiag is an OUTPUT array of length n which contains the
 *        diagonal elements of r.
 *
 *      acnorm is an OUTPUT array of length n which contains the
 *        norms of the corresponding columns of the input matrix a.
 *        If this information is not needed, then acnorm can coincide
 *        with rdiag.
 *
 *      wa is a work array of length n. If pivot is false, then wa
 *        can coincide with rdiag.
 *
 */
    int i, j, k, kmax, minmn;
    double ajnorm;

/*** qrfac: compute initial column norms and initialize several arrays. ***/

    for (j = 0; j < n; j++) {
        acnorm[j] = lm_enorm(m, &a[j*m]);
        rdiag[j] = acnorm[j];
        wa[j] = rdiag[j];
        if (pivot)
            ipvt[j] = j;
    }
#ifdef LMFIT_DEBUG_MESSAGES
    printf("qrfac\n");
#endif

/*** qrfac: reduce a to r with householder transformations. ***/

    minmn = MINF(m, n);
    for (j = 0; j < minmn; j++) {
        if (!pivot)
            goto pivot_ok;

        /** bring the column of largest norm into the pivot position. **/

        kmax = j;
        for (k = j + 1; k < n; k++)
            if (rdiag[k] > rdiag[kmax])
                kmax = k;
        if (kmax == j)
            goto pivot_ok;

        for (i = 0; i < m; i++) {
            double temp = a[j*m+i];
            a[j*m+i] = a[kmax*m+i];
            a[kmax*m+i] = temp;
        }
        rdiag[kmax] = rdiag[j];
        wa[kmax] = wa[j];
        k = ipvt[j];
        ipvt[j] = ipvt[kmax];
        ipvt[kmax] = k;

      pivot_ok:
        /** compute the Householder transformation to reduce the
            j-th column of a to a multiple of the j-th unit vector. **/

        ajnorm = lm_enorm(m-j, &a[j*m+j]);
        if (ajnorm == 0.) {
            rdiag[j] = 0;
            continue;
        }

        if (a[j*m+j] < 0.)
            ajnorm = -ajnorm;
        for (i = j; i < m; i++)
            a[j*m+i] /= ajnorm;
        a[j*m+j] += 1;

        /** apply the transformation to the remaining columns
            and update the norms. **/

        for (k = j + 1; k < n; k++) {
            double sum = 0;

            for (i = j; i < m; i++)
                sum += a[j*m+i] * a[k*m+i];

            double temp = sum / a[j + m * j];

            for (i = j; i < m; i++)
                a[k*m+i] -= temp * a[j*m+i];

            if (pivot && rdiag[k] != 0.) {
                temp = a[m * k + j] / rdiag[k];
                temp = MAXF(0., 1 - temp * temp);
                rdiag[k] *= sqrt(temp);
                temp = rdiag[k] / wa[k];
                if ( 0.05 * SQUARE(temp) <= LM_MACHEP ) {
                    rdiag[k] = lm_enorm(m-j-1, &a[m*k+j+1]);
                    wa[k] = rdiag[k];
                }
            }
        }

        rdiag[j] = -ajnorm;
    }
}


/*****************************************************************************/
/*  lm_qrsolv (linear least-squares)                                         */
/*****************************************************************************/

void lm_qrsolv(int n, double *r, int ldr, int *ipvt, double *diag,
               double *qtb, double *x, double *sdiag, double *wa)
{
/*
 *     Given an m by n matrix a, an n by n diagonal matrix d,
 *     and an m-vector b, the problem is to determine an x which
 *     solves the system
 *
 *          a*x = b  and  d*x = 0
 *
 *     in the least squares sense.
 *
 *     This subroutine completes the solution of the problem
 *     if it is provided with the necessary information from the
 *     qr factorization, with column pivoting, of a. That is, if
 *     a*p = q*r, where p is a permutation matrix, q has orthogonal
 *     columns, and r is an upper triangular matrix with diagonal
 *     elements of nonincreasing magnitude, then qrsolv expects
 *     the full upper triangle of r, the permutation matrix p,
 *     and the first n components of (q transpose)*b. The system
 *     a*x = b, d*x = 0, is then equivalent to
 *
 *          r*z = qT*b,  pT*d*p*z = 0,
 *
 *     where x = p*z. If this system does not have full rank,
 *     then a least squares solution is obtained. On output qrsolv
 *     also provides an upper triangular matrix s such that
 *
 *          pT *(aT *a + d*d)*p = sT *s.
 *
 *     s is computed within qrsolv and may be of separate interest.
 *
 *     Parameters
 *
 *      n is a positive integer input variable set to the order of r.
 *
 *      r is an n by n array. On input the full upper triangle
 *        must contain the full upper triangle of the matrix r.
 *        On OUTPUT the full upper triangle is unaltered, and the
 *        strict lower triangle contains the strict upper triangle
 *        (transposed) of the upper triangular matrix s.
 *
 *      ldr is a positive integer input variable not less than n
 *        which specifies the leading dimension of the array r.
 *
 *      ipvt is an integer input array of length n which defines the
 *        permutation matrix p such that a*p = q*r. Column j of p
 *        is column ipvt(j) of the identity matrix.
 *
 *      diag is an input array of length n which must contain the
 *        diagonal elements of the matrix d.
 *
 *      qtb is an input array of length n which must contain the first
 *        n elements of the vector (q transpose)*b.
 *
 *      x is an OUTPUT array of length n which contains the least
 *        squares solution of the system a*x = b, d*x = 0.
 *
 *      sdiag is an OUTPUT array of length n which contains the
 *        diagonal elements of the upper triangular matrix s.
 *
 *      wa is a work array of length n.
 *
 */
    int i, kk, j, k, nsing;
    double qtbpj;
    double _sin, _cos, _tan, _cot; /* local variables, not functions */

/*** qrsolv: copy r and (q transpose)*b to preserve input and initialize s.
     in particular, save the diagonal elements of r in x. ***/

    for (j = 0; j < n; j++) {
        for (i = j; i < n; i++)
            r[j * ldr + i] = r[i * ldr + j];
        x[j] = r[j * ldr + j];
        wa[j] = qtb[j];
    }
#ifdef LMFIT_DEBUG_MESSAGES
    printf("qrsolv\n");
#endif

/*** qrsolv: eliminate the diagonal matrix d using a Givens rotation. ***/

    for (j = 0; j < n; j++) {

/*** qrsolv: prepare the row of d to be eliminated, locating the
     diagonal element using p from the qr factorization. ***/

        if (diag) {
            if (diag[ipvt[j]] == 0.)
                goto L90;
            for (k = j+1; k < n; k++)
                sdiag[k] = 0.;
            sdiag[j] = diag[ipvt[j]];
        } else {
            for (k = j+1; k < n; k++)
                sdiag[k] = 0.;
            sdiag[j] = 1.;
        }

/*** qrsolv: the transformations to eliminate the row of d modify only
     a single element of qT*b beyond the first n, which is initially 0. ***/

        qtbpj = 0.;
        for (k = j; k < n; k++) {

            /** determine a Givens rotation which eliminates the
                appropriate element in the current row of d. **/

            if (sdiag[k] == 0.)
                continue;
            kk = k + ldr * k;
            if (fabs(r[kk]) < fabs(sdiag[k])) {
                _cot = r[kk] / sdiag[k];
                _sin = 1 / sqrt(1 + SQUARE(_cot));
                _cos = _sin * _cot;
            } else {
                _tan = sdiag[k] / r[kk];
                _cos = 1 / sqrt(1 + SQUARE(_tan));
                _sin = _cos * _tan;
            }

            /** compute the modified diagonal element of r and
                the modified element of ((q transpose)*b,0). **/

            r[kk] = _cos * r[kk] + _sin * sdiag[k];
            double temp = _cos * wa[k] + _sin * qtbpj;
            qtbpj = -_sin * wa[k] + _cos * qtbpj;
            wa[k] = temp;

            /** accumulate the tranformation in the row of s. **/

            for (i = k + 1; i < n; i++) {
                temp = _cos * r[k * ldr + i] + _sin * sdiag[i];
                sdiag[i] = -_sin * r[k * ldr + i] + _cos * sdiag[i];
                r[k * ldr + i] = temp;
            }
        }

      L90:
        /** store the diagonal element of s and restore
            the corresponding diagonal element of r. **/

        sdiag[j] = r[j * ldr + j];
        r[j * ldr + j] = x[j];
    }

/*** qrsolv: solve the triangular system for z. if the system is
     singular, then obtain a least squares solution. ***/

    nsing = n;
    for (j = 0; j < n; j++) {
        if (sdiag[j] == 0. && nsing == n)
            nsing = j;
        if (nsing < n)
            wa[j] = 0;
    }

    for (j = nsing - 1; j >= 0; j--) {
        double sum = 0;
        for (i = j + 1; i < nsing; i++)
            sum += r[j * ldr + i] * wa[i];
        wa[j] = (wa[j] - sum) / sdiag[j];
    }

/*** qrsolv: permute the components of z back to components of x. ***/

    for (j = 0; j < n; j++)
        x[ipvt[j]] = wa[j];

} /*** lm_qrsolv. ***/


/*****************************************************************************/
/*  lm_enorm (Euclidean norm)                                                */
/*****************************************************************************/

double lm_enorm(int n, const double *x)
{
/*     Given an n-vector x, this function calculates the
 *     euclidean norm of x.
 *
 *     The euclidean norm is computed by accumulating the sum of
 *     squares in three different sums. The sums of squares for the
 *     small and large components are scaled so that no overflows
 *     occur. Non-destructive underflows are permitted. Underflows
 *     and overflows do not occur in the computation of the unscaled
 *     sum of squares for the intermediate components.
 *     The definitions of small, intermediate and large components
 *     depend on two constants, LM_SQRT_DWARF and LM_SQRT_GIANT. The main
 *     restrictions on these constants are that LM_SQRT_DWARF**2 not
 *     underflow and LM_SQRT_GIANT**2 not overflow.
 *
 *     Parameters
 *
 *      n is a positive integer input variable.
 *
 *      x is an input array of length n.
 */
    int i;
    double agiant, s1, s2, s3, xabs, x1max, x3max, temp;

    s1 = 0;
    s2 = 0;
    s3 = 0;
    x1max = 0;
    x3max = 0;
    agiant = LM_SQRT_GIANT / n;

    /** sum squares. **/

    for (i = 0; i < n; i++) {
        xabs = fabs(x[i]);
        if (xabs > LM_SQRT_DWARF) {
            if ( xabs < agiant ) {
                s2 += xabs * xabs;
            } else if ( xabs > x1max ) {
                temp = x1max / xabs;
                s1 = 1 + s1 * SQUARE(temp);
                x1max = xabs;
            } else {
                temp = xabs / x1max;
                s1 += SQUARE(temp);
            }
        } else if ( xabs > x3max ) {
            temp = x3max / xabs;
            s3 = 1 + s3 * SQUARE(temp);
            x3max = xabs;
        } else if (xabs != 0.) {
            temp = xabs / x3max;
            s3 += SQUARE(temp);
        }
    }

    /** calculation of norm. **/

    if (s1 != 0)
        return x1max * sqrt(s1 + (s2 / x1max) / x1max);
    else if (s2 != 0)
        if (s2 >= x3max)
            return sqrt(s2 * (1 + (x3max / s2) * (x3max * s3)));
        else
            return sqrt(x3max * ((s2 / x3max) + (x3max * s3)));
    else
        return x3max * sqrt(s3);

} /*** lm_enorm. ***/
