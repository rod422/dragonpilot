#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5853957832074453940) {
   out_5853957832074453940[0] = delta_x[0] + nom_x[0];
   out_5853957832074453940[1] = delta_x[1] + nom_x[1];
   out_5853957832074453940[2] = delta_x[2] + nom_x[2];
   out_5853957832074453940[3] = delta_x[3] + nom_x[3];
   out_5853957832074453940[4] = delta_x[4] + nom_x[4];
   out_5853957832074453940[5] = delta_x[5] + nom_x[5];
   out_5853957832074453940[6] = delta_x[6] + nom_x[6];
   out_5853957832074453940[7] = delta_x[7] + nom_x[7];
   out_5853957832074453940[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_737226202460666342) {
   out_737226202460666342[0] = -nom_x[0] + true_x[0];
   out_737226202460666342[1] = -nom_x[1] + true_x[1];
   out_737226202460666342[2] = -nom_x[2] + true_x[2];
   out_737226202460666342[3] = -nom_x[3] + true_x[3];
   out_737226202460666342[4] = -nom_x[4] + true_x[4];
   out_737226202460666342[5] = -nom_x[5] + true_x[5];
   out_737226202460666342[6] = -nom_x[6] + true_x[6];
   out_737226202460666342[7] = -nom_x[7] + true_x[7];
   out_737226202460666342[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1375279494024935349) {
   out_1375279494024935349[0] = 1.0;
   out_1375279494024935349[1] = 0;
   out_1375279494024935349[2] = 0;
   out_1375279494024935349[3] = 0;
   out_1375279494024935349[4] = 0;
   out_1375279494024935349[5] = 0;
   out_1375279494024935349[6] = 0;
   out_1375279494024935349[7] = 0;
   out_1375279494024935349[8] = 0;
   out_1375279494024935349[9] = 0;
   out_1375279494024935349[10] = 1.0;
   out_1375279494024935349[11] = 0;
   out_1375279494024935349[12] = 0;
   out_1375279494024935349[13] = 0;
   out_1375279494024935349[14] = 0;
   out_1375279494024935349[15] = 0;
   out_1375279494024935349[16] = 0;
   out_1375279494024935349[17] = 0;
   out_1375279494024935349[18] = 0;
   out_1375279494024935349[19] = 0;
   out_1375279494024935349[20] = 1.0;
   out_1375279494024935349[21] = 0;
   out_1375279494024935349[22] = 0;
   out_1375279494024935349[23] = 0;
   out_1375279494024935349[24] = 0;
   out_1375279494024935349[25] = 0;
   out_1375279494024935349[26] = 0;
   out_1375279494024935349[27] = 0;
   out_1375279494024935349[28] = 0;
   out_1375279494024935349[29] = 0;
   out_1375279494024935349[30] = 1.0;
   out_1375279494024935349[31] = 0;
   out_1375279494024935349[32] = 0;
   out_1375279494024935349[33] = 0;
   out_1375279494024935349[34] = 0;
   out_1375279494024935349[35] = 0;
   out_1375279494024935349[36] = 0;
   out_1375279494024935349[37] = 0;
   out_1375279494024935349[38] = 0;
   out_1375279494024935349[39] = 0;
   out_1375279494024935349[40] = 1.0;
   out_1375279494024935349[41] = 0;
   out_1375279494024935349[42] = 0;
   out_1375279494024935349[43] = 0;
   out_1375279494024935349[44] = 0;
   out_1375279494024935349[45] = 0;
   out_1375279494024935349[46] = 0;
   out_1375279494024935349[47] = 0;
   out_1375279494024935349[48] = 0;
   out_1375279494024935349[49] = 0;
   out_1375279494024935349[50] = 1.0;
   out_1375279494024935349[51] = 0;
   out_1375279494024935349[52] = 0;
   out_1375279494024935349[53] = 0;
   out_1375279494024935349[54] = 0;
   out_1375279494024935349[55] = 0;
   out_1375279494024935349[56] = 0;
   out_1375279494024935349[57] = 0;
   out_1375279494024935349[58] = 0;
   out_1375279494024935349[59] = 0;
   out_1375279494024935349[60] = 1.0;
   out_1375279494024935349[61] = 0;
   out_1375279494024935349[62] = 0;
   out_1375279494024935349[63] = 0;
   out_1375279494024935349[64] = 0;
   out_1375279494024935349[65] = 0;
   out_1375279494024935349[66] = 0;
   out_1375279494024935349[67] = 0;
   out_1375279494024935349[68] = 0;
   out_1375279494024935349[69] = 0;
   out_1375279494024935349[70] = 1.0;
   out_1375279494024935349[71] = 0;
   out_1375279494024935349[72] = 0;
   out_1375279494024935349[73] = 0;
   out_1375279494024935349[74] = 0;
   out_1375279494024935349[75] = 0;
   out_1375279494024935349[76] = 0;
   out_1375279494024935349[77] = 0;
   out_1375279494024935349[78] = 0;
   out_1375279494024935349[79] = 0;
   out_1375279494024935349[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4251696200088534572) {
   out_4251696200088534572[0] = state[0];
   out_4251696200088534572[1] = state[1];
   out_4251696200088534572[2] = state[2];
   out_4251696200088534572[3] = state[3];
   out_4251696200088534572[4] = state[4];
   out_4251696200088534572[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4251696200088534572[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4251696200088534572[7] = state[7];
   out_4251696200088534572[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1908927242052538900) {
   out_1908927242052538900[0] = 1;
   out_1908927242052538900[1] = 0;
   out_1908927242052538900[2] = 0;
   out_1908927242052538900[3] = 0;
   out_1908927242052538900[4] = 0;
   out_1908927242052538900[5] = 0;
   out_1908927242052538900[6] = 0;
   out_1908927242052538900[7] = 0;
   out_1908927242052538900[8] = 0;
   out_1908927242052538900[9] = 0;
   out_1908927242052538900[10] = 1;
   out_1908927242052538900[11] = 0;
   out_1908927242052538900[12] = 0;
   out_1908927242052538900[13] = 0;
   out_1908927242052538900[14] = 0;
   out_1908927242052538900[15] = 0;
   out_1908927242052538900[16] = 0;
   out_1908927242052538900[17] = 0;
   out_1908927242052538900[18] = 0;
   out_1908927242052538900[19] = 0;
   out_1908927242052538900[20] = 1;
   out_1908927242052538900[21] = 0;
   out_1908927242052538900[22] = 0;
   out_1908927242052538900[23] = 0;
   out_1908927242052538900[24] = 0;
   out_1908927242052538900[25] = 0;
   out_1908927242052538900[26] = 0;
   out_1908927242052538900[27] = 0;
   out_1908927242052538900[28] = 0;
   out_1908927242052538900[29] = 0;
   out_1908927242052538900[30] = 1;
   out_1908927242052538900[31] = 0;
   out_1908927242052538900[32] = 0;
   out_1908927242052538900[33] = 0;
   out_1908927242052538900[34] = 0;
   out_1908927242052538900[35] = 0;
   out_1908927242052538900[36] = 0;
   out_1908927242052538900[37] = 0;
   out_1908927242052538900[38] = 0;
   out_1908927242052538900[39] = 0;
   out_1908927242052538900[40] = 1;
   out_1908927242052538900[41] = 0;
   out_1908927242052538900[42] = 0;
   out_1908927242052538900[43] = 0;
   out_1908927242052538900[44] = 0;
   out_1908927242052538900[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1908927242052538900[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1908927242052538900[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1908927242052538900[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1908927242052538900[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1908927242052538900[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1908927242052538900[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1908927242052538900[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1908927242052538900[53] = -9.8000000000000007*dt;
   out_1908927242052538900[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1908927242052538900[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1908927242052538900[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1908927242052538900[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1908927242052538900[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1908927242052538900[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1908927242052538900[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1908927242052538900[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1908927242052538900[62] = 0;
   out_1908927242052538900[63] = 0;
   out_1908927242052538900[64] = 0;
   out_1908927242052538900[65] = 0;
   out_1908927242052538900[66] = 0;
   out_1908927242052538900[67] = 0;
   out_1908927242052538900[68] = 0;
   out_1908927242052538900[69] = 0;
   out_1908927242052538900[70] = 1;
   out_1908927242052538900[71] = 0;
   out_1908927242052538900[72] = 0;
   out_1908927242052538900[73] = 0;
   out_1908927242052538900[74] = 0;
   out_1908927242052538900[75] = 0;
   out_1908927242052538900[76] = 0;
   out_1908927242052538900[77] = 0;
   out_1908927242052538900[78] = 0;
   out_1908927242052538900[79] = 0;
   out_1908927242052538900[80] = 1;
}
void h_25(double *state, double *unused, double *out_6553259077698250498) {
   out_6553259077698250498[0] = state[6];
}
void H_25(double *state, double *unused, double *out_75408611009746463) {
   out_75408611009746463[0] = 0;
   out_75408611009746463[1] = 0;
   out_75408611009746463[2] = 0;
   out_75408611009746463[3] = 0;
   out_75408611009746463[4] = 0;
   out_75408611009746463[5] = 0;
   out_75408611009746463[6] = 1;
   out_75408611009746463[7] = 0;
   out_75408611009746463[8] = 0;
}
void h_24(double *state, double *unused, double *out_6203684692643907106) {
   out_6203684692643907106[0] = state[4];
   out_6203684692643907106[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3633334209812069599) {
   out_3633334209812069599[0] = 0;
   out_3633334209812069599[1] = 0;
   out_3633334209812069599[2] = 0;
   out_3633334209812069599[3] = 0;
   out_3633334209812069599[4] = 1;
   out_3633334209812069599[5] = 0;
   out_3633334209812069599[6] = 0;
   out_3633334209812069599[7] = 0;
   out_3633334209812069599[8] = 0;
   out_3633334209812069599[9] = 0;
   out_3633334209812069599[10] = 0;
   out_3633334209812069599[11] = 0;
   out_3633334209812069599[12] = 0;
   out_3633334209812069599[13] = 0;
   out_3633334209812069599[14] = 1;
   out_3633334209812069599[15] = 0;
   out_3633334209812069599[16] = 0;
   out_3633334209812069599[17] = 0;
}
void h_30(double *state, double *unused, double *out_3936353391069076669) {
   out_3936353391069076669[0] = state[4];
}
void H_30(double *state, double *unused, double *out_6992098952501363218) {
   out_6992098952501363218[0] = 0;
   out_6992098952501363218[1] = 0;
   out_6992098952501363218[2] = 0;
   out_6992098952501363218[3] = 0;
   out_6992098952501363218[4] = 1;
   out_6992098952501363218[5] = 0;
   out_6992098952501363218[6] = 0;
   out_6992098952501363218[7] = 0;
   out_6992098952501363218[8] = 0;
}
void h_26(double *state, double *unused, double *out_6290393570737091919) {
   out_6290393570737091919[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3666094707864309761) {
   out_3666094707864309761[0] = 0;
   out_3666094707864309761[1] = 0;
   out_3666094707864309761[2] = 0;
   out_3666094707864309761[3] = 0;
   out_3666094707864309761[4] = 0;
   out_3666094707864309761[5] = 0;
   out_3666094707864309761[6] = 0;
   out_3666094707864309761[7] = 1;
   out_3666094707864309761[8] = 0;
}
void h_27(double *state, double *unused, double *out_1931073048242142184) {
   out_1931073048242142184[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4817335640700938307) {
   out_4817335640700938307[0] = 0;
   out_4817335640700938307[1] = 0;
   out_4817335640700938307[2] = 0;
   out_4817335640700938307[3] = 1;
   out_4817335640700938307[4] = 0;
   out_4817335640700938307[5] = 0;
   out_4817335640700938307[6] = 0;
   out_4817335640700938307[7] = 0;
   out_4817335640700938307[8] = 0;
}
void h_29(double *state, double *unused, double *out_8752405768646736991) {
   out_8752405768646736991[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3103972913831387274) {
   out_3103972913831387274[0] = 0;
   out_3103972913831387274[1] = 1;
   out_3103972913831387274[2] = 0;
   out_3103972913831387274[3] = 0;
   out_3103972913831387274[4] = 0;
   out_3103972913831387274[5] = 0;
   out_3103972913831387274[6] = 0;
   out_3103972913831387274[7] = 0;
   out_3103972913831387274[8] = 0;
}
void h_28(double *state, double *unused, double *out_4967859099686474869) {
   out_4967859099686474869[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1978426103238143300) {
   out_1978426103238143300[0] = 1;
   out_1978426103238143300[1] = 0;
   out_1978426103238143300[2] = 0;
   out_1978426103238143300[3] = 0;
   out_1978426103238143300[4] = 0;
   out_1978426103238143300[5] = 0;
   out_1978426103238143300[6] = 0;
   out_1978426103238143300[7] = 0;
   out_1978426103238143300[8] = 0;
}
void h_31(double *state, double *unused, double *out_7848996981607604908) {
   out_7848996981607604908[0] = state[8];
}
void H_31(double *state, double *unused, double *out_106054572886706891) {
   out_106054572886706891[0] = 0;
   out_106054572886706891[1] = 0;
   out_106054572886706891[2] = 0;
   out_106054572886706891[3] = 0;
   out_106054572886706891[4] = 0;
   out_106054572886706891[5] = 0;
   out_106054572886706891[6] = 0;
   out_106054572886706891[7] = 0;
   out_106054572886706891[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_5853957832074453940) {
  err_fun(nom_x, delta_x, out_5853957832074453940);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_737226202460666342) {
  inv_err_fun(nom_x, true_x, out_737226202460666342);
}
void car_H_mod_fun(double *state, double *out_1375279494024935349) {
  H_mod_fun(state, out_1375279494024935349);
}
void car_f_fun(double *state, double dt, double *out_4251696200088534572) {
  f_fun(state,  dt, out_4251696200088534572);
}
void car_F_fun(double *state, double dt, double *out_1908927242052538900) {
  F_fun(state,  dt, out_1908927242052538900);
}
void car_h_25(double *state, double *unused, double *out_6553259077698250498) {
  h_25(state, unused, out_6553259077698250498);
}
void car_H_25(double *state, double *unused, double *out_75408611009746463) {
  H_25(state, unused, out_75408611009746463);
}
void car_h_24(double *state, double *unused, double *out_6203684692643907106) {
  h_24(state, unused, out_6203684692643907106);
}
void car_H_24(double *state, double *unused, double *out_3633334209812069599) {
  H_24(state, unused, out_3633334209812069599);
}
void car_h_30(double *state, double *unused, double *out_3936353391069076669) {
  h_30(state, unused, out_3936353391069076669);
}
void car_H_30(double *state, double *unused, double *out_6992098952501363218) {
  H_30(state, unused, out_6992098952501363218);
}
void car_h_26(double *state, double *unused, double *out_6290393570737091919) {
  h_26(state, unused, out_6290393570737091919);
}
void car_H_26(double *state, double *unused, double *out_3666094707864309761) {
  H_26(state, unused, out_3666094707864309761);
}
void car_h_27(double *state, double *unused, double *out_1931073048242142184) {
  h_27(state, unused, out_1931073048242142184);
}
void car_H_27(double *state, double *unused, double *out_4817335640700938307) {
  H_27(state, unused, out_4817335640700938307);
}
void car_h_29(double *state, double *unused, double *out_8752405768646736991) {
  h_29(state, unused, out_8752405768646736991);
}
void car_H_29(double *state, double *unused, double *out_3103972913831387274) {
  H_29(state, unused, out_3103972913831387274);
}
void car_h_28(double *state, double *unused, double *out_4967859099686474869) {
  h_28(state, unused, out_4967859099686474869);
}
void car_H_28(double *state, double *unused, double *out_1978426103238143300) {
  H_28(state, unused, out_1978426103238143300);
}
void car_h_31(double *state, double *unused, double *out_7848996981607604908) {
  h_31(state, unused, out_7848996981607604908);
}
void car_H_31(double *state, double *unused, double *out_106054572886706891) {
  H_31(state, unused, out_106054572886706891);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
