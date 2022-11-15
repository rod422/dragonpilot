#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_533379885840528252) {
   out_533379885840528252[0] = delta_x[0] + nom_x[0];
   out_533379885840528252[1] = delta_x[1] + nom_x[1];
   out_533379885840528252[2] = delta_x[2] + nom_x[2];
   out_533379885840528252[3] = delta_x[3] + nom_x[3];
   out_533379885840528252[4] = delta_x[4] + nom_x[4];
   out_533379885840528252[5] = delta_x[5] + nom_x[5];
   out_533379885840528252[6] = delta_x[6] + nom_x[6];
   out_533379885840528252[7] = delta_x[7] + nom_x[7];
   out_533379885840528252[8] = delta_x[8] + nom_x[8];
   out_533379885840528252[9] = delta_x[9] + nom_x[9];
   out_533379885840528252[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8671545501629514785) {
   out_8671545501629514785[0] = -nom_x[0] + true_x[0];
   out_8671545501629514785[1] = -nom_x[1] + true_x[1];
   out_8671545501629514785[2] = -nom_x[2] + true_x[2];
   out_8671545501629514785[3] = -nom_x[3] + true_x[3];
   out_8671545501629514785[4] = -nom_x[4] + true_x[4];
   out_8671545501629514785[5] = -nom_x[5] + true_x[5];
   out_8671545501629514785[6] = -nom_x[6] + true_x[6];
   out_8671545501629514785[7] = -nom_x[7] + true_x[7];
   out_8671545501629514785[8] = -nom_x[8] + true_x[8];
   out_8671545501629514785[9] = -nom_x[9] + true_x[9];
   out_8671545501629514785[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_6049361165244039606) {
   out_6049361165244039606[0] = 1.0;
   out_6049361165244039606[1] = 0;
   out_6049361165244039606[2] = 0;
   out_6049361165244039606[3] = 0;
   out_6049361165244039606[4] = 0;
   out_6049361165244039606[5] = 0;
   out_6049361165244039606[6] = 0;
   out_6049361165244039606[7] = 0;
   out_6049361165244039606[8] = 0;
   out_6049361165244039606[9] = 0;
   out_6049361165244039606[10] = 0;
   out_6049361165244039606[11] = 0;
   out_6049361165244039606[12] = 1.0;
   out_6049361165244039606[13] = 0;
   out_6049361165244039606[14] = 0;
   out_6049361165244039606[15] = 0;
   out_6049361165244039606[16] = 0;
   out_6049361165244039606[17] = 0;
   out_6049361165244039606[18] = 0;
   out_6049361165244039606[19] = 0;
   out_6049361165244039606[20] = 0;
   out_6049361165244039606[21] = 0;
   out_6049361165244039606[22] = 0;
   out_6049361165244039606[23] = 0;
   out_6049361165244039606[24] = 1.0;
   out_6049361165244039606[25] = 0;
   out_6049361165244039606[26] = 0;
   out_6049361165244039606[27] = 0;
   out_6049361165244039606[28] = 0;
   out_6049361165244039606[29] = 0;
   out_6049361165244039606[30] = 0;
   out_6049361165244039606[31] = 0;
   out_6049361165244039606[32] = 0;
   out_6049361165244039606[33] = 0;
   out_6049361165244039606[34] = 0;
   out_6049361165244039606[35] = 0;
   out_6049361165244039606[36] = 1.0;
   out_6049361165244039606[37] = 0;
   out_6049361165244039606[38] = 0;
   out_6049361165244039606[39] = 0;
   out_6049361165244039606[40] = 0;
   out_6049361165244039606[41] = 0;
   out_6049361165244039606[42] = 0;
   out_6049361165244039606[43] = 0;
   out_6049361165244039606[44] = 0;
   out_6049361165244039606[45] = 0;
   out_6049361165244039606[46] = 0;
   out_6049361165244039606[47] = 0;
   out_6049361165244039606[48] = 1.0;
   out_6049361165244039606[49] = 0;
   out_6049361165244039606[50] = 0;
   out_6049361165244039606[51] = 0;
   out_6049361165244039606[52] = 0;
   out_6049361165244039606[53] = 0;
   out_6049361165244039606[54] = 0;
   out_6049361165244039606[55] = 0;
   out_6049361165244039606[56] = 0;
   out_6049361165244039606[57] = 0;
   out_6049361165244039606[58] = 0;
   out_6049361165244039606[59] = 0;
   out_6049361165244039606[60] = 1.0;
   out_6049361165244039606[61] = 0;
   out_6049361165244039606[62] = 0;
   out_6049361165244039606[63] = 0;
   out_6049361165244039606[64] = 0;
   out_6049361165244039606[65] = 0;
   out_6049361165244039606[66] = 0;
   out_6049361165244039606[67] = 0;
   out_6049361165244039606[68] = 0;
   out_6049361165244039606[69] = 0;
   out_6049361165244039606[70] = 0;
   out_6049361165244039606[71] = 0;
   out_6049361165244039606[72] = 1.0;
   out_6049361165244039606[73] = 0;
   out_6049361165244039606[74] = 0;
   out_6049361165244039606[75] = 0;
   out_6049361165244039606[76] = 0;
   out_6049361165244039606[77] = 0;
   out_6049361165244039606[78] = 0;
   out_6049361165244039606[79] = 0;
   out_6049361165244039606[80] = 0;
   out_6049361165244039606[81] = 0;
   out_6049361165244039606[82] = 0;
   out_6049361165244039606[83] = 0;
   out_6049361165244039606[84] = 1.0;
   out_6049361165244039606[85] = 0;
   out_6049361165244039606[86] = 0;
   out_6049361165244039606[87] = 0;
   out_6049361165244039606[88] = 0;
   out_6049361165244039606[89] = 0;
   out_6049361165244039606[90] = 0;
   out_6049361165244039606[91] = 0;
   out_6049361165244039606[92] = 0;
   out_6049361165244039606[93] = 0;
   out_6049361165244039606[94] = 0;
   out_6049361165244039606[95] = 0;
   out_6049361165244039606[96] = 1.0;
   out_6049361165244039606[97] = 0;
   out_6049361165244039606[98] = 0;
   out_6049361165244039606[99] = 0;
   out_6049361165244039606[100] = 0;
   out_6049361165244039606[101] = 0;
   out_6049361165244039606[102] = 0;
   out_6049361165244039606[103] = 0;
   out_6049361165244039606[104] = 0;
   out_6049361165244039606[105] = 0;
   out_6049361165244039606[106] = 0;
   out_6049361165244039606[107] = 0;
   out_6049361165244039606[108] = 1.0;
   out_6049361165244039606[109] = 0;
   out_6049361165244039606[110] = 0;
   out_6049361165244039606[111] = 0;
   out_6049361165244039606[112] = 0;
   out_6049361165244039606[113] = 0;
   out_6049361165244039606[114] = 0;
   out_6049361165244039606[115] = 0;
   out_6049361165244039606[116] = 0;
   out_6049361165244039606[117] = 0;
   out_6049361165244039606[118] = 0;
   out_6049361165244039606[119] = 0;
   out_6049361165244039606[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_2606141003197075150) {
   out_2606141003197075150[0] = dt*state[3] + state[0];
   out_2606141003197075150[1] = dt*state[4] + state[1];
   out_2606141003197075150[2] = dt*state[5] + state[2];
   out_2606141003197075150[3] = state[3];
   out_2606141003197075150[4] = state[4];
   out_2606141003197075150[5] = state[5];
   out_2606141003197075150[6] = dt*state[7] + state[6];
   out_2606141003197075150[7] = dt*state[8] + state[7];
   out_2606141003197075150[8] = state[8];
   out_2606141003197075150[9] = state[9];
   out_2606141003197075150[10] = state[10];
}
void F_fun(double *state, double dt, double *out_6801608056846298175) {
   out_6801608056846298175[0] = 1;
   out_6801608056846298175[1] = 0;
   out_6801608056846298175[2] = 0;
   out_6801608056846298175[3] = dt;
   out_6801608056846298175[4] = 0;
   out_6801608056846298175[5] = 0;
   out_6801608056846298175[6] = 0;
   out_6801608056846298175[7] = 0;
   out_6801608056846298175[8] = 0;
   out_6801608056846298175[9] = 0;
   out_6801608056846298175[10] = 0;
   out_6801608056846298175[11] = 0;
   out_6801608056846298175[12] = 1;
   out_6801608056846298175[13] = 0;
   out_6801608056846298175[14] = 0;
   out_6801608056846298175[15] = dt;
   out_6801608056846298175[16] = 0;
   out_6801608056846298175[17] = 0;
   out_6801608056846298175[18] = 0;
   out_6801608056846298175[19] = 0;
   out_6801608056846298175[20] = 0;
   out_6801608056846298175[21] = 0;
   out_6801608056846298175[22] = 0;
   out_6801608056846298175[23] = 0;
   out_6801608056846298175[24] = 1;
   out_6801608056846298175[25] = 0;
   out_6801608056846298175[26] = 0;
   out_6801608056846298175[27] = dt;
   out_6801608056846298175[28] = 0;
   out_6801608056846298175[29] = 0;
   out_6801608056846298175[30] = 0;
   out_6801608056846298175[31] = 0;
   out_6801608056846298175[32] = 0;
   out_6801608056846298175[33] = 0;
   out_6801608056846298175[34] = 0;
   out_6801608056846298175[35] = 0;
   out_6801608056846298175[36] = 1;
   out_6801608056846298175[37] = 0;
   out_6801608056846298175[38] = 0;
   out_6801608056846298175[39] = 0;
   out_6801608056846298175[40] = 0;
   out_6801608056846298175[41] = 0;
   out_6801608056846298175[42] = 0;
   out_6801608056846298175[43] = 0;
   out_6801608056846298175[44] = 0;
   out_6801608056846298175[45] = 0;
   out_6801608056846298175[46] = 0;
   out_6801608056846298175[47] = 0;
   out_6801608056846298175[48] = 1;
   out_6801608056846298175[49] = 0;
   out_6801608056846298175[50] = 0;
   out_6801608056846298175[51] = 0;
   out_6801608056846298175[52] = 0;
   out_6801608056846298175[53] = 0;
   out_6801608056846298175[54] = 0;
   out_6801608056846298175[55] = 0;
   out_6801608056846298175[56] = 0;
   out_6801608056846298175[57] = 0;
   out_6801608056846298175[58] = 0;
   out_6801608056846298175[59] = 0;
   out_6801608056846298175[60] = 1;
   out_6801608056846298175[61] = 0;
   out_6801608056846298175[62] = 0;
   out_6801608056846298175[63] = 0;
   out_6801608056846298175[64] = 0;
   out_6801608056846298175[65] = 0;
   out_6801608056846298175[66] = 0;
   out_6801608056846298175[67] = 0;
   out_6801608056846298175[68] = 0;
   out_6801608056846298175[69] = 0;
   out_6801608056846298175[70] = 0;
   out_6801608056846298175[71] = 0;
   out_6801608056846298175[72] = 1;
   out_6801608056846298175[73] = dt;
   out_6801608056846298175[74] = 0;
   out_6801608056846298175[75] = 0;
   out_6801608056846298175[76] = 0;
   out_6801608056846298175[77] = 0;
   out_6801608056846298175[78] = 0;
   out_6801608056846298175[79] = 0;
   out_6801608056846298175[80] = 0;
   out_6801608056846298175[81] = 0;
   out_6801608056846298175[82] = 0;
   out_6801608056846298175[83] = 0;
   out_6801608056846298175[84] = 1;
   out_6801608056846298175[85] = dt;
   out_6801608056846298175[86] = 0;
   out_6801608056846298175[87] = 0;
   out_6801608056846298175[88] = 0;
   out_6801608056846298175[89] = 0;
   out_6801608056846298175[90] = 0;
   out_6801608056846298175[91] = 0;
   out_6801608056846298175[92] = 0;
   out_6801608056846298175[93] = 0;
   out_6801608056846298175[94] = 0;
   out_6801608056846298175[95] = 0;
   out_6801608056846298175[96] = 1;
   out_6801608056846298175[97] = 0;
   out_6801608056846298175[98] = 0;
   out_6801608056846298175[99] = 0;
   out_6801608056846298175[100] = 0;
   out_6801608056846298175[101] = 0;
   out_6801608056846298175[102] = 0;
   out_6801608056846298175[103] = 0;
   out_6801608056846298175[104] = 0;
   out_6801608056846298175[105] = 0;
   out_6801608056846298175[106] = 0;
   out_6801608056846298175[107] = 0;
   out_6801608056846298175[108] = 1;
   out_6801608056846298175[109] = 0;
   out_6801608056846298175[110] = 0;
   out_6801608056846298175[111] = 0;
   out_6801608056846298175[112] = 0;
   out_6801608056846298175[113] = 0;
   out_6801608056846298175[114] = 0;
   out_6801608056846298175[115] = 0;
   out_6801608056846298175[116] = 0;
   out_6801608056846298175[117] = 0;
   out_6801608056846298175[118] = 0;
   out_6801608056846298175[119] = 0;
   out_6801608056846298175[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_8879001659981122149) {
   out_8879001659981122149[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_7680411267998003626) {
   out_7680411267998003626[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7680411267998003626[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7680411267998003626[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7680411267998003626[3] = 0;
   out_7680411267998003626[4] = 0;
   out_7680411267998003626[5] = 0;
   out_7680411267998003626[6] = 1;
   out_7680411267998003626[7] = 0;
   out_7680411267998003626[8] = 0;
   out_7680411267998003626[9] = 0;
   out_7680411267998003626[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_407258811367348151) {
   out_407258811367348151[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_7476595035387964760) {
   out_7476595035387964760[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7476595035387964760[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7476595035387964760[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7476595035387964760[3] = 0;
   out_7476595035387964760[4] = 0;
   out_7476595035387964760[5] = 0;
   out_7476595035387964760[6] = 1;
   out_7476595035387964760[7] = 0;
   out_7476595035387964760[8] = 0;
   out_7476595035387964760[9] = 1;
   out_7476595035387964760[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_7261540144599269436) {
   out_7261540144599269436[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_7983380131701770826) {
   out_7983380131701770826[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7983380131701770826[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7983380131701770826[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7983380131701770826[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7983380131701770826[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7983380131701770826[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7983380131701770826[6] = 0;
   out_7983380131701770826[7] = 1;
   out_7983380131701770826[8] = 0;
   out_7983380131701770826[9] = 0;
   out_7983380131701770826[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_7261540144599269436) {
   out_7261540144599269436[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_7983380131701770826) {
   out_7983380131701770826[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7983380131701770826[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7983380131701770826[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7983380131701770826[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7983380131701770826[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7983380131701770826[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_7983380131701770826[6] = 0;
   out_7983380131701770826[7] = 1;
   out_7983380131701770826[8] = 0;
   out_7983380131701770826[9] = 0;
   out_7983380131701770826[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_533379885840528252) {
  err_fun(nom_x, delta_x, out_533379885840528252);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8671545501629514785) {
  inv_err_fun(nom_x, true_x, out_8671545501629514785);
}
void gnss_H_mod_fun(double *state, double *out_6049361165244039606) {
  H_mod_fun(state, out_6049361165244039606);
}
void gnss_f_fun(double *state, double dt, double *out_2606141003197075150) {
  f_fun(state,  dt, out_2606141003197075150);
}
void gnss_F_fun(double *state, double dt, double *out_6801608056846298175) {
  F_fun(state,  dt, out_6801608056846298175);
}
void gnss_h_6(double *state, double *sat_pos, double *out_8879001659981122149) {
  h_6(state, sat_pos, out_8879001659981122149);
}
void gnss_H_6(double *state, double *sat_pos, double *out_7680411267998003626) {
  H_6(state, sat_pos, out_7680411267998003626);
}
void gnss_h_20(double *state, double *sat_pos, double *out_407258811367348151) {
  h_20(state, sat_pos, out_407258811367348151);
}
void gnss_H_20(double *state, double *sat_pos, double *out_7476595035387964760) {
  H_20(state, sat_pos, out_7476595035387964760);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_7261540144599269436) {
  h_7(state, sat_pos_vel, out_7261540144599269436);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_7983380131701770826) {
  H_7(state, sat_pos_vel, out_7983380131701770826);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_7261540144599269436) {
  h_21(state, sat_pos_vel, out_7261540144599269436);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_7983380131701770826) {
  H_21(state, sat_pos_vel, out_7983380131701770826);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
