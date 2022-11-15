#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_533379885840528252);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8671545501629514785);
void gnss_H_mod_fun(double *state, double *out_6049361165244039606);
void gnss_f_fun(double *state, double dt, double *out_2606141003197075150);
void gnss_F_fun(double *state, double dt, double *out_6801608056846298175);
void gnss_h_6(double *state, double *sat_pos, double *out_8879001659981122149);
void gnss_H_6(double *state, double *sat_pos, double *out_7680411267998003626);
void gnss_h_20(double *state, double *sat_pos, double *out_407258811367348151);
void gnss_H_20(double *state, double *sat_pos, double *out_7476595035387964760);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_7261540144599269436);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_7983380131701770826);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_7261540144599269436);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_7983380131701770826);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}