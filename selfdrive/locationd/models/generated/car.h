#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_5853957832074453940);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_737226202460666342);
void car_H_mod_fun(double *state, double *out_1375279494024935349);
void car_f_fun(double *state, double dt, double *out_4251696200088534572);
void car_F_fun(double *state, double dt, double *out_1908927242052538900);
void car_h_25(double *state, double *unused, double *out_6553259077698250498);
void car_H_25(double *state, double *unused, double *out_75408611009746463);
void car_h_24(double *state, double *unused, double *out_6203684692643907106);
void car_H_24(double *state, double *unused, double *out_3633334209812069599);
void car_h_30(double *state, double *unused, double *out_3936353391069076669);
void car_H_30(double *state, double *unused, double *out_6992098952501363218);
void car_h_26(double *state, double *unused, double *out_6290393570737091919);
void car_H_26(double *state, double *unused, double *out_3666094707864309761);
void car_h_27(double *state, double *unused, double *out_1931073048242142184);
void car_H_27(double *state, double *unused, double *out_4817335640700938307);
void car_h_29(double *state, double *unused, double *out_8752405768646736991);
void car_H_29(double *state, double *unused, double *out_3103972913831387274);
void car_h_28(double *state, double *unused, double *out_4967859099686474869);
void car_H_28(double *state, double *unused, double *out_1978426103238143300);
void car_h_31(double *state, double *unused, double *out_7848996981607604908);
void car_H_31(double *state, double *unused, double *out_106054572886706891);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}