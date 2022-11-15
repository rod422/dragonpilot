#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_209133747298070962);
void live_err_fun(double *nom_x, double *delta_x, double *out_1127464374514398162);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8453671828857282665);
void live_H_mod_fun(double *state, double *out_6465515215284678187);
void live_f_fun(double *state, double dt, double *out_5601687001448067779);
void live_F_fun(double *state, double dt, double *out_3647176865632245870);
void live_h_4(double *state, double *unused, double *out_8686664304179300181);
void live_H_4(double *state, double *unused, double *out_4737414633741096164);
void live_h_9(double *state, double *unused, double *out_5565585308076199796);
void live_H_9(double *state, double *unused, double *out_6422110504704007982);
void live_h_10(double *state, double *unused, double *out_1353507015058286203);
void live_H_10(double *state, double *unused, double *out_6027635784935919290);
void live_h_12(double *state, double *unused, double *out_7631493099303276238);
void live_H_12(double *state, double *unused, double *out_8689873031936493657);
void live_h_35(double *state, double *unused, double *out_3508652193024788518);
void live_H_35(double *state, double *unused, double *out_5944309999611479948);
void live_h_32(double *state, double *unused, double *out_8916862548391282088);
void live_H_32(double *state, double *unused, double *out_6256931694498924976);
void live_h_13(double *state, double *unused, double *out_108113376412138945);
void live_H_13(double *state, double *unused, double *out_5820312086696149770);
void live_h_14(double *state, double *unused, double *out_5565585308076199796);
void live_H_14(double *state, double *unused, double *out_6422110504704007982);
void live_h_33(double *state, double *unused, double *out_6196126748769845638);
void live_H_33(double *state, double *unused, double *out_7192110377956990472);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}