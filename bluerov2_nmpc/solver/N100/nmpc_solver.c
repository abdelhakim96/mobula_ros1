/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "nmpc_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int nmpc_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
nmpcWorkspace.state[0] = nmpcVariables.x[lRun1 * 8];
nmpcWorkspace.state[1] = nmpcVariables.x[lRun1 * 8 + 1];
nmpcWorkspace.state[2] = nmpcVariables.x[lRun1 * 8 + 2];
nmpcWorkspace.state[3] = nmpcVariables.x[lRun1 * 8 + 3];
nmpcWorkspace.state[4] = nmpcVariables.x[lRun1 * 8 + 4];
nmpcWorkspace.state[5] = nmpcVariables.x[lRun1 * 8 + 5];
nmpcWorkspace.state[6] = nmpcVariables.x[lRun1 * 8 + 6];
nmpcWorkspace.state[7] = nmpcVariables.x[lRun1 * 8 + 7];

nmpcWorkspace.state[104] = nmpcVariables.u[lRun1 * 4];
nmpcWorkspace.state[105] = nmpcVariables.u[lRun1 * 4 + 1];
nmpcWorkspace.state[106] = nmpcVariables.u[lRun1 * 4 + 2];
nmpcWorkspace.state[107] = nmpcVariables.u[lRun1 * 4 + 3];
nmpcWorkspace.state[108] = nmpcVariables.od[lRun1 * 3];
nmpcWorkspace.state[109] = nmpcVariables.od[lRun1 * 3 + 1];
nmpcWorkspace.state[110] = nmpcVariables.od[lRun1 * 3 + 2];

ret = nmpc_integrate(nmpcWorkspace.state, 1);

nmpcWorkspace.d[lRun1 * 8] = nmpcWorkspace.state[0] - nmpcVariables.x[lRun1 * 8 + 8];
nmpcWorkspace.d[lRun1 * 8 + 1] = nmpcWorkspace.state[1] - nmpcVariables.x[lRun1 * 8 + 9];
nmpcWorkspace.d[lRun1 * 8 + 2] = nmpcWorkspace.state[2] - nmpcVariables.x[lRun1 * 8 + 10];
nmpcWorkspace.d[lRun1 * 8 + 3] = nmpcWorkspace.state[3] - nmpcVariables.x[lRun1 * 8 + 11];
nmpcWorkspace.d[lRun1 * 8 + 4] = nmpcWorkspace.state[4] - nmpcVariables.x[lRun1 * 8 + 12];
nmpcWorkspace.d[lRun1 * 8 + 5] = nmpcWorkspace.state[5] - nmpcVariables.x[lRun1 * 8 + 13];
nmpcWorkspace.d[lRun1 * 8 + 6] = nmpcWorkspace.state[6] - nmpcVariables.x[lRun1 * 8 + 14];
nmpcWorkspace.d[lRun1 * 8 + 7] = nmpcWorkspace.state[7] - nmpcVariables.x[lRun1 * 8 + 15];

nmpcWorkspace.evGx[lRun1 * 64] = nmpcWorkspace.state[8];
nmpcWorkspace.evGx[lRun1 * 64 + 1] = nmpcWorkspace.state[9];
nmpcWorkspace.evGx[lRun1 * 64 + 2] = nmpcWorkspace.state[10];
nmpcWorkspace.evGx[lRun1 * 64 + 3] = nmpcWorkspace.state[11];
nmpcWorkspace.evGx[lRun1 * 64 + 4] = nmpcWorkspace.state[12];
nmpcWorkspace.evGx[lRun1 * 64 + 5] = nmpcWorkspace.state[13];
nmpcWorkspace.evGx[lRun1 * 64 + 6] = nmpcWorkspace.state[14];
nmpcWorkspace.evGx[lRun1 * 64 + 7] = nmpcWorkspace.state[15];
nmpcWorkspace.evGx[lRun1 * 64 + 8] = nmpcWorkspace.state[16];
nmpcWorkspace.evGx[lRun1 * 64 + 9] = nmpcWorkspace.state[17];
nmpcWorkspace.evGx[lRun1 * 64 + 10] = nmpcWorkspace.state[18];
nmpcWorkspace.evGx[lRun1 * 64 + 11] = nmpcWorkspace.state[19];
nmpcWorkspace.evGx[lRun1 * 64 + 12] = nmpcWorkspace.state[20];
nmpcWorkspace.evGx[lRun1 * 64 + 13] = nmpcWorkspace.state[21];
nmpcWorkspace.evGx[lRun1 * 64 + 14] = nmpcWorkspace.state[22];
nmpcWorkspace.evGx[lRun1 * 64 + 15] = nmpcWorkspace.state[23];
nmpcWorkspace.evGx[lRun1 * 64 + 16] = nmpcWorkspace.state[24];
nmpcWorkspace.evGx[lRun1 * 64 + 17] = nmpcWorkspace.state[25];
nmpcWorkspace.evGx[lRun1 * 64 + 18] = nmpcWorkspace.state[26];
nmpcWorkspace.evGx[lRun1 * 64 + 19] = nmpcWorkspace.state[27];
nmpcWorkspace.evGx[lRun1 * 64 + 20] = nmpcWorkspace.state[28];
nmpcWorkspace.evGx[lRun1 * 64 + 21] = nmpcWorkspace.state[29];
nmpcWorkspace.evGx[lRun1 * 64 + 22] = nmpcWorkspace.state[30];
nmpcWorkspace.evGx[lRun1 * 64 + 23] = nmpcWorkspace.state[31];
nmpcWorkspace.evGx[lRun1 * 64 + 24] = nmpcWorkspace.state[32];
nmpcWorkspace.evGx[lRun1 * 64 + 25] = nmpcWorkspace.state[33];
nmpcWorkspace.evGx[lRun1 * 64 + 26] = nmpcWorkspace.state[34];
nmpcWorkspace.evGx[lRun1 * 64 + 27] = nmpcWorkspace.state[35];
nmpcWorkspace.evGx[lRun1 * 64 + 28] = nmpcWorkspace.state[36];
nmpcWorkspace.evGx[lRun1 * 64 + 29] = nmpcWorkspace.state[37];
nmpcWorkspace.evGx[lRun1 * 64 + 30] = nmpcWorkspace.state[38];
nmpcWorkspace.evGx[lRun1 * 64 + 31] = nmpcWorkspace.state[39];
nmpcWorkspace.evGx[lRun1 * 64 + 32] = nmpcWorkspace.state[40];
nmpcWorkspace.evGx[lRun1 * 64 + 33] = nmpcWorkspace.state[41];
nmpcWorkspace.evGx[lRun1 * 64 + 34] = nmpcWorkspace.state[42];
nmpcWorkspace.evGx[lRun1 * 64 + 35] = nmpcWorkspace.state[43];
nmpcWorkspace.evGx[lRun1 * 64 + 36] = nmpcWorkspace.state[44];
nmpcWorkspace.evGx[lRun1 * 64 + 37] = nmpcWorkspace.state[45];
nmpcWorkspace.evGx[lRun1 * 64 + 38] = nmpcWorkspace.state[46];
nmpcWorkspace.evGx[lRun1 * 64 + 39] = nmpcWorkspace.state[47];
nmpcWorkspace.evGx[lRun1 * 64 + 40] = nmpcWorkspace.state[48];
nmpcWorkspace.evGx[lRun1 * 64 + 41] = nmpcWorkspace.state[49];
nmpcWorkspace.evGx[lRun1 * 64 + 42] = nmpcWorkspace.state[50];
nmpcWorkspace.evGx[lRun1 * 64 + 43] = nmpcWorkspace.state[51];
nmpcWorkspace.evGx[lRun1 * 64 + 44] = nmpcWorkspace.state[52];
nmpcWorkspace.evGx[lRun1 * 64 + 45] = nmpcWorkspace.state[53];
nmpcWorkspace.evGx[lRun1 * 64 + 46] = nmpcWorkspace.state[54];
nmpcWorkspace.evGx[lRun1 * 64 + 47] = nmpcWorkspace.state[55];
nmpcWorkspace.evGx[lRun1 * 64 + 48] = nmpcWorkspace.state[56];
nmpcWorkspace.evGx[lRun1 * 64 + 49] = nmpcWorkspace.state[57];
nmpcWorkspace.evGx[lRun1 * 64 + 50] = nmpcWorkspace.state[58];
nmpcWorkspace.evGx[lRun1 * 64 + 51] = nmpcWorkspace.state[59];
nmpcWorkspace.evGx[lRun1 * 64 + 52] = nmpcWorkspace.state[60];
nmpcWorkspace.evGx[lRun1 * 64 + 53] = nmpcWorkspace.state[61];
nmpcWorkspace.evGx[lRun1 * 64 + 54] = nmpcWorkspace.state[62];
nmpcWorkspace.evGx[lRun1 * 64 + 55] = nmpcWorkspace.state[63];
nmpcWorkspace.evGx[lRun1 * 64 + 56] = nmpcWorkspace.state[64];
nmpcWorkspace.evGx[lRun1 * 64 + 57] = nmpcWorkspace.state[65];
nmpcWorkspace.evGx[lRun1 * 64 + 58] = nmpcWorkspace.state[66];
nmpcWorkspace.evGx[lRun1 * 64 + 59] = nmpcWorkspace.state[67];
nmpcWorkspace.evGx[lRun1 * 64 + 60] = nmpcWorkspace.state[68];
nmpcWorkspace.evGx[lRun1 * 64 + 61] = nmpcWorkspace.state[69];
nmpcWorkspace.evGx[lRun1 * 64 + 62] = nmpcWorkspace.state[70];
nmpcWorkspace.evGx[lRun1 * 64 + 63] = nmpcWorkspace.state[71];

nmpcWorkspace.evGu[lRun1 * 32] = nmpcWorkspace.state[72];
nmpcWorkspace.evGu[lRun1 * 32 + 1] = nmpcWorkspace.state[73];
nmpcWorkspace.evGu[lRun1 * 32 + 2] = nmpcWorkspace.state[74];
nmpcWorkspace.evGu[lRun1 * 32 + 3] = nmpcWorkspace.state[75];
nmpcWorkspace.evGu[lRun1 * 32 + 4] = nmpcWorkspace.state[76];
nmpcWorkspace.evGu[lRun1 * 32 + 5] = nmpcWorkspace.state[77];
nmpcWorkspace.evGu[lRun1 * 32 + 6] = nmpcWorkspace.state[78];
nmpcWorkspace.evGu[lRun1 * 32 + 7] = nmpcWorkspace.state[79];
nmpcWorkspace.evGu[lRun1 * 32 + 8] = nmpcWorkspace.state[80];
nmpcWorkspace.evGu[lRun1 * 32 + 9] = nmpcWorkspace.state[81];
nmpcWorkspace.evGu[lRun1 * 32 + 10] = nmpcWorkspace.state[82];
nmpcWorkspace.evGu[lRun1 * 32 + 11] = nmpcWorkspace.state[83];
nmpcWorkspace.evGu[lRun1 * 32 + 12] = nmpcWorkspace.state[84];
nmpcWorkspace.evGu[lRun1 * 32 + 13] = nmpcWorkspace.state[85];
nmpcWorkspace.evGu[lRun1 * 32 + 14] = nmpcWorkspace.state[86];
nmpcWorkspace.evGu[lRun1 * 32 + 15] = nmpcWorkspace.state[87];
nmpcWorkspace.evGu[lRun1 * 32 + 16] = nmpcWorkspace.state[88];
nmpcWorkspace.evGu[lRun1 * 32 + 17] = nmpcWorkspace.state[89];
nmpcWorkspace.evGu[lRun1 * 32 + 18] = nmpcWorkspace.state[90];
nmpcWorkspace.evGu[lRun1 * 32 + 19] = nmpcWorkspace.state[91];
nmpcWorkspace.evGu[lRun1 * 32 + 20] = nmpcWorkspace.state[92];
nmpcWorkspace.evGu[lRun1 * 32 + 21] = nmpcWorkspace.state[93];
nmpcWorkspace.evGu[lRun1 * 32 + 22] = nmpcWorkspace.state[94];
nmpcWorkspace.evGu[lRun1 * 32 + 23] = nmpcWorkspace.state[95];
nmpcWorkspace.evGu[lRun1 * 32 + 24] = nmpcWorkspace.state[96];
nmpcWorkspace.evGu[lRun1 * 32 + 25] = nmpcWorkspace.state[97];
nmpcWorkspace.evGu[lRun1 * 32 + 26] = nmpcWorkspace.state[98];
nmpcWorkspace.evGu[lRun1 * 32 + 27] = nmpcWorkspace.state[99];
nmpcWorkspace.evGu[lRun1 * 32 + 28] = nmpcWorkspace.state[100];
nmpcWorkspace.evGu[lRun1 * 32 + 29] = nmpcWorkspace.state[101];
nmpcWorkspace.evGu[lRun1 * 32 + 30] = nmpcWorkspace.state[102];
nmpcWorkspace.evGu[lRun1 * 32 + 31] = nmpcWorkspace.state[103];
}
return ret;
}

void nmpc_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 8;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = u[0];
out[9] = u[1];
out[10] = u[2];
out[11] = u[3];
}

void nmpc_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
}

void nmpc_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ2[20] = +tmpObjS[20];
tmpQ2[21] = +tmpObjS[21];
tmpQ2[22] = +tmpObjS[22];
tmpQ2[23] = +tmpObjS[23];
tmpQ2[24] = +tmpObjS[24];
tmpQ2[25] = +tmpObjS[25];
tmpQ2[26] = +tmpObjS[26];
tmpQ2[27] = +tmpObjS[27];
tmpQ2[28] = +tmpObjS[28];
tmpQ2[29] = +tmpObjS[29];
tmpQ2[30] = +tmpObjS[30];
tmpQ2[31] = +tmpObjS[31];
tmpQ2[32] = +tmpObjS[32];
tmpQ2[33] = +tmpObjS[33];
tmpQ2[34] = +tmpObjS[34];
tmpQ2[35] = +tmpObjS[35];
tmpQ2[36] = +tmpObjS[36];
tmpQ2[37] = +tmpObjS[37];
tmpQ2[38] = +tmpObjS[38];
tmpQ2[39] = +tmpObjS[39];
tmpQ2[40] = +tmpObjS[40];
tmpQ2[41] = +tmpObjS[41];
tmpQ2[42] = +tmpObjS[42];
tmpQ2[43] = +tmpObjS[43];
tmpQ2[44] = +tmpObjS[44];
tmpQ2[45] = +tmpObjS[45];
tmpQ2[46] = +tmpObjS[46];
tmpQ2[47] = +tmpObjS[47];
tmpQ2[48] = +tmpObjS[48];
tmpQ2[49] = +tmpObjS[49];
tmpQ2[50] = +tmpObjS[50];
tmpQ2[51] = +tmpObjS[51];
tmpQ2[52] = +tmpObjS[52];
tmpQ2[53] = +tmpObjS[53];
tmpQ2[54] = +tmpObjS[54];
tmpQ2[55] = +tmpObjS[55];
tmpQ2[56] = +tmpObjS[56];
tmpQ2[57] = +tmpObjS[57];
tmpQ2[58] = +tmpObjS[58];
tmpQ2[59] = +tmpObjS[59];
tmpQ2[60] = +tmpObjS[60];
tmpQ2[61] = +tmpObjS[61];
tmpQ2[62] = +tmpObjS[62];
tmpQ2[63] = +tmpObjS[63];
tmpQ2[64] = +tmpObjS[64];
tmpQ2[65] = +tmpObjS[65];
tmpQ2[66] = +tmpObjS[66];
tmpQ2[67] = +tmpObjS[67];
tmpQ2[68] = +tmpObjS[68];
tmpQ2[69] = +tmpObjS[69];
tmpQ2[70] = +tmpObjS[70];
tmpQ2[71] = +tmpObjS[71];
tmpQ2[72] = +tmpObjS[72];
tmpQ2[73] = +tmpObjS[73];
tmpQ2[74] = +tmpObjS[74];
tmpQ2[75] = +tmpObjS[75];
tmpQ2[76] = +tmpObjS[76];
tmpQ2[77] = +tmpObjS[77];
tmpQ2[78] = +tmpObjS[78];
tmpQ2[79] = +tmpObjS[79];
tmpQ2[80] = +tmpObjS[80];
tmpQ2[81] = +tmpObjS[81];
tmpQ2[82] = +tmpObjS[82];
tmpQ2[83] = +tmpObjS[83];
tmpQ2[84] = +tmpObjS[84];
tmpQ2[85] = +tmpObjS[85];
tmpQ2[86] = +tmpObjS[86];
tmpQ2[87] = +tmpObjS[87];
tmpQ2[88] = +tmpObjS[88];
tmpQ2[89] = +tmpObjS[89];
tmpQ2[90] = +tmpObjS[90];
tmpQ2[91] = +tmpObjS[91];
tmpQ2[92] = +tmpObjS[92];
tmpQ2[93] = +tmpObjS[93];
tmpQ2[94] = +tmpObjS[94];
tmpQ2[95] = +tmpObjS[95];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[4];
tmpQ1[5] = + tmpQ2[5];
tmpQ1[6] = + tmpQ2[6];
tmpQ1[7] = + tmpQ2[7];
tmpQ1[8] = + tmpQ2[12];
tmpQ1[9] = + tmpQ2[13];
tmpQ1[10] = + tmpQ2[14];
tmpQ1[11] = + tmpQ2[15];
tmpQ1[12] = + tmpQ2[16];
tmpQ1[13] = + tmpQ2[17];
tmpQ1[14] = + tmpQ2[18];
tmpQ1[15] = + tmpQ2[19];
tmpQ1[16] = + tmpQ2[24];
tmpQ1[17] = + tmpQ2[25];
tmpQ1[18] = + tmpQ2[26];
tmpQ1[19] = + tmpQ2[27];
tmpQ1[20] = + tmpQ2[28];
tmpQ1[21] = + tmpQ2[29];
tmpQ1[22] = + tmpQ2[30];
tmpQ1[23] = + tmpQ2[31];
tmpQ1[24] = + tmpQ2[36];
tmpQ1[25] = + tmpQ2[37];
tmpQ1[26] = + tmpQ2[38];
tmpQ1[27] = + tmpQ2[39];
tmpQ1[28] = + tmpQ2[40];
tmpQ1[29] = + tmpQ2[41];
tmpQ1[30] = + tmpQ2[42];
tmpQ1[31] = + tmpQ2[43];
tmpQ1[32] = + tmpQ2[48];
tmpQ1[33] = + tmpQ2[49];
tmpQ1[34] = + tmpQ2[50];
tmpQ1[35] = + tmpQ2[51];
tmpQ1[36] = + tmpQ2[52];
tmpQ1[37] = + tmpQ2[53];
tmpQ1[38] = + tmpQ2[54];
tmpQ1[39] = + tmpQ2[55];
tmpQ1[40] = + tmpQ2[60];
tmpQ1[41] = + tmpQ2[61];
tmpQ1[42] = + tmpQ2[62];
tmpQ1[43] = + tmpQ2[63];
tmpQ1[44] = + tmpQ2[64];
tmpQ1[45] = + tmpQ2[65];
tmpQ1[46] = + tmpQ2[66];
tmpQ1[47] = + tmpQ2[67];
tmpQ1[48] = + tmpQ2[72];
tmpQ1[49] = + tmpQ2[73];
tmpQ1[50] = + tmpQ2[74];
tmpQ1[51] = + tmpQ2[75];
tmpQ1[52] = + tmpQ2[76];
tmpQ1[53] = + tmpQ2[77];
tmpQ1[54] = + tmpQ2[78];
tmpQ1[55] = + tmpQ2[79];
tmpQ1[56] = + tmpQ2[84];
tmpQ1[57] = + tmpQ2[85];
tmpQ1[58] = + tmpQ2[86];
tmpQ1[59] = + tmpQ2[87];
tmpQ1[60] = + tmpQ2[88];
tmpQ1[61] = + tmpQ2[89];
tmpQ1[62] = + tmpQ2[90];
tmpQ1[63] = + tmpQ2[91];
}

void nmpc_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[96];
tmpR2[1] = +tmpObjS[97];
tmpR2[2] = +tmpObjS[98];
tmpR2[3] = +tmpObjS[99];
tmpR2[4] = +tmpObjS[100];
tmpR2[5] = +tmpObjS[101];
tmpR2[6] = +tmpObjS[102];
tmpR2[7] = +tmpObjS[103];
tmpR2[8] = +tmpObjS[104];
tmpR2[9] = +tmpObjS[105];
tmpR2[10] = +tmpObjS[106];
tmpR2[11] = +tmpObjS[107];
tmpR2[12] = +tmpObjS[108];
tmpR2[13] = +tmpObjS[109];
tmpR2[14] = +tmpObjS[110];
tmpR2[15] = +tmpObjS[111];
tmpR2[16] = +tmpObjS[112];
tmpR2[17] = +tmpObjS[113];
tmpR2[18] = +tmpObjS[114];
tmpR2[19] = +tmpObjS[115];
tmpR2[20] = +tmpObjS[116];
tmpR2[21] = +tmpObjS[117];
tmpR2[22] = +tmpObjS[118];
tmpR2[23] = +tmpObjS[119];
tmpR2[24] = +tmpObjS[120];
tmpR2[25] = +tmpObjS[121];
tmpR2[26] = +tmpObjS[122];
tmpR2[27] = +tmpObjS[123];
tmpR2[28] = +tmpObjS[124];
tmpR2[29] = +tmpObjS[125];
tmpR2[30] = +tmpObjS[126];
tmpR2[31] = +tmpObjS[127];
tmpR2[32] = +tmpObjS[128];
tmpR2[33] = +tmpObjS[129];
tmpR2[34] = +tmpObjS[130];
tmpR2[35] = +tmpObjS[131];
tmpR2[36] = +tmpObjS[132];
tmpR2[37] = +tmpObjS[133];
tmpR2[38] = +tmpObjS[134];
tmpR2[39] = +tmpObjS[135];
tmpR2[40] = +tmpObjS[136];
tmpR2[41] = +tmpObjS[137];
tmpR2[42] = +tmpObjS[138];
tmpR2[43] = +tmpObjS[139];
tmpR2[44] = +tmpObjS[140];
tmpR2[45] = +tmpObjS[141];
tmpR2[46] = +tmpObjS[142];
tmpR2[47] = +tmpObjS[143];
tmpR1[0] = + tmpR2[8];
tmpR1[1] = + tmpR2[9];
tmpR1[2] = + tmpR2[10];
tmpR1[3] = + tmpR2[11];
tmpR1[4] = + tmpR2[20];
tmpR1[5] = + tmpR2[21];
tmpR1[6] = + tmpR2[22];
tmpR1[7] = + tmpR2[23];
tmpR1[8] = + tmpR2[32];
tmpR1[9] = + tmpR2[33];
tmpR1[10] = + tmpR2[34];
tmpR1[11] = + tmpR2[35];
tmpR1[12] = + tmpR2[44];
tmpR1[13] = + tmpR2[45];
tmpR1[14] = + tmpR2[46];
tmpR1[15] = + tmpR2[47];
}

void nmpc_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN2[16] = +tmpObjSEndTerm[16];
tmpQN2[17] = +tmpObjSEndTerm[17];
tmpQN2[18] = +tmpObjSEndTerm[18];
tmpQN2[19] = +tmpObjSEndTerm[19];
tmpQN2[20] = +tmpObjSEndTerm[20];
tmpQN2[21] = +tmpObjSEndTerm[21];
tmpQN2[22] = +tmpObjSEndTerm[22];
tmpQN2[23] = +tmpObjSEndTerm[23];
tmpQN2[24] = +tmpObjSEndTerm[24];
tmpQN2[25] = +tmpObjSEndTerm[25];
tmpQN2[26] = +tmpObjSEndTerm[26];
tmpQN2[27] = +tmpObjSEndTerm[27];
tmpQN2[28] = +tmpObjSEndTerm[28];
tmpQN2[29] = +tmpObjSEndTerm[29];
tmpQN2[30] = +tmpObjSEndTerm[30];
tmpQN2[31] = +tmpObjSEndTerm[31];
tmpQN2[32] = +tmpObjSEndTerm[32];
tmpQN2[33] = +tmpObjSEndTerm[33];
tmpQN2[34] = +tmpObjSEndTerm[34];
tmpQN2[35] = +tmpObjSEndTerm[35];
tmpQN2[36] = +tmpObjSEndTerm[36];
tmpQN2[37] = +tmpObjSEndTerm[37];
tmpQN2[38] = +tmpObjSEndTerm[38];
tmpQN2[39] = +tmpObjSEndTerm[39];
tmpQN2[40] = +tmpObjSEndTerm[40];
tmpQN2[41] = +tmpObjSEndTerm[41];
tmpQN2[42] = +tmpObjSEndTerm[42];
tmpQN2[43] = +tmpObjSEndTerm[43];
tmpQN2[44] = +tmpObjSEndTerm[44];
tmpQN2[45] = +tmpObjSEndTerm[45];
tmpQN2[46] = +tmpObjSEndTerm[46];
tmpQN2[47] = +tmpObjSEndTerm[47];
tmpQN2[48] = +tmpObjSEndTerm[48];
tmpQN2[49] = +tmpObjSEndTerm[49];
tmpQN2[50] = +tmpObjSEndTerm[50];
tmpQN2[51] = +tmpObjSEndTerm[51];
tmpQN2[52] = +tmpObjSEndTerm[52];
tmpQN2[53] = +tmpObjSEndTerm[53];
tmpQN2[54] = +tmpObjSEndTerm[54];
tmpQN2[55] = +tmpObjSEndTerm[55];
tmpQN2[56] = +tmpObjSEndTerm[56];
tmpQN2[57] = +tmpObjSEndTerm[57];
tmpQN2[58] = +tmpObjSEndTerm[58];
tmpQN2[59] = +tmpObjSEndTerm[59];
tmpQN2[60] = +tmpObjSEndTerm[60];
tmpQN2[61] = +tmpObjSEndTerm[61];
tmpQN2[62] = +tmpObjSEndTerm[62];
tmpQN2[63] = +tmpObjSEndTerm[63];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
tmpQN1[9] = + tmpQN2[9];
tmpQN1[10] = + tmpQN2[10];
tmpQN1[11] = + tmpQN2[11];
tmpQN1[12] = + tmpQN2[12];
tmpQN1[13] = + tmpQN2[13];
tmpQN1[14] = + tmpQN2[14];
tmpQN1[15] = + tmpQN2[15];
tmpQN1[16] = + tmpQN2[16];
tmpQN1[17] = + tmpQN2[17];
tmpQN1[18] = + tmpQN2[18];
tmpQN1[19] = + tmpQN2[19];
tmpQN1[20] = + tmpQN2[20];
tmpQN1[21] = + tmpQN2[21];
tmpQN1[22] = + tmpQN2[22];
tmpQN1[23] = + tmpQN2[23];
tmpQN1[24] = + tmpQN2[24];
tmpQN1[25] = + tmpQN2[25];
tmpQN1[26] = + tmpQN2[26];
tmpQN1[27] = + tmpQN2[27];
tmpQN1[28] = + tmpQN2[28];
tmpQN1[29] = + tmpQN2[29];
tmpQN1[30] = + tmpQN2[30];
tmpQN1[31] = + tmpQN2[31];
tmpQN1[32] = + tmpQN2[32];
tmpQN1[33] = + tmpQN2[33];
tmpQN1[34] = + tmpQN2[34];
tmpQN1[35] = + tmpQN2[35];
tmpQN1[36] = + tmpQN2[36];
tmpQN1[37] = + tmpQN2[37];
tmpQN1[38] = + tmpQN2[38];
tmpQN1[39] = + tmpQN2[39];
tmpQN1[40] = + tmpQN2[40];
tmpQN1[41] = + tmpQN2[41];
tmpQN1[42] = + tmpQN2[42];
tmpQN1[43] = + tmpQN2[43];
tmpQN1[44] = + tmpQN2[44];
tmpQN1[45] = + tmpQN2[45];
tmpQN1[46] = + tmpQN2[46];
tmpQN1[47] = + tmpQN2[47];
tmpQN1[48] = + tmpQN2[48];
tmpQN1[49] = + tmpQN2[49];
tmpQN1[50] = + tmpQN2[50];
tmpQN1[51] = + tmpQN2[51];
tmpQN1[52] = + tmpQN2[52];
tmpQN1[53] = + tmpQN2[53];
tmpQN1[54] = + tmpQN2[54];
tmpQN1[55] = + tmpQN2[55];
tmpQN1[56] = + tmpQN2[56];
tmpQN1[57] = + tmpQN2[57];
tmpQN1[58] = + tmpQN2[58];
tmpQN1[59] = + tmpQN2[59];
tmpQN1[60] = + tmpQN2[60];
tmpQN1[61] = + tmpQN2[61];
tmpQN1[62] = + tmpQN2[62];
tmpQN1[63] = + tmpQN2[63];
}

void nmpc_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 100; ++runObj)
{
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[runObj * 8];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[runObj * 8 + 1];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[runObj * 8 + 2];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[runObj * 8 + 3];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[runObj * 8 + 4];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[runObj * 8 + 5];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[runObj * 8 + 6];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[runObj * 8 + 7];
nmpcWorkspace.objValueIn[8] = nmpcVariables.u[runObj * 4];
nmpcWorkspace.objValueIn[9] = nmpcVariables.u[runObj * 4 + 1];
nmpcWorkspace.objValueIn[10] = nmpcVariables.u[runObj * 4 + 2];
nmpcWorkspace.objValueIn[11] = nmpcVariables.u[runObj * 4 + 3];
nmpcWorkspace.objValueIn[12] = nmpcVariables.od[runObj * 3];
nmpcWorkspace.objValueIn[13] = nmpcVariables.od[runObj * 3 + 1];
nmpcWorkspace.objValueIn[14] = nmpcVariables.od[runObj * 3 + 2];

nmpc_evaluateLSQ( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.Dy[runObj * 12] = nmpcWorkspace.objValueOut[0];
nmpcWorkspace.Dy[runObj * 12 + 1] = nmpcWorkspace.objValueOut[1];
nmpcWorkspace.Dy[runObj * 12 + 2] = nmpcWorkspace.objValueOut[2];
nmpcWorkspace.Dy[runObj * 12 + 3] = nmpcWorkspace.objValueOut[3];
nmpcWorkspace.Dy[runObj * 12 + 4] = nmpcWorkspace.objValueOut[4];
nmpcWorkspace.Dy[runObj * 12 + 5] = nmpcWorkspace.objValueOut[5];
nmpcWorkspace.Dy[runObj * 12 + 6] = nmpcWorkspace.objValueOut[6];
nmpcWorkspace.Dy[runObj * 12 + 7] = nmpcWorkspace.objValueOut[7];
nmpcWorkspace.Dy[runObj * 12 + 8] = nmpcWorkspace.objValueOut[8];
nmpcWorkspace.Dy[runObj * 12 + 9] = nmpcWorkspace.objValueOut[9];
nmpcWorkspace.Dy[runObj * 12 + 10] = nmpcWorkspace.objValueOut[10];
nmpcWorkspace.Dy[runObj * 12 + 11] = nmpcWorkspace.objValueOut[11];

nmpc_setObjQ1Q2( nmpcVariables.W, &(nmpcWorkspace.Q1[ runObj * 64 ]), &(nmpcWorkspace.Q2[ runObj * 96 ]) );

nmpc_setObjR1R2( nmpcVariables.W, &(nmpcWorkspace.R1[ runObj * 16 ]), &(nmpcWorkspace.R2[ runObj * 48 ]) );

}
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[800];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[801];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[802];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[803];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[804];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[805];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[806];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[807];
nmpcWorkspace.objValueIn[8] = nmpcVariables.od[300];
nmpcWorkspace.objValueIn[9] = nmpcVariables.od[301];
nmpcWorkspace.objValueIn[10] = nmpcVariables.od[302];
nmpc_evaluateLSQEndTerm( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );

nmpcWorkspace.DyN[0] = nmpcWorkspace.objValueOut[0];
nmpcWorkspace.DyN[1] = nmpcWorkspace.objValueOut[1];
nmpcWorkspace.DyN[2] = nmpcWorkspace.objValueOut[2];
nmpcWorkspace.DyN[3] = nmpcWorkspace.objValueOut[3];
nmpcWorkspace.DyN[4] = nmpcWorkspace.objValueOut[4];
nmpcWorkspace.DyN[5] = nmpcWorkspace.objValueOut[5];
nmpcWorkspace.DyN[6] = nmpcWorkspace.objValueOut[6];
nmpcWorkspace.DyN[7] = nmpcWorkspace.objValueOut[7];

nmpc_setObjQN1QN2( nmpcVariables.WN, nmpcWorkspace.QN1, nmpcWorkspace.QN2 );

}

void nmpc_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7];
dNew[1] += + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3] + Gx1[12]*dOld[4] + Gx1[13]*dOld[5] + Gx1[14]*dOld[6] + Gx1[15]*dOld[7];
dNew[2] += + Gx1[16]*dOld[0] + Gx1[17]*dOld[1] + Gx1[18]*dOld[2] + Gx1[19]*dOld[3] + Gx1[20]*dOld[4] + Gx1[21]*dOld[5] + Gx1[22]*dOld[6] + Gx1[23]*dOld[7];
dNew[3] += + Gx1[24]*dOld[0] + Gx1[25]*dOld[1] + Gx1[26]*dOld[2] + Gx1[27]*dOld[3] + Gx1[28]*dOld[4] + Gx1[29]*dOld[5] + Gx1[30]*dOld[6] + Gx1[31]*dOld[7];
dNew[4] += + Gx1[32]*dOld[0] + Gx1[33]*dOld[1] + Gx1[34]*dOld[2] + Gx1[35]*dOld[3] + Gx1[36]*dOld[4] + Gx1[37]*dOld[5] + Gx1[38]*dOld[6] + Gx1[39]*dOld[7];
dNew[5] += + Gx1[40]*dOld[0] + Gx1[41]*dOld[1] + Gx1[42]*dOld[2] + Gx1[43]*dOld[3] + Gx1[44]*dOld[4] + Gx1[45]*dOld[5] + Gx1[46]*dOld[6] + Gx1[47]*dOld[7];
dNew[6] += + Gx1[48]*dOld[0] + Gx1[49]*dOld[1] + Gx1[50]*dOld[2] + Gx1[51]*dOld[3] + Gx1[52]*dOld[4] + Gx1[53]*dOld[5] + Gx1[54]*dOld[6] + Gx1[55]*dOld[7];
dNew[7] += + Gx1[56]*dOld[0] + Gx1[57]*dOld[1] + Gx1[58]*dOld[2] + Gx1[59]*dOld[3] + Gx1[60]*dOld[4] + Gx1[61]*dOld[5] + Gx1[62]*dOld[6] + Gx1[63]*dOld[7];
}

void nmpc_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
Gx2[25] = Gx1[25];
Gx2[26] = Gx1[26];
Gx2[27] = Gx1[27];
Gx2[28] = Gx1[28];
Gx2[29] = Gx1[29];
Gx2[30] = Gx1[30];
Gx2[31] = Gx1[31];
Gx2[32] = Gx1[32];
Gx2[33] = Gx1[33];
Gx2[34] = Gx1[34];
Gx2[35] = Gx1[35];
Gx2[36] = Gx1[36];
Gx2[37] = Gx1[37];
Gx2[38] = Gx1[38];
Gx2[39] = Gx1[39];
Gx2[40] = Gx1[40];
Gx2[41] = Gx1[41];
Gx2[42] = Gx1[42];
Gx2[43] = Gx1[43];
Gx2[44] = Gx1[44];
Gx2[45] = Gx1[45];
Gx2[46] = Gx1[46];
Gx2[47] = Gx1[47];
Gx2[48] = Gx1[48];
Gx2[49] = Gx1[49];
Gx2[50] = Gx1[50];
Gx2[51] = Gx1[51];
Gx2[52] = Gx1[52];
Gx2[53] = Gx1[53];
Gx2[54] = Gx1[54];
Gx2[55] = Gx1[55];
Gx2[56] = Gx1[56];
Gx2[57] = Gx1[57];
Gx2[58] = Gx1[58];
Gx2[59] = Gx1[59];
Gx2[60] = Gx1[60];
Gx2[61] = Gx1[61];
Gx2[62] = Gx1[62];
Gx2[63] = Gx1[63];
}

void nmpc_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[16] + Gx1[3]*Gx2[24] + Gx1[4]*Gx2[32] + Gx1[5]*Gx2[40] + Gx1[6]*Gx2[48] + Gx1[7]*Gx2[56];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[17] + Gx1[3]*Gx2[25] + Gx1[4]*Gx2[33] + Gx1[5]*Gx2[41] + Gx1[6]*Gx2[49] + Gx1[7]*Gx2[57];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[10] + Gx1[2]*Gx2[18] + Gx1[3]*Gx2[26] + Gx1[4]*Gx2[34] + Gx1[5]*Gx2[42] + Gx1[6]*Gx2[50] + Gx1[7]*Gx2[58];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[11] + Gx1[2]*Gx2[19] + Gx1[3]*Gx2[27] + Gx1[4]*Gx2[35] + Gx1[5]*Gx2[43] + Gx1[6]*Gx2[51] + Gx1[7]*Gx2[59];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[12] + Gx1[2]*Gx2[20] + Gx1[3]*Gx2[28] + Gx1[4]*Gx2[36] + Gx1[5]*Gx2[44] + Gx1[6]*Gx2[52] + Gx1[7]*Gx2[60];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[13] + Gx1[2]*Gx2[21] + Gx1[3]*Gx2[29] + Gx1[4]*Gx2[37] + Gx1[5]*Gx2[45] + Gx1[6]*Gx2[53] + Gx1[7]*Gx2[61];
Gx3[6] = + Gx1[0]*Gx2[6] + Gx1[1]*Gx2[14] + Gx1[2]*Gx2[22] + Gx1[3]*Gx2[30] + Gx1[4]*Gx2[38] + Gx1[5]*Gx2[46] + Gx1[6]*Gx2[54] + Gx1[7]*Gx2[62];
Gx3[7] = + Gx1[0]*Gx2[7] + Gx1[1]*Gx2[15] + Gx1[2]*Gx2[23] + Gx1[3]*Gx2[31] + Gx1[4]*Gx2[39] + Gx1[5]*Gx2[47] + Gx1[6]*Gx2[55] + Gx1[7]*Gx2[63];
Gx3[8] = + Gx1[8]*Gx2[0] + Gx1[9]*Gx2[8] + Gx1[10]*Gx2[16] + Gx1[11]*Gx2[24] + Gx1[12]*Gx2[32] + Gx1[13]*Gx2[40] + Gx1[14]*Gx2[48] + Gx1[15]*Gx2[56];
Gx3[9] = + Gx1[8]*Gx2[1] + Gx1[9]*Gx2[9] + Gx1[10]*Gx2[17] + Gx1[11]*Gx2[25] + Gx1[12]*Gx2[33] + Gx1[13]*Gx2[41] + Gx1[14]*Gx2[49] + Gx1[15]*Gx2[57];
Gx3[10] = + Gx1[8]*Gx2[2] + Gx1[9]*Gx2[10] + Gx1[10]*Gx2[18] + Gx1[11]*Gx2[26] + Gx1[12]*Gx2[34] + Gx1[13]*Gx2[42] + Gx1[14]*Gx2[50] + Gx1[15]*Gx2[58];
Gx3[11] = + Gx1[8]*Gx2[3] + Gx1[9]*Gx2[11] + Gx1[10]*Gx2[19] + Gx1[11]*Gx2[27] + Gx1[12]*Gx2[35] + Gx1[13]*Gx2[43] + Gx1[14]*Gx2[51] + Gx1[15]*Gx2[59];
Gx3[12] = + Gx1[8]*Gx2[4] + Gx1[9]*Gx2[12] + Gx1[10]*Gx2[20] + Gx1[11]*Gx2[28] + Gx1[12]*Gx2[36] + Gx1[13]*Gx2[44] + Gx1[14]*Gx2[52] + Gx1[15]*Gx2[60];
Gx3[13] = + Gx1[8]*Gx2[5] + Gx1[9]*Gx2[13] + Gx1[10]*Gx2[21] + Gx1[11]*Gx2[29] + Gx1[12]*Gx2[37] + Gx1[13]*Gx2[45] + Gx1[14]*Gx2[53] + Gx1[15]*Gx2[61];
Gx3[14] = + Gx1[8]*Gx2[6] + Gx1[9]*Gx2[14] + Gx1[10]*Gx2[22] + Gx1[11]*Gx2[30] + Gx1[12]*Gx2[38] + Gx1[13]*Gx2[46] + Gx1[14]*Gx2[54] + Gx1[15]*Gx2[62];
Gx3[15] = + Gx1[8]*Gx2[7] + Gx1[9]*Gx2[15] + Gx1[10]*Gx2[23] + Gx1[11]*Gx2[31] + Gx1[12]*Gx2[39] + Gx1[13]*Gx2[47] + Gx1[14]*Gx2[55] + Gx1[15]*Gx2[63];
Gx3[16] = + Gx1[16]*Gx2[0] + Gx1[17]*Gx2[8] + Gx1[18]*Gx2[16] + Gx1[19]*Gx2[24] + Gx1[20]*Gx2[32] + Gx1[21]*Gx2[40] + Gx1[22]*Gx2[48] + Gx1[23]*Gx2[56];
Gx3[17] = + Gx1[16]*Gx2[1] + Gx1[17]*Gx2[9] + Gx1[18]*Gx2[17] + Gx1[19]*Gx2[25] + Gx1[20]*Gx2[33] + Gx1[21]*Gx2[41] + Gx1[22]*Gx2[49] + Gx1[23]*Gx2[57];
Gx3[18] = + Gx1[16]*Gx2[2] + Gx1[17]*Gx2[10] + Gx1[18]*Gx2[18] + Gx1[19]*Gx2[26] + Gx1[20]*Gx2[34] + Gx1[21]*Gx2[42] + Gx1[22]*Gx2[50] + Gx1[23]*Gx2[58];
Gx3[19] = + Gx1[16]*Gx2[3] + Gx1[17]*Gx2[11] + Gx1[18]*Gx2[19] + Gx1[19]*Gx2[27] + Gx1[20]*Gx2[35] + Gx1[21]*Gx2[43] + Gx1[22]*Gx2[51] + Gx1[23]*Gx2[59];
Gx3[20] = + Gx1[16]*Gx2[4] + Gx1[17]*Gx2[12] + Gx1[18]*Gx2[20] + Gx1[19]*Gx2[28] + Gx1[20]*Gx2[36] + Gx1[21]*Gx2[44] + Gx1[22]*Gx2[52] + Gx1[23]*Gx2[60];
Gx3[21] = + Gx1[16]*Gx2[5] + Gx1[17]*Gx2[13] + Gx1[18]*Gx2[21] + Gx1[19]*Gx2[29] + Gx1[20]*Gx2[37] + Gx1[21]*Gx2[45] + Gx1[22]*Gx2[53] + Gx1[23]*Gx2[61];
Gx3[22] = + Gx1[16]*Gx2[6] + Gx1[17]*Gx2[14] + Gx1[18]*Gx2[22] + Gx1[19]*Gx2[30] + Gx1[20]*Gx2[38] + Gx1[21]*Gx2[46] + Gx1[22]*Gx2[54] + Gx1[23]*Gx2[62];
Gx3[23] = + Gx1[16]*Gx2[7] + Gx1[17]*Gx2[15] + Gx1[18]*Gx2[23] + Gx1[19]*Gx2[31] + Gx1[20]*Gx2[39] + Gx1[21]*Gx2[47] + Gx1[22]*Gx2[55] + Gx1[23]*Gx2[63];
Gx3[24] = + Gx1[24]*Gx2[0] + Gx1[25]*Gx2[8] + Gx1[26]*Gx2[16] + Gx1[27]*Gx2[24] + Gx1[28]*Gx2[32] + Gx1[29]*Gx2[40] + Gx1[30]*Gx2[48] + Gx1[31]*Gx2[56];
Gx3[25] = + Gx1[24]*Gx2[1] + Gx1[25]*Gx2[9] + Gx1[26]*Gx2[17] + Gx1[27]*Gx2[25] + Gx1[28]*Gx2[33] + Gx1[29]*Gx2[41] + Gx1[30]*Gx2[49] + Gx1[31]*Gx2[57];
Gx3[26] = + Gx1[24]*Gx2[2] + Gx1[25]*Gx2[10] + Gx1[26]*Gx2[18] + Gx1[27]*Gx2[26] + Gx1[28]*Gx2[34] + Gx1[29]*Gx2[42] + Gx1[30]*Gx2[50] + Gx1[31]*Gx2[58];
Gx3[27] = + Gx1[24]*Gx2[3] + Gx1[25]*Gx2[11] + Gx1[26]*Gx2[19] + Gx1[27]*Gx2[27] + Gx1[28]*Gx2[35] + Gx1[29]*Gx2[43] + Gx1[30]*Gx2[51] + Gx1[31]*Gx2[59];
Gx3[28] = + Gx1[24]*Gx2[4] + Gx1[25]*Gx2[12] + Gx1[26]*Gx2[20] + Gx1[27]*Gx2[28] + Gx1[28]*Gx2[36] + Gx1[29]*Gx2[44] + Gx1[30]*Gx2[52] + Gx1[31]*Gx2[60];
Gx3[29] = + Gx1[24]*Gx2[5] + Gx1[25]*Gx2[13] + Gx1[26]*Gx2[21] + Gx1[27]*Gx2[29] + Gx1[28]*Gx2[37] + Gx1[29]*Gx2[45] + Gx1[30]*Gx2[53] + Gx1[31]*Gx2[61];
Gx3[30] = + Gx1[24]*Gx2[6] + Gx1[25]*Gx2[14] + Gx1[26]*Gx2[22] + Gx1[27]*Gx2[30] + Gx1[28]*Gx2[38] + Gx1[29]*Gx2[46] + Gx1[30]*Gx2[54] + Gx1[31]*Gx2[62];
Gx3[31] = + Gx1[24]*Gx2[7] + Gx1[25]*Gx2[15] + Gx1[26]*Gx2[23] + Gx1[27]*Gx2[31] + Gx1[28]*Gx2[39] + Gx1[29]*Gx2[47] + Gx1[30]*Gx2[55] + Gx1[31]*Gx2[63];
Gx3[32] = + Gx1[32]*Gx2[0] + Gx1[33]*Gx2[8] + Gx1[34]*Gx2[16] + Gx1[35]*Gx2[24] + Gx1[36]*Gx2[32] + Gx1[37]*Gx2[40] + Gx1[38]*Gx2[48] + Gx1[39]*Gx2[56];
Gx3[33] = + Gx1[32]*Gx2[1] + Gx1[33]*Gx2[9] + Gx1[34]*Gx2[17] + Gx1[35]*Gx2[25] + Gx1[36]*Gx2[33] + Gx1[37]*Gx2[41] + Gx1[38]*Gx2[49] + Gx1[39]*Gx2[57];
Gx3[34] = + Gx1[32]*Gx2[2] + Gx1[33]*Gx2[10] + Gx1[34]*Gx2[18] + Gx1[35]*Gx2[26] + Gx1[36]*Gx2[34] + Gx1[37]*Gx2[42] + Gx1[38]*Gx2[50] + Gx1[39]*Gx2[58];
Gx3[35] = + Gx1[32]*Gx2[3] + Gx1[33]*Gx2[11] + Gx1[34]*Gx2[19] + Gx1[35]*Gx2[27] + Gx1[36]*Gx2[35] + Gx1[37]*Gx2[43] + Gx1[38]*Gx2[51] + Gx1[39]*Gx2[59];
Gx3[36] = + Gx1[32]*Gx2[4] + Gx1[33]*Gx2[12] + Gx1[34]*Gx2[20] + Gx1[35]*Gx2[28] + Gx1[36]*Gx2[36] + Gx1[37]*Gx2[44] + Gx1[38]*Gx2[52] + Gx1[39]*Gx2[60];
Gx3[37] = + Gx1[32]*Gx2[5] + Gx1[33]*Gx2[13] + Gx1[34]*Gx2[21] + Gx1[35]*Gx2[29] + Gx1[36]*Gx2[37] + Gx1[37]*Gx2[45] + Gx1[38]*Gx2[53] + Gx1[39]*Gx2[61];
Gx3[38] = + Gx1[32]*Gx2[6] + Gx1[33]*Gx2[14] + Gx1[34]*Gx2[22] + Gx1[35]*Gx2[30] + Gx1[36]*Gx2[38] + Gx1[37]*Gx2[46] + Gx1[38]*Gx2[54] + Gx1[39]*Gx2[62];
Gx3[39] = + Gx1[32]*Gx2[7] + Gx1[33]*Gx2[15] + Gx1[34]*Gx2[23] + Gx1[35]*Gx2[31] + Gx1[36]*Gx2[39] + Gx1[37]*Gx2[47] + Gx1[38]*Gx2[55] + Gx1[39]*Gx2[63];
Gx3[40] = + Gx1[40]*Gx2[0] + Gx1[41]*Gx2[8] + Gx1[42]*Gx2[16] + Gx1[43]*Gx2[24] + Gx1[44]*Gx2[32] + Gx1[45]*Gx2[40] + Gx1[46]*Gx2[48] + Gx1[47]*Gx2[56];
Gx3[41] = + Gx1[40]*Gx2[1] + Gx1[41]*Gx2[9] + Gx1[42]*Gx2[17] + Gx1[43]*Gx2[25] + Gx1[44]*Gx2[33] + Gx1[45]*Gx2[41] + Gx1[46]*Gx2[49] + Gx1[47]*Gx2[57];
Gx3[42] = + Gx1[40]*Gx2[2] + Gx1[41]*Gx2[10] + Gx1[42]*Gx2[18] + Gx1[43]*Gx2[26] + Gx1[44]*Gx2[34] + Gx1[45]*Gx2[42] + Gx1[46]*Gx2[50] + Gx1[47]*Gx2[58];
Gx3[43] = + Gx1[40]*Gx2[3] + Gx1[41]*Gx2[11] + Gx1[42]*Gx2[19] + Gx1[43]*Gx2[27] + Gx1[44]*Gx2[35] + Gx1[45]*Gx2[43] + Gx1[46]*Gx2[51] + Gx1[47]*Gx2[59];
Gx3[44] = + Gx1[40]*Gx2[4] + Gx1[41]*Gx2[12] + Gx1[42]*Gx2[20] + Gx1[43]*Gx2[28] + Gx1[44]*Gx2[36] + Gx1[45]*Gx2[44] + Gx1[46]*Gx2[52] + Gx1[47]*Gx2[60];
Gx3[45] = + Gx1[40]*Gx2[5] + Gx1[41]*Gx2[13] + Gx1[42]*Gx2[21] + Gx1[43]*Gx2[29] + Gx1[44]*Gx2[37] + Gx1[45]*Gx2[45] + Gx1[46]*Gx2[53] + Gx1[47]*Gx2[61];
Gx3[46] = + Gx1[40]*Gx2[6] + Gx1[41]*Gx2[14] + Gx1[42]*Gx2[22] + Gx1[43]*Gx2[30] + Gx1[44]*Gx2[38] + Gx1[45]*Gx2[46] + Gx1[46]*Gx2[54] + Gx1[47]*Gx2[62];
Gx3[47] = + Gx1[40]*Gx2[7] + Gx1[41]*Gx2[15] + Gx1[42]*Gx2[23] + Gx1[43]*Gx2[31] + Gx1[44]*Gx2[39] + Gx1[45]*Gx2[47] + Gx1[46]*Gx2[55] + Gx1[47]*Gx2[63];
Gx3[48] = + Gx1[48]*Gx2[0] + Gx1[49]*Gx2[8] + Gx1[50]*Gx2[16] + Gx1[51]*Gx2[24] + Gx1[52]*Gx2[32] + Gx1[53]*Gx2[40] + Gx1[54]*Gx2[48] + Gx1[55]*Gx2[56];
Gx3[49] = + Gx1[48]*Gx2[1] + Gx1[49]*Gx2[9] + Gx1[50]*Gx2[17] + Gx1[51]*Gx2[25] + Gx1[52]*Gx2[33] + Gx1[53]*Gx2[41] + Gx1[54]*Gx2[49] + Gx1[55]*Gx2[57];
Gx3[50] = + Gx1[48]*Gx2[2] + Gx1[49]*Gx2[10] + Gx1[50]*Gx2[18] + Gx1[51]*Gx2[26] + Gx1[52]*Gx2[34] + Gx1[53]*Gx2[42] + Gx1[54]*Gx2[50] + Gx1[55]*Gx2[58];
Gx3[51] = + Gx1[48]*Gx2[3] + Gx1[49]*Gx2[11] + Gx1[50]*Gx2[19] + Gx1[51]*Gx2[27] + Gx1[52]*Gx2[35] + Gx1[53]*Gx2[43] + Gx1[54]*Gx2[51] + Gx1[55]*Gx2[59];
Gx3[52] = + Gx1[48]*Gx2[4] + Gx1[49]*Gx2[12] + Gx1[50]*Gx2[20] + Gx1[51]*Gx2[28] + Gx1[52]*Gx2[36] + Gx1[53]*Gx2[44] + Gx1[54]*Gx2[52] + Gx1[55]*Gx2[60];
Gx3[53] = + Gx1[48]*Gx2[5] + Gx1[49]*Gx2[13] + Gx1[50]*Gx2[21] + Gx1[51]*Gx2[29] + Gx1[52]*Gx2[37] + Gx1[53]*Gx2[45] + Gx1[54]*Gx2[53] + Gx1[55]*Gx2[61];
Gx3[54] = + Gx1[48]*Gx2[6] + Gx1[49]*Gx2[14] + Gx1[50]*Gx2[22] + Gx1[51]*Gx2[30] + Gx1[52]*Gx2[38] + Gx1[53]*Gx2[46] + Gx1[54]*Gx2[54] + Gx1[55]*Gx2[62];
Gx3[55] = + Gx1[48]*Gx2[7] + Gx1[49]*Gx2[15] + Gx1[50]*Gx2[23] + Gx1[51]*Gx2[31] + Gx1[52]*Gx2[39] + Gx1[53]*Gx2[47] + Gx1[54]*Gx2[55] + Gx1[55]*Gx2[63];
Gx3[56] = + Gx1[56]*Gx2[0] + Gx1[57]*Gx2[8] + Gx1[58]*Gx2[16] + Gx1[59]*Gx2[24] + Gx1[60]*Gx2[32] + Gx1[61]*Gx2[40] + Gx1[62]*Gx2[48] + Gx1[63]*Gx2[56];
Gx3[57] = + Gx1[56]*Gx2[1] + Gx1[57]*Gx2[9] + Gx1[58]*Gx2[17] + Gx1[59]*Gx2[25] + Gx1[60]*Gx2[33] + Gx1[61]*Gx2[41] + Gx1[62]*Gx2[49] + Gx1[63]*Gx2[57];
Gx3[58] = + Gx1[56]*Gx2[2] + Gx1[57]*Gx2[10] + Gx1[58]*Gx2[18] + Gx1[59]*Gx2[26] + Gx1[60]*Gx2[34] + Gx1[61]*Gx2[42] + Gx1[62]*Gx2[50] + Gx1[63]*Gx2[58];
Gx3[59] = + Gx1[56]*Gx2[3] + Gx1[57]*Gx2[11] + Gx1[58]*Gx2[19] + Gx1[59]*Gx2[27] + Gx1[60]*Gx2[35] + Gx1[61]*Gx2[43] + Gx1[62]*Gx2[51] + Gx1[63]*Gx2[59];
Gx3[60] = + Gx1[56]*Gx2[4] + Gx1[57]*Gx2[12] + Gx1[58]*Gx2[20] + Gx1[59]*Gx2[28] + Gx1[60]*Gx2[36] + Gx1[61]*Gx2[44] + Gx1[62]*Gx2[52] + Gx1[63]*Gx2[60];
Gx3[61] = + Gx1[56]*Gx2[5] + Gx1[57]*Gx2[13] + Gx1[58]*Gx2[21] + Gx1[59]*Gx2[29] + Gx1[60]*Gx2[37] + Gx1[61]*Gx2[45] + Gx1[62]*Gx2[53] + Gx1[63]*Gx2[61];
Gx3[62] = + Gx1[56]*Gx2[6] + Gx1[57]*Gx2[14] + Gx1[58]*Gx2[22] + Gx1[59]*Gx2[30] + Gx1[60]*Gx2[38] + Gx1[61]*Gx2[46] + Gx1[62]*Gx2[54] + Gx1[63]*Gx2[62];
Gx3[63] = + Gx1[56]*Gx2[7] + Gx1[57]*Gx2[15] + Gx1[58]*Gx2[23] + Gx1[59]*Gx2[31] + Gx1[60]*Gx2[39] + Gx1[61]*Gx2[47] + Gx1[62]*Gx2[55] + Gx1[63]*Gx2[63];
}

void nmpc_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[12] + Gx1[4]*Gu1[16] + Gx1[5]*Gu1[20] + Gx1[6]*Gu1[24] + Gx1[7]*Gu1[28];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[9] + Gx1[3]*Gu1[13] + Gx1[4]*Gu1[17] + Gx1[5]*Gu1[21] + Gx1[6]*Gu1[25] + Gx1[7]*Gu1[29];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[6] + Gx1[2]*Gu1[10] + Gx1[3]*Gu1[14] + Gx1[4]*Gu1[18] + Gx1[5]*Gu1[22] + Gx1[6]*Gu1[26] + Gx1[7]*Gu1[30];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[1]*Gu1[7] + Gx1[2]*Gu1[11] + Gx1[3]*Gu1[15] + Gx1[4]*Gu1[19] + Gx1[5]*Gu1[23] + Gx1[6]*Gu1[27] + Gx1[7]*Gu1[31];
Gu2[4] = + Gx1[8]*Gu1[0] + Gx1[9]*Gu1[4] + Gx1[10]*Gu1[8] + Gx1[11]*Gu1[12] + Gx1[12]*Gu1[16] + Gx1[13]*Gu1[20] + Gx1[14]*Gu1[24] + Gx1[15]*Gu1[28];
Gu2[5] = + Gx1[8]*Gu1[1] + Gx1[9]*Gu1[5] + Gx1[10]*Gu1[9] + Gx1[11]*Gu1[13] + Gx1[12]*Gu1[17] + Gx1[13]*Gu1[21] + Gx1[14]*Gu1[25] + Gx1[15]*Gu1[29];
Gu2[6] = + Gx1[8]*Gu1[2] + Gx1[9]*Gu1[6] + Gx1[10]*Gu1[10] + Gx1[11]*Gu1[14] + Gx1[12]*Gu1[18] + Gx1[13]*Gu1[22] + Gx1[14]*Gu1[26] + Gx1[15]*Gu1[30];
Gu2[7] = + Gx1[8]*Gu1[3] + Gx1[9]*Gu1[7] + Gx1[10]*Gu1[11] + Gx1[11]*Gu1[15] + Gx1[12]*Gu1[19] + Gx1[13]*Gu1[23] + Gx1[14]*Gu1[27] + Gx1[15]*Gu1[31];
Gu2[8] = + Gx1[16]*Gu1[0] + Gx1[17]*Gu1[4] + Gx1[18]*Gu1[8] + Gx1[19]*Gu1[12] + Gx1[20]*Gu1[16] + Gx1[21]*Gu1[20] + Gx1[22]*Gu1[24] + Gx1[23]*Gu1[28];
Gu2[9] = + Gx1[16]*Gu1[1] + Gx1[17]*Gu1[5] + Gx1[18]*Gu1[9] + Gx1[19]*Gu1[13] + Gx1[20]*Gu1[17] + Gx1[21]*Gu1[21] + Gx1[22]*Gu1[25] + Gx1[23]*Gu1[29];
Gu2[10] = + Gx1[16]*Gu1[2] + Gx1[17]*Gu1[6] + Gx1[18]*Gu1[10] + Gx1[19]*Gu1[14] + Gx1[20]*Gu1[18] + Gx1[21]*Gu1[22] + Gx1[22]*Gu1[26] + Gx1[23]*Gu1[30];
Gu2[11] = + Gx1[16]*Gu1[3] + Gx1[17]*Gu1[7] + Gx1[18]*Gu1[11] + Gx1[19]*Gu1[15] + Gx1[20]*Gu1[19] + Gx1[21]*Gu1[23] + Gx1[22]*Gu1[27] + Gx1[23]*Gu1[31];
Gu2[12] = + Gx1[24]*Gu1[0] + Gx1[25]*Gu1[4] + Gx1[26]*Gu1[8] + Gx1[27]*Gu1[12] + Gx1[28]*Gu1[16] + Gx1[29]*Gu1[20] + Gx1[30]*Gu1[24] + Gx1[31]*Gu1[28];
Gu2[13] = + Gx1[24]*Gu1[1] + Gx1[25]*Gu1[5] + Gx1[26]*Gu1[9] + Gx1[27]*Gu1[13] + Gx1[28]*Gu1[17] + Gx1[29]*Gu1[21] + Gx1[30]*Gu1[25] + Gx1[31]*Gu1[29];
Gu2[14] = + Gx1[24]*Gu1[2] + Gx1[25]*Gu1[6] + Gx1[26]*Gu1[10] + Gx1[27]*Gu1[14] + Gx1[28]*Gu1[18] + Gx1[29]*Gu1[22] + Gx1[30]*Gu1[26] + Gx1[31]*Gu1[30];
Gu2[15] = + Gx1[24]*Gu1[3] + Gx1[25]*Gu1[7] + Gx1[26]*Gu1[11] + Gx1[27]*Gu1[15] + Gx1[28]*Gu1[19] + Gx1[29]*Gu1[23] + Gx1[30]*Gu1[27] + Gx1[31]*Gu1[31];
Gu2[16] = + Gx1[32]*Gu1[0] + Gx1[33]*Gu1[4] + Gx1[34]*Gu1[8] + Gx1[35]*Gu1[12] + Gx1[36]*Gu1[16] + Gx1[37]*Gu1[20] + Gx1[38]*Gu1[24] + Gx1[39]*Gu1[28];
Gu2[17] = + Gx1[32]*Gu1[1] + Gx1[33]*Gu1[5] + Gx1[34]*Gu1[9] + Gx1[35]*Gu1[13] + Gx1[36]*Gu1[17] + Gx1[37]*Gu1[21] + Gx1[38]*Gu1[25] + Gx1[39]*Gu1[29];
Gu2[18] = + Gx1[32]*Gu1[2] + Gx1[33]*Gu1[6] + Gx1[34]*Gu1[10] + Gx1[35]*Gu1[14] + Gx1[36]*Gu1[18] + Gx1[37]*Gu1[22] + Gx1[38]*Gu1[26] + Gx1[39]*Gu1[30];
Gu2[19] = + Gx1[32]*Gu1[3] + Gx1[33]*Gu1[7] + Gx1[34]*Gu1[11] + Gx1[35]*Gu1[15] + Gx1[36]*Gu1[19] + Gx1[37]*Gu1[23] + Gx1[38]*Gu1[27] + Gx1[39]*Gu1[31];
Gu2[20] = + Gx1[40]*Gu1[0] + Gx1[41]*Gu1[4] + Gx1[42]*Gu1[8] + Gx1[43]*Gu1[12] + Gx1[44]*Gu1[16] + Gx1[45]*Gu1[20] + Gx1[46]*Gu1[24] + Gx1[47]*Gu1[28];
Gu2[21] = + Gx1[40]*Gu1[1] + Gx1[41]*Gu1[5] + Gx1[42]*Gu1[9] + Gx1[43]*Gu1[13] + Gx1[44]*Gu1[17] + Gx1[45]*Gu1[21] + Gx1[46]*Gu1[25] + Gx1[47]*Gu1[29];
Gu2[22] = + Gx1[40]*Gu1[2] + Gx1[41]*Gu1[6] + Gx1[42]*Gu1[10] + Gx1[43]*Gu1[14] + Gx1[44]*Gu1[18] + Gx1[45]*Gu1[22] + Gx1[46]*Gu1[26] + Gx1[47]*Gu1[30];
Gu2[23] = + Gx1[40]*Gu1[3] + Gx1[41]*Gu1[7] + Gx1[42]*Gu1[11] + Gx1[43]*Gu1[15] + Gx1[44]*Gu1[19] + Gx1[45]*Gu1[23] + Gx1[46]*Gu1[27] + Gx1[47]*Gu1[31];
Gu2[24] = + Gx1[48]*Gu1[0] + Gx1[49]*Gu1[4] + Gx1[50]*Gu1[8] + Gx1[51]*Gu1[12] + Gx1[52]*Gu1[16] + Gx1[53]*Gu1[20] + Gx1[54]*Gu1[24] + Gx1[55]*Gu1[28];
Gu2[25] = + Gx1[48]*Gu1[1] + Gx1[49]*Gu1[5] + Gx1[50]*Gu1[9] + Gx1[51]*Gu1[13] + Gx1[52]*Gu1[17] + Gx1[53]*Gu1[21] + Gx1[54]*Gu1[25] + Gx1[55]*Gu1[29];
Gu2[26] = + Gx1[48]*Gu1[2] + Gx1[49]*Gu1[6] + Gx1[50]*Gu1[10] + Gx1[51]*Gu1[14] + Gx1[52]*Gu1[18] + Gx1[53]*Gu1[22] + Gx1[54]*Gu1[26] + Gx1[55]*Gu1[30];
Gu2[27] = + Gx1[48]*Gu1[3] + Gx1[49]*Gu1[7] + Gx1[50]*Gu1[11] + Gx1[51]*Gu1[15] + Gx1[52]*Gu1[19] + Gx1[53]*Gu1[23] + Gx1[54]*Gu1[27] + Gx1[55]*Gu1[31];
Gu2[28] = + Gx1[56]*Gu1[0] + Gx1[57]*Gu1[4] + Gx1[58]*Gu1[8] + Gx1[59]*Gu1[12] + Gx1[60]*Gu1[16] + Gx1[61]*Gu1[20] + Gx1[62]*Gu1[24] + Gx1[63]*Gu1[28];
Gu2[29] = + Gx1[56]*Gu1[1] + Gx1[57]*Gu1[5] + Gx1[58]*Gu1[9] + Gx1[59]*Gu1[13] + Gx1[60]*Gu1[17] + Gx1[61]*Gu1[21] + Gx1[62]*Gu1[25] + Gx1[63]*Gu1[29];
Gu2[30] = + Gx1[56]*Gu1[2] + Gx1[57]*Gu1[6] + Gx1[58]*Gu1[10] + Gx1[59]*Gu1[14] + Gx1[60]*Gu1[18] + Gx1[61]*Gu1[22] + Gx1[62]*Gu1[26] + Gx1[63]*Gu1[30];
Gu2[31] = + Gx1[56]*Gu1[3] + Gx1[57]*Gu1[7] + Gx1[58]*Gu1[11] + Gx1[59]*Gu1[15] + Gx1[60]*Gu1[19] + Gx1[61]*Gu1[23] + Gx1[62]*Gu1[27] + Gx1[63]*Gu1[31];
}

void nmpc_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
Gu2[27] = Gu1[27];
Gu2[28] = Gu1[28];
Gu2[29] = Gu1[29];
Gu2[30] = Gu1[30];
Gu2[31] = Gu1[31];
}

void nmpc_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
nmpcWorkspace.H[(iRow * 1632 + 3264) + (iCol * 4 + 8)] += + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28];
nmpcWorkspace.H[(iRow * 1632 + 3264) + (iCol * 4 + 9)] += + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29];
nmpcWorkspace.H[(iRow * 1632 + 3264) + (iCol * 4 + 10)] += + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30];
nmpcWorkspace.H[(iRow * 1632 + 3264) + (iCol * 4 + 11)] += + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31];
nmpcWorkspace.H[(iRow * 1632 + 3672) + (iCol * 4 + 8)] += + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28];
nmpcWorkspace.H[(iRow * 1632 + 3672) + (iCol * 4 + 9)] += + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29];
nmpcWorkspace.H[(iRow * 1632 + 3672) + (iCol * 4 + 10)] += + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30];
nmpcWorkspace.H[(iRow * 1632 + 3672) + (iCol * 4 + 11)] += + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31];
nmpcWorkspace.H[(iRow * 1632 + 4080) + (iCol * 4 + 8)] += + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28];
nmpcWorkspace.H[(iRow * 1632 + 4080) + (iCol * 4 + 9)] += + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29];
nmpcWorkspace.H[(iRow * 1632 + 4080) + (iCol * 4 + 10)] += + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30];
nmpcWorkspace.H[(iRow * 1632 + 4080) + (iCol * 4 + 11)] += + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31];
nmpcWorkspace.H[(iRow * 1632 + 4488) + (iCol * 4 + 8)] += + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28];
nmpcWorkspace.H[(iRow * 1632 + 4488) + (iCol * 4 + 9)] += + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29];
nmpcWorkspace.H[(iRow * 1632 + 4488) + (iCol * 4 + 10)] += + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30];
nmpcWorkspace.H[(iRow * 1632 + 4488) + (iCol * 4 + 11)] += + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31];
}

void nmpc_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
nmpcWorkspace.H[(iRow * 1632 + 3264) + (iCol * 4 + 8)] = R11[0];
nmpcWorkspace.H[(iRow * 1632 + 3264) + (iCol * 4 + 9)] = R11[1];
nmpcWorkspace.H[(iRow * 1632 + 3264) + (iCol * 4 + 10)] = R11[2];
nmpcWorkspace.H[(iRow * 1632 + 3264) + (iCol * 4 + 11)] = R11[3];
nmpcWorkspace.H[(iRow * 1632 + 3672) + (iCol * 4 + 8)] = R11[4];
nmpcWorkspace.H[(iRow * 1632 + 3672) + (iCol * 4 + 9)] = R11[5];
nmpcWorkspace.H[(iRow * 1632 + 3672) + (iCol * 4 + 10)] = R11[6];
nmpcWorkspace.H[(iRow * 1632 + 3672) + (iCol * 4 + 11)] = R11[7];
nmpcWorkspace.H[(iRow * 1632 + 4080) + (iCol * 4 + 8)] = R11[8];
nmpcWorkspace.H[(iRow * 1632 + 4080) + (iCol * 4 + 9)] = R11[9];
nmpcWorkspace.H[(iRow * 1632 + 4080) + (iCol * 4 + 10)] = R11[10];
nmpcWorkspace.H[(iRow * 1632 + 4080) + (iCol * 4 + 11)] = R11[11];
nmpcWorkspace.H[(iRow * 1632 + 4488) + (iCol * 4 + 8)] = R11[12];
nmpcWorkspace.H[(iRow * 1632 + 4488) + (iCol * 4 + 9)] = R11[13];
nmpcWorkspace.H[(iRow * 1632 + 4488) + (iCol * 4 + 10)] = R11[14];
nmpcWorkspace.H[(iRow * 1632 + 4488) + (iCol * 4 + 11)] = R11[15];
}

void nmpc_zeroBlockH11( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 1632 + 3264) + (iCol * 4 + 8)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 1632 + 3264) + (iCol * 4 + 9)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 1632 + 3264) + (iCol * 4 + 10)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 1632 + 3264) + (iCol * 4 + 11)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 1632 + 3672) + (iCol * 4 + 8)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 1632 + 3672) + (iCol * 4 + 9)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 1632 + 3672) + (iCol * 4 + 10)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 1632 + 3672) + (iCol * 4 + 11)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 1632 + 4080) + (iCol * 4 + 8)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 1632 + 4080) + (iCol * 4 + 9)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 1632 + 4080) + (iCol * 4 + 10)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 1632 + 4080) + (iCol * 4 + 11)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 1632 + 4488) + (iCol * 4 + 8)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 1632 + 4488) + (iCol * 4 + 9)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 1632 + 4488) + (iCol * 4 + 10)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 1632 + 4488) + (iCol * 4 + 11)] = 0.0000000000000000e+00;
}

void nmpc_copyHTH( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 1632 + 3264) + (iCol * 4 + 8)] = nmpcWorkspace.H[(iCol * 1632 + 3264) + (iRow * 4 + 8)];
nmpcWorkspace.H[(iRow * 1632 + 3264) + (iCol * 4 + 9)] = nmpcWorkspace.H[(iCol * 1632 + 3672) + (iRow * 4 + 8)];
nmpcWorkspace.H[(iRow * 1632 + 3264) + (iCol * 4 + 10)] = nmpcWorkspace.H[(iCol * 1632 + 4080) + (iRow * 4 + 8)];
nmpcWorkspace.H[(iRow * 1632 + 3264) + (iCol * 4 + 11)] = nmpcWorkspace.H[(iCol * 1632 + 4488) + (iRow * 4 + 8)];
nmpcWorkspace.H[(iRow * 1632 + 3672) + (iCol * 4 + 8)] = nmpcWorkspace.H[(iCol * 1632 + 3264) + (iRow * 4 + 9)];
nmpcWorkspace.H[(iRow * 1632 + 3672) + (iCol * 4 + 9)] = nmpcWorkspace.H[(iCol * 1632 + 3672) + (iRow * 4 + 9)];
nmpcWorkspace.H[(iRow * 1632 + 3672) + (iCol * 4 + 10)] = nmpcWorkspace.H[(iCol * 1632 + 4080) + (iRow * 4 + 9)];
nmpcWorkspace.H[(iRow * 1632 + 3672) + (iCol * 4 + 11)] = nmpcWorkspace.H[(iCol * 1632 + 4488) + (iRow * 4 + 9)];
nmpcWorkspace.H[(iRow * 1632 + 4080) + (iCol * 4 + 8)] = nmpcWorkspace.H[(iCol * 1632 + 3264) + (iRow * 4 + 10)];
nmpcWorkspace.H[(iRow * 1632 + 4080) + (iCol * 4 + 9)] = nmpcWorkspace.H[(iCol * 1632 + 3672) + (iRow * 4 + 10)];
nmpcWorkspace.H[(iRow * 1632 + 4080) + (iCol * 4 + 10)] = nmpcWorkspace.H[(iCol * 1632 + 4080) + (iRow * 4 + 10)];
nmpcWorkspace.H[(iRow * 1632 + 4080) + (iCol * 4 + 11)] = nmpcWorkspace.H[(iCol * 1632 + 4488) + (iRow * 4 + 10)];
nmpcWorkspace.H[(iRow * 1632 + 4488) + (iCol * 4 + 8)] = nmpcWorkspace.H[(iCol * 1632 + 3264) + (iRow * 4 + 11)];
nmpcWorkspace.H[(iRow * 1632 + 4488) + (iCol * 4 + 9)] = nmpcWorkspace.H[(iCol * 1632 + 3672) + (iRow * 4 + 11)];
nmpcWorkspace.H[(iRow * 1632 + 4488) + (iCol * 4 + 10)] = nmpcWorkspace.H[(iCol * 1632 + 4080) + (iRow * 4 + 11)];
nmpcWorkspace.H[(iRow * 1632 + 4488) + (iCol * 4 + 11)] = nmpcWorkspace.H[(iCol * 1632 + 4488) + (iRow * 4 + 11)];
}

void nmpc_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7];
dNew[1] = + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3] + Gx1[12]*dOld[4] + Gx1[13]*dOld[5] + Gx1[14]*dOld[6] + Gx1[15]*dOld[7];
dNew[2] = + Gx1[16]*dOld[0] + Gx1[17]*dOld[1] + Gx1[18]*dOld[2] + Gx1[19]*dOld[3] + Gx1[20]*dOld[4] + Gx1[21]*dOld[5] + Gx1[22]*dOld[6] + Gx1[23]*dOld[7];
dNew[3] = + Gx1[24]*dOld[0] + Gx1[25]*dOld[1] + Gx1[26]*dOld[2] + Gx1[27]*dOld[3] + Gx1[28]*dOld[4] + Gx1[29]*dOld[5] + Gx1[30]*dOld[6] + Gx1[31]*dOld[7];
dNew[4] = + Gx1[32]*dOld[0] + Gx1[33]*dOld[1] + Gx1[34]*dOld[2] + Gx1[35]*dOld[3] + Gx1[36]*dOld[4] + Gx1[37]*dOld[5] + Gx1[38]*dOld[6] + Gx1[39]*dOld[7];
dNew[5] = + Gx1[40]*dOld[0] + Gx1[41]*dOld[1] + Gx1[42]*dOld[2] + Gx1[43]*dOld[3] + Gx1[44]*dOld[4] + Gx1[45]*dOld[5] + Gx1[46]*dOld[6] + Gx1[47]*dOld[7];
dNew[6] = + Gx1[48]*dOld[0] + Gx1[49]*dOld[1] + Gx1[50]*dOld[2] + Gx1[51]*dOld[3] + Gx1[52]*dOld[4] + Gx1[53]*dOld[5] + Gx1[54]*dOld[6] + Gx1[55]*dOld[7];
dNew[7] = + Gx1[56]*dOld[0] + Gx1[57]*dOld[1] + Gx1[58]*dOld[2] + Gx1[59]*dOld[3] + Gx1[60]*dOld[4] + Gx1[61]*dOld[5] + Gx1[62]*dOld[6] + Gx1[63]*dOld[7];
}

void nmpc_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + nmpcWorkspace.QN1[0]*dOld[0] + nmpcWorkspace.QN1[1]*dOld[1] + nmpcWorkspace.QN1[2]*dOld[2] + nmpcWorkspace.QN1[3]*dOld[3] + nmpcWorkspace.QN1[4]*dOld[4] + nmpcWorkspace.QN1[5]*dOld[5] + nmpcWorkspace.QN1[6]*dOld[6] + nmpcWorkspace.QN1[7]*dOld[7];
dNew[1] = + nmpcWorkspace.QN1[8]*dOld[0] + nmpcWorkspace.QN1[9]*dOld[1] + nmpcWorkspace.QN1[10]*dOld[2] + nmpcWorkspace.QN1[11]*dOld[3] + nmpcWorkspace.QN1[12]*dOld[4] + nmpcWorkspace.QN1[13]*dOld[5] + nmpcWorkspace.QN1[14]*dOld[6] + nmpcWorkspace.QN1[15]*dOld[7];
dNew[2] = + nmpcWorkspace.QN1[16]*dOld[0] + nmpcWorkspace.QN1[17]*dOld[1] + nmpcWorkspace.QN1[18]*dOld[2] + nmpcWorkspace.QN1[19]*dOld[3] + nmpcWorkspace.QN1[20]*dOld[4] + nmpcWorkspace.QN1[21]*dOld[5] + nmpcWorkspace.QN1[22]*dOld[6] + nmpcWorkspace.QN1[23]*dOld[7];
dNew[3] = + nmpcWorkspace.QN1[24]*dOld[0] + nmpcWorkspace.QN1[25]*dOld[1] + nmpcWorkspace.QN1[26]*dOld[2] + nmpcWorkspace.QN1[27]*dOld[3] + nmpcWorkspace.QN1[28]*dOld[4] + nmpcWorkspace.QN1[29]*dOld[5] + nmpcWorkspace.QN1[30]*dOld[6] + nmpcWorkspace.QN1[31]*dOld[7];
dNew[4] = + nmpcWorkspace.QN1[32]*dOld[0] + nmpcWorkspace.QN1[33]*dOld[1] + nmpcWorkspace.QN1[34]*dOld[2] + nmpcWorkspace.QN1[35]*dOld[3] + nmpcWorkspace.QN1[36]*dOld[4] + nmpcWorkspace.QN1[37]*dOld[5] + nmpcWorkspace.QN1[38]*dOld[6] + nmpcWorkspace.QN1[39]*dOld[7];
dNew[5] = + nmpcWorkspace.QN1[40]*dOld[0] + nmpcWorkspace.QN1[41]*dOld[1] + nmpcWorkspace.QN1[42]*dOld[2] + nmpcWorkspace.QN1[43]*dOld[3] + nmpcWorkspace.QN1[44]*dOld[4] + nmpcWorkspace.QN1[45]*dOld[5] + nmpcWorkspace.QN1[46]*dOld[6] + nmpcWorkspace.QN1[47]*dOld[7];
dNew[6] = + nmpcWorkspace.QN1[48]*dOld[0] + nmpcWorkspace.QN1[49]*dOld[1] + nmpcWorkspace.QN1[50]*dOld[2] + nmpcWorkspace.QN1[51]*dOld[3] + nmpcWorkspace.QN1[52]*dOld[4] + nmpcWorkspace.QN1[53]*dOld[5] + nmpcWorkspace.QN1[54]*dOld[6] + nmpcWorkspace.QN1[55]*dOld[7];
dNew[7] = + nmpcWorkspace.QN1[56]*dOld[0] + nmpcWorkspace.QN1[57]*dOld[1] + nmpcWorkspace.QN1[58]*dOld[2] + nmpcWorkspace.QN1[59]*dOld[3] + nmpcWorkspace.QN1[60]*dOld[4] + nmpcWorkspace.QN1[61]*dOld[5] + nmpcWorkspace.QN1[62]*dOld[6] + nmpcWorkspace.QN1[63]*dOld[7];
}

void nmpc_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10] + R2[11]*Dy1[11];
RDy1[1] = + R2[12]*Dy1[0] + R2[13]*Dy1[1] + R2[14]*Dy1[2] + R2[15]*Dy1[3] + R2[16]*Dy1[4] + R2[17]*Dy1[5] + R2[18]*Dy1[6] + R2[19]*Dy1[7] + R2[20]*Dy1[8] + R2[21]*Dy1[9] + R2[22]*Dy1[10] + R2[23]*Dy1[11];
RDy1[2] = + R2[24]*Dy1[0] + R2[25]*Dy1[1] + R2[26]*Dy1[2] + R2[27]*Dy1[3] + R2[28]*Dy1[4] + R2[29]*Dy1[5] + R2[30]*Dy1[6] + R2[31]*Dy1[7] + R2[32]*Dy1[8] + R2[33]*Dy1[9] + R2[34]*Dy1[10] + R2[35]*Dy1[11];
RDy1[3] = + R2[36]*Dy1[0] + R2[37]*Dy1[1] + R2[38]*Dy1[2] + R2[39]*Dy1[3] + R2[40]*Dy1[4] + R2[41]*Dy1[5] + R2[42]*Dy1[6] + R2[43]*Dy1[7] + R2[44]*Dy1[8] + R2[45]*Dy1[9] + R2[46]*Dy1[10] + R2[47]*Dy1[11];
}

void nmpc_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10] + Q2[11]*Dy1[11];
QDy1[1] = + Q2[12]*Dy1[0] + Q2[13]*Dy1[1] + Q2[14]*Dy1[2] + Q2[15]*Dy1[3] + Q2[16]*Dy1[4] + Q2[17]*Dy1[5] + Q2[18]*Dy1[6] + Q2[19]*Dy1[7] + Q2[20]*Dy1[8] + Q2[21]*Dy1[9] + Q2[22]*Dy1[10] + Q2[23]*Dy1[11];
QDy1[2] = + Q2[24]*Dy1[0] + Q2[25]*Dy1[1] + Q2[26]*Dy1[2] + Q2[27]*Dy1[3] + Q2[28]*Dy1[4] + Q2[29]*Dy1[5] + Q2[30]*Dy1[6] + Q2[31]*Dy1[7] + Q2[32]*Dy1[8] + Q2[33]*Dy1[9] + Q2[34]*Dy1[10] + Q2[35]*Dy1[11];
QDy1[3] = + Q2[36]*Dy1[0] + Q2[37]*Dy1[1] + Q2[38]*Dy1[2] + Q2[39]*Dy1[3] + Q2[40]*Dy1[4] + Q2[41]*Dy1[5] + Q2[42]*Dy1[6] + Q2[43]*Dy1[7] + Q2[44]*Dy1[8] + Q2[45]*Dy1[9] + Q2[46]*Dy1[10] + Q2[47]*Dy1[11];
QDy1[4] = + Q2[48]*Dy1[0] + Q2[49]*Dy1[1] + Q2[50]*Dy1[2] + Q2[51]*Dy1[3] + Q2[52]*Dy1[4] + Q2[53]*Dy1[5] + Q2[54]*Dy1[6] + Q2[55]*Dy1[7] + Q2[56]*Dy1[8] + Q2[57]*Dy1[9] + Q2[58]*Dy1[10] + Q2[59]*Dy1[11];
QDy1[5] = + Q2[60]*Dy1[0] + Q2[61]*Dy1[1] + Q2[62]*Dy1[2] + Q2[63]*Dy1[3] + Q2[64]*Dy1[4] + Q2[65]*Dy1[5] + Q2[66]*Dy1[6] + Q2[67]*Dy1[7] + Q2[68]*Dy1[8] + Q2[69]*Dy1[9] + Q2[70]*Dy1[10] + Q2[71]*Dy1[11];
QDy1[6] = + Q2[72]*Dy1[0] + Q2[73]*Dy1[1] + Q2[74]*Dy1[2] + Q2[75]*Dy1[3] + Q2[76]*Dy1[4] + Q2[77]*Dy1[5] + Q2[78]*Dy1[6] + Q2[79]*Dy1[7] + Q2[80]*Dy1[8] + Q2[81]*Dy1[9] + Q2[82]*Dy1[10] + Q2[83]*Dy1[11];
QDy1[7] = + Q2[84]*Dy1[0] + Q2[85]*Dy1[1] + Q2[86]*Dy1[2] + Q2[87]*Dy1[3] + Q2[88]*Dy1[4] + Q2[89]*Dy1[5] + Q2[90]*Dy1[6] + Q2[91]*Dy1[7] + Q2[92]*Dy1[8] + Q2[93]*Dy1[9] + Q2[94]*Dy1[10] + Q2[95]*Dy1[11];
}

void nmpc_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[4]*QDy1[1] + E1[8]*QDy1[2] + E1[12]*QDy1[3] + E1[16]*QDy1[4] + E1[20]*QDy1[5] + E1[24]*QDy1[6] + E1[28]*QDy1[7];
U1[1] += + E1[1]*QDy1[0] + E1[5]*QDy1[1] + E1[9]*QDy1[2] + E1[13]*QDy1[3] + E1[17]*QDy1[4] + E1[21]*QDy1[5] + E1[25]*QDy1[6] + E1[29]*QDy1[7];
U1[2] += + E1[2]*QDy1[0] + E1[6]*QDy1[1] + E1[10]*QDy1[2] + E1[14]*QDy1[3] + E1[18]*QDy1[4] + E1[22]*QDy1[5] + E1[26]*QDy1[6] + E1[30]*QDy1[7];
U1[3] += + E1[3]*QDy1[0] + E1[7]*QDy1[1] + E1[11]*QDy1[2] + E1[15]*QDy1[3] + E1[19]*QDy1[4] + E1[23]*QDy1[5] + E1[27]*QDy1[6] + E1[31]*QDy1[7];
}

void nmpc_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[4]*Gx1[8] + E1[8]*Gx1[16] + E1[12]*Gx1[24] + E1[16]*Gx1[32] + E1[20]*Gx1[40] + E1[24]*Gx1[48] + E1[28]*Gx1[56];
H101[1] += + E1[0]*Gx1[1] + E1[4]*Gx1[9] + E1[8]*Gx1[17] + E1[12]*Gx1[25] + E1[16]*Gx1[33] + E1[20]*Gx1[41] + E1[24]*Gx1[49] + E1[28]*Gx1[57];
H101[2] += + E1[0]*Gx1[2] + E1[4]*Gx1[10] + E1[8]*Gx1[18] + E1[12]*Gx1[26] + E1[16]*Gx1[34] + E1[20]*Gx1[42] + E1[24]*Gx1[50] + E1[28]*Gx1[58];
H101[3] += + E1[0]*Gx1[3] + E1[4]*Gx1[11] + E1[8]*Gx1[19] + E1[12]*Gx1[27] + E1[16]*Gx1[35] + E1[20]*Gx1[43] + E1[24]*Gx1[51] + E1[28]*Gx1[59];
H101[4] += + E1[0]*Gx1[4] + E1[4]*Gx1[12] + E1[8]*Gx1[20] + E1[12]*Gx1[28] + E1[16]*Gx1[36] + E1[20]*Gx1[44] + E1[24]*Gx1[52] + E1[28]*Gx1[60];
H101[5] += + E1[0]*Gx1[5] + E1[4]*Gx1[13] + E1[8]*Gx1[21] + E1[12]*Gx1[29] + E1[16]*Gx1[37] + E1[20]*Gx1[45] + E1[24]*Gx1[53] + E1[28]*Gx1[61];
H101[6] += + E1[0]*Gx1[6] + E1[4]*Gx1[14] + E1[8]*Gx1[22] + E1[12]*Gx1[30] + E1[16]*Gx1[38] + E1[20]*Gx1[46] + E1[24]*Gx1[54] + E1[28]*Gx1[62];
H101[7] += + E1[0]*Gx1[7] + E1[4]*Gx1[15] + E1[8]*Gx1[23] + E1[12]*Gx1[31] + E1[16]*Gx1[39] + E1[20]*Gx1[47] + E1[24]*Gx1[55] + E1[28]*Gx1[63];
H101[8] += + E1[1]*Gx1[0] + E1[5]*Gx1[8] + E1[9]*Gx1[16] + E1[13]*Gx1[24] + E1[17]*Gx1[32] + E1[21]*Gx1[40] + E1[25]*Gx1[48] + E1[29]*Gx1[56];
H101[9] += + E1[1]*Gx1[1] + E1[5]*Gx1[9] + E1[9]*Gx1[17] + E1[13]*Gx1[25] + E1[17]*Gx1[33] + E1[21]*Gx1[41] + E1[25]*Gx1[49] + E1[29]*Gx1[57];
H101[10] += + E1[1]*Gx1[2] + E1[5]*Gx1[10] + E1[9]*Gx1[18] + E1[13]*Gx1[26] + E1[17]*Gx1[34] + E1[21]*Gx1[42] + E1[25]*Gx1[50] + E1[29]*Gx1[58];
H101[11] += + E1[1]*Gx1[3] + E1[5]*Gx1[11] + E1[9]*Gx1[19] + E1[13]*Gx1[27] + E1[17]*Gx1[35] + E1[21]*Gx1[43] + E1[25]*Gx1[51] + E1[29]*Gx1[59];
H101[12] += + E1[1]*Gx1[4] + E1[5]*Gx1[12] + E1[9]*Gx1[20] + E1[13]*Gx1[28] + E1[17]*Gx1[36] + E1[21]*Gx1[44] + E1[25]*Gx1[52] + E1[29]*Gx1[60];
H101[13] += + E1[1]*Gx1[5] + E1[5]*Gx1[13] + E1[9]*Gx1[21] + E1[13]*Gx1[29] + E1[17]*Gx1[37] + E1[21]*Gx1[45] + E1[25]*Gx1[53] + E1[29]*Gx1[61];
H101[14] += + E1[1]*Gx1[6] + E1[5]*Gx1[14] + E1[9]*Gx1[22] + E1[13]*Gx1[30] + E1[17]*Gx1[38] + E1[21]*Gx1[46] + E1[25]*Gx1[54] + E1[29]*Gx1[62];
H101[15] += + E1[1]*Gx1[7] + E1[5]*Gx1[15] + E1[9]*Gx1[23] + E1[13]*Gx1[31] + E1[17]*Gx1[39] + E1[21]*Gx1[47] + E1[25]*Gx1[55] + E1[29]*Gx1[63];
H101[16] += + E1[2]*Gx1[0] + E1[6]*Gx1[8] + E1[10]*Gx1[16] + E1[14]*Gx1[24] + E1[18]*Gx1[32] + E1[22]*Gx1[40] + E1[26]*Gx1[48] + E1[30]*Gx1[56];
H101[17] += + E1[2]*Gx1[1] + E1[6]*Gx1[9] + E1[10]*Gx1[17] + E1[14]*Gx1[25] + E1[18]*Gx1[33] + E1[22]*Gx1[41] + E1[26]*Gx1[49] + E1[30]*Gx1[57];
H101[18] += + E1[2]*Gx1[2] + E1[6]*Gx1[10] + E1[10]*Gx1[18] + E1[14]*Gx1[26] + E1[18]*Gx1[34] + E1[22]*Gx1[42] + E1[26]*Gx1[50] + E1[30]*Gx1[58];
H101[19] += + E1[2]*Gx1[3] + E1[6]*Gx1[11] + E1[10]*Gx1[19] + E1[14]*Gx1[27] + E1[18]*Gx1[35] + E1[22]*Gx1[43] + E1[26]*Gx1[51] + E1[30]*Gx1[59];
H101[20] += + E1[2]*Gx1[4] + E1[6]*Gx1[12] + E1[10]*Gx1[20] + E1[14]*Gx1[28] + E1[18]*Gx1[36] + E1[22]*Gx1[44] + E1[26]*Gx1[52] + E1[30]*Gx1[60];
H101[21] += + E1[2]*Gx1[5] + E1[6]*Gx1[13] + E1[10]*Gx1[21] + E1[14]*Gx1[29] + E1[18]*Gx1[37] + E1[22]*Gx1[45] + E1[26]*Gx1[53] + E1[30]*Gx1[61];
H101[22] += + E1[2]*Gx1[6] + E1[6]*Gx1[14] + E1[10]*Gx1[22] + E1[14]*Gx1[30] + E1[18]*Gx1[38] + E1[22]*Gx1[46] + E1[26]*Gx1[54] + E1[30]*Gx1[62];
H101[23] += + E1[2]*Gx1[7] + E1[6]*Gx1[15] + E1[10]*Gx1[23] + E1[14]*Gx1[31] + E1[18]*Gx1[39] + E1[22]*Gx1[47] + E1[26]*Gx1[55] + E1[30]*Gx1[63];
H101[24] += + E1[3]*Gx1[0] + E1[7]*Gx1[8] + E1[11]*Gx1[16] + E1[15]*Gx1[24] + E1[19]*Gx1[32] + E1[23]*Gx1[40] + E1[27]*Gx1[48] + E1[31]*Gx1[56];
H101[25] += + E1[3]*Gx1[1] + E1[7]*Gx1[9] + E1[11]*Gx1[17] + E1[15]*Gx1[25] + E1[19]*Gx1[33] + E1[23]*Gx1[41] + E1[27]*Gx1[49] + E1[31]*Gx1[57];
H101[26] += + E1[3]*Gx1[2] + E1[7]*Gx1[10] + E1[11]*Gx1[18] + E1[15]*Gx1[26] + E1[19]*Gx1[34] + E1[23]*Gx1[42] + E1[27]*Gx1[50] + E1[31]*Gx1[58];
H101[27] += + E1[3]*Gx1[3] + E1[7]*Gx1[11] + E1[11]*Gx1[19] + E1[15]*Gx1[27] + E1[19]*Gx1[35] + E1[23]*Gx1[43] + E1[27]*Gx1[51] + E1[31]*Gx1[59];
H101[28] += + E1[3]*Gx1[4] + E1[7]*Gx1[12] + E1[11]*Gx1[20] + E1[15]*Gx1[28] + E1[19]*Gx1[36] + E1[23]*Gx1[44] + E1[27]*Gx1[52] + E1[31]*Gx1[60];
H101[29] += + E1[3]*Gx1[5] + E1[7]*Gx1[13] + E1[11]*Gx1[21] + E1[15]*Gx1[29] + E1[19]*Gx1[37] + E1[23]*Gx1[45] + E1[27]*Gx1[53] + E1[31]*Gx1[61];
H101[30] += + E1[3]*Gx1[6] + E1[7]*Gx1[14] + E1[11]*Gx1[22] + E1[15]*Gx1[30] + E1[19]*Gx1[38] + E1[23]*Gx1[46] + E1[27]*Gx1[54] + E1[31]*Gx1[62];
H101[31] += + E1[3]*Gx1[7] + E1[7]*Gx1[15] + E1[11]*Gx1[23] + E1[15]*Gx1[31] + E1[19]*Gx1[39] + E1[23]*Gx1[47] + E1[27]*Gx1[55] + E1[31]*Gx1[63];
}

void nmpc_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 32; lCopy++) H101[ lCopy ] = 0; }
}

void nmpc_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1] + E1[2]*U1[2] + E1[3]*U1[3];
dNew[1] += + E1[4]*U1[0] + E1[5]*U1[1] + E1[6]*U1[2] + E1[7]*U1[3];
dNew[2] += + E1[8]*U1[0] + E1[9]*U1[1] + E1[10]*U1[2] + E1[11]*U1[3];
dNew[3] += + E1[12]*U1[0] + E1[13]*U1[1] + E1[14]*U1[2] + E1[15]*U1[3];
dNew[4] += + E1[16]*U1[0] + E1[17]*U1[1] + E1[18]*U1[2] + E1[19]*U1[3];
dNew[5] += + E1[20]*U1[0] + E1[21]*U1[1] + E1[22]*U1[2] + E1[23]*U1[3];
dNew[6] += + E1[24]*U1[0] + E1[25]*U1[1] + E1[26]*U1[2] + E1[27]*U1[3];
dNew[7] += + E1[28]*U1[0] + E1[29]*U1[1] + E1[30]*U1[2] + E1[31]*U1[3];
}

void nmpc_zeroBlockH00(  )
{
nmpcWorkspace.H[0] = 0.0000000000000000e+00;
nmpcWorkspace.H[1] = 0.0000000000000000e+00;
nmpcWorkspace.H[2] = 0.0000000000000000e+00;
nmpcWorkspace.H[3] = 0.0000000000000000e+00;
nmpcWorkspace.H[4] = 0.0000000000000000e+00;
nmpcWorkspace.H[5] = 0.0000000000000000e+00;
nmpcWorkspace.H[6] = 0.0000000000000000e+00;
nmpcWorkspace.H[7] = 0.0000000000000000e+00;
nmpcWorkspace.H[408] = 0.0000000000000000e+00;
nmpcWorkspace.H[409] = 0.0000000000000000e+00;
nmpcWorkspace.H[410] = 0.0000000000000000e+00;
nmpcWorkspace.H[411] = 0.0000000000000000e+00;
nmpcWorkspace.H[412] = 0.0000000000000000e+00;
nmpcWorkspace.H[413] = 0.0000000000000000e+00;
nmpcWorkspace.H[414] = 0.0000000000000000e+00;
nmpcWorkspace.H[415] = 0.0000000000000000e+00;
nmpcWorkspace.H[816] = 0.0000000000000000e+00;
nmpcWorkspace.H[817] = 0.0000000000000000e+00;
nmpcWorkspace.H[818] = 0.0000000000000000e+00;
nmpcWorkspace.H[819] = 0.0000000000000000e+00;
nmpcWorkspace.H[820] = 0.0000000000000000e+00;
nmpcWorkspace.H[821] = 0.0000000000000000e+00;
nmpcWorkspace.H[822] = 0.0000000000000000e+00;
nmpcWorkspace.H[823] = 0.0000000000000000e+00;
nmpcWorkspace.H[1224] = 0.0000000000000000e+00;
nmpcWorkspace.H[1225] = 0.0000000000000000e+00;
nmpcWorkspace.H[1226] = 0.0000000000000000e+00;
nmpcWorkspace.H[1227] = 0.0000000000000000e+00;
nmpcWorkspace.H[1228] = 0.0000000000000000e+00;
nmpcWorkspace.H[1229] = 0.0000000000000000e+00;
nmpcWorkspace.H[1230] = 0.0000000000000000e+00;
nmpcWorkspace.H[1231] = 0.0000000000000000e+00;
nmpcWorkspace.H[1632] = 0.0000000000000000e+00;
nmpcWorkspace.H[1633] = 0.0000000000000000e+00;
nmpcWorkspace.H[1634] = 0.0000000000000000e+00;
nmpcWorkspace.H[1635] = 0.0000000000000000e+00;
nmpcWorkspace.H[1636] = 0.0000000000000000e+00;
nmpcWorkspace.H[1637] = 0.0000000000000000e+00;
nmpcWorkspace.H[1638] = 0.0000000000000000e+00;
nmpcWorkspace.H[1639] = 0.0000000000000000e+00;
nmpcWorkspace.H[2040] = 0.0000000000000000e+00;
nmpcWorkspace.H[2041] = 0.0000000000000000e+00;
nmpcWorkspace.H[2042] = 0.0000000000000000e+00;
nmpcWorkspace.H[2043] = 0.0000000000000000e+00;
nmpcWorkspace.H[2044] = 0.0000000000000000e+00;
nmpcWorkspace.H[2045] = 0.0000000000000000e+00;
nmpcWorkspace.H[2046] = 0.0000000000000000e+00;
nmpcWorkspace.H[2047] = 0.0000000000000000e+00;
nmpcWorkspace.H[2448] = 0.0000000000000000e+00;
nmpcWorkspace.H[2449] = 0.0000000000000000e+00;
nmpcWorkspace.H[2450] = 0.0000000000000000e+00;
nmpcWorkspace.H[2451] = 0.0000000000000000e+00;
nmpcWorkspace.H[2452] = 0.0000000000000000e+00;
nmpcWorkspace.H[2453] = 0.0000000000000000e+00;
nmpcWorkspace.H[2454] = 0.0000000000000000e+00;
nmpcWorkspace.H[2455] = 0.0000000000000000e+00;
nmpcWorkspace.H[2856] = 0.0000000000000000e+00;
nmpcWorkspace.H[2857] = 0.0000000000000000e+00;
nmpcWorkspace.H[2858] = 0.0000000000000000e+00;
nmpcWorkspace.H[2859] = 0.0000000000000000e+00;
nmpcWorkspace.H[2860] = 0.0000000000000000e+00;
nmpcWorkspace.H[2861] = 0.0000000000000000e+00;
nmpcWorkspace.H[2862] = 0.0000000000000000e+00;
nmpcWorkspace.H[2863] = 0.0000000000000000e+00;
}

void nmpc_multCTQC( real_t* const Gx1, real_t* const Gx2 )
{
nmpcWorkspace.H[0] += + Gx1[0]*Gx2[0] + Gx1[8]*Gx2[8] + Gx1[16]*Gx2[16] + Gx1[24]*Gx2[24] + Gx1[32]*Gx2[32] + Gx1[40]*Gx2[40] + Gx1[48]*Gx2[48] + Gx1[56]*Gx2[56];
nmpcWorkspace.H[1] += + Gx1[0]*Gx2[1] + Gx1[8]*Gx2[9] + Gx1[16]*Gx2[17] + Gx1[24]*Gx2[25] + Gx1[32]*Gx2[33] + Gx1[40]*Gx2[41] + Gx1[48]*Gx2[49] + Gx1[56]*Gx2[57];
nmpcWorkspace.H[2] += + Gx1[0]*Gx2[2] + Gx1[8]*Gx2[10] + Gx1[16]*Gx2[18] + Gx1[24]*Gx2[26] + Gx1[32]*Gx2[34] + Gx1[40]*Gx2[42] + Gx1[48]*Gx2[50] + Gx1[56]*Gx2[58];
nmpcWorkspace.H[3] += + Gx1[0]*Gx2[3] + Gx1[8]*Gx2[11] + Gx1[16]*Gx2[19] + Gx1[24]*Gx2[27] + Gx1[32]*Gx2[35] + Gx1[40]*Gx2[43] + Gx1[48]*Gx2[51] + Gx1[56]*Gx2[59];
nmpcWorkspace.H[4] += + Gx1[0]*Gx2[4] + Gx1[8]*Gx2[12] + Gx1[16]*Gx2[20] + Gx1[24]*Gx2[28] + Gx1[32]*Gx2[36] + Gx1[40]*Gx2[44] + Gx1[48]*Gx2[52] + Gx1[56]*Gx2[60];
nmpcWorkspace.H[5] += + Gx1[0]*Gx2[5] + Gx1[8]*Gx2[13] + Gx1[16]*Gx2[21] + Gx1[24]*Gx2[29] + Gx1[32]*Gx2[37] + Gx1[40]*Gx2[45] + Gx1[48]*Gx2[53] + Gx1[56]*Gx2[61];
nmpcWorkspace.H[6] += + Gx1[0]*Gx2[6] + Gx1[8]*Gx2[14] + Gx1[16]*Gx2[22] + Gx1[24]*Gx2[30] + Gx1[32]*Gx2[38] + Gx1[40]*Gx2[46] + Gx1[48]*Gx2[54] + Gx1[56]*Gx2[62];
nmpcWorkspace.H[7] += + Gx1[0]*Gx2[7] + Gx1[8]*Gx2[15] + Gx1[16]*Gx2[23] + Gx1[24]*Gx2[31] + Gx1[32]*Gx2[39] + Gx1[40]*Gx2[47] + Gx1[48]*Gx2[55] + Gx1[56]*Gx2[63];
nmpcWorkspace.H[408] += + Gx1[1]*Gx2[0] + Gx1[9]*Gx2[8] + Gx1[17]*Gx2[16] + Gx1[25]*Gx2[24] + Gx1[33]*Gx2[32] + Gx1[41]*Gx2[40] + Gx1[49]*Gx2[48] + Gx1[57]*Gx2[56];
nmpcWorkspace.H[409] += + Gx1[1]*Gx2[1] + Gx1[9]*Gx2[9] + Gx1[17]*Gx2[17] + Gx1[25]*Gx2[25] + Gx1[33]*Gx2[33] + Gx1[41]*Gx2[41] + Gx1[49]*Gx2[49] + Gx1[57]*Gx2[57];
nmpcWorkspace.H[410] += + Gx1[1]*Gx2[2] + Gx1[9]*Gx2[10] + Gx1[17]*Gx2[18] + Gx1[25]*Gx2[26] + Gx1[33]*Gx2[34] + Gx1[41]*Gx2[42] + Gx1[49]*Gx2[50] + Gx1[57]*Gx2[58];
nmpcWorkspace.H[411] += + Gx1[1]*Gx2[3] + Gx1[9]*Gx2[11] + Gx1[17]*Gx2[19] + Gx1[25]*Gx2[27] + Gx1[33]*Gx2[35] + Gx1[41]*Gx2[43] + Gx1[49]*Gx2[51] + Gx1[57]*Gx2[59];
nmpcWorkspace.H[412] += + Gx1[1]*Gx2[4] + Gx1[9]*Gx2[12] + Gx1[17]*Gx2[20] + Gx1[25]*Gx2[28] + Gx1[33]*Gx2[36] + Gx1[41]*Gx2[44] + Gx1[49]*Gx2[52] + Gx1[57]*Gx2[60];
nmpcWorkspace.H[413] += + Gx1[1]*Gx2[5] + Gx1[9]*Gx2[13] + Gx1[17]*Gx2[21] + Gx1[25]*Gx2[29] + Gx1[33]*Gx2[37] + Gx1[41]*Gx2[45] + Gx1[49]*Gx2[53] + Gx1[57]*Gx2[61];
nmpcWorkspace.H[414] += + Gx1[1]*Gx2[6] + Gx1[9]*Gx2[14] + Gx1[17]*Gx2[22] + Gx1[25]*Gx2[30] + Gx1[33]*Gx2[38] + Gx1[41]*Gx2[46] + Gx1[49]*Gx2[54] + Gx1[57]*Gx2[62];
nmpcWorkspace.H[415] += + Gx1[1]*Gx2[7] + Gx1[9]*Gx2[15] + Gx1[17]*Gx2[23] + Gx1[25]*Gx2[31] + Gx1[33]*Gx2[39] + Gx1[41]*Gx2[47] + Gx1[49]*Gx2[55] + Gx1[57]*Gx2[63];
nmpcWorkspace.H[816] += + Gx1[2]*Gx2[0] + Gx1[10]*Gx2[8] + Gx1[18]*Gx2[16] + Gx1[26]*Gx2[24] + Gx1[34]*Gx2[32] + Gx1[42]*Gx2[40] + Gx1[50]*Gx2[48] + Gx1[58]*Gx2[56];
nmpcWorkspace.H[817] += + Gx1[2]*Gx2[1] + Gx1[10]*Gx2[9] + Gx1[18]*Gx2[17] + Gx1[26]*Gx2[25] + Gx1[34]*Gx2[33] + Gx1[42]*Gx2[41] + Gx1[50]*Gx2[49] + Gx1[58]*Gx2[57];
nmpcWorkspace.H[818] += + Gx1[2]*Gx2[2] + Gx1[10]*Gx2[10] + Gx1[18]*Gx2[18] + Gx1[26]*Gx2[26] + Gx1[34]*Gx2[34] + Gx1[42]*Gx2[42] + Gx1[50]*Gx2[50] + Gx1[58]*Gx2[58];
nmpcWorkspace.H[819] += + Gx1[2]*Gx2[3] + Gx1[10]*Gx2[11] + Gx1[18]*Gx2[19] + Gx1[26]*Gx2[27] + Gx1[34]*Gx2[35] + Gx1[42]*Gx2[43] + Gx1[50]*Gx2[51] + Gx1[58]*Gx2[59];
nmpcWorkspace.H[820] += + Gx1[2]*Gx2[4] + Gx1[10]*Gx2[12] + Gx1[18]*Gx2[20] + Gx1[26]*Gx2[28] + Gx1[34]*Gx2[36] + Gx1[42]*Gx2[44] + Gx1[50]*Gx2[52] + Gx1[58]*Gx2[60];
nmpcWorkspace.H[821] += + Gx1[2]*Gx2[5] + Gx1[10]*Gx2[13] + Gx1[18]*Gx2[21] + Gx1[26]*Gx2[29] + Gx1[34]*Gx2[37] + Gx1[42]*Gx2[45] + Gx1[50]*Gx2[53] + Gx1[58]*Gx2[61];
nmpcWorkspace.H[822] += + Gx1[2]*Gx2[6] + Gx1[10]*Gx2[14] + Gx1[18]*Gx2[22] + Gx1[26]*Gx2[30] + Gx1[34]*Gx2[38] + Gx1[42]*Gx2[46] + Gx1[50]*Gx2[54] + Gx1[58]*Gx2[62];
nmpcWorkspace.H[823] += + Gx1[2]*Gx2[7] + Gx1[10]*Gx2[15] + Gx1[18]*Gx2[23] + Gx1[26]*Gx2[31] + Gx1[34]*Gx2[39] + Gx1[42]*Gx2[47] + Gx1[50]*Gx2[55] + Gx1[58]*Gx2[63];
nmpcWorkspace.H[1224] += + Gx1[3]*Gx2[0] + Gx1[11]*Gx2[8] + Gx1[19]*Gx2[16] + Gx1[27]*Gx2[24] + Gx1[35]*Gx2[32] + Gx1[43]*Gx2[40] + Gx1[51]*Gx2[48] + Gx1[59]*Gx2[56];
nmpcWorkspace.H[1225] += + Gx1[3]*Gx2[1] + Gx1[11]*Gx2[9] + Gx1[19]*Gx2[17] + Gx1[27]*Gx2[25] + Gx1[35]*Gx2[33] + Gx1[43]*Gx2[41] + Gx1[51]*Gx2[49] + Gx1[59]*Gx2[57];
nmpcWorkspace.H[1226] += + Gx1[3]*Gx2[2] + Gx1[11]*Gx2[10] + Gx1[19]*Gx2[18] + Gx1[27]*Gx2[26] + Gx1[35]*Gx2[34] + Gx1[43]*Gx2[42] + Gx1[51]*Gx2[50] + Gx1[59]*Gx2[58];
nmpcWorkspace.H[1227] += + Gx1[3]*Gx2[3] + Gx1[11]*Gx2[11] + Gx1[19]*Gx2[19] + Gx1[27]*Gx2[27] + Gx1[35]*Gx2[35] + Gx1[43]*Gx2[43] + Gx1[51]*Gx2[51] + Gx1[59]*Gx2[59];
nmpcWorkspace.H[1228] += + Gx1[3]*Gx2[4] + Gx1[11]*Gx2[12] + Gx1[19]*Gx2[20] + Gx1[27]*Gx2[28] + Gx1[35]*Gx2[36] + Gx1[43]*Gx2[44] + Gx1[51]*Gx2[52] + Gx1[59]*Gx2[60];
nmpcWorkspace.H[1229] += + Gx1[3]*Gx2[5] + Gx1[11]*Gx2[13] + Gx1[19]*Gx2[21] + Gx1[27]*Gx2[29] + Gx1[35]*Gx2[37] + Gx1[43]*Gx2[45] + Gx1[51]*Gx2[53] + Gx1[59]*Gx2[61];
nmpcWorkspace.H[1230] += + Gx1[3]*Gx2[6] + Gx1[11]*Gx2[14] + Gx1[19]*Gx2[22] + Gx1[27]*Gx2[30] + Gx1[35]*Gx2[38] + Gx1[43]*Gx2[46] + Gx1[51]*Gx2[54] + Gx1[59]*Gx2[62];
nmpcWorkspace.H[1231] += + Gx1[3]*Gx2[7] + Gx1[11]*Gx2[15] + Gx1[19]*Gx2[23] + Gx1[27]*Gx2[31] + Gx1[35]*Gx2[39] + Gx1[43]*Gx2[47] + Gx1[51]*Gx2[55] + Gx1[59]*Gx2[63];
nmpcWorkspace.H[1632] += + Gx1[4]*Gx2[0] + Gx1[12]*Gx2[8] + Gx1[20]*Gx2[16] + Gx1[28]*Gx2[24] + Gx1[36]*Gx2[32] + Gx1[44]*Gx2[40] + Gx1[52]*Gx2[48] + Gx1[60]*Gx2[56];
nmpcWorkspace.H[1633] += + Gx1[4]*Gx2[1] + Gx1[12]*Gx2[9] + Gx1[20]*Gx2[17] + Gx1[28]*Gx2[25] + Gx1[36]*Gx2[33] + Gx1[44]*Gx2[41] + Gx1[52]*Gx2[49] + Gx1[60]*Gx2[57];
nmpcWorkspace.H[1634] += + Gx1[4]*Gx2[2] + Gx1[12]*Gx2[10] + Gx1[20]*Gx2[18] + Gx1[28]*Gx2[26] + Gx1[36]*Gx2[34] + Gx1[44]*Gx2[42] + Gx1[52]*Gx2[50] + Gx1[60]*Gx2[58];
nmpcWorkspace.H[1635] += + Gx1[4]*Gx2[3] + Gx1[12]*Gx2[11] + Gx1[20]*Gx2[19] + Gx1[28]*Gx2[27] + Gx1[36]*Gx2[35] + Gx1[44]*Gx2[43] + Gx1[52]*Gx2[51] + Gx1[60]*Gx2[59];
nmpcWorkspace.H[1636] += + Gx1[4]*Gx2[4] + Gx1[12]*Gx2[12] + Gx1[20]*Gx2[20] + Gx1[28]*Gx2[28] + Gx1[36]*Gx2[36] + Gx1[44]*Gx2[44] + Gx1[52]*Gx2[52] + Gx1[60]*Gx2[60];
nmpcWorkspace.H[1637] += + Gx1[4]*Gx2[5] + Gx1[12]*Gx2[13] + Gx1[20]*Gx2[21] + Gx1[28]*Gx2[29] + Gx1[36]*Gx2[37] + Gx1[44]*Gx2[45] + Gx1[52]*Gx2[53] + Gx1[60]*Gx2[61];
nmpcWorkspace.H[1638] += + Gx1[4]*Gx2[6] + Gx1[12]*Gx2[14] + Gx1[20]*Gx2[22] + Gx1[28]*Gx2[30] + Gx1[36]*Gx2[38] + Gx1[44]*Gx2[46] + Gx1[52]*Gx2[54] + Gx1[60]*Gx2[62];
nmpcWorkspace.H[1639] += + Gx1[4]*Gx2[7] + Gx1[12]*Gx2[15] + Gx1[20]*Gx2[23] + Gx1[28]*Gx2[31] + Gx1[36]*Gx2[39] + Gx1[44]*Gx2[47] + Gx1[52]*Gx2[55] + Gx1[60]*Gx2[63];
nmpcWorkspace.H[2040] += + Gx1[5]*Gx2[0] + Gx1[13]*Gx2[8] + Gx1[21]*Gx2[16] + Gx1[29]*Gx2[24] + Gx1[37]*Gx2[32] + Gx1[45]*Gx2[40] + Gx1[53]*Gx2[48] + Gx1[61]*Gx2[56];
nmpcWorkspace.H[2041] += + Gx1[5]*Gx2[1] + Gx1[13]*Gx2[9] + Gx1[21]*Gx2[17] + Gx1[29]*Gx2[25] + Gx1[37]*Gx2[33] + Gx1[45]*Gx2[41] + Gx1[53]*Gx2[49] + Gx1[61]*Gx2[57];
nmpcWorkspace.H[2042] += + Gx1[5]*Gx2[2] + Gx1[13]*Gx2[10] + Gx1[21]*Gx2[18] + Gx1[29]*Gx2[26] + Gx1[37]*Gx2[34] + Gx1[45]*Gx2[42] + Gx1[53]*Gx2[50] + Gx1[61]*Gx2[58];
nmpcWorkspace.H[2043] += + Gx1[5]*Gx2[3] + Gx1[13]*Gx2[11] + Gx1[21]*Gx2[19] + Gx1[29]*Gx2[27] + Gx1[37]*Gx2[35] + Gx1[45]*Gx2[43] + Gx1[53]*Gx2[51] + Gx1[61]*Gx2[59];
nmpcWorkspace.H[2044] += + Gx1[5]*Gx2[4] + Gx1[13]*Gx2[12] + Gx1[21]*Gx2[20] + Gx1[29]*Gx2[28] + Gx1[37]*Gx2[36] + Gx1[45]*Gx2[44] + Gx1[53]*Gx2[52] + Gx1[61]*Gx2[60];
nmpcWorkspace.H[2045] += + Gx1[5]*Gx2[5] + Gx1[13]*Gx2[13] + Gx1[21]*Gx2[21] + Gx1[29]*Gx2[29] + Gx1[37]*Gx2[37] + Gx1[45]*Gx2[45] + Gx1[53]*Gx2[53] + Gx1[61]*Gx2[61];
nmpcWorkspace.H[2046] += + Gx1[5]*Gx2[6] + Gx1[13]*Gx2[14] + Gx1[21]*Gx2[22] + Gx1[29]*Gx2[30] + Gx1[37]*Gx2[38] + Gx1[45]*Gx2[46] + Gx1[53]*Gx2[54] + Gx1[61]*Gx2[62];
nmpcWorkspace.H[2047] += + Gx1[5]*Gx2[7] + Gx1[13]*Gx2[15] + Gx1[21]*Gx2[23] + Gx1[29]*Gx2[31] + Gx1[37]*Gx2[39] + Gx1[45]*Gx2[47] + Gx1[53]*Gx2[55] + Gx1[61]*Gx2[63];
nmpcWorkspace.H[2448] += + Gx1[6]*Gx2[0] + Gx1[14]*Gx2[8] + Gx1[22]*Gx2[16] + Gx1[30]*Gx2[24] + Gx1[38]*Gx2[32] + Gx1[46]*Gx2[40] + Gx1[54]*Gx2[48] + Gx1[62]*Gx2[56];
nmpcWorkspace.H[2449] += + Gx1[6]*Gx2[1] + Gx1[14]*Gx2[9] + Gx1[22]*Gx2[17] + Gx1[30]*Gx2[25] + Gx1[38]*Gx2[33] + Gx1[46]*Gx2[41] + Gx1[54]*Gx2[49] + Gx1[62]*Gx2[57];
nmpcWorkspace.H[2450] += + Gx1[6]*Gx2[2] + Gx1[14]*Gx2[10] + Gx1[22]*Gx2[18] + Gx1[30]*Gx2[26] + Gx1[38]*Gx2[34] + Gx1[46]*Gx2[42] + Gx1[54]*Gx2[50] + Gx1[62]*Gx2[58];
nmpcWorkspace.H[2451] += + Gx1[6]*Gx2[3] + Gx1[14]*Gx2[11] + Gx1[22]*Gx2[19] + Gx1[30]*Gx2[27] + Gx1[38]*Gx2[35] + Gx1[46]*Gx2[43] + Gx1[54]*Gx2[51] + Gx1[62]*Gx2[59];
nmpcWorkspace.H[2452] += + Gx1[6]*Gx2[4] + Gx1[14]*Gx2[12] + Gx1[22]*Gx2[20] + Gx1[30]*Gx2[28] + Gx1[38]*Gx2[36] + Gx1[46]*Gx2[44] + Gx1[54]*Gx2[52] + Gx1[62]*Gx2[60];
nmpcWorkspace.H[2453] += + Gx1[6]*Gx2[5] + Gx1[14]*Gx2[13] + Gx1[22]*Gx2[21] + Gx1[30]*Gx2[29] + Gx1[38]*Gx2[37] + Gx1[46]*Gx2[45] + Gx1[54]*Gx2[53] + Gx1[62]*Gx2[61];
nmpcWorkspace.H[2454] += + Gx1[6]*Gx2[6] + Gx1[14]*Gx2[14] + Gx1[22]*Gx2[22] + Gx1[30]*Gx2[30] + Gx1[38]*Gx2[38] + Gx1[46]*Gx2[46] + Gx1[54]*Gx2[54] + Gx1[62]*Gx2[62];
nmpcWorkspace.H[2455] += + Gx1[6]*Gx2[7] + Gx1[14]*Gx2[15] + Gx1[22]*Gx2[23] + Gx1[30]*Gx2[31] + Gx1[38]*Gx2[39] + Gx1[46]*Gx2[47] + Gx1[54]*Gx2[55] + Gx1[62]*Gx2[63];
nmpcWorkspace.H[2856] += + Gx1[7]*Gx2[0] + Gx1[15]*Gx2[8] + Gx1[23]*Gx2[16] + Gx1[31]*Gx2[24] + Gx1[39]*Gx2[32] + Gx1[47]*Gx2[40] + Gx1[55]*Gx2[48] + Gx1[63]*Gx2[56];
nmpcWorkspace.H[2857] += + Gx1[7]*Gx2[1] + Gx1[15]*Gx2[9] + Gx1[23]*Gx2[17] + Gx1[31]*Gx2[25] + Gx1[39]*Gx2[33] + Gx1[47]*Gx2[41] + Gx1[55]*Gx2[49] + Gx1[63]*Gx2[57];
nmpcWorkspace.H[2858] += + Gx1[7]*Gx2[2] + Gx1[15]*Gx2[10] + Gx1[23]*Gx2[18] + Gx1[31]*Gx2[26] + Gx1[39]*Gx2[34] + Gx1[47]*Gx2[42] + Gx1[55]*Gx2[50] + Gx1[63]*Gx2[58];
nmpcWorkspace.H[2859] += + Gx1[7]*Gx2[3] + Gx1[15]*Gx2[11] + Gx1[23]*Gx2[19] + Gx1[31]*Gx2[27] + Gx1[39]*Gx2[35] + Gx1[47]*Gx2[43] + Gx1[55]*Gx2[51] + Gx1[63]*Gx2[59];
nmpcWorkspace.H[2860] += + Gx1[7]*Gx2[4] + Gx1[15]*Gx2[12] + Gx1[23]*Gx2[20] + Gx1[31]*Gx2[28] + Gx1[39]*Gx2[36] + Gx1[47]*Gx2[44] + Gx1[55]*Gx2[52] + Gx1[63]*Gx2[60];
nmpcWorkspace.H[2861] += + Gx1[7]*Gx2[5] + Gx1[15]*Gx2[13] + Gx1[23]*Gx2[21] + Gx1[31]*Gx2[29] + Gx1[39]*Gx2[37] + Gx1[47]*Gx2[45] + Gx1[55]*Gx2[53] + Gx1[63]*Gx2[61];
nmpcWorkspace.H[2862] += + Gx1[7]*Gx2[6] + Gx1[15]*Gx2[14] + Gx1[23]*Gx2[22] + Gx1[31]*Gx2[30] + Gx1[39]*Gx2[38] + Gx1[47]*Gx2[46] + Gx1[55]*Gx2[54] + Gx1[63]*Gx2[62];
nmpcWorkspace.H[2863] += + Gx1[7]*Gx2[7] + Gx1[15]*Gx2[15] + Gx1[23]*Gx2[23] + Gx1[31]*Gx2[31] + Gx1[39]*Gx2[39] + Gx1[47]*Gx2[47] + Gx1[55]*Gx2[55] + Gx1[63]*Gx2[63];
}

void nmpc_macCTSlx( real_t* const C0, real_t* const g0 )
{
g0[0] += 0.0;
;
g0[1] += 0.0;
;
g0[2] += 0.0;
;
g0[3] += 0.0;
;
g0[4] += 0.0;
;
g0[5] += 0.0;
;
g0[6] += 0.0;
;
g0[7] += 0.0;
;
}

void nmpc_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
g1[2] += 0.0;
;
g1[3] += 0.0;
;
}

void nmpc_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
nmpc_moveGuE( nmpcWorkspace.evGu, nmpcWorkspace.E );
for (lRun1 = 1; lRun1 < 100; ++lRun1)
{
nmpc_moveGxT( &(nmpcWorkspace.evGx[ lRun1 * 64 ]), nmpcWorkspace.T );
nmpc_multGxd( &(nmpcWorkspace.d[ lRun1 * 8-8 ]), &(nmpcWorkspace.evGx[ lRun1 * 64 ]), &(nmpcWorkspace.d[ lRun1 * 8 ]) );
nmpc_multGxGx( nmpcWorkspace.T, &(nmpcWorkspace.evGx[ lRun1 * 64-64 ]), &(nmpcWorkspace.evGx[ lRun1 * 64 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ lRun4 * 32 ]), &(nmpcWorkspace.E[ lRun3 * 32 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_moveGuE( &(nmpcWorkspace.evGu[ lRun1 * 32 ]), &(nmpcWorkspace.E[ lRun3 * 32 ]) );
}

nmpc_multGxGx( &(nmpcWorkspace.Q1[ 64 ]), nmpcWorkspace.evGx, nmpcWorkspace.QGx );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 128 ]), &(nmpcWorkspace.evGx[ 64 ]), &(nmpcWorkspace.QGx[ 64 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 192 ]), &(nmpcWorkspace.evGx[ 128 ]), &(nmpcWorkspace.QGx[ 128 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 256 ]), &(nmpcWorkspace.evGx[ 192 ]), &(nmpcWorkspace.QGx[ 192 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 320 ]), &(nmpcWorkspace.evGx[ 256 ]), &(nmpcWorkspace.QGx[ 256 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 384 ]), &(nmpcWorkspace.evGx[ 320 ]), &(nmpcWorkspace.QGx[ 320 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 448 ]), &(nmpcWorkspace.evGx[ 384 ]), &(nmpcWorkspace.QGx[ 384 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 512 ]), &(nmpcWorkspace.evGx[ 448 ]), &(nmpcWorkspace.QGx[ 448 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 576 ]), &(nmpcWorkspace.evGx[ 512 ]), &(nmpcWorkspace.QGx[ 512 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 640 ]), &(nmpcWorkspace.evGx[ 576 ]), &(nmpcWorkspace.QGx[ 576 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 704 ]), &(nmpcWorkspace.evGx[ 640 ]), &(nmpcWorkspace.QGx[ 640 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 768 ]), &(nmpcWorkspace.evGx[ 704 ]), &(nmpcWorkspace.QGx[ 704 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 832 ]), &(nmpcWorkspace.evGx[ 768 ]), &(nmpcWorkspace.QGx[ 768 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 896 ]), &(nmpcWorkspace.evGx[ 832 ]), &(nmpcWorkspace.QGx[ 832 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 960 ]), &(nmpcWorkspace.evGx[ 896 ]), &(nmpcWorkspace.QGx[ 896 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1024 ]), &(nmpcWorkspace.evGx[ 960 ]), &(nmpcWorkspace.QGx[ 960 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1088 ]), &(nmpcWorkspace.evGx[ 1024 ]), &(nmpcWorkspace.QGx[ 1024 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1152 ]), &(nmpcWorkspace.evGx[ 1088 ]), &(nmpcWorkspace.QGx[ 1088 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1216 ]), &(nmpcWorkspace.evGx[ 1152 ]), &(nmpcWorkspace.QGx[ 1152 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1280 ]), &(nmpcWorkspace.evGx[ 1216 ]), &(nmpcWorkspace.QGx[ 1216 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1344 ]), &(nmpcWorkspace.evGx[ 1280 ]), &(nmpcWorkspace.QGx[ 1280 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1408 ]), &(nmpcWorkspace.evGx[ 1344 ]), &(nmpcWorkspace.QGx[ 1344 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1472 ]), &(nmpcWorkspace.evGx[ 1408 ]), &(nmpcWorkspace.QGx[ 1408 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1536 ]), &(nmpcWorkspace.evGx[ 1472 ]), &(nmpcWorkspace.QGx[ 1472 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1600 ]), &(nmpcWorkspace.evGx[ 1536 ]), &(nmpcWorkspace.QGx[ 1536 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1664 ]), &(nmpcWorkspace.evGx[ 1600 ]), &(nmpcWorkspace.QGx[ 1600 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1728 ]), &(nmpcWorkspace.evGx[ 1664 ]), &(nmpcWorkspace.QGx[ 1664 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1792 ]), &(nmpcWorkspace.evGx[ 1728 ]), &(nmpcWorkspace.QGx[ 1728 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1856 ]), &(nmpcWorkspace.evGx[ 1792 ]), &(nmpcWorkspace.QGx[ 1792 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1920 ]), &(nmpcWorkspace.evGx[ 1856 ]), &(nmpcWorkspace.QGx[ 1856 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1984 ]), &(nmpcWorkspace.evGx[ 1920 ]), &(nmpcWorkspace.QGx[ 1920 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2048 ]), &(nmpcWorkspace.evGx[ 1984 ]), &(nmpcWorkspace.QGx[ 1984 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2112 ]), &(nmpcWorkspace.evGx[ 2048 ]), &(nmpcWorkspace.QGx[ 2048 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2176 ]), &(nmpcWorkspace.evGx[ 2112 ]), &(nmpcWorkspace.QGx[ 2112 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2240 ]), &(nmpcWorkspace.evGx[ 2176 ]), &(nmpcWorkspace.QGx[ 2176 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2304 ]), &(nmpcWorkspace.evGx[ 2240 ]), &(nmpcWorkspace.QGx[ 2240 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2368 ]), &(nmpcWorkspace.evGx[ 2304 ]), &(nmpcWorkspace.QGx[ 2304 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2432 ]), &(nmpcWorkspace.evGx[ 2368 ]), &(nmpcWorkspace.QGx[ 2368 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2496 ]), &(nmpcWorkspace.evGx[ 2432 ]), &(nmpcWorkspace.QGx[ 2432 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2560 ]), &(nmpcWorkspace.evGx[ 2496 ]), &(nmpcWorkspace.QGx[ 2496 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2624 ]), &(nmpcWorkspace.evGx[ 2560 ]), &(nmpcWorkspace.QGx[ 2560 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2688 ]), &(nmpcWorkspace.evGx[ 2624 ]), &(nmpcWorkspace.QGx[ 2624 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2752 ]), &(nmpcWorkspace.evGx[ 2688 ]), &(nmpcWorkspace.QGx[ 2688 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2816 ]), &(nmpcWorkspace.evGx[ 2752 ]), &(nmpcWorkspace.QGx[ 2752 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2880 ]), &(nmpcWorkspace.evGx[ 2816 ]), &(nmpcWorkspace.QGx[ 2816 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 2944 ]), &(nmpcWorkspace.evGx[ 2880 ]), &(nmpcWorkspace.QGx[ 2880 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3008 ]), &(nmpcWorkspace.evGx[ 2944 ]), &(nmpcWorkspace.QGx[ 2944 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3072 ]), &(nmpcWorkspace.evGx[ 3008 ]), &(nmpcWorkspace.QGx[ 3008 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3136 ]), &(nmpcWorkspace.evGx[ 3072 ]), &(nmpcWorkspace.QGx[ 3072 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3200 ]), &(nmpcWorkspace.evGx[ 3136 ]), &(nmpcWorkspace.QGx[ 3136 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3264 ]), &(nmpcWorkspace.evGx[ 3200 ]), &(nmpcWorkspace.QGx[ 3200 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3328 ]), &(nmpcWorkspace.evGx[ 3264 ]), &(nmpcWorkspace.QGx[ 3264 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3392 ]), &(nmpcWorkspace.evGx[ 3328 ]), &(nmpcWorkspace.QGx[ 3328 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3456 ]), &(nmpcWorkspace.evGx[ 3392 ]), &(nmpcWorkspace.QGx[ 3392 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3520 ]), &(nmpcWorkspace.evGx[ 3456 ]), &(nmpcWorkspace.QGx[ 3456 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3584 ]), &(nmpcWorkspace.evGx[ 3520 ]), &(nmpcWorkspace.QGx[ 3520 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3648 ]), &(nmpcWorkspace.evGx[ 3584 ]), &(nmpcWorkspace.QGx[ 3584 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3712 ]), &(nmpcWorkspace.evGx[ 3648 ]), &(nmpcWorkspace.QGx[ 3648 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3776 ]), &(nmpcWorkspace.evGx[ 3712 ]), &(nmpcWorkspace.QGx[ 3712 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3840 ]), &(nmpcWorkspace.evGx[ 3776 ]), &(nmpcWorkspace.QGx[ 3776 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3904 ]), &(nmpcWorkspace.evGx[ 3840 ]), &(nmpcWorkspace.QGx[ 3840 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 3968 ]), &(nmpcWorkspace.evGx[ 3904 ]), &(nmpcWorkspace.QGx[ 3904 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4032 ]), &(nmpcWorkspace.evGx[ 3968 ]), &(nmpcWorkspace.QGx[ 3968 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4096 ]), &(nmpcWorkspace.evGx[ 4032 ]), &(nmpcWorkspace.QGx[ 4032 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4160 ]), &(nmpcWorkspace.evGx[ 4096 ]), &(nmpcWorkspace.QGx[ 4096 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4224 ]), &(nmpcWorkspace.evGx[ 4160 ]), &(nmpcWorkspace.QGx[ 4160 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4288 ]), &(nmpcWorkspace.evGx[ 4224 ]), &(nmpcWorkspace.QGx[ 4224 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4352 ]), &(nmpcWorkspace.evGx[ 4288 ]), &(nmpcWorkspace.QGx[ 4288 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4416 ]), &(nmpcWorkspace.evGx[ 4352 ]), &(nmpcWorkspace.QGx[ 4352 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4480 ]), &(nmpcWorkspace.evGx[ 4416 ]), &(nmpcWorkspace.QGx[ 4416 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4544 ]), &(nmpcWorkspace.evGx[ 4480 ]), &(nmpcWorkspace.QGx[ 4480 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4608 ]), &(nmpcWorkspace.evGx[ 4544 ]), &(nmpcWorkspace.QGx[ 4544 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4672 ]), &(nmpcWorkspace.evGx[ 4608 ]), &(nmpcWorkspace.QGx[ 4608 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4736 ]), &(nmpcWorkspace.evGx[ 4672 ]), &(nmpcWorkspace.QGx[ 4672 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4800 ]), &(nmpcWorkspace.evGx[ 4736 ]), &(nmpcWorkspace.QGx[ 4736 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4864 ]), &(nmpcWorkspace.evGx[ 4800 ]), &(nmpcWorkspace.QGx[ 4800 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4928 ]), &(nmpcWorkspace.evGx[ 4864 ]), &(nmpcWorkspace.QGx[ 4864 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 4992 ]), &(nmpcWorkspace.evGx[ 4928 ]), &(nmpcWorkspace.QGx[ 4928 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5056 ]), &(nmpcWorkspace.evGx[ 4992 ]), &(nmpcWorkspace.QGx[ 4992 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5120 ]), &(nmpcWorkspace.evGx[ 5056 ]), &(nmpcWorkspace.QGx[ 5056 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5184 ]), &(nmpcWorkspace.evGx[ 5120 ]), &(nmpcWorkspace.QGx[ 5120 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5248 ]), &(nmpcWorkspace.evGx[ 5184 ]), &(nmpcWorkspace.QGx[ 5184 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5312 ]), &(nmpcWorkspace.evGx[ 5248 ]), &(nmpcWorkspace.QGx[ 5248 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5376 ]), &(nmpcWorkspace.evGx[ 5312 ]), &(nmpcWorkspace.QGx[ 5312 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5440 ]), &(nmpcWorkspace.evGx[ 5376 ]), &(nmpcWorkspace.QGx[ 5376 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5504 ]), &(nmpcWorkspace.evGx[ 5440 ]), &(nmpcWorkspace.QGx[ 5440 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5568 ]), &(nmpcWorkspace.evGx[ 5504 ]), &(nmpcWorkspace.QGx[ 5504 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5632 ]), &(nmpcWorkspace.evGx[ 5568 ]), &(nmpcWorkspace.QGx[ 5568 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5696 ]), &(nmpcWorkspace.evGx[ 5632 ]), &(nmpcWorkspace.QGx[ 5632 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5760 ]), &(nmpcWorkspace.evGx[ 5696 ]), &(nmpcWorkspace.QGx[ 5696 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5824 ]), &(nmpcWorkspace.evGx[ 5760 ]), &(nmpcWorkspace.QGx[ 5760 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5888 ]), &(nmpcWorkspace.evGx[ 5824 ]), &(nmpcWorkspace.QGx[ 5824 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 5952 ]), &(nmpcWorkspace.evGx[ 5888 ]), &(nmpcWorkspace.QGx[ 5888 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 6016 ]), &(nmpcWorkspace.evGx[ 5952 ]), &(nmpcWorkspace.QGx[ 5952 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 6080 ]), &(nmpcWorkspace.evGx[ 6016 ]), &(nmpcWorkspace.QGx[ 6016 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 6144 ]), &(nmpcWorkspace.evGx[ 6080 ]), &(nmpcWorkspace.QGx[ 6080 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 6208 ]), &(nmpcWorkspace.evGx[ 6144 ]), &(nmpcWorkspace.QGx[ 6144 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 6272 ]), &(nmpcWorkspace.evGx[ 6208 ]), &(nmpcWorkspace.QGx[ 6208 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 6336 ]), &(nmpcWorkspace.evGx[ 6272 ]), &(nmpcWorkspace.QGx[ 6272 ]) );
nmpc_multGxGx( nmpcWorkspace.QN1, &(nmpcWorkspace.evGx[ 6336 ]), &(nmpcWorkspace.QGx[ 6336 ]) );

for (lRun1 = 0; lRun1 < 99; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( &(nmpcWorkspace.Q1[ lRun1 * 64 + 64 ]), &(nmpcWorkspace.E[ lRun3 * 32 ]), &(nmpcWorkspace.QE[ lRun3 * 32 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( nmpcWorkspace.QN1, &(nmpcWorkspace.E[ lRun3 * 32 ]), &(nmpcWorkspace.QE[ lRun3 * 32 ]) );
}

nmpc_zeroBlockH00(  );
nmpc_multCTQC( nmpcWorkspace.evGx, nmpcWorkspace.QGx );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 64 ]), &(nmpcWorkspace.QGx[ 64 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 128 ]), &(nmpcWorkspace.QGx[ 128 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 192 ]), &(nmpcWorkspace.QGx[ 192 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 256 ]), &(nmpcWorkspace.QGx[ 256 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 320 ]), &(nmpcWorkspace.QGx[ 320 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 384 ]), &(nmpcWorkspace.QGx[ 384 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 448 ]), &(nmpcWorkspace.QGx[ 448 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 512 ]), &(nmpcWorkspace.QGx[ 512 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 576 ]), &(nmpcWorkspace.QGx[ 576 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 640 ]), &(nmpcWorkspace.QGx[ 640 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 704 ]), &(nmpcWorkspace.QGx[ 704 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 768 ]), &(nmpcWorkspace.QGx[ 768 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 832 ]), &(nmpcWorkspace.QGx[ 832 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 896 ]), &(nmpcWorkspace.QGx[ 896 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 960 ]), &(nmpcWorkspace.QGx[ 960 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1024 ]), &(nmpcWorkspace.QGx[ 1024 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1088 ]), &(nmpcWorkspace.QGx[ 1088 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1152 ]), &(nmpcWorkspace.QGx[ 1152 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1216 ]), &(nmpcWorkspace.QGx[ 1216 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1280 ]), &(nmpcWorkspace.QGx[ 1280 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1344 ]), &(nmpcWorkspace.QGx[ 1344 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1408 ]), &(nmpcWorkspace.QGx[ 1408 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1472 ]), &(nmpcWorkspace.QGx[ 1472 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1536 ]), &(nmpcWorkspace.QGx[ 1536 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1600 ]), &(nmpcWorkspace.QGx[ 1600 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1664 ]), &(nmpcWorkspace.QGx[ 1664 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1728 ]), &(nmpcWorkspace.QGx[ 1728 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1792 ]), &(nmpcWorkspace.QGx[ 1792 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1856 ]), &(nmpcWorkspace.QGx[ 1856 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1920 ]), &(nmpcWorkspace.QGx[ 1920 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1984 ]), &(nmpcWorkspace.QGx[ 1984 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2048 ]), &(nmpcWorkspace.QGx[ 2048 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2112 ]), &(nmpcWorkspace.QGx[ 2112 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2176 ]), &(nmpcWorkspace.QGx[ 2176 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2240 ]), &(nmpcWorkspace.QGx[ 2240 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2304 ]), &(nmpcWorkspace.QGx[ 2304 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2368 ]), &(nmpcWorkspace.QGx[ 2368 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2432 ]), &(nmpcWorkspace.QGx[ 2432 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2496 ]), &(nmpcWorkspace.QGx[ 2496 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2560 ]), &(nmpcWorkspace.QGx[ 2560 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2624 ]), &(nmpcWorkspace.QGx[ 2624 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2688 ]), &(nmpcWorkspace.QGx[ 2688 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2752 ]), &(nmpcWorkspace.QGx[ 2752 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2816 ]), &(nmpcWorkspace.QGx[ 2816 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2880 ]), &(nmpcWorkspace.QGx[ 2880 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 2944 ]), &(nmpcWorkspace.QGx[ 2944 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3008 ]), &(nmpcWorkspace.QGx[ 3008 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3072 ]), &(nmpcWorkspace.QGx[ 3072 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3136 ]), &(nmpcWorkspace.QGx[ 3136 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3200 ]), &(nmpcWorkspace.QGx[ 3200 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3264 ]), &(nmpcWorkspace.QGx[ 3264 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3328 ]), &(nmpcWorkspace.QGx[ 3328 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3392 ]), &(nmpcWorkspace.QGx[ 3392 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3456 ]), &(nmpcWorkspace.QGx[ 3456 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3520 ]), &(nmpcWorkspace.QGx[ 3520 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3584 ]), &(nmpcWorkspace.QGx[ 3584 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3648 ]), &(nmpcWorkspace.QGx[ 3648 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3712 ]), &(nmpcWorkspace.QGx[ 3712 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3776 ]), &(nmpcWorkspace.QGx[ 3776 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3840 ]), &(nmpcWorkspace.QGx[ 3840 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3904 ]), &(nmpcWorkspace.QGx[ 3904 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 3968 ]), &(nmpcWorkspace.QGx[ 3968 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4032 ]), &(nmpcWorkspace.QGx[ 4032 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4096 ]), &(nmpcWorkspace.QGx[ 4096 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4160 ]), &(nmpcWorkspace.QGx[ 4160 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4224 ]), &(nmpcWorkspace.QGx[ 4224 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4288 ]), &(nmpcWorkspace.QGx[ 4288 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4352 ]), &(nmpcWorkspace.QGx[ 4352 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4416 ]), &(nmpcWorkspace.QGx[ 4416 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4480 ]), &(nmpcWorkspace.QGx[ 4480 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4544 ]), &(nmpcWorkspace.QGx[ 4544 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4608 ]), &(nmpcWorkspace.QGx[ 4608 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4672 ]), &(nmpcWorkspace.QGx[ 4672 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4736 ]), &(nmpcWorkspace.QGx[ 4736 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4800 ]), &(nmpcWorkspace.QGx[ 4800 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4864 ]), &(nmpcWorkspace.QGx[ 4864 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4928 ]), &(nmpcWorkspace.QGx[ 4928 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 4992 ]), &(nmpcWorkspace.QGx[ 4992 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5056 ]), &(nmpcWorkspace.QGx[ 5056 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5120 ]), &(nmpcWorkspace.QGx[ 5120 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5184 ]), &(nmpcWorkspace.QGx[ 5184 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5248 ]), &(nmpcWorkspace.QGx[ 5248 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5312 ]), &(nmpcWorkspace.QGx[ 5312 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5376 ]), &(nmpcWorkspace.QGx[ 5376 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5440 ]), &(nmpcWorkspace.QGx[ 5440 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5504 ]), &(nmpcWorkspace.QGx[ 5504 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5568 ]), &(nmpcWorkspace.QGx[ 5568 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5632 ]), &(nmpcWorkspace.QGx[ 5632 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5696 ]), &(nmpcWorkspace.QGx[ 5696 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5760 ]), &(nmpcWorkspace.QGx[ 5760 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5824 ]), &(nmpcWorkspace.QGx[ 5824 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5888 ]), &(nmpcWorkspace.QGx[ 5888 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 5952 ]), &(nmpcWorkspace.QGx[ 5952 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 6016 ]), &(nmpcWorkspace.QGx[ 6016 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 6080 ]), &(nmpcWorkspace.QGx[ 6080 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 6144 ]), &(nmpcWorkspace.QGx[ 6144 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 6208 ]), &(nmpcWorkspace.QGx[ 6208 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 6272 ]), &(nmpcWorkspace.QGx[ 6272 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 6336 ]), &(nmpcWorkspace.QGx[ 6336 ]) );

for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
nmpc_zeroBlockH10( &(nmpcWorkspace.H10[ lRun1 * 32 ]) );
for (lRun2 = lRun1; lRun2 < 100; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_multQETGx( &(nmpcWorkspace.QE[ lRun3 * 32 ]), &(nmpcWorkspace.evGx[ lRun2 * 64 ]), &(nmpcWorkspace.H10[ lRun1 * 32 ]) );
}
}

for (lRun1 = 0;lRun1 < 8; ++lRun1)
for (lRun2 = 0;lRun2 < 400; ++lRun2)
nmpcWorkspace.H[(lRun1 * 408) + (lRun2 + 8)] = nmpcWorkspace.H10[(lRun2 * 8) + (lRun1)];

for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
nmpc_setBlockH11_R1( lRun1, lRun1, &(nmpcWorkspace.R1[ lRun1 * 16 ]) );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 100; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmpc_setBlockH11( lRun1, lRun2, &(nmpcWorkspace.E[ lRun4 * 32 ]), &(nmpcWorkspace.QE[ lRun5 * 32 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 100; ++lRun2)
{
nmpc_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 100; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmpc_setBlockH11( lRun1, lRun2, &(nmpcWorkspace.E[ lRun4 * 32 ]), &(nmpcWorkspace.QE[ lRun5 * 32 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
nmpc_copyHTH( lRun1, lRun2 );
}
}

for (lRun1 = 0;lRun1 < 400; ++lRun1)
for (lRun2 = 0;lRun2 < 8; ++lRun2)
nmpcWorkspace.H[(lRun1 * 408 + 3264) + (lRun2)] = nmpcWorkspace.H10[(lRun1 * 8) + (lRun2)];

nmpc_multQ1d( &(nmpcWorkspace.Q1[ 64 ]), nmpcWorkspace.d, nmpcWorkspace.Qd );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 128 ]), &(nmpcWorkspace.d[ 8 ]), &(nmpcWorkspace.Qd[ 8 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 192 ]), &(nmpcWorkspace.d[ 16 ]), &(nmpcWorkspace.Qd[ 16 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 256 ]), &(nmpcWorkspace.d[ 24 ]), &(nmpcWorkspace.Qd[ 24 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 320 ]), &(nmpcWorkspace.d[ 32 ]), &(nmpcWorkspace.Qd[ 32 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 384 ]), &(nmpcWorkspace.d[ 40 ]), &(nmpcWorkspace.Qd[ 40 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 448 ]), &(nmpcWorkspace.d[ 48 ]), &(nmpcWorkspace.Qd[ 48 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 512 ]), &(nmpcWorkspace.d[ 56 ]), &(nmpcWorkspace.Qd[ 56 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 576 ]), &(nmpcWorkspace.d[ 64 ]), &(nmpcWorkspace.Qd[ 64 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 640 ]), &(nmpcWorkspace.d[ 72 ]), &(nmpcWorkspace.Qd[ 72 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 704 ]), &(nmpcWorkspace.d[ 80 ]), &(nmpcWorkspace.Qd[ 80 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 768 ]), &(nmpcWorkspace.d[ 88 ]), &(nmpcWorkspace.Qd[ 88 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 832 ]), &(nmpcWorkspace.d[ 96 ]), &(nmpcWorkspace.Qd[ 96 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 896 ]), &(nmpcWorkspace.d[ 104 ]), &(nmpcWorkspace.Qd[ 104 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 960 ]), &(nmpcWorkspace.d[ 112 ]), &(nmpcWorkspace.Qd[ 112 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1024 ]), &(nmpcWorkspace.d[ 120 ]), &(nmpcWorkspace.Qd[ 120 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1088 ]), &(nmpcWorkspace.d[ 128 ]), &(nmpcWorkspace.Qd[ 128 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1152 ]), &(nmpcWorkspace.d[ 136 ]), &(nmpcWorkspace.Qd[ 136 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1216 ]), &(nmpcWorkspace.d[ 144 ]), &(nmpcWorkspace.Qd[ 144 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1280 ]), &(nmpcWorkspace.d[ 152 ]), &(nmpcWorkspace.Qd[ 152 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1344 ]), &(nmpcWorkspace.d[ 160 ]), &(nmpcWorkspace.Qd[ 160 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1408 ]), &(nmpcWorkspace.d[ 168 ]), &(nmpcWorkspace.Qd[ 168 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1472 ]), &(nmpcWorkspace.d[ 176 ]), &(nmpcWorkspace.Qd[ 176 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1536 ]), &(nmpcWorkspace.d[ 184 ]), &(nmpcWorkspace.Qd[ 184 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1600 ]), &(nmpcWorkspace.d[ 192 ]), &(nmpcWorkspace.Qd[ 192 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1664 ]), &(nmpcWorkspace.d[ 200 ]), &(nmpcWorkspace.Qd[ 200 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1728 ]), &(nmpcWorkspace.d[ 208 ]), &(nmpcWorkspace.Qd[ 208 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1792 ]), &(nmpcWorkspace.d[ 216 ]), &(nmpcWorkspace.Qd[ 216 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1856 ]), &(nmpcWorkspace.d[ 224 ]), &(nmpcWorkspace.Qd[ 224 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1920 ]), &(nmpcWorkspace.d[ 232 ]), &(nmpcWorkspace.Qd[ 232 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1984 ]), &(nmpcWorkspace.d[ 240 ]), &(nmpcWorkspace.Qd[ 240 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2048 ]), &(nmpcWorkspace.d[ 248 ]), &(nmpcWorkspace.Qd[ 248 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2112 ]), &(nmpcWorkspace.d[ 256 ]), &(nmpcWorkspace.Qd[ 256 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2176 ]), &(nmpcWorkspace.d[ 264 ]), &(nmpcWorkspace.Qd[ 264 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2240 ]), &(nmpcWorkspace.d[ 272 ]), &(nmpcWorkspace.Qd[ 272 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2304 ]), &(nmpcWorkspace.d[ 280 ]), &(nmpcWorkspace.Qd[ 280 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2368 ]), &(nmpcWorkspace.d[ 288 ]), &(nmpcWorkspace.Qd[ 288 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2432 ]), &(nmpcWorkspace.d[ 296 ]), &(nmpcWorkspace.Qd[ 296 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2496 ]), &(nmpcWorkspace.d[ 304 ]), &(nmpcWorkspace.Qd[ 304 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2560 ]), &(nmpcWorkspace.d[ 312 ]), &(nmpcWorkspace.Qd[ 312 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2624 ]), &(nmpcWorkspace.d[ 320 ]), &(nmpcWorkspace.Qd[ 320 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2688 ]), &(nmpcWorkspace.d[ 328 ]), &(nmpcWorkspace.Qd[ 328 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2752 ]), &(nmpcWorkspace.d[ 336 ]), &(nmpcWorkspace.Qd[ 336 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2816 ]), &(nmpcWorkspace.d[ 344 ]), &(nmpcWorkspace.Qd[ 344 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2880 ]), &(nmpcWorkspace.d[ 352 ]), &(nmpcWorkspace.Qd[ 352 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 2944 ]), &(nmpcWorkspace.d[ 360 ]), &(nmpcWorkspace.Qd[ 360 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3008 ]), &(nmpcWorkspace.d[ 368 ]), &(nmpcWorkspace.Qd[ 368 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3072 ]), &(nmpcWorkspace.d[ 376 ]), &(nmpcWorkspace.Qd[ 376 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3136 ]), &(nmpcWorkspace.d[ 384 ]), &(nmpcWorkspace.Qd[ 384 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3200 ]), &(nmpcWorkspace.d[ 392 ]), &(nmpcWorkspace.Qd[ 392 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3264 ]), &(nmpcWorkspace.d[ 400 ]), &(nmpcWorkspace.Qd[ 400 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3328 ]), &(nmpcWorkspace.d[ 408 ]), &(nmpcWorkspace.Qd[ 408 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3392 ]), &(nmpcWorkspace.d[ 416 ]), &(nmpcWorkspace.Qd[ 416 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3456 ]), &(nmpcWorkspace.d[ 424 ]), &(nmpcWorkspace.Qd[ 424 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3520 ]), &(nmpcWorkspace.d[ 432 ]), &(nmpcWorkspace.Qd[ 432 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3584 ]), &(nmpcWorkspace.d[ 440 ]), &(nmpcWorkspace.Qd[ 440 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3648 ]), &(nmpcWorkspace.d[ 448 ]), &(nmpcWorkspace.Qd[ 448 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3712 ]), &(nmpcWorkspace.d[ 456 ]), &(nmpcWorkspace.Qd[ 456 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3776 ]), &(nmpcWorkspace.d[ 464 ]), &(nmpcWorkspace.Qd[ 464 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3840 ]), &(nmpcWorkspace.d[ 472 ]), &(nmpcWorkspace.Qd[ 472 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3904 ]), &(nmpcWorkspace.d[ 480 ]), &(nmpcWorkspace.Qd[ 480 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 3968 ]), &(nmpcWorkspace.d[ 488 ]), &(nmpcWorkspace.Qd[ 488 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4032 ]), &(nmpcWorkspace.d[ 496 ]), &(nmpcWorkspace.Qd[ 496 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4096 ]), &(nmpcWorkspace.d[ 504 ]), &(nmpcWorkspace.Qd[ 504 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4160 ]), &(nmpcWorkspace.d[ 512 ]), &(nmpcWorkspace.Qd[ 512 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4224 ]), &(nmpcWorkspace.d[ 520 ]), &(nmpcWorkspace.Qd[ 520 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4288 ]), &(nmpcWorkspace.d[ 528 ]), &(nmpcWorkspace.Qd[ 528 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4352 ]), &(nmpcWorkspace.d[ 536 ]), &(nmpcWorkspace.Qd[ 536 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4416 ]), &(nmpcWorkspace.d[ 544 ]), &(nmpcWorkspace.Qd[ 544 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4480 ]), &(nmpcWorkspace.d[ 552 ]), &(nmpcWorkspace.Qd[ 552 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4544 ]), &(nmpcWorkspace.d[ 560 ]), &(nmpcWorkspace.Qd[ 560 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4608 ]), &(nmpcWorkspace.d[ 568 ]), &(nmpcWorkspace.Qd[ 568 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4672 ]), &(nmpcWorkspace.d[ 576 ]), &(nmpcWorkspace.Qd[ 576 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4736 ]), &(nmpcWorkspace.d[ 584 ]), &(nmpcWorkspace.Qd[ 584 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4800 ]), &(nmpcWorkspace.d[ 592 ]), &(nmpcWorkspace.Qd[ 592 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4864 ]), &(nmpcWorkspace.d[ 600 ]), &(nmpcWorkspace.Qd[ 600 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4928 ]), &(nmpcWorkspace.d[ 608 ]), &(nmpcWorkspace.Qd[ 608 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 4992 ]), &(nmpcWorkspace.d[ 616 ]), &(nmpcWorkspace.Qd[ 616 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5056 ]), &(nmpcWorkspace.d[ 624 ]), &(nmpcWorkspace.Qd[ 624 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5120 ]), &(nmpcWorkspace.d[ 632 ]), &(nmpcWorkspace.Qd[ 632 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5184 ]), &(nmpcWorkspace.d[ 640 ]), &(nmpcWorkspace.Qd[ 640 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5248 ]), &(nmpcWorkspace.d[ 648 ]), &(nmpcWorkspace.Qd[ 648 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5312 ]), &(nmpcWorkspace.d[ 656 ]), &(nmpcWorkspace.Qd[ 656 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5376 ]), &(nmpcWorkspace.d[ 664 ]), &(nmpcWorkspace.Qd[ 664 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5440 ]), &(nmpcWorkspace.d[ 672 ]), &(nmpcWorkspace.Qd[ 672 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5504 ]), &(nmpcWorkspace.d[ 680 ]), &(nmpcWorkspace.Qd[ 680 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5568 ]), &(nmpcWorkspace.d[ 688 ]), &(nmpcWorkspace.Qd[ 688 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5632 ]), &(nmpcWorkspace.d[ 696 ]), &(nmpcWorkspace.Qd[ 696 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5696 ]), &(nmpcWorkspace.d[ 704 ]), &(nmpcWorkspace.Qd[ 704 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5760 ]), &(nmpcWorkspace.d[ 712 ]), &(nmpcWorkspace.Qd[ 712 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5824 ]), &(nmpcWorkspace.d[ 720 ]), &(nmpcWorkspace.Qd[ 720 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5888 ]), &(nmpcWorkspace.d[ 728 ]), &(nmpcWorkspace.Qd[ 728 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 5952 ]), &(nmpcWorkspace.d[ 736 ]), &(nmpcWorkspace.Qd[ 736 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 6016 ]), &(nmpcWorkspace.d[ 744 ]), &(nmpcWorkspace.Qd[ 744 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 6080 ]), &(nmpcWorkspace.d[ 752 ]), &(nmpcWorkspace.Qd[ 752 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 6144 ]), &(nmpcWorkspace.d[ 760 ]), &(nmpcWorkspace.Qd[ 760 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 6208 ]), &(nmpcWorkspace.d[ 768 ]), &(nmpcWorkspace.Qd[ 768 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 6272 ]), &(nmpcWorkspace.d[ 776 ]), &(nmpcWorkspace.Qd[ 776 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 6336 ]), &(nmpcWorkspace.d[ 784 ]), &(nmpcWorkspace.Qd[ 784 ]) );
nmpc_multQN1d( nmpcWorkspace.QN1, &(nmpcWorkspace.d[ 792 ]), &(nmpcWorkspace.Qd[ 792 ]) );

nmpc_macCTSlx( nmpcWorkspace.evGx, nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 64 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 128 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 192 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 256 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 320 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 384 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 448 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 512 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 576 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 640 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 704 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 768 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 832 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 896 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 960 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1024 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1088 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1152 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1216 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1280 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1344 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1408 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1472 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1536 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1600 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1664 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1728 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1792 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1856 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1920 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1984 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2048 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2112 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2176 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2240 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2304 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2368 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2432 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2496 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2560 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2624 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2688 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2752 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2816 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2880 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 2944 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3008 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3072 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3136 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3200 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3264 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3328 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3392 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3456 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3520 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3584 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3648 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3712 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3776 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3840 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3904 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 3968 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4032 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4096 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4160 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4224 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4288 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4352 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4416 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4480 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4544 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4608 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4672 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4736 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4800 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4864 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4928 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 4992 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5056 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5120 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5184 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5248 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5312 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5376 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5440 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5504 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5568 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5632 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5696 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5760 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5824 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5888 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 5952 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 6016 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 6080 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 6144 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 6208 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 6272 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 6336 ]), nmpcWorkspace.g );
for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 100; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_macETSlu( &(nmpcWorkspace.QE[ lRun3 * 32 ]), &(nmpcWorkspace.g[ lRun1 * 4 + 8 ]) );
}
}
nmpcWorkspace.lb[8] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[0];
nmpcWorkspace.lb[9] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[1];
nmpcWorkspace.lb[10] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[2];
nmpcWorkspace.lb[11] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[3];
nmpcWorkspace.lb[12] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[4];
nmpcWorkspace.lb[13] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[5];
nmpcWorkspace.lb[14] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[6];
nmpcWorkspace.lb[15] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[7];
nmpcWorkspace.lb[16] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[8];
nmpcWorkspace.lb[17] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[9];
nmpcWorkspace.lb[18] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[10];
nmpcWorkspace.lb[19] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[11];
nmpcWorkspace.lb[20] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[12];
nmpcWorkspace.lb[21] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[13];
nmpcWorkspace.lb[22] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[14];
nmpcWorkspace.lb[23] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[15];
nmpcWorkspace.lb[24] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[16];
nmpcWorkspace.lb[25] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[17];
nmpcWorkspace.lb[26] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[18];
nmpcWorkspace.lb[27] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[19];
nmpcWorkspace.lb[28] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[20];
nmpcWorkspace.lb[29] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[21];
nmpcWorkspace.lb[30] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[22];
nmpcWorkspace.lb[31] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[23];
nmpcWorkspace.lb[32] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[24];
nmpcWorkspace.lb[33] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[25];
nmpcWorkspace.lb[34] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[26];
nmpcWorkspace.lb[35] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[27];
nmpcWorkspace.lb[36] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[28];
nmpcWorkspace.lb[37] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[29];
nmpcWorkspace.lb[38] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[30];
nmpcWorkspace.lb[39] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[31];
nmpcWorkspace.lb[40] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[32];
nmpcWorkspace.lb[41] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[33];
nmpcWorkspace.lb[42] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[34];
nmpcWorkspace.lb[43] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[35];
nmpcWorkspace.lb[44] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[36];
nmpcWorkspace.lb[45] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[37];
nmpcWorkspace.lb[46] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[38];
nmpcWorkspace.lb[47] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[39];
nmpcWorkspace.lb[48] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[40];
nmpcWorkspace.lb[49] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[41];
nmpcWorkspace.lb[50] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[42];
nmpcWorkspace.lb[51] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[43];
nmpcWorkspace.lb[52] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[44];
nmpcWorkspace.lb[53] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[45];
nmpcWorkspace.lb[54] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[46];
nmpcWorkspace.lb[55] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[47];
nmpcWorkspace.lb[56] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[48];
nmpcWorkspace.lb[57] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[49];
nmpcWorkspace.lb[58] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[50];
nmpcWorkspace.lb[59] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[51];
nmpcWorkspace.lb[60] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[52];
nmpcWorkspace.lb[61] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[53];
nmpcWorkspace.lb[62] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[54];
nmpcWorkspace.lb[63] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[55];
nmpcWorkspace.lb[64] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[56];
nmpcWorkspace.lb[65] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[57];
nmpcWorkspace.lb[66] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[58];
nmpcWorkspace.lb[67] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[59];
nmpcWorkspace.lb[68] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[60];
nmpcWorkspace.lb[69] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[61];
nmpcWorkspace.lb[70] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[62];
nmpcWorkspace.lb[71] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[63];
nmpcWorkspace.lb[72] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[64];
nmpcWorkspace.lb[73] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[65];
nmpcWorkspace.lb[74] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[66];
nmpcWorkspace.lb[75] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[67];
nmpcWorkspace.lb[76] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[68];
nmpcWorkspace.lb[77] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[69];
nmpcWorkspace.lb[78] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[70];
nmpcWorkspace.lb[79] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[71];
nmpcWorkspace.lb[80] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[72];
nmpcWorkspace.lb[81] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[73];
nmpcWorkspace.lb[82] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[74];
nmpcWorkspace.lb[83] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[75];
nmpcWorkspace.lb[84] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[76];
nmpcWorkspace.lb[85] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[77];
nmpcWorkspace.lb[86] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[78];
nmpcWorkspace.lb[87] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[79];
nmpcWorkspace.lb[88] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[80];
nmpcWorkspace.lb[89] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[81];
nmpcWorkspace.lb[90] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[82];
nmpcWorkspace.lb[91] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[83];
nmpcWorkspace.lb[92] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[84];
nmpcWorkspace.lb[93] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[85];
nmpcWorkspace.lb[94] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[86];
nmpcWorkspace.lb[95] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[87];
nmpcWorkspace.lb[96] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[88];
nmpcWorkspace.lb[97] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[89];
nmpcWorkspace.lb[98] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[90];
nmpcWorkspace.lb[99] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[91];
nmpcWorkspace.lb[100] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[92];
nmpcWorkspace.lb[101] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[93];
nmpcWorkspace.lb[102] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[94];
nmpcWorkspace.lb[103] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[95];
nmpcWorkspace.lb[104] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[96];
nmpcWorkspace.lb[105] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[97];
nmpcWorkspace.lb[106] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[98];
nmpcWorkspace.lb[107] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[99];
nmpcWorkspace.lb[108] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[100];
nmpcWorkspace.lb[109] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[101];
nmpcWorkspace.lb[110] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[102];
nmpcWorkspace.lb[111] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[103];
nmpcWorkspace.lb[112] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[104];
nmpcWorkspace.lb[113] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[105];
nmpcWorkspace.lb[114] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[106];
nmpcWorkspace.lb[115] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[107];
nmpcWorkspace.lb[116] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[108];
nmpcWorkspace.lb[117] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[109];
nmpcWorkspace.lb[118] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[110];
nmpcWorkspace.lb[119] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[111];
nmpcWorkspace.lb[120] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[112];
nmpcWorkspace.lb[121] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[113];
nmpcWorkspace.lb[122] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[114];
nmpcWorkspace.lb[123] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[115];
nmpcWorkspace.lb[124] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[116];
nmpcWorkspace.lb[125] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[117];
nmpcWorkspace.lb[126] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[118];
nmpcWorkspace.lb[127] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[119];
nmpcWorkspace.lb[128] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[120];
nmpcWorkspace.lb[129] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[121];
nmpcWorkspace.lb[130] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[122];
nmpcWorkspace.lb[131] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[123];
nmpcWorkspace.lb[132] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[124];
nmpcWorkspace.lb[133] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[125];
nmpcWorkspace.lb[134] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[126];
nmpcWorkspace.lb[135] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[127];
nmpcWorkspace.lb[136] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[128];
nmpcWorkspace.lb[137] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[129];
nmpcWorkspace.lb[138] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[130];
nmpcWorkspace.lb[139] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[131];
nmpcWorkspace.lb[140] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[132];
nmpcWorkspace.lb[141] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[133];
nmpcWorkspace.lb[142] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[134];
nmpcWorkspace.lb[143] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[135];
nmpcWorkspace.lb[144] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[136];
nmpcWorkspace.lb[145] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[137];
nmpcWorkspace.lb[146] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[138];
nmpcWorkspace.lb[147] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[139];
nmpcWorkspace.lb[148] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[140];
nmpcWorkspace.lb[149] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[141];
nmpcWorkspace.lb[150] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[142];
nmpcWorkspace.lb[151] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[143];
nmpcWorkspace.lb[152] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[144];
nmpcWorkspace.lb[153] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[145];
nmpcWorkspace.lb[154] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[146];
nmpcWorkspace.lb[155] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[147];
nmpcWorkspace.lb[156] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[148];
nmpcWorkspace.lb[157] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[149];
nmpcWorkspace.lb[158] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[150];
nmpcWorkspace.lb[159] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[151];
nmpcWorkspace.lb[160] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[152];
nmpcWorkspace.lb[161] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[153];
nmpcWorkspace.lb[162] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[154];
nmpcWorkspace.lb[163] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[155];
nmpcWorkspace.lb[164] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[156];
nmpcWorkspace.lb[165] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[157];
nmpcWorkspace.lb[166] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[158];
nmpcWorkspace.lb[167] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[159];
nmpcWorkspace.lb[168] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[160];
nmpcWorkspace.lb[169] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[161];
nmpcWorkspace.lb[170] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[162];
nmpcWorkspace.lb[171] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[163];
nmpcWorkspace.lb[172] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[164];
nmpcWorkspace.lb[173] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[165];
nmpcWorkspace.lb[174] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[166];
nmpcWorkspace.lb[175] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[167];
nmpcWorkspace.lb[176] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[168];
nmpcWorkspace.lb[177] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[169];
nmpcWorkspace.lb[178] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[170];
nmpcWorkspace.lb[179] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[171];
nmpcWorkspace.lb[180] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[172];
nmpcWorkspace.lb[181] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[173];
nmpcWorkspace.lb[182] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[174];
nmpcWorkspace.lb[183] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[175];
nmpcWorkspace.lb[184] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[176];
nmpcWorkspace.lb[185] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[177];
nmpcWorkspace.lb[186] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[178];
nmpcWorkspace.lb[187] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[179];
nmpcWorkspace.lb[188] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[180];
nmpcWorkspace.lb[189] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[181];
nmpcWorkspace.lb[190] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[182];
nmpcWorkspace.lb[191] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[183];
nmpcWorkspace.lb[192] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[184];
nmpcWorkspace.lb[193] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[185];
nmpcWorkspace.lb[194] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[186];
nmpcWorkspace.lb[195] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[187];
nmpcWorkspace.lb[196] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[188];
nmpcWorkspace.lb[197] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[189];
nmpcWorkspace.lb[198] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[190];
nmpcWorkspace.lb[199] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[191];
nmpcWorkspace.lb[200] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[192];
nmpcWorkspace.lb[201] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[193];
nmpcWorkspace.lb[202] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[194];
nmpcWorkspace.lb[203] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[195];
nmpcWorkspace.lb[204] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[196];
nmpcWorkspace.lb[205] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[197];
nmpcWorkspace.lb[206] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[198];
nmpcWorkspace.lb[207] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[199];
nmpcWorkspace.lb[208] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[200];
nmpcWorkspace.lb[209] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[201];
nmpcWorkspace.lb[210] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[202];
nmpcWorkspace.lb[211] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[203];
nmpcWorkspace.lb[212] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[204];
nmpcWorkspace.lb[213] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[205];
nmpcWorkspace.lb[214] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[206];
nmpcWorkspace.lb[215] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[207];
nmpcWorkspace.lb[216] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[208];
nmpcWorkspace.lb[217] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[209];
nmpcWorkspace.lb[218] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[210];
nmpcWorkspace.lb[219] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[211];
nmpcWorkspace.lb[220] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[212];
nmpcWorkspace.lb[221] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[213];
nmpcWorkspace.lb[222] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[214];
nmpcWorkspace.lb[223] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[215];
nmpcWorkspace.lb[224] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[216];
nmpcWorkspace.lb[225] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[217];
nmpcWorkspace.lb[226] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[218];
nmpcWorkspace.lb[227] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[219];
nmpcWorkspace.lb[228] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[220];
nmpcWorkspace.lb[229] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[221];
nmpcWorkspace.lb[230] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[222];
nmpcWorkspace.lb[231] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[223];
nmpcWorkspace.lb[232] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[224];
nmpcWorkspace.lb[233] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[225];
nmpcWorkspace.lb[234] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[226];
nmpcWorkspace.lb[235] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[227];
nmpcWorkspace.lb[236] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[228];
nmpcWorkspace.lb[237] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[229];
nmpcWorkspace.lb[238] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[230];
nmpcWorkspace.lb[239] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[231];
nmpcWorkspace.lb[240] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[232];
nmpcWorkspace.lb[241] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[233];
nmpcWorkspace.lb[242] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[234];
nmpcWorkspace.lb[243] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[235];
nmpcWorkspace.lb[244] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[236];
nmpcWorkspace.lb[245] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[237];
nmpcWorkspace.lb[246] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[238];
nmpcWorkspace.lb[247] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[239];
nmpcWorkspace.lb[248] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[240];
nmpcWorkspace.lb[249] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[241];
nmpcWorkspace.lb[250] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[242];
nmpcWorkspace.lb[251] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[243];
nmpcWorkspace.lb[252] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[244];
nmpcWorkspace.lb[253] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[245];
nmpcWorkspace.lb[254] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[246];
nmpcWorkspace.lb[255] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[247];
nmpcWorkspace.lb[256] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[248];
nmpcWorkspace.lb[257] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[249];
nmpcWorkspace.lb[258] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[250];
nmpcWorkspace.lb[259] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[251];
nmpcWorkspace.lb[260] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[252];
nmpcWorkspace.lb[261] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[253];
nmpcWorkspace.lb[262] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[254];
nmpcWorkspace.lb[263] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[255];
nmpcWorkspace.lb[264] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[256];
nmpcWorkspace.lb[265] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[257];
nmpcWorkspace.lb[266] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[258];
nmpcWorkspace.lb[267] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[259];
nmpcWorkspace.lb[268] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[260];
nmpcWorkspace.lb[269] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[261];
nmpcWorkspace.lb[270] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[262];
nmpcWorkspace.lb[271] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[263];
nmpcWorkspace.lb[272] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[264];
nmpcWorkspace.lb[273] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[265];
nmpcWorkspace.lb[274] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[266];
nmpcWorkspace.lb[275] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[267];
nmpcWorkspace.lb[276] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[268];
nmpcWorkspace.lb[277] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[269];
nmpcWorkspace.lb[278] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[270];
nmpcWorkspace.lb[279] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[271];
nmpcWorkspace.lb[280] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[272];
nmpcWorkspace.lb[281] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[273];
nmpcWorkspace.lb[282] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[274];
nmpcWorkspace.lb[283] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[275];
nmpcWorkspace.lb[284] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[276];
nmpcWorkspace.lb[285] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[277];
nmpcWorkspace.lb[286] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[278];
nmpcWorkspace.lb[287] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[279];
nmpcWorkspace.lb[288] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[280];
nmpcWorkspace.lb[289] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[281];
nmpcWorkspace.lb[290] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[282];
nmpcWorkspace.lb[291] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[283];
nmpcWorkspace.lb[292] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[284];
nmpcWorkspace.lb[293] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[285];
nmpcWorkspace.lb[294] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[286];
nmpcWorkspace.lb[295] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[287];
nmpcWorkspace.lb[296] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[288];
nmpcWorkspace.lb[297] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[289];
nmpcWorkspace.lb[298] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[290];
nmpcWorkspace.lb[299] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[291];
nmpcWorkspace.lb[300] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[292];
nmpcWorkspace.lb[301] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[293];
nmpcWorkspace.lb[302] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[294];
nmpcWorkspace.lb[303] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[295];
nmpcWorkspace.lb[304] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[296];
nmpcWorkspace.lb[305] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[297];
nmpcWorkspace.lb[306] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[298];
nmpcWorkspace.lb[307] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[299];
nmpcWorkspace.lb[308] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[300];
nmpcWorkspace.lb[309] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[301];
nmpcWorkspace.lb[310] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[302];
nmpcWorkspace.lb[311] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[303];
nmpcWorkspace.lb[312] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[304];
nmpcWorkspace.lb[313] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[305];
nmpcWorkspace.lb[314] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[306];
nmpcWorkspace.lb[315] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[307];
nmpcWorkspace.lb[316] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[308];
nmpcWorkspace.lb[317] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[309];
nmpcWorkspace.lb[318] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[310];
nmpcWorkspace.lb[319] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[311];
nmpcWorkspace.lb[320] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[312];
nmpcWorkspace.lb[321] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[313];
nmpcWorkspace.lb[322] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[314];
nmpcWorkspace.lb[323] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[315];
nmpcWorkspace.lb[324] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[316];
nmpcWorkspace.lb[325] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[317];
nmpcWorkspace.lb[326] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[318];
nmpcWorkspace.lb[327] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[319];
nmpcWorkspace.lb[328] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[320];
nmpcWorkspace.lb[329] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[321];
nmpcWorkspace.lb[330] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[322];
nmpcWorkspace.lb[331] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[323];
nmpcWorkspace.lb[332] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[324];
nmpcWorkspace.lb[333] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[325];
nmpcWorkspace.lb[334] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[326];
nmpcWorkspace.lb[335] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[327];
nmpcWorkspace.lb[336] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[328];
nmpcWorkspace.lb[337] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[329];
nmpcWorkspace.lb[338] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[330];
nmpcWorkspace.lb[339] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[331];
nmpcWorkspace.lb[340] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[332];
nmpcWorkspace.lb[341] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[333];
nmpcWorkspace.lb[342] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[334];
nmpcWorkspace.lb[343] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[335];
nmpcWorkspace.lb[344] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[336];
nmpcWorkspace.lb[345] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[337];
nmpcWorkspace.lb[346] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[338];
nmpcWorkspace.lb[347] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[339];
nmpcWorkspace.lb[348] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[340];
nmpcWorkspace.lb[349] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[341];
nmpcWorkspace.lb[350] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[342];
nmpcWorkspace.lb[351] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[343];
nmpcWorkspace.lb[352] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[344];
nmpcWorkspace.lb[353] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[345];
nmpcWorkspace.lb[354] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[346];
nmpcWorkspace.lb[355] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[347];
nmpcWorkspace.lb[356] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[348];
nmpcWorkspace.lb[357] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[349];
nmpcWorkspace.lb[358] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[350];
nmpcWorkspace.lb[359] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[351];
nmpcWorkspace.lb[360] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[352];
nmpcWorkspace.lb[361] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[353];
nmpcWorkspace.lb[362] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[354];
nmpcWorkspace.lb[363] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[355];
nmpcWorkspace.lb[364] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[356];
nmpcWorkspace.lb[365] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[357];
nmpcWorkspace.lb[366] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[358];
nmpcWorkspace.lb[367] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[359];
nmpcWorkspace.lb[368] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[360];
nmpcWorkspace.lb[369] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[361];
nmpcWorkspace.lb[370] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[362];
nmpcWorkspace.lb[371] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[363];
nmpcWorkspace.lb[372] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[364];
nmpcWorkspace.lb[373] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[365];
nmpcWorkspace.lb[374] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[366];
nmpcWorkspace.lb[375] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[367];
nmpcWorkspace.lb[376] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[368];
nmpcWorkspace.lb[377] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[369];
nmpcWorkspace.lb[378] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[370];
nmpcWorkspace.lb[379] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[371];
nmpcWorkspace.lb[380] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[372];
nmpcWorkspace.lb[381] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[373];
nmpcWorkspace.lb[382] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[374];
nmpcWorkspace.lb[383] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[375];
nmpcWorkspace.lb[384] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[376];
nmpcWorkspace.lb[385] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[377];
nmpcWorkspace.lb[386] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[378];
nmpcWorkspace.lb[387] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[379];
nmpcWorkspace.lb[388] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[380];
nmpcWorkspace.lb[389] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[381];
nmpcWorkspace.lb[390] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[382];
nmpcWorkspace.lb[391] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[383];
nmpcWorkspace.lb[392] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[384];
nmpcWorkspace.lb[393] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[385];
nmpcWorkspace.lb[394] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[386];
nmpcWorkspace.lb[395] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[387];
nmpcWorkspace.lb[396] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[388];
nmpcWorkspace.lb[397] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[389];
nmpcWorkspace.lb[398] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[390];
nmpcWorkspace.lb[399] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[391];
nmpcWorkspace.lb[400] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[392];
nmpcWorkspace.lb[401] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[393];
nmpcWorkspace.lb[402] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[394];
nmpcWorkspace.lb[403] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[395];
nmpcWorkspace.lb[404] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[396];
nmpcWorkspace.lb[405] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[397];
nmpcWorkspace.lb[406] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[398];
nmpcWorkspace.lb[407] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[399];
nmpcWorkspace.ub[8] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[0];
nmpcWorkspace.ub[9] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[1];
nmpcWorkspace.ub[10] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[2];
nmpcWorkspace.ub[11] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[3];
nmpcWorkspace.ub[12] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[4];
nmpcWorkspace.ub[13] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[5];
nmpcWorkspace.ub[14] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[6];
nmpcWorkspace.ub[15] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[7];
nmpcWorkspace.ub[16] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[8];
nmpcWorkspace.ub[17] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[9];
nmpcWorkspace.ub[18] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[10];
nmpcWorkspace.ub[19] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[11];
nmpcWorkspace.ub[20] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[12];
nmpcWorkspace.ub[21] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[13];
nmpcWorkspace.ub[22] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[14];
nmpcWorkspace.ub[23] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[15];
nmpcWorkspace.ub[24] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[16];
nmpcWorkspace.ub[25] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[17];
nmpcWorkspace.ub[26] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[18];
nmpcWorkspace.ub[27] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[19];
nmpcWorkspace.ub[28] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[20];
nmpcWorkspace.ub[29] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[21];
nmpcWorkspace.ub[30] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[22];
nmpcWorkspace.ub[31] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[23];
nmpcWorkspace.ub[32] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[24];
nmpcWorkspace.ub[33] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[25];
nmpcWorkspace.ub[34] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[26];
nmpcWorkspace.ub[35] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[27];
nmpcWorkspace.ub[36] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[28];
nmpcWorkspace.ub[37] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[29];
nmpcWorkspace.ub[38] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[30];
nmpcWorkspace.ub[39] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[31];
nmpcWorkspace.ub[40] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[32];
nmpcWorkspace.ub[41] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[33];
nmpcWorkspace.ub[42] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[34];
nmpcWorkspace.ub[43] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[35];
nmpcWorkspace.ub[44] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[36];
nmpcWorkspace.ub[45] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[37];
nmpcWorkspace.ub[46] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[38];
nmpcWorkspace.ub[47] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[39];
nmpcWorkspace.ub[48] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[40];
nmpcWorkspace.ub[49] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[41];
nmpcWorkspace.ub[50] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[42];
nmpcWorkspace.ub[51] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[43];
nmpcWorkspace.ub[52] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[44];
nmpcWorkspace.ub[53] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[45];
nmpcWorkspace.ub[54] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[46];
nmpcWorkspace.ub[55] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[47];
nmpcWorkspace.ub[56] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[48];
nmpcWorkspace.ub[57] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[49];
nmpcWorkspace.ub[58] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[50];
nmpcWorkspace.ub[59] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[51];
nmpcWorkspace.ub[60] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[52];
nmpcWorkspace.ub[61] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[53];
nmpcWorkspace.ub[62] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[54];
nmpcWorkspace.ub[63] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[55];
nmpcWorkspace.ub[64] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[56];
nmpcWorkspace.ub[65] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[57];
nmpcWorkspace.ub[66] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[58];
nmpcWorkspace.ub[67] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[59];
nmpcWorkspace.ub[68] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[60];
nmpcWorkspace.ub[69] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[61];
nmpcWorkspace.ub[70] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[62];
nmpcWorkspace.ub[71] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[63];
nmpcWorkspace.ub[72] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[64];
nmpcWorkspace.ub[73] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[65];
nmpcWorkspace.ub[74] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[66];
nmpcWorkspace.ub[75] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[67];
nmpcWorkspace.ub[76] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[68];
nmpcWorkspace.ub[77] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[69];
nmpcWorkspace.ub[78] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[70];
nmpcWorkspace.ub[79] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[71];
nmpcWorkspace.ub[80] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[72];
nmpcWorkspace.ub[81] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[73];
nmpcWorkspace.ub[82] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[74];
nmpcWorkspace.ub[83] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[75];
nmpcWorkspace.ub[84] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[76];
nmpcWorkspace.ub[85] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[77];
nmpcWorkspace.ub[86] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[78];
nmpcWorkspace.ub[87] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[79];
nmpcWorkspace.ub[88] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[80];
nmpcWorkspace.ub[89] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[81];
nmpcWorkspace.ub[90] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[82];
nmpcWorkspace.ub[91] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[83];
nmpcWorkspace.ub[92] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[84];
nmpcWorkspace.ub[93] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[85];
nmpcWorkspace.ub[94] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[86];
nmpcWorkspace.ub[95] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[87];
nmpcWorkspace.ub[96] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[88];
nmpcWorkspace.ub[97] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[89];
nmpcWorkspace.ub[98] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[90];
nmpcWorkspace.ub[99] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[91];
nmpcWorkspace.ub[100] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[92];
nmpcWorkspace.ub[101] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[93];
nmpcWorkspace.ub[102] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[94];
nmpcWorkspace.ub[103] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[95];
nmpcWorkspace.ub[104] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[96];
nmpcWorkspace.ub[105] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[97];
nmpcWorkspace.ub[106] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[98];
nmpcWorkspace.ub[107] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[99];
nmpcWorkspace.ub[108] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[100];
nmpcWorkspace.ub[109] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[101];
nmpcWorkspace.ub[110] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[102];
nmpcWorkspace.ub[111] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[103];
nmpcWorkspace.ub[112] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[104];
nmpcWorkspace.ub[113] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[105];
nmpcWorkspace.ub[114] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[106];
nmpcWorkspace.ub[115] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[107];
nmpcWorkspace.ub[116] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[108];
nmpcWorkspace.ub[117] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[109];
nmpcWorkspace.ub[118] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[110];
nmpcWorkspace.ub[119] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[111];
nmpcWorkspace.ub[120] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[112];
nmpcWorkspace.ub[121] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[113];
nmpcWorkspace.ub[122] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[114];
nmpcWorkspace.ub[123] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[115];
nmpcWorkspace.ub[124] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[116];
nmpcWorkspace.ub[125] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[117];
nmpcWorkspace.ub[126] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[118];
nmpcWorkspace.ub[127] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[119];
nmpcWorkspace.ub[128] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[120];
nmpcWorkspace.ub[129] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[121];
nmpcWorkspace.ub[130] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[122];
nmpcWorkspace.ub[131] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[123];
nmpcWorkspace.ub[132] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[124];
nmpcWorkspace.ub[133] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[125];
nmpcWorkspace.ub[134] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[126];
nmpcWorkspace.ub[135] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[127];
nmpcWorkspace.ub[136] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[128];
nmpcWorkspace.ub[137] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[129];
nmpcWorkspace.ub[138] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[130];
nmpcWorkspace.ub[139] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[131];
nmpcWorkspace.ub[140] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[132];
nmpcWorkspace.ub[141] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[133];
nmpcWorkspace.ub[142] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[134];
nmpcWorkspace.ub[143] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[135];
nmpcWorkspace.ub[144] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[136];
nmpcWorkspace.ub[145] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[137];
nmpcWorkspace.ub[146] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[138];
nmpcWorkspace.ub[147] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[139];
nmpcWorkspace.ub[148] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[140];
nmpcWorkspace.ub[149] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[141];
nmpcWorkspace.ub[150] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[142];
nmpcWorkspace.ub[151] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[143];
nmpcWorkspace.ub[152] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[144];
nmpcWorkspace.ub[153] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[145];
nmpcWorkspace.ub[154] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[146];
nmpcWorkspace.ub[155] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[147];
nmpcWorkspace.ub[156] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[148];
nmpcWorkspace.ub[157] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[149];
nmpcWorkspace.ub[158] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[150];
nmpcWorkspace.ub[159] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[151];
nmpcWorkspace.ub[160] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[152];
nmpcWorkspace.ub[161] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[153];
nmpcWorkspace.ub[162] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[154];
nmpcWorkspace.ub[163] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[155];
nmpcWorkspace.ub[164] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[156];
nmpcWorkspace.ub[165] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[157];
nmpcWorkspace.ub[166] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[158];
nmpcWorkspace.ub[167] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[159];
nmpcWorkspace.ub[168] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[160];
nmpcWorkspace.ub[169] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[161];
nmpcWorkspace.ub[170] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[162];
nmpcWorkspace.ub[171] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[163];
nmpcWorkspace.ub[172] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[164];
nmpcWorkspace.ub[173] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[165];
nmpcWorkspace.ub[174] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[166];
nmpcWorkspace.ub[175] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[167];
nmpcWorkspace.ub[176] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[168];
nmpcWorkspace.ub[177] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[169];
nmpcWorkspace.ub[178] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[170];
nmpcWorkspace.ub[179] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[171];
nmpcWorkspace.ub[180] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[172];
nmpcWorkspace.ub[181] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[173];
nmpcWorkspace.ub[182] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[174];
nmpcWorkspace.ub[183] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[175];
nmpcWorkspace.ub[184] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[176];
nmpcWorkspace.ub[185] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[177];
nmpcWorkspace.ub[186] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[178];
nmpcWorkspace.ub[187] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[179];
nmpcWorkspace.ub[188] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[180];
nmpcWorkspace.ub[189] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[181];
nmpcWorkspace.ub[190] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[182];
nmpcWorkspace.ub[191] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[183];
nmpcWorkspace.ub[192] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[184];
nmpcWorkspace.ub[193] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[185];
nmpcWorkspace.ub[194] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[186];
nmpcWorkspace.ub[195] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[187];
nmpcWorkspace.ub[196] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[188];
nmpcWorkspace.ub[197] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[189];
nmpcWorkspace.ub[198] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[190];
nmpcWorkspace.ub[199] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[191];
nmpcWorkspace.ub[200] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[192];
nmpcWorkspace.ub[201] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[193];
nmpcWorkspace.ub[202] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[194];
nmpcWorkspace.ub[203] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[195];
nmpcWorkspace.ub[204] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[196];
nmpcWorkspace.ub[205] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[197];
nmpcWorkspace.ub[206] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[198];
nmpcWorkspace.ub[207] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[199];
nmpcWorkspace.ub[208] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[200];
nmpcWorkspace.ub[209] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[201];
nmpcWorkspace.ub[210] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[202];
nmpcWorkspace.ub[211] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[203];
nmpcWorkspace.ub[212] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[204];
nmpcWorkspace.ub[213] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[205];
nmpcWorkspace.ub[214] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[206];
nmpcWorkspace.ub[215] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[207];
nmpcWorkspace.ub[216] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[208];
nmpcWorkspace.ub[217] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[209];
nmpcWorkspace.ub[218] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[210];
nmpcWorkspace.ub[219] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[211];
nmpcWorkspace.ub[220] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[212];
nmpcWorkspace.ub[221] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[213];
nmpcWorkspace.ub[222] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[214];
nmpcWorkspace.ub[223] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[215];
nmpcWorkspace.ub[224] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[216];
nmpcWorkspace.ub[225] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[217];
nmpcWorkspace.ub[226] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[218];
nmpcWorkspace.ub[227] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[219];
nmpcWorkspace.ub[228] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[220];
nmpcWorkspace.ub[229] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[221];
nmpcWorkspace.ub[230] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[222];
nmpcWorkspace.ub[231] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[223];
nmpcWorkspace.ub[232] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[224];
nmpcWorkspace.ub[233] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[225];
nmpcWorkspace.ub[234] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[226];
nmpcWorkspace.ub[235] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[227];
nmpcWorkspace.ub[236] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[228];
nmpcWorkspace.ub[237] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[229];
nmpcWorkspace.ub[238] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[230];
nmpcWorkspace.ub[239] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[231];
nmpcWorkspace.ub[240] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[232];
nmpcWorkspace.ub[241] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[233];
nmpcWorkspace.ub[242] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[234];
nmpcWorkspace.ub[243] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[235];
nmpcWorkspace.ub[244] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[236];
nmpcWorkspace.ub[245] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[237];
nmpcWorkspace.ub[246] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[238];
nmpcWorkspace.ub[247] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[239];
nmpcWorkspace.ub[248] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[240];
nmpcWorkspace.ub[249] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[241];
nmpcWorkspace.ub[250] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[242];
nmpcWorkspace.ub[251] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[243];
nmpcWorkspace.ub[252] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[244];
nmpcWorkspace.ub[253] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[245];
nmpcWorkspace.ub[254] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[246];
nmpcWorkspace.ub[255] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[247];
nmpcWorkspace.ub[256] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[248];
nmpcWorkspace.ub[257] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[249];
nmpcWorkspace.ub[258] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[250];
nmpcWorkspace.ub[259] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[251];
nmpcWorkspace.ub[260] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[252];
nmpcWorkspace.ub[261] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[253];
nmpcWorkspace.ub[262] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[254];
nmpcWorkspace.ub[263] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[255];
nmpcWorkspace.ub[264] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[256];
nmpcWorkspace.ub[265] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[257];
nmpcWorkspace.ub[266] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[258];
nmpcWorkspace.ub[267] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[259];
nmpcWorkspace.ub[268] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[260];
nmpcWorkspace.ub[269] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[261];
nmpcWorkspace.ub[270] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[262];
nmpcWorkspace.ub[271] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[263];
nmpcWorkspace.ub[272] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[264];
nmpcWorkspace.ub[273] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[265];
nmpcWorkspace.ub[274] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[266];
nmpcWorkspace.ub[275] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[267];
nmpcWorkspace.ub[276] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[268];
nmpcWorkspace.ub[277] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[269];
nmpcWorkspace.ub[278] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[270];
nmpcWorkspace.ub[279] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[271];
nmpcWorkspace.ub[280] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[272];
nmpcWorkspace.ub[281] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[273];
nmpcWorkspace.ub[282] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[274];
nmpcWorkspace.ub[283] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[275];
nmpcWorkspace.ub[284] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[276];
nmpcWorkspace.ub[285] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[277];
nmpcWorkspace.ub[286] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[278];
nmpcWorkspace.ub[287] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[279];
nmpcWorkspace.ub[288] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[280];
nmpcWorkspace.ub[289] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[281];
nmpcWorkspace.ub[290] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[282];
nmpcWorkspace.ub[291] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[283];
nmpcWorkspace.ub[292] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[284];
nmpcWorkspace.ub[293] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[285];
nmpcWorkspace.ub[294] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[286];
nmpcWorkspace.ub[295] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[287];
nmpcWorkspace.ub[296] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[288];
nmpcWorkspace.ub[297] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[289];
nmpcWorkspace.ub[298] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[290];
nmpcWorkspace.ub[299] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[291];
nmpcWorkspace.ub[300] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[292];
nmpcWorkspace.ub[301] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[293];
nmpcWorkspace.ub[302] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[294];
nmpcWorkspace.ub[303] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[295];
nmpcWorkspace.ub[304] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[296];
nmpcWorkspace.ub[305] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[297];
nmpcWorkspace.ub[306] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[298];
nmpcWorkspace.ub[307] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[299];
nmpcWorkspace.ub[308] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[300];
nmpcWorkspace.ub[309] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[301];
nmpcWorkspace.ub[310] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[302];
nmpcWorkspace.ub[311] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[303];
nmpcWorkspace.ub[312] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[304];
nmpcWorkspace.ub[313] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[305];
nmpcWorkspace.ub[314] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[306];
nmpcWorkspace.ub[315] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[307];
nmpcWorkspace.ub[316] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[308];
nmpcWorkspace.ub[317] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[309];
nmpcWorkspace.ub[318] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[310];
nmpcWorkspace.ub[319] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[311];
nmpcWorkspace.ub[320] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[312];
nmpcWorkspace.ub[321] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[313];
nmpcWorkspace.ub[322] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[314];
nmpcWorkspace.ub[323] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[315];
nmpcWorkspace.ub[324] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[316];
nmpcWorkspace.ub[325] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[317];
nmpcWorkspace.ub[326] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[318];
nmpcWorkspace.ub[327] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[319];
nmpcWorkspace.ub[328] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[320];
nmpcWorkspace.ub[329] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[321];
nmpcWorkspace.ub[330] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[322];
nmpcWorkspace.ub[331] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[323];
nmpcWorkspace.ub[332] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[324];
nmpcWorkspace.ub[333] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[325];
nmpcWorkspace.ub[334] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[326];
nmpcWorkspace.ub[335] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[327];
nmpcWorkspace.ub[336] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[328];
nmpcWorkspace.ub[337] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[329];
nmpcWorkspace.ub[338] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[330];
nmpcWorkspace.ub[339] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[331];
nmpcWorkspace.ub[340] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[332];
nmpcWorkspace.ub[341] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[333];
nmpcWorkspace.ub[342] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[334];
nmpcWorkspace.ub[343] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[335];
nmpcWorkspace.ub[344] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[336];
nmpcWorkspace.ub[345] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[337];
nmpcWorkspace.ub[346] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[338];
nmpcWorkspace.ub[347] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[339];
nmpcWorkspace.ub[348] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[340];
nmpcWorkspace.ub[349] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[341];
nmpcWorkspace.ub[350] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[342];
nmpcWorkspace.ub[351] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[343];
nmpcWorkspace.ub[352] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[344];
nmpcWorkspace.ub[353] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[345];
nmpcWorkspace.ub[354] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[346];
nmpcWorkspace.ub[355] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[347];
nmpcWorkspace.ub[356] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[348];
nmpcWorkspace.ub[357] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[349];
nmpcWorkspace.ub[358] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[350];
nmpcWorkspace.ub[359] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[351];
nmpcWorkspace.ub[360] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[352];
nmpcWorkspace.ub[361] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[353];
nmpcWorkspace.ub[362] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[354];
nmpcWorkspace.ub[363] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[355];
nmpcWorkspace.ub[364] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[356];
nmpcWorkspace.ub[365] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[357];
nmpcWorkspace.ub[366] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[358];
nmpcWorkspace.ub[367] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[359];
nmpcWorkspace.ub[368] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[360];
nmpcWorkspace.ub[369] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[361];
nmpcWorkspace.ub[370] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[362];
nmpcWorkspace.ub[371] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[363];
nmpcWorkspace.ub[372] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[364];
nmpcWorkspace.ub[373] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[365];
nmpcWorkspace.ub[374] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[366];
nmpcWorkspace.ub[375] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[367];
nmpcWorkspace.ub[376] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[368];
nmpcWorkspace.ub[377] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[369];
nmpcWorkspace.ub[378] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[370];
nmpcWorkspace.ub[379] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[371];
nmpcWorkspace.ub[380] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[372];
nmpcWorkspace.ub[381] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[373];
nmpcWorkspace.ub[382] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[374];
nmpcWorkspace.ub[383] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[375];
nmpcWorkspace.ub[384] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[376];
nmpcWorkspace.ub[385] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[377];
nmpcWorkspace.ub[386] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[378];
nmpcWorkspace.ub[387] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[379];
nmpcWorkspace.ub[388] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[380];
nmpcWorkspace.ub[389] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[381];
nmpcWorkspace.ub[390] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[382];
nmpcWorkspace.ub[391] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[383];
nmpcWorkspace.ub[392] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[384];
nmpcWorkspace.ub[393] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[385];
nmpcWorkspace.ub[394] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[386];
nmpcWorkspace.ub[395] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[387];
nmpcWorkspace.ub[396] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[388];
nmpcWorkspace.ub[397] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[389];
nmpcWorkspace.ub[398] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[390];
nmpcWorkspace.ub[399] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[391];
nmpcWorkspace.ub[400] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[392];
nmpcWorkspace.ub[401] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[393];
nmpcWorkspace.ub[402] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[394];
nmpcWorkspace.ub[403] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[395];
nmpcWorkspace.ub[404] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[396];
nmpcWorkspace.ub[405] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[397];
nmpcWorkspace.ub[406] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[398];
nmpcWorkspace.ub[407] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[399];

}

void nmpc_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
nmpcWorkspace.Dx0[0] = nmpcVariables.x0[0] - nmpcVariables.x[0];
nmpcWorkspace.Dx0[1] = nmpcVariables.x0[1] - nmpcVariables.x[1];
nmpcWorkspace.Dx0[2] = nmpcVariables.x0[2] - nmpcVariables.x[2];
nmpcWorkspace.Dx0[3] = nmpcVariables.x0[3] - nmpcVariables.x[3];
nmpcWorkspace.Dx0[4] = nmpcVariables.x0[4] - nmpcVariables.x[4];
nmpcWorkspace.Dx0[5] = nmpcVariables.x0[5] - nmpcVariables.x[5];
nmpcWorkspace.Dx0[6] = nmpcVariables.x0[6] - nmpcVariables.x[6];
nmpcWorkspace.Dx0[7] = nmpcVariables.x0[7] - nmpcVariables.x[7];

for (lRun2 = 0; lRun2 < 1200; ++lRun2)
nmpcWorkspace.Dy[lRun2] -= nmpcVariables.y[lRun2];

nmpcWorkspace.DyN[0] -= nmpcVariables.yN[0];
nmpcWorkspace.DyN[1] -= nmpcVariables.yN[1];
nmpcWorkspace.DyN[2] -= nmpcVariables.yN[2];
nmpcWorkspace.DyN[3] -= nmpcVariables.yN[3];
nmpcWorkspace.DyN[4] -= nmpcVariables.yN[4];
nmpcWorkspace.DyN[5] -= nmpcVariables.yN[5];
nmpcWorkspace.DyN[6] -= nmpcVariables.yN[6];
nmpcWorkspace.DyN[7] -= nmpcVariables.yN[7];

nmpc_multRDy( nmpcWorkspace.R2, nmpcWorkspace.Dy, &(nmpcWorkspace.g[ 8 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 48 ]), &(nmpcWorkspace.Dy[ 12 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 96 ]), &(nmpcWorkspace.Dy[ 24 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 144 ]), &(nmpcWorkspace.Dy[ 36 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 192 ]), &(nmpcWorkspace.Dy[ 48 ]), &(nmpcWorkspace.g[ 24 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 240 ]), &(nmpcWorkspace.Dy[ 60 ]), &(nmpcWorkspace.g[ 28 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 288 ]), &(nmpcWorkspace.Dy[ 72 ]), &(nmpcWorkspace.g[ 32 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 336 ]), &(nmpcWorkspace.Dy[ 84 ]), &(nmpcWorkspace.g[ 36 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 384 ]), &(nmpcWorkspace.Dy[ 96 ]), &(nmpcWorkspace.g[ 40 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 432 ]), &(nmpcWorkspace.Dy[ 108 ]), &(nmpcWorkspace.g[ 44 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 480 ]), &(nmpcWorkspace.Dy[ 120 ]), &(nmpcWorkspace.g[ 48 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 528 ]), &(nmpcWorkspace.Dy[ 132 ]), &(nmpcWorkspace.g[ 52 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 576 ]), &(nmpcWorkspace.Dy[ 144 ]), &(nmpcWorkspace.g[ 56 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 624 ]), &(nmpcWorkspace.Dy[ 156 ]), &(nmpcWorkspace.g[ 60 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 672 ]), &(nmpcWorkspace.Dy[ 168 ]), &(nmpcWorkspace.g[ 64 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 720 ]), &(nmpcWorkspace.Dy[ 180 ]), &(nmpcWorkspace.g[ 68 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 768 ]), &(nmpcWorkspace.Dy[ 192 ]), &(nmpcWorkspace.g[ 72 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 816 ]), &(nmpcWorkspace.Dy[ 204 ]), &(nmpcWorkspace.g[ 76 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 864 ]), &(nmpcWorkspace.Dy[ 216 ]), &(nmpcWorkspace.g[ 80 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 912 ]), &(nmpcWorkspace.Dy[ 228 ]), &(nmpcWorkspace.g[ 84 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 960 ]), &(nmpcWorkspace.Dy[ 240 ]), &(nmpcWorkspace.g[ 88 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1008 ]), &(nmpcWorkspace.Dy[ 252 ]), &(nmpcWorkspace.g[ 92 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1056 ]), &(nmpcWorkspace.Dy[ 264 ]), &(nmpcWorkspace.g[ 96 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1104 ]), &(nmpcWorkspace.Dy[ 276 ]), &(nmpcWorkspace.g[ 100 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1152 ]), &(nmpcWorkspace.Dy[ 288 ]), &(nmpcWorkspace.g[ 104 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1200 ]), &(nmpcWorkspace.Dy[ 300 ]), &(nmpcWorkspace.g[ 108 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1248 ]), &(nmpcWorkspace.Dy[ 312 ]), &(nmpcWorkspace.g[ 112 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1296 ]), &(nmpcWorkspace.Dy[ 324 ]), &(nmpcWorkspace.g[ 116 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1344 ]), &(nmpcWorkspace.Dy[ 336 ]), &(nmpcWorkspace.g[ 120 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1392 ]), &(nmpcWorkspace.Dy[ 348 ]), &(nmpcWorkspace.g[ 124 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1440 ]), &(nmpcWorkspace.Dy[ 360 ]), &(nmpcWorkspace.g[ 128 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1488 ]), &(nmpcWorkspace.Dy[ 372 ]), &(nmpcWorkspace.g[ 132 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1536 ]), &(nmpcWorkspace.Dy[ 384 ]), &(nmpcWorkspace.g[ 136 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1584 ]), &(nmpcWorkspace.Dy[ 396 ]), &(nmpcWorkspace.g[ 140 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1632 ]), &(nmpcWorkspace.Dy[ 408 ]), &(nmpcWorkspace.g[ 144 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1680 ]), &(nmpcWorkspace.Dy[ 420 ]), &(nmpcWorkspace.g[ 148 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1728 ]), &(nmpcWorkspace.Dy[ 432 ]), &(nmpcWorkspace.g[ 152 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1776 ]), &(nmpcWorkspace.Dy[ 444 ]), &(nmpcWorkspace.g[ 156 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1824 ]), &(nmpcWorkspace.Dy[ 456 ]), &(nmpcWorkspace.g[ 160 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1872 ]), &(nmpcWorkspace.Dy[ 468 ]), &(nmpcWorkspace.g[ 164 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1920 ]), &(nmpcWorkspace.Dy[ 480 ]), &(nmpcWorkspace.g[ 168 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1968 ]), &(nmpcWorkspace.Dy[ 492 ]), &(nmpcWorkspace.g[ 172 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2016 ]), &(nmpcWorkspace.Dy[ 504 ]), &(nmpcWorkspace.g[ 176 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2064 ]), &(nmpcWorkspace.Dy[ 516 ]), &(nmpcWorkspace.g[ 180 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2112 ]), &(nmpcWorkspace.Dy[ 528 ]), &(nmpcWorkspace.g[ 184 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2160 ]), &(nmpcWorkspace.Dy[ 540 ]), &(nmpcWorkspace.g[ 188 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2208 ]), &(nmpcWorkspace.Dy[ 552 ]), &(nmpcWorkspace.g[ 192 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2256 ]), &(nmpcWorkspace.Dy[ 564 ]), &(nmpcWorkspace.g[ 196 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2304 ]), &(nmpcWorkspace.Dy[ 576 ]), &(nmpcWorkspace.g[ 200 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2352 ]), &(nmpcWorkspace.Dy[ 588 ]), &(nmpcWorkspace.g[ 204 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2400 ]), &(nmpcWorkspace.Dy[ 600 ]), &(nmpcWorkspace.g[ 208 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2448 ]), &(nmpcWorkspace.Dy[ 612 ]), &(nmpcWorkspace.g[ 212 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2496 ]), &(nmpcWorkspace.Dy[ 624 ]), &(nmpcWorkspace.g[ 216 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2544 ]), &(nmpcWorkspace.Dy[ 636 ]), &(nmpcWorkspace.g[ 220 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2592 ]), &(nmpcWorkspace.Dy[ 648 ]), &(nmpcWorkspace.g[ 224 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2640 ]), &(nmpcWorkspace.Dy[ 660 ]), &(nmpcWorkspace.g[ 228 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2688 ]), &(nmpcWorkspace.Dy[ 672 ]), &(nmpcWorkspace.g[ 232 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2736 ]), &(nmpcWorkspace.Dy[ 684 ]), &(nmpcWorkspace.g[ 236 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2784 ]), &(nmpcWorkspace.Dy[ 696 ]), &(nmpcWorkspace.g[ 240 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2832 ]), &(nmpcWorkspace.Dy[ 708 ]), &(nmpcWorkspace.g[ 244 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2880 ]), &(nmpcWorkspace.Dy[ 720 ]), &(nmpcWorkspace.g[ 248 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2928 ]), &(nmpcWorkspace.Dy[ 732 ]), &(nmpcWorkspace.g[ 252 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 2976 ]), &(nmpcWorkspace.Dy[ 744 ]), &(nmpcWorkspace.g[ 256 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3024 ]), &(nmpcWorkspace.Dy[ 756 ]), &(nmpcWorkspace.g[ 260 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3072 ]), &(nmpcWorkspace.Dy[ 768 ]), &(nmpcWorkspace.g[ 264 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3120 ]), &(nmpcWorkspace.Dy[ 780 ]), &(nmpcWorkspace.g[ 268 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3168 ]), &(nmpcWorkspace.Dy[ 792 ]), &(nmpcWorkspace.g[ 272 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3216 ]), &(nmpcWorkspace.Dy[ 804 ]), &(nmpcWorkspace.g[ 276 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3264 ]), &(nmpcWorkspace.Dy[ 816 ]), &(nmpcWorkspace.g[ 280 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3312 ]), &(nmpcWorkspace.Dy[ 828 ]), &(nmpcWorkspace.g[ 284 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3360 ]), &(nmpcWorkspace.Dy[ 840 ]), &(nmpcWorkspace.g[ 288 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3408 ]), &(nmpcWorkspace.Dy[ 852 ]), &(nmpcWorkspace.g[ 292 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3456 ]), &(nmpcWorkspace.Dy[ 864 ]), &(nmpcWorkspace.g[ 296 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3504 ]), &(nmpcWorkspace.Dy[ 876 ]), &(nmpcWorkspace.g[ 300 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3552 ]), &(nmpcWorkspace.Dy[ 888 ]), &(nmpcWorkspace.g[ 304 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3600 ]), &(nmpcWorkspace.Dy[ 900 ]), &(nmpcWorkspace.g[ 308 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3648 ]), &(nmpcWorkspace.Dy[ 912 ]), &(nmpcWorkspace.g[ 312 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3696 ]), &(nmpcWorkspace.Dy[ 924 ]), &(nmpcWorkspace.g[ 316 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3744 ]), &(nmpcWorkspace.Dy[ 936 ]), &(nmpcWorkspace.g[ 320 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3792 ]), &(nmpcWorkspace.Dy[ 948 ]), &(nmpcWorkspace.g[ 324 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3840 ]), &(nmpcWorkspace.Dy[ 960 ]), &(nmpcWorkspace.g[ 328 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3888 ]), &(nmpcWorkspace.Dy[ 972 ]), &(nmpcWorkspace.g[ 332 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3936 ]), &(nmpcWorkspace.Dy[ 984 ]), &(nmpcWorkspace.g[ 336 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 3984 ]), &(nmpcWorkspace.Dy[ 996 ]), &(nmpcWorkspace.g[ 340 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 4032 ]), &(nmpcWorkspace.Dy[ 1008 ]), &(nmpcWorkspace.g[ 344 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 4080 ]), &(nmpcWorkspace.Dy[ 1020 ]), &(nmpcWorkspace.g[ 348 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 4128 ]), &(nmpcWorkspace.Dy[ 1032 ]), &(nmpcWorkspace.g[ 352 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 4176 ]), &(nmpcWorkspace.Dy[ 1044 ]), &(nmpcWorkspace.g[ 356 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 4224 ]), &(nmpcWorkspace.Dy[ 1056 ]), &(nmpcWorkspace.g[ 360 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 4272 ]), &(nmpcWorkspace.Dy[ 1068 ]), &(nmpcWorkspace.g[ 364 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 4320 ]), &(nmpcWorkspace.Dy[ 1080 ]), &(nmpcWorkspace.g[ 368 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 4368 ]), &(nmpcWorkspace.Dy[ 1092 ]), &(nmpcWorkspace.g[ 372 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 4416 ]), &(nmpcWorkspace.Dy[ 1104 ]), &(nmpcWorkspace.g[ 376 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 4464 ]), &(nmpcWorkspace.Dy[ 1116 ]), &(nmpcWorkspace.g[ 380 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 4512 ]), &(nmpcWorkspace.Dy[ 1128 ]), &(nmpcWorkspace.g[ 384 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 4560 ]), &(nmpcWorkspace.Dy[ 1140 ]), &(nmpcWorkspace.g[ 388 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 4608 ]), &(nmpcWorkspace.Dy[ 1152 ]), &(nmpcWorkspace.g[ 392 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 4656 ]), &(nmpcWorkspace.Dy[ 1164 ]), &(nmpcWorkspace.g[ 396 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 4704 ]), &(nmpcWorkspace.Dy[ 1176 ]), &(nmpcWorkspace.g[ 400 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 4752 ]), &(nmpcWorkspace.Dy[ 1188 ]), &(nmpcWorkspace.g[ 404 ]) );

nmpc_multQDy( nmpcWorkspace.Q2, nmpcWorkspace.Dy, nmpcWorkspace.QDy );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 96 ]), &(nmpcWorkspace.Dy[ 12 ]), &(nmpcWorkspace.QDy[ 8 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 192 ]), &(nmpcWorkspace.Dy[ 24 ]), &(nmpcWorkspace.QDy[ 16 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 288 ]), &(nmpcWorkspace.Dy[ 36 ]), &(nmpcWorkspace.QDy[ 24 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 384 ]), &(nmpcWorkspace.Dy[ 48 ]), &(nmpcWorkspace.QDy[ 32 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 480 ]), &(nmpcWorkspace.Dy[ 60 ]), &(nmpcWorkspace.QDy[ 40 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 576 ]), &(nmpcWorkspace.Dy[ 72 ]), &(nmpcWorkspace.QDy[ 48 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 672 ]), &(nmpcWorkspace.Dy[ 84 ]), &(nmpcWorkspace.QDy[ 56 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 768 ]), &(nmpcWorkspace.Dy[ 96 ]), &(nmpcWorkspace.QDy[ 64 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 864 ]), &(nmpcWorkspace.Dy[ 108 ]), &(nmpcWorkspace.QDy[ 72 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 960 ]), &(nmpcWorkspace.Dy[ 120 ]), &(nmpcWorkspace.QDy[ 80 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1056 ]), &(nmpcWorkspace.Dy[ 132 ]), &(nmpcWorkspace.QDy[ 88 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1152 ]), &(nmpcWorkspace.Dy[ 144 ]), &(nmpcWorkspace.QDy[ 96 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1248 ]), &(nmpcWorkspace.Dy[ 156 ]), &(nmpcWorkspace.QDy[ 104 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1344 ]), &(nmpcWorkspace.Dy[ 168 ]), &(nmpcWorkspace.QDy[ 112 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1440 ]), &(nmpcWorkspace.Dy[ 180 ]), &(nmpcWorkspace.QDy[ 120 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1536 ]), &(nmpcWorkspace.Dy[ 192 ]), &(nmpcWorkspace.QDy[ 128 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1632 ]), &(nmpcWorkspace.Dy[ 204 ]), &(nmpcWorkspace.QDy[ 136 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1728 ]), &(nmpcWorkspace.Dy[ 216 ]), &(nmpcWorkspace.QDy[ 144 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1824 ]), &(nmpcWorkspace.Dy[ 228 ]), &(nmpcWorkspace.QDy[ 152 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1920 ]), &(nmpcWorkspace.Dy[ 240 ]), &(nmpcWorkspace.QDy[ 160 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2016 ]), &(nmpcWorkspace.Dy[ 252 ]), &(nmpcWorkspace.QDy[ 168 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2112 ]), &(nmpcWorkspace.Dy[ 264 ]), &(nmpcWorkspace.QDy[ 176 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2208 ]), &(nmpcWorkspace.Dy[ 276 ]), &(nmpcWorkspace.QDy[ 184 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2304 ]), &(nmpcWorkspace.Dy[ 288 ]), &(nmpcWorkspace.QDy[ 192 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2400 ]), &(nmpcWorkspace.Dy[ 300 ]), &(nmpcWorkspace.QDy[ 200 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2496 ]), &(nmpcWorkspace.Dy[ 312 ]), &(nmpcWorkspace.QDy[ 208 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2592 ]), &(nmpcWorkspace.Dy[ 324 ]), &(nmpcWorkspace.QDy[ 216 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2688 ]), &(nmpcWorkspace.Dy[ 336 ]), &(nmpcWorkspace.QDy[ 224 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2784 ]), &(nmpcWorkspace.Dy[ 348 ]), &(nmpcWorkspace.QDy[ 232 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2880 ]), &(nmpcWorkspace.Dy[ 360 ]), &(nmpcWorkspace.QDy[ 240 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2976 ]), &(nmpcWorkspace.Dy[ 372 ]), &(nmpcWorkspace.QDy[ 248 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3072 ]), &(nmpcWorkspace.Dy[ 384 ]), &(nmpcWorkspace.QDy[ 256 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3168 ]), &(nmpcWorkspace.Dy[ 396 ]), &(nmpcWorkspace.QDy[ 264 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3264 ]), &(nmpcWorkspace.Dy[ 408 ]), &(nmpcWorkspace.QDy[ 272 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3360 ]), &(nmpcWorkspace.Dy[ 420 ]), &(nmpcWorkspace.QDy[ 280 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3456 ]), &(nmpcWorkspace.Dy[ 432 ]), &(nmpcWorkspace.QDy[ 288 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3552 ]), &(nmpcWorkspace.Dy[ 444 ]), &(nmpcWorkspace.QDy[ 296 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3648 ]), &(nmpcWorkspace.Dy[ 456 ]), &(nmpcWorkspace.QDy[ 304 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3744 ]), &(nmpcWorkspace.Dy[ 468 ]), &(nmpcWorkspace.QDy[ 312 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3840 ]), &(nmpcWorkspace.Dy[ 480 ]), &(nmpcWorkspace.QDy[ 320 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 3936 ]), &(nmpcWorkspace.Dy[ 492 ]), &(nmpcWorkspace.QDy[ 328 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4032 ]), &(nmpcWorkspace.Dy[ 504 ]), &(nmpcWorkspace.QDy[ 336 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4128 ]), &(nmpcWorkspace.Dy[ 516 ]), &(nmpcWorkspace.QDy[ 344 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4224 ]), &(nmpcWorkspace.Dy[ 528 ]), &(nmpcWorkspace.QDy[ 352 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4320 ]), &(nmpcWorkspace.Dy[ 540 ]), &(nmpcWorkspace.QDy[ 360 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4416 ]), &(nmpcWorkspace.Dy[ 552 ]), &(nmpcWorkspace.QDy[ 368 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4512 ]), &(nmpcWorkspace.Dy[ 564 ]), &(nmpcWorkspace.QDy[ 376 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4608 ]), &(nmpcWorkspace.Dy[ 576 ]), &(nmpcWorkspace.QDy[ 384 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4704 ]), &(nmpcWorkspace.Dy[ 588 ]), &(nmpcWorkspace.QDy[ 392 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4800 ]), &(nmpcWorkspace.Dy[ 600 ]), &(nmpcWorkspace.QDy[ 400 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4896 ]), &(nmpcWorkspace.Dy[ 612 ]), &(nmpcWorkspace.QDy[ 408 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 4992 ]), &(nmpcWorkspace.Dy[ 624 ]), &(nmpcWorkspace.QDy[ 416 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 5088 ]), &(nmpcWorkspace.Dy[ 636 ]), &(nmpcWorkspace.QDy[ 424 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 5184 ]), &(nmpcWorkspace.Dy[ 648 ]), &(nmpcWorkspace.QDy[ 432 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 5280 ]), &(nmpcWorkspace.Dy[ 660 ]), &(nmpcWorkspace.QDy[ 440 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 5376 ]), &(nmpcWorkspace.Dy[ 672 ]), &(nmpcWorkspace.QDy[ 448 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 5472 ]), &(nmpcWorkspace.Dy[ 684 ]), &(nmpcWorkspace.QDy[ 456 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 5568 ]), &(nmpcWorkspace.Dy[ 696 ]), &(nmpcWorkspace.QDy[ 464 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 5664 ]), &(nmpcWorkspace.Dy[ 708 ]), &(nmpcWorkspace.QDy[ 472 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 5760 ]), &(nmpcWorkspace.Dy[ 720 ]), &(nmpcWorkspace.QDy[ 480 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 5856 ]), &(nmpcWorkspace.Dy[ 732 ]), &(nmpcWorkspace.QDy[ 488 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 5952 ]), &(nmpcWorkspace.Dy[ 744 ]), &(nmpcWorkspace.QDy[ 496 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 6048 ]), &(nmpcWorkspace.Dy[ 756 ]), &(nmpcWorkspace.QDy[ 504 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 6144 ]), &(nmpcWorkspace.Dy[ 768 ]), &(nmpcWorkspace.QDy[ 512 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 6240 ]), &(nmpcWorkspace.Dy[ 780 ]), &(nmpcWorkspace.QDy[ 520 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 6336 ]), &(nmpcWorkspace.Dy[ 792 ]), &(nmpcWorkspace.QDy[ 528 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 6432 ]), &(nmpcWorkspace.Dy[ 804 ]), &(nmpcWorkspace.QDy[ 536 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 6528 ]), &(nmpcWorkspace.Dy[ 816 ]), &(nmpcWorkspace.QDy[ 544 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 6624 ]), &(nmpcWorkspace.Dy[ 828 ]), &(nmpcWorkspace.QDy[ 552 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 6720 ]), &(nmpcWorkspace.Dy[ 840 ]), &(nmpcWorkspace.QDy[ 560 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 6816 ]), &(nmpcWorkspace.Dy[ 852 ]), &(nmpcWorkspace.QDy[ 568 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 6912 ]), &(nmpcWorkspace.Dy[ 864 ]), &(nmpcWorkspace.QDy[ 576 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 7008 ]), &(nmpcWorkspace.Dy[ 876 ]), &(nmpcWorkspace.QDy[ 584 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 7104 ]), &(nmpcWorkspace.Dy[ 888 ]), &(nmpcWorkspace.QDy[ 592 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 7200 ]), &(nmpcWorkspace.Dy[ 900 ]), &(nmpcWorkspace.QDy[ 600 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 7296 ]), &(nmpcWorkspace.Dy[ 912 ]), &(nmpcWorkspace.QDy[ 608 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 7392 ]), &(nmpcWorkspace.Dy[ 924 ]), &(nmpcWorkspace.QDy[ 616 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 7488 ]), &(nmpcWorkspace.Dy[ 936 ]), &(nmpcWorkspace.QDy[ 624 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 7584 ]), &(nmpcWorkspace.Dy[ 948 ]), &(nmpcWorkspace.QDy[ 632 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 7680 ]), &(nmpcWorkspace.Dy[ 960 ]), &(nmpcWorkspace.QDy[ 640 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 7776 ]), &(nmpcWorkspace.Dy[ 972 ]), &(nmpcWorkspace.QDy[ 648 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 7872 ]), &(nmpcWorkspace.Dy[ 984 ]), &(nmpcWorkspace.QDy[ 656 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 7968 ]), &(nmpcWorkspace.Dy[ 996 ]), &(nmpcWorkspace.QDy[ 664 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 8064 ]), &(nmpcWorkspace.Dy[ 1008 ]), &(nmpcWorkspace.QDy[ 672 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 8160 ]), &(nmpcWorkspace.Dy[ 1020 ]), &(nmpcWorkspace.QDy[ 680 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 8256 ]), &(nmpcWorkspace.Dy[ 1032 ]), &(nmpcWorkspace.QDy[ 688 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 8352 ]), &(nmpcWorkspace.Dy[ 1044 ]), &(nmpcWorkspace.QDy[ 696 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 8448 ]), &(nmpcWorkspace.Dy[ 1056 ]), &(nmpcWorkspace.QDy[ 704 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 8544 ]), &(nmpcWorkspace.Dy[ 1068 ]), &(nmpcWorkspace.QDy[ 712 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 8640 ]), &(nmpcWorkspace.Dy[ 1080 ]), &(nmpcWorkspace.QDy[ 720 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 8736 ]), &(nmpcWorkspace.Dy[ 1092 ]), &(nmpcWorkspace.QDy[ 728 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 8832 ]), &(nmpcWorkspace.Dy[ 1104 ]), &(nmpcWorkspace.QDy[ 736 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 8928 ]), &(nmpcWorkspace.Dy[ 1116 ]), &(nmpcWorkspace.QDy[ 744 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 9024 ]), &(nmpcWorkspace.Dy[ 1128 ]), &(nmpcWorkspace.QDy[ 752 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 9120 ]), &(nmpcWorkspace.Dy[ 1140 ]), &(nmpcWorkspace.QDy[ 760 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 9216 ]), &(nmpcWorkspace.Dy[ 1152 ]), &(nmpcWorkspace.QDy[ 768 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 9312 ]), &(nmpcWorkspace.Dy[ 1164 ]), &(nmpcWorkspace.QDy[ 776 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 9408 ]), &(nmpcWorkspace.Dy[ 1176 ]), &(nmpcWorkspace.QDy[ 784 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 9504 ]), &(nmpcWorkspace.Dy[ 1188 ]), &(nmpcWorkspace.QDy[ 792 ]) );

nmpcWorkspace.QDy[800] = + nmpcWorkspace.QN2[0]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[1]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[2]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[3]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[4]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[5]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[6]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[7]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[801] = + nmpcWorkspace.QN2[8]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[9]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[10]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[11]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[12]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[13]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[14]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[15]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[802] = + nmpcWorkspace.QN2[16]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[17]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[18]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[19]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[20]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[21]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[22]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[23]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[803] = + nmpcWorkspace.QN2[24]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[25]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[26]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[27]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[28]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[29]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[30]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[31]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[804] = + nmpcWorkspace.QN2[32]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[33]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[34]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[35]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[36]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[37]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[38]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[39]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[805] = + nmpcWorkspace.QN2[40]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[41]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[42]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[43]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[44]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[45]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[46]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[47]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[806] = + nmpcWorkspace.QN2[48]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[49]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[50]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[51]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[52]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[53]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[54]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[55]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[807] = + nmpcWorkspace.QN2[56]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[57]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[58]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[59]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[60]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[61]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[62]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[63]*nmpcWorkspace.DyN[7];

for (lRun2 = 0; lRun2 < 800; ++lRun2)
nmpcWorkspace.QDy[lRun2 + 8] += nmpcWorkspace.Qd[lRun2];


for (lRun2 = 0; lRun2 < 8; ++lRun2)
{
for (lRun4 = 0; lRun4 < 1; ++lRun4)
{
real_t t = 0.0;
for (lRun5 = 0; lRun5 < 800; ++lRun5)
{
t += + nmpcWorkspace.evGx[(lRun5 * 8) + (lRun2)]*nmpcWorkspace.QDy[(lRun5 + 8) + (lRun4)];
}
nmpcWorkspace.g[(lRun2) + (lRun4)] = t;
}
}


for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 100; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_multEQDy( &(nmpcWorkspace.E[ lRun3 * 32 ]), &(nmpcWorkspace.QDy[ lRun2 * 8 + 8 ]), &(nmpcWorkspace.g[ lRun1 * 4 + 8 ]) );
}
}

nmpcWorkspace.lb[0] = nmpcWorkspace.Dx0[0];
nmpcWorkspace.lb[1] = nmpcWorkspace.Dx0[1];
nmpcWorkspace.lb[2] = nmpcWorkspace.Dx0[2];
nmpcWorkspace.lb[3] = nmpcWorkspace.Dx0[3];
nmpcWorkspace.lb[4] = nmpcWorkspace.Dx0[4];
nmpcWorkspace.lb[5] = nmpcWorkspace.Dx0[5];
nmpcWorkspace.lb[6] = nmpcWorkspace.Dx0[6];
nmpcWorkspace.lb[7] = nmpcWorkspace.Dx0[7];
nmpcWorkspace.ub[0] = nmpcWorkspace.Dx0[0];
nmpcWorkspace.ub[1] = nmpcWorkspace.Dx0[1];
nmpcWorkspace.ub[2] = nmpcWorkspace.Dx0[2];
nmpcWorkspace.ub[3] = nmpcWorkspace.Dx0[3];
nmpcWorkspace.ub[4] = nmpcWorkspace.Dx0[4];
nmpcWorkspace.ub[5] = nmpcWorkspace.Dx0[5];
nmpcWorkspace.ub[6] = nmpcWorkspace.Dx0[6];
nmpcWorkspace.ub[7] = nmpcWorkspace.Dx0[7];
}

void nmpc_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
nmpcVariables.x[0] += nmpcWorkspace.x[0];
nmpcVariables.x[1] += nmpcWorkspace.x[1];
nmpcVariables.x[2] += nmpcWorkspace.x[2];
nmpcVariables.x[3] += nmpcWorkspace.x[3];
nmpcVariables.x[4] += nmpcWorkspace.x[4];
nmpcVariables.x[5] += nmpcWorkspace.x[5];
nmpcVariables.x[6] += nmpcWorkspace.x[6];
nmpcVariables.x[7] += nmpcWorkspace.x[7];

for (lRun1 = 0; lRun1 < 400; ++lRun1)
nmpcVariables.u[lRun1] += nmpcWorkspace.x[lRun1 + 8];


for (lRun1 = 0; lRun1 < 800; ++lRun1)
{
for (lRun2 = 0; lRun2 < 1; ++lRun2)
{
real_t t = 0.0;
for (lRun3 = 0; lRun3 < 8; ++lRun3)
{
t += + nmpcWorkspace.evGx[(lRun1 * 8) + (lRun3)]*nmpcWorkspace.x[(lRun3) + (lRun2)];
}
nmpcVariables.x[(lRun1 + 8) + (lRun2)] += t + nmpcWorkspace.d[(lRun1) + (lRun2)];
}
}

for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multEDu( &(nmpcWorkspace.E[ lRun3 * 32 ]), &(nmpcWorkspace.x[ lRun2 * 4 + 8 ]), &(nmpcVariables.x[ lRun1 * 8 + 8 ]) );
}
}
}

int nmpc_preparationStep(  )
{
int ret;

ret = nmpc_modelSimulation();
nmpc_evaluateObjective(  );
nmpc_condensePrep(  );
return ret;
}

int nmpc_feedbackStep(  )
{
int tmp;

nmpc_condenseFdb(  );

tmp = nmpc_solve( );

nmpc_expand(  );
return tmp;
}

int nmpc_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&nmpcWorkspace, 0, sizeof( nmpcWorkspace ));
return ret;
}

void nmpc_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 100; ++index)
{
nmpcWorkspace.state[0] = nmpcVariables.x[index * 8];
nmpcWorkspace.state[1] = nmpcVariables.x[index * 8 + 1];
nmpcWorkspace.state[2] = nmpcVariables.x[index * 8 + 2];
nmpcWorkspace.state[3] = nmpcVariables.x[index * 8 + 3];
nmpcWorkspace.state[4] = nmpcVariables.x[index * 8 + 4];
nmpcWorkspace.state[5] = nmpcVariables.x[index * 8 + 5];
nmpcWorkspace.state[6] = nmpcVariables.x[index * 8 + 6];
nmpcWorkspace.state[7] = nmpcVariables.x[index * 8 + 7];
nmpcWorkspace.state[104] = nmpcVariables.u[index * 4];
nmpcWorkspace.state[105] = nmpcVariables.u[index * 4 + 1];
nmpcWorkspace.state[106] = nmpcVariables.u[index * 4 + 2];
nmpcWorkspace.state[107] = nmpcVariables.u[index * 4 + 3];
nmpcWorkspace.state[108] = nmpcVariables.od[index * 3];
nmpcWorkspace.state[109] = nmpcVariables.od[index * 3 + 1];
nmpcWorkspace.state[110] = nmpcVariables.od[index * 3 + 2];

nmpc_integrate(nmpcWorkspace.state, index == 0);

nmpcVariables.x[index * 8 + 8] = nmpcWorkspace.state[0];
nmpcVariables.x[index * 8 + 9] = nmpcWorkspace.state[1];
nmpcVariables.x[index * 8 + 10] = nmpcWorkspace.state[2];
nmpcVariables.x[index * 8 + 11] = nmpcWorkspace.state[3];
nmpcVariables.x[index * 8 + 12] = nmpcWorkspace.state[4];
nmpcVariables.x[index * 8 + 13] = nmpcWorkspace.state[5];
nmpcVariables.x[index * 8 + 14] = nmpcWorkspace.state[6];
nmpcVariables.x[index * 8 + 15] = nmpcWorkspace.state[7];
}
}

void nmpc_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 100; ++index)
{
nmpcVariables.x[index * 8] = nmpcVariables.x[index * 8 + 8];
nmpcVariables.x[index * 8 + 1] = nmpcVariables.x[index * 8 + 9];
nmpcVariables.x[index * 8 + 2] = nmpcVariables.x[index * 8 + 10];
nmpcVariables.x[index * 8 + 3] = nmpcVariables.x[index * 8 + 11];
nmpcVariables.x[index * 8 + 4] = nmpcVariables.x[index * 8 + 12];
nmpcVariables.x[index * 8 + 5] = nmpcVariables.x[index * 8 + 13];
nmpcVariables.x[index * 8 + 6] = nmpcVariables.x[index * 8 + 14];
nmpcVariables.x[index * 8 + 7] = nmpcVariables.x[index * 8 + 15];
}

if (strategy == 1 && xEnd != 0)
{
nmpcVariables.x[800] = xEnd[0];
nmpcVariables.x[801] = xEnd[1];
nmpcVariables.x[802] = xEnd[2];
nmpcVariables.x[803] = xEnd[3];
nmpcVariables.x[804] = xEnd[4];
nmpcVariables.x[805] = xEnd[5];
nmpcVariables.x[806] = xEnd[6];
nmpcVariables.x[807] = xEnd[7];
}
else if (strategy == 2) 
{
nmpcWorkspace.state[0] = nmpcVariables.x[800];
nmpcWorkspace.state[1] = nmpcVariables.x[801];
nmpcWorkspace.state[2] = nmpcVariables.x[802];
nmpcWorkspace.state[3] = nmpcVariables.x[803];
nmpcWorkspace.state[4] = nmpcVariables.x[804];
nmpcWorkspace.state[5] = nmpcVariables.x[805];
nmpcWorkspace.state[6] = nmpcVariables.x[806];
nmpcWorkspace.state[7] = nmpcVariables.x[807];
if (uEnd != 0)
{
nmpcWorkspace.state[104] = uEnd[0];
nmpcWorkspace.state[105] = uEnd[1];
nmpcWorkspace.state[106] = uEnd[2];
nmpcWorkspace.state[107] = uEnd[3];
}
else
{
nmpcWorkspace.state[104] = nmpcVariables.u[396];
nmpcWorkspace.state[105] = nmpcVariables.u[397];
nmpcWorkspace.state[106] = nmpcVariables.u[398];
nmpcWorkspace.state[107] = nmpcVariables.u[399];
}
nmpcWorkspace.state[108] = nmpcVariables.od[300];
nmpcWorkspace.state[109] = nmpcVariables.od[301];
nmpcWorkspace.state[110] = nmpcVariables.od[302];

nmpc_integrate(nmpcWorkspace.state, 1);

nmpcVariables.x[800] = nmpcWorkspace.state[0];
nmpcVariables.x[801] = nmpcWorkspace.state[1];
nmpcVariables.x[802] = nmpcWorkspace.state[2];
nmpcVariables.x[803] = nmpcWorkspace.state[3];
nmpcVariables.x[804] = nmpcWorkspace.state[4];
nmpcVariables.x[805] = nmpcWorkspace.state[5];
nmpcVariables.x[806] = nmpcWorkspace.state[6];
nmpcVariables.x[807] = nmpcWorkspace.state[7];
}
}

void nmpc_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 99; ++index)
{
nmpcVariables.u[index * 4] = nmpcVariables.u[index * 4 + 4];
nmpcVariables.u[index * 4 + 1] = nmpcVariables.u[index * 4 + 5];
nmpcVariables.u[index * 4 + 2] = nmpcVariables.u[index * 4 + 6];
nmpcVariables.u[index * 4 + 3] = nmpcVariables.u[index * 4 + 7];
}

if (uEnd != 0)
{
nmpcVariables.u[396] = uEnd[0];
nmpcVariables.u[397] = uEnd[1];
nmpcVariables.u[398] = uEnd[2];
nmpcVariables.u[399] = uEnd[3];
}
}

real_t nmpc_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + nmpcWorkspace.g[0]*nmpcWorkspace.x[0] + nmpcWorkspace.g[1]*nmpcWorkspace.x[1] + nmpcWorkspace.g[2]*nmpcWorkspace.x[2] + nmpcWorkspace.g[3]*nmpcWorkspace.x[3] + nmpcWorkspace.g[4]*nmpcWorkspace.x[4] + nmpcWorkspace.g[5]*nmpcWorkspace.x[5] + nmpcWorkspace.g[6]*nmpcWorkspace.x[6] + nmpcWorkspace.g[7]*nmpcWorkspace.x[7] + nmpcWorkspace.g[8]*nmpcWorkspace.x[8] + nmpcWorkspace.g[9]*nmpcWorkspace.x[9] + nmpcWorkspace.g[10]*nmpcWorkspace.x[10] + nmpcWorkspace.g[11]*nmpcWorkspace.x[11] + nmpcWorkspace.g[12]*nmpcWorkspace.x[12] + nmpcWorkspace.g[13]*nmpcWorkspace.x[13] + nmpcWorkspace.g[14]*nmpcWorkspace.x[14] + nmpcWorkspace.g[15]*nmpcWorkspace.x[15] + nmpcWorkspace.g[16]*nmpcWorkspace.x[16] + nmpcWorkspace.g[17]*nmpcWorkspace.x[17] + nmpcWorkspace.g[18]*nmpcWorkspace.x[18] + nmpcWorkspace.g[19]*nmpcWorkspace.x[19] + nmpcWorkspace.g[20]*nmpcWorkspace.x[20] + nmpcWorkspace.g[21]*nmpcWorkspace.x[21] + nmpcWorkspace.g[22]*nmpcWorkspace.x[22] + nmpcWorkspace.g[23]*nmpcWorkspace.x[23] + nmpcWorkspace.g[24]*nmpcWorkspace.x[24] + nmpcWorkspace.g[25]*nmpcWorkspace.x[25] + nmpcWorkspace.g[26]*nmpcWorkspace.x[26] + nmpcWorkspace.g[27]*nmpcWorkspace.x[27] + nmpcWorkspace.g[28]*nmpcWorkspace.x[28] + nmpcWorkspace.g[29]*nmpcWorkspace.x[29] + nmpcWorkspace.g[30]*nmpcWorkspace.x[30] + nmpcWorkspace.g[31]*nmpcWorkspace.x[31] + nmpcWorkspace.g[32]*nmpcWorkspace.x[32] + nmpcWorkspace.g[33]*nmpcWorkspace.x[33] + nmpcWorkspace.g[34]*nmpcWorkspace.x[34] + nmpcWorkspace.g[35]*nmpcWorkspace.x[35] + nmpcWorkspace.g[36]*nmpcWorkspace.x[36] + nmpcWorkspace.g[37]*nmpcWorkspace.x[37] + nmpcWorkspace.g[38]*nmpcWorkspace.x[38] + nmpcWorkspace.g[39]*nmpcWorkspace.x[39] + nmpcWorkspace.g[40]*nmpcWorkspace.x[40] + nmpcWorkspace.g[41]*nmpcWorkspace.x[41] + nmpcWorkspace.g[42]*nmpcWorkspace.x[42] + nmpcWorkspace.g[43]*nmpcWorkspace.x[43] + nmpcWorkspace.g[44]*nmpcWorkspace.x[44] + nmpcWorkspace.g[45]*nmpcWorkspace.x[45] + nmpcWorkspace.g[46]*nmpcWorkspace.x[46] + nmpcWorkspace.g[47]*nmpcWorkspace.x[47] + nmpcWorkspace.g[48]*nmpcWorkspace.x[48] + nmpcWorkspace.g[49]*nmpcWorkspace.x[49] + nmpcWorkspace.g[50]*nmpcWorkspace.x[50] + nmpcWorkspace.g[51]*nmpcWorkspace.x[51] + nmpcWorkspace.g[52]*nmpcWorkspace.x[52] + nmpcWorkspace.g[53]*nmpcWorkspace.x[53] + nmpcWorkspace.g[54]*nmpcWorkspace.x[54] + nmpcWorkspace.g[55]*nmpcWorkspace.x[55] + nmpcWorkspace.g[56]*nmpcWorkspace.x[56] + nmpcWorkspace.g[57]*nmpcWorkspace.x[57] + nmpcWorkspace.g[58]*nmpcWorkspace.x[58] + nmpcWorkspace.g[59]*nmpcWorkspace.x[59] + nmpcWorkspace.g[60]*nmpcWorkspace.x[60] + nmpcWorkspace.g[61]*nmpcWorkspace.x[61] + nmpcWorkspace.g[62]*nmpcWorkspace.x[62] + nmpcWorkspace.g[63]*nmpcWorkspace.x[63] + nmpcWorkspace.g[64]*nmpcWorkspace.x[64] + nmpcWorkspace.g[65]*nmpcWorkspace.x[65] + nmpcWorkspace.g[66]*nmpcWorkspace.x[66] + nmpcWorkspace.g[67]*nmpcWorkspace.x[67] + nmpcWorkspace.g[68]*nmpcWorkspace.x[68] + nmpcWorkspace.g[69]*nmpcWorkspace.x[69] + nmpcWorkspace.g[70]*nmpcWorkspace.x[70] + nmpcWorkspace.g[71]*nmpcWorkspace.x[71] + nmpcWorkspace.g[72]*nmpcWorkspace.x[72] + nmpcWorkspace.g[73]*nmpcWorkspace.x[73] + nmpcWorkspace.g[74]*nmpcWorkspace.x[74] + nmpcWorkspace.g[75]*nmpcWorkspace.x[75] + nmpcWorkspace.g[76]*nmpcWorkspace.x[76] + nmpcWorkspace.g[77]*nmpcWorkspace.x[77] + nmpcWorkspace.g[78]*nmpcWorkspace.x[78] + nmpcWorkspace.g[79]*nmpcWorkspace.x[79] + nmpcWorkspace.g[80]*nmpcWorkspace.x[80] + nmpcWorkspace.g[81]*nmpcWorkspace.x[81] + nmpcWorkspace.g[82]*nmpcWorkspace.x[82] + nmpcWorkspace.g[83]*nmpcWorkspace.x[83] + nmpcWorkspace.g[84]*nmpcWorkspace.x[84] + nmpcWorkspace.g[85]*nmpcWorkspace.x[85] + nmpcWorkspace.g[86]*nmpcWorkspace.x[86] + nmpcWorkspace.g[87]*nmpcWorkspace.x[87] + nmpcWorkspace.g[88]*nmpcWorkspace.x[88] + nmpcWorkspace.g[89]*nmpcWorkspace.x[89] + nmpcWorkspace.g[90]*nmpcWorkspace.x[90] + nmpcWorkspace.g[91]*nmpcWorkspace.x[91] + nmpcWorkspace.g[92]*nmpcWorkspace.x[92] + nmpcWorkspace.g[93]*nmpcWorkspace.x[93] + nmpcWorkspace.g[94]*nmpcWorkspace.x[94] + nmpcWorkspace.g[95]*nmpcWorkspace.x[95] + nmpcWorkspace.g[96]*nmpcWorkspace.x[96] + nmpcWorkspace.g[97]*nmpcWorkspace.x[97] + nmpcWorkspace.g[98]*nmpcWorkspace.x[98] + nmpcWorkspace.g[99]*nmpcWorkspace.x[99] + nmpcWorkspace.g[100]*nmpcWorkspace.x[100] + nmpcWorkspace.g[101]*nmpcWorkspace.x[101] + nmpcWorkspace.g[102]*nmpcWorkspace.x[102] + nmpcWorkspace.g[103]*nmpcWorkspace.x[103] + nmpcWorkspace.g[104]*nmpcWorkspace.x[104] + nmpcWorkspace.g[105]*nmpcWorkspace.x[105] + nmpcWorkspace.g[106]*nmpcWorkspace.x[106] + nmpcWorkspace.g[107]*nmpcWorkspace.x[107] + nmpcWorkspace.g[108]*nmpcWorkspace.x[108] + nmpcWorkspace.g[109]*nmpcWorkspace.x[109] + nmpcWorkspace.g[110]*nmpcWorkspace.x[110] + nmpcWorkspace.g[111]*nmpcWorkspace.x[111] + nmpcWorkspace.g[112]*nmpcWorkspace.x[112] + nmpcWorkspace.g[113]*nmpcWorkspace.x[113] + nmpcWorkspace.g[114]*nmpcWorkspace.x[114] + nmpcWorkspace.g[115]*nmpcWorkspace.x[115] + nmpcWorkspace.g[116]*nmpcWorkspace.x[116] + nmpcWorkspace.g[117]*nmpcWorkspace.x[117] + nmpcWorkspace.g[118]*nmpcWorkspace.x[118] + nmpcWorkspace.g[119]*nmpcWorkspace.x[119] + nmpcWorkspace.g[120]*nmpcWorkspace.x[120] + nmpcWorkspace.g[121]*nmpcWorkspace.x[121] + nmpcWorkspace.g[122]*nmpcWorkspace.x[122] + nmpcWorkspace.g[123]*nmpcWorkspace.x[123] + nmpcWorkspace.g[124]*nmpcWorkspace.x[124] + nmpcWorkspace.g[125]*nmpcWorkspace.x[125] + nmpcWorkspace.g[126]*nmpcWorkspace.x[126] + nmpcWorkspace.g[127]*nmpcWorkspace.x[127] + nmpcWorkspace.g[128]*nmpcWorkspace.x[128] + nmpcWorkspace.g[129]*nmpcWorkspace.x[129] + nmpcWorkspace.g[130]*nmpcWorkspace.x[130] + nmpcWorkspace.g[131]*nmpcWorkspace.x[131] + nmpcWorkspace.g[132]*nmpcWorkspace.x[132] + nmpcWorkspace.g[133]*nmpcWorkspace.x[133] + nmpcWorkspace.g[134]*nmpcWorkspace.x[134] + nmpcWorkspace.g[135]*nmpcWorkspace.x[135] + nmpcWorkspace.g[136]*nmpcWorkspace.x[136] + nmpcWorkspace.g[137]*nmpcWorkspace.x[137] + nmpcWorkspace.g[138]*nmpcWorkspace.x[138] + nmpcWorkspace.g[139]*nmpcWorkspace.x[139] + nmpcWorkspace.g[140]*nmpcWorkspace.x[140] + nmpcWorkspace.g[141]*nmpcWorkspace.x[141] + nmpcWorkspace.g[142]*nmpcWorkspace.x[142] + nmpcWorkspace.g[143]*nmpcWorkspace.x[143] + nmpcWorkspace.g[144]*nmpcWorkspace.x[144] + nmpcWorkspace.g[145]*nmpcWorkspace.x[145] + nmpcWorkspace.g[146]*nmpcWorkspace.x[146] + nmpcWorkspace.g[147]*nmpcWorkspace.x[147] + nmpcWorkspace.g[148]*nmpcWorkspace.x[148] + nmpcWorkspace.g[149]*nmpcWorkspace.x[149] + nmpcWorkspace.g[150]*nmpcWorkspace.x[150] + nmpcWorkspace.g[151]*nmpcWorkspace.x[151] + nmpcWorkspace.g[152]*nmpcWorkspace.x[152] + nmpcWorkspace.g[153]*nmpcWorkspace.x[153] + nmpcWorkspace.g[154]*nmpcWorkspace.x[154] + nmpcWorkspace.g[155]*nmpcWorkspace.x[155] + nmpcWorkspace.g[156]*nmpcWorkspace.x[156] + nmpcWorkspace.g[157]*nmpcWorkspace.x[157] + nmpcWorkspace.g[158]*nmpcWorkspace.x[158] + nmpcWorkspace.g[159]*nmpcWorkspace.x[159] + nmpcWorkspace.g[160]*nmpcWorkspace.x[160] + nmpcWorkspace.g[161]*nmpcWorkspace.x[161] + nmpcWorkspace.g[162]*nmpcWorkspace.x[162] + nmpcWorkspace.g[163]*nmpcWorkspace.x[163] + nmpcWorkspace.g[164]*nmpcWorkspace.x[164] + nmpcWorkspace.g[165]*nmpcWorkspace.x[165] + nmpcWorkspace.g[166]*nmpcWorkspace.x[166] + nmpcWorkspace.g[167]*nmpcWorkspace.x[167] + nmpcWorkspace.g[168]*nmpcWorkspace.x[168] + nmpcWorkspace.g[169]*nmpcWorkspace.x[169] + nmpcWorkspace.g[170]*nmpcWorkspace.x[170] + nmpcWorkspace.g[171]*nmpcWorkspace.x[171] + nmpcWorkspace.g[172]*nmpcWorkspace.x[172] + nmpcWorkspace.g[173]*nmpcWorkspace.x[173] + nmpcWorkspace.g[174]*nmpcWorkspace.x[174] + nmpcWorkspace.g[175]*nmpcWorkspace.x[175] + nmpcWorkspace.g[176]*nmpcWorkspace.x[176] + nmpcWorkspace.g[177]*nmpcWorkspace.x[177] + nmpcWorkspace.g[178]*nmpcWorkspace.x[178] + nmpcWorkspace.g[179]*nmpcWorkspace.x[179] + nmpcWorkspace.g[180]*nmpcWorkspace.x[180] + nmpcWorkspace.g[181]*nmpcWorkspace.x[181] + nmpcWorkspace.g[182]*nmpcWorkspace.x[182] + nmpcWorkspace.g[183]*nmpcWorkspace.x[183] + nmpcWorkspace.g[184]*nmpcWorkspace.x[184] + nmpcWorkspace.g[185]*nmpcWorkspace.x[185] + nmpcWorkspace.g[186]*nmpcWorkspace.x[186] + nmpcWorkspace.g[187]*nmpcWorkspace.x[187] + nmpcWorkspace.g[188]*nmpcWorkspace.x[188] + nmpcWorkspace.g[189]*nmpcWorkspace.x[189] + nmpcWorkspace.g[190]*nmpcWorkspace.x[190] + nmpcWorkspace.g[191]*nmpcWorkspace.x[191] + nmpcWorkspace.g[192]*nmpcWorkspace.x[192] + nmpcWorkspace.g[193]*nmpcWorkspace.x[193] + nmpcWorkspace.g[194]*nmpcWorkspace.x[194] + nmpcWorkspace.g[195]*nmpcWorkspace.x[195] + nmpcWorkspace.g[196]*nmpcWorkspace.x[196] + nmpcWorkspace.g[197]*nmpcWorkspace.x[197] + nmpcWorkspace.g[198]*nmpcWorkspace.x[198] + nmpcWorkspace.g[199]*nmpcWorkspace.x[199] + nmpcWorkspace.g[200]*nmpcWorkspace.x[200] + nmpcWorkspace.g[201]*nmpcWorkspace.x[201] + nmpcWorkspace.g[202]*nmpcWorkspace.x[202] + nmpcWorkspace.g[203]*nmpcWorkspace.x[203] + nmpcWorkspace.g[204]*nmpcWorkspace.x[204] + nmpcWorkspace.g[205]*nmpcWorkspace.x[205] + nmpcWorkspace.g[206]*nmpcWorkspace.x[206] + nmpcWorkspace.g[207]*nmpcWorkspace.x[207] + nmpcWorkspace.g[208]*nmpcWorkspace.x[208] + nmpcWorkspace.g[209]*nmpcWorkspace.x[209] + nmpcWorkspace.g[210]*nmpcWorkspace.x[210] + nmpcWorkspace.g[211]*nmpcWorkspace.x[211] + nmpcWorkspace.g[212]*nmpcWorkspace.x[212] + nmpcWorkspace.g[213]*nmpcWorkspace.x[213] + nmpcWorkspace.g[214]*nmpcWorkspace.x[214] + nmpcWorkspace.g[215]*nmpcWorkspace.x[215] + nmpcWorkspace.g[216]*nmpcWorkspace.x[216] + nmpcWorkspace.g[217]*nmpcWorkspace.x[217] + nmpcWorkspace.g[218]*nmpcWorkspace.x[218] + nmpcWorkspace.g[219]*nmpcWorkspace.x[219] + nmpcWorkspace.g[220]*nmpcWorkspace.x[220] + nmpcWorkspace.g[221]*nmpcWorkspace.x[221] + nmpcWorkspace.g[222]*nmpcWorkspace.x[222] + nmpcWorkspace.g[223]*nmpcWorkspace.x[223] + nmpcWorkspace.g[224]*nmpcWorkspace.x[224] + nmpcWorkspace.g[225]*nmpcWorkspace.x[225] + nmpcWorkspace.g[226]*nmpcWorkspace.x[226] + nmpcWorkspace.g[227]*nmpcWorkspace.x[227] + nmpcWorkspace.g[228]*nmpcWorkspace.x[228] + nmpcWorkspace.g[229]*nmpcWorkspace.x[229] + nmpcWorkspace.g[230]*nmpcWorkspace.x[230] + nmpcWorkspace.g[231]*nmpcWorkspace.x[231] + nmpcWorkspace.g[232]*nmpcWorkspace.x[232] + nmpcWorkspace.g[233]*nmpcWorkspace.x[233] + nmpcWorkspace.g[234]*nmpcWorkspace.x[234] + nmpcWorkspace.g[235]*nmpcWorkspace.x[235] + nmpcWorkspace.g[236]*nmpcWorkspace.x[236] + nmpcWorkspace.g[237]*nmpcWorkspace.x[237] + nmpcWorkspace.g[238]*nmpcWorkspace.x[238] + nmpcWorkspace.g[239]*nmpcWorkspace.x[239] + nmpcWorkspace.g[240]*nmpcWorkspace.x[240] + nmpcWorkspace.g[241]*nmpcWorkspace.x[241] + nmpcWorkspace.g[242]*nmpcWorkspace.x[242] + nmpcWorkspace.g[243]*nmpcWorkspace.x[243] + nmpcWorkspace.g[244]*nmpcWorkspace.x[244] + nmpcWorkspace.g[245]*nmpcWorkspace.x[245] + nmpcWorkspace.g[246]*nmpcWorkspace.x[246] + nmpcWorkspace.g[247]*nmpcWorkspace.x[247] + nmpcWorkspace.g[248]*nmpcWorkspace.x[248] + nmpcWorkspace.g[249]*nmpcWorkspace.x[249] + nmpcWorkspace.g[250]*nmpcWorkspace.x[250] + nmpcWorkspace.g[251]*nmpcWorkspace.x[251] + nmpcWorkspace.g[252]*nmpcWorkspace.x[252] + nmpcWorkspace.g[253]*nmpcWorkspace.x[253] + nmpcWorkspace.g[254]*nmpcWorkspace.x[254] + nmpcWorkspace.g[255]*nmpcWorkspace.x[255] + nmpcWorkspace.g[256]*nmpcWorkspace.x[256] + nmpcWorkspace.g[257]*nmpcWorkspace.x[257] + nmpcWorkspace.g[258]*nmpcWorkspace.x[258] + nmpcWorkspace.g[259]*nmpcWorkspace.x[259] + nmpcWorkspace.g[260]*nmpcWorkspace.x[260] + nmpcWorkspace.g[261]*nmpcWorkspace.x[261] + nmpcWorkspace.g[262]*nmpcWorkspace.x[262] + nmpcWorkspace.g[263]*nmpcWorkspace.x[263] + nmpcWorkspace.g[264]*nmpcWorkspace.x[264] + nmpcWorkspace.g[265]*nmpcWorkspace.x[265] + nmpcWorkspace.g[266]*nmpcWorkspace.x[266] + nmpcWorkspace.g[267]*nmpcWorkspace.x[267] + nmpcWorkspace.g[268]*nmpcWorkspace.x[268] + nmpcWorkspace.g[269]*nmpcWorkspace.x[269] + nmpcWorkspace.g[270]*nmpcWorkspace.x[270] + nmpcWorkspace.g[271]*nmpcWorkspace.x[271] + nmpcWorkspace.g[272]*nmpcWorkspace.x[272] + nmpcWorkspace.g[273]*nmpcWorkspace.x[273] + nmpcWorkspace.g[274]*nmpcWorkspace.x[274] + nmpcWorkspace.g[275]*nmpcWorkspace.x[275] + nmpcWorkspace.g[276]*nmpcWorkspace.x[276] + nmpcWorkspace.g[277]*nmpcWorkspace.x[277] + nmpcWorkspace.g[278]*nmpcWorkspace.x[278] + nmpcWorkspace.g[279]*nmpcWorkspace.x[279] + nmpcWorkspace.g[280]*nmpcWorkspace.x[280] + nmpcWorkspace.g[281]*nmpcWorkspace.x[281] + nmpcWorkspace.g[282]*nmpcWorkspace.x[282] + nmpcWorkspace.g[283]*nmpcWorkspace.x[283] + nmpcWorkspace.g[284]*nmpcWorkspace.x[284] + nmpcWorkspace.g[285]*nmpcWorkspace.x[285] + nmpcWorkspace.g[286]*nmpcWorkspace.x[286] + nmpcWorkspace.g[287]*nmpcWorkspace.x[287] + nmpcWorkspace.g[288]*nmpcWorkspace.x[288] + nmpcWorkspace.g[289]*nmpcWorkspace.x[289] + nmpcWorkspace.g[290]*nmpcWorkspace.x[290] + nmpcWorkspace.g[291]*nmpcWorkspace.x[291] + nmpcWorkspace.g[292]*nmpcWorkspace.x[292] + nmpcWorkspace.g[293]*nmpcWorkspace.x[293] + nmpcWorkspace.g[294]*nmpcWorkspace.x[294] + nmpcWorkspace.g[295]*nmpcWorkspace.x[295] + nmpcWorkspace.g[296]*nmpcWorkspace.x[296] + nmpcWorkspace.g[297]*nmpcWorkspace.x[297] + nmpcWorkspace.g[298]*nmpcWorkspace.x[298] + nmpcWorkspace.g[299]*nmpcWorkspace.x[299] + nmpcWorkspace.g[300]*nmpcWorkspace.x[300] + nmpcWorkspace.g[301]*nmpcWorkspace.x[301] + nmpcWorkspace.g[302]*nmpcWorkspace.x[302] + nmpcWorkspace.g[303]*nmpcWorkspace.x[303] + nmpcWorkspace.g[304]*nmpcWorkspace.x[304] + nmpcWorkspace.g[305]*nmpcWorkspace.x[305] + nmpcWorkspace.g[306]*nmpcWorkspace.x[306] + nmpcWorkspace.g[307]*nmpcWorkspace.x[307] + nmpcWorkspace.g[308]*nmpcWorkspace.x[308] + nmpcWorkspace.g[309]*nmpcWorkspace.x[309] + nmpcWorkspace.g[310]*nmpcWorkspace.x[310] + nmpcWorkspace.g[311]*nmpcWorkspace.x[311] + nmpcWorkspace.g[312]*nmpcWorkspace.x[312] + nmpcWorkspace.g[313]*nmpcWorkspace.x[313] + nmpcWorkspace.g[314]*nmpcWorkspace.x[314] + nmpcWorkspace.g[315]*nmpcWorkspace.x[315] + nmpcWorkspace.g[316]*nmpcWorkspace.x[316] + nmpcWorkspace.g[317]*nmpcWorkspace.x[317] + nmpcWorkspace.g[318]*nmpcWorkspace.x[318] + nmpcWorkspace.g[319]*nmpcWorkspace.x[319] + nmpcWorkspace.g[320]*nmpcWorkspace.x[320] + nmpcWorkspace.g[321]*nmpcWorkspace.x[321] + nmpcWorkspace.g[322]*nmpcWorkspace.x[322] + nmpcWorkspace.g[323]*nmpcWorkspace.x[323] + nmpcWorkspace.g[324]*nmpcWorkspace.x[324] + nmpcWorkspace.g[325]*nmpcWorkspace.x[325] + nmpcWorkspace.g[326]*nmpcWorkspace.x[326] + nmpcWorkspace.g[327]*nmpcWorkspace.x[327] + nmpcWorkspace.g[328]*nmpcWorkspace.x[328] + nmpcWorkspace.g[329]*nmpcWorkspace.x[329] + nmpcWorkspace.g[330]*nmpcWorkspace.x[330] + nmpcWorkspace.g[331]*nmpcWorkspace.x[331] + nmpcWorkspace.g[332]*nmpcWorkspace.x[332] + nmpcWorkspace.g[333]*nmpcWorkspace.x[333] + nmpcWorkspace.g[334]*nmpcWorkspace.x[334] + nmpcWorkspace.g[335]*nmpcWorkspace.x[335] + nmpcWorkspace.g[336]*nmpcWorkspace.x[336] + nmpcWorkspace.g[337]*nmpcWorkspace.x[337] + nmpcWorkspace.g[338]*nmpcWorkspace.x[338] + nmpcWorkspace.g[339]*nmpcWorkspace.x[339] + nmpcWorkspace.g[340]*nmpcWorkspace.x[340] + nmpcWorkspace.g[341]*nmpcWorkspace.x[341] + nmpcWorkspace.g[342]*nmpcWorkspace.x[342] + nmpcWorkspace.g[343]*nmpcWorkspace.x[343] + nmpcWorkspace.g[344]*nmpcWorkspace.x[344] + nmpcWorkspace.g[345]*nmpcWorkspace.x[345] + nmpcWorkspace.g[346]*nmpcWorkspace.x[346] + nmpcWorkspace.g[347]*nmpcWorkspace.x[347] + nmpcWorkspace.g[348]*nmpcWorkspace.x[348] + nmpcWorkspace.g[349]*nmpcWorkspace.x[349] + nmpcWorkspace.g[350]*nmpcWorkspace.x[350] + nmpcWorkspace.g[351]*nmpcWorkspace.x[351] + nmpcWorkspace.g[352]*nmpcWorkspace.x[352] + nmpcWorkspace.g[353]*nmpcWorkspace.x[353] + nmpcWorkspace.g[354]*nmpcWorkspace.x[354] + nmpcWorkspace.g[355]*nmpcWorkspace.x[355] + nmpcWorkspace.g[356]*nmpcWorkspace.x[356] + nmpcWorkspace.g[357]*nmpcWorkspace.x[357] + nmpcWorkspace.g[358]*nmpcWorkspace.x[358] + nmpcWorkspace.g[359]*nmpcWorkspace.x[359] + nmpcWorkspace.g[360]*nmpcWorkspace.x[360] + nmpcWorkspace.g[361]*nmpcWorkspace.x[361] + nmpcWorkspace.g[362]*nmpcWorkspace.x[362] + nmpcWorkspace.g[363]*nmpcWorkspace.x[363] + nmpcWorkspace.g[364]*nmpcWorkspace.x[364] + nmpcWorkspace.g[365]*nmpcWorkspace.x[365] + nmpcWorkspace.g[366]*nmpcWorkspace.x[366] + nmpcWorkspace.g[367]*nmpcWorkspace.x[367] + nmpcWorkspace.g[368]*nmpcWorkspace.x[368] + nmpcWorkspace.g[369]*nmpcWorkspace.x[369] + nmpcWorkspace.g[370]*nmpcWorkspace.x[370] + nmpcWorkspace.g[371]*nmpcWorkspace.x[371] + nmpcWorkspace.g[372]*nmpcWorkspace.x[372] + nmpcWorkspace.g[373]*nmpcWorkspace.x[373] + nmpcWorkspace.g[374]*nmpcWorkspace.x[374] + nmpcWorkspace.g[375]*nmpcWorkspace.x[375] + nmpcWorkspace.g[376]*nmpcWorkspace.x[376] + nmpcWorkspace.g[377]*nmpcWorkspace.x[377] + nmpcWorkspace.g[378]*nmpcWorkspace.x[378] + nmpcWorkspace.g[379]*nmpcWorkspace.x[379] + nmpcWorkspace.g[380]*nmpcWorkspace.x[380] + nmpcWorkspace.g[381]*nmpcWorkspace.x[381] + nmpcWorkspace.g[382]*nmpcWorkspace.x[382] + nmpcWorkspace.g[383]*nmpcWorkspace.x[383] + nmpcWorkspace.g[384]*nmpcWorkspace.x[384] + nmpcWorkspace.g[385]*nmpcWorkspace.x[385] + nmpcWorkspace.g[386]*nmpcWorkspace.x[386] + nmpcWorkspace.g[387]*nmpcWorkspace.x[387] + nmpcWorkspace.g[388]*nmpcWorkspace.x[388] + nmpcWorkspace.g[389]*nmpcWorkspace.x[389] + nmpcWorkspace.g[390]*nmpcWorkspace.x[390] + nmpcWorkspace.g[391]*nmpcWorkspace.x[391] + nmpcWorkspace.g[392]*nmpcWorkspace.x[392] + nmpcWorkspace.g[393]*nmpcWorkspace.x[393] + nmpcWorkspace.g[394]*nmpcWorkspace.x[394] + nmpcWorkspace.g[395]*nmpcWorkspace.x[395] + nmpcWorkspace.g[396]*nmpcWorkspace.x[396] + nmpcWorkspace.g[397]*nmpcWorkspace.x[397] + nmpcWorkspace.g[398]*nmpcWorkspace.x[398] + nmpcWorkspace.g[399]*nmpcWorkspace.x[399] + nmpcWorkspace.g[400]*nmpcWorkspace.x[400] + nmpcWorkspace.g[401]*nmpcWorkspace.x[401] + nmpcWorkspace.g[402]*nmpcWorkspace.x[402] + nmpcWorkspace.g[403]*nmpcWorkspace.x[403] + nmpcWorkspace.g[404]*nmpcWorkspace.x[404] + nmpcWorkspace.g[405]*nmpcWorkspace.x[405] + nmpcWorkspace.g[406]*nmpcWorkspace.x[406] + nmpcWorkspace.g[407]*nmpcWorkspace.x[407];
kkt = fabs( kkt );
for (index = 0; index < 408; ++index)
{
prd = nmpcWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(nmpcWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(nmpcWorkspace.ub[index] * prd);
}
return kkt;
}

real_t nmpc_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 12 */
real_t tmpDy[ 12 ];

/** Row vector of size: 8 */
real_t tmpDyN[ 8 ];

for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[lRun1 * 8];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[lRun1 * 8 + 1];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[lRun1 * 8 + 2];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[lRun1 * 8 + 3];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[lRun1 * 8 + 4];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[lRun1 * 8 + 5];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[lRun1 * 8 + 6];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[lRun1 * 8 + 7];
nmpcWorkspace.objValueIn[8] = nmpcVariables.u[lRun1 * 4];
nmpcWorkspace.objValueIn[9] = nmpcVariables.u[lRun1 * 4 + 1];
nmpcWorkspace.objValueIn[10] = nmpcVariables.u[lRun1 * 4 + 2];
nmpcWorkspace.objValueIn[11] = nmpcVariables.u[lRun1 * 4 + 3];
nmpcWorkspace.objValueIn[12] = nmpcVariables.od[lRun1 * 3];
nmpcWorkspace.objValueIn[13] = nmpcVariables.od[lRun1 * 3 + 1];
nmpcWorkspace.objValueIn[14] = nmpcVariables.od[lRun1 * 3 + 2];

nmpc_evaluateLSQ( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.Dy[lRun1 * 12] = nmpcWorkspace.objValueOut[0] - nmpcVariables.y[lRun1 * 12];
nmpcWorkspace.Dy[lRun1 * 12 + 1] = nmpcWorkspace.objValueOut[1] - nmpcVariables.y[lRun1 * 12 + 1];
nmpcWorkspace.Dy[lRun1 * 12 + 2] = nmpcWorkspace.objValueOut[2] - nmpcVariables.y[lRun1 * 12 + 2];
nmpcWorkspace.Dy[lRun1 * 12 + 3] = nmpcWorkspace.objValueOut[3] - nmpcVariables.y[lRun1 * 12 + 3];
nmpcWorkspace.Dy[lRun1 * 12 + 4] = nmpcWorkspace.objValueOut[4] - nmpcVariables.y[lRun1 * 12 + 4];
nmpcWorkspace.Dy[lRun1 * 12 + 5] = nmpcWorkspace.objValueOut[5] - nmpcVariables.y[lRun1 * 12 + 5];
nmpcWorkspace.Dy[lRun1 * 12 + 6] = nmpcWorkspace.objValueOut[6] - nmpcVariables.y[lRun1 * 12 + 6];
nmpcWorkspace.Dy[lRun1 * 12 + 7] = nmpcWorkspace.objValueOut[7] - nmpcVariables.y[lRun1 * 12 + 7];
nmpcWorkspace.Dy[lRun1 * 12 + 8] = nmpcWorkspace.objValueOut[8] - nmpcVariables.y[lRun1 * 12 + 8];
nmpcWorkspace.Dy[lRun1 * 12 + 9] = nmpcWorkspace.objValueOut[9] - nmpcVariables.y[lRun1 * 12 + 9];
nmpcWorkspace.Dy[lRun1 * 12 + 10] = nmpcWorkspace.objValueOut[10] - nmpcVariables.y[lRun1 * 12 + 10];
nmpcWorkspace.Dy[lRun1 * 12 + 11] = nmpcWorkspace.objValueOut[11] - nmpcVariables.y[lRun1 * 12 + 11];
}
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[800];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[801];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[802];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[803];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[804];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[805];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[806];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[807];
nmpcWorkspace.objValueIn[8] = nmpcVariables.od[300];
nmpcWorkspace.objValueIn[9] = nmpcVariables.od[301];
nmpcWorkspace.objValueIn[10] = nmpcVariables.od[302];
nmpc_evaluateLSQEndTerm( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.DyN[0] = nmpcWorkspace.objValueOut[0] - nmpcVariables.yN[0];
nmpcWorkspace.DyN[1] = nmpcWorkspace.objValueOut[1] - nmpcVariables.yN[1];
nmpcWorkspace.DyN[2] = nmpcWorkspace.objValueOut[2] - nmpcVariables.yN[2];
nmpcWorkspace.DyN[3] = nmpcWorkspace.objValueOut[3] - nmpcVariables.yN[3];
nmpcWorkspace.DyN[4] = nmpcWorkspace.objValueOut[4] - nmpcVariables.yN[4];
nmpcWorkspace.DyN[5] = nmpcWorkspace.objValueOut[5] - nmpcVariables.yN[5];
nmpcWorkspace.DyN[6] = nmpcWorkspace.objValueOut[6] - nmpcVariables.yN[6];
nmpcWorkspace.DyN[7] = nmpcWorkspace.objValueOut[7] - nmpcVariables.yN[7];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
tmpDy[0] = + nmpcWorkspace.Dy[lRun1 * 12]*nmpcVariables.W[0];
tmpDy[1] = + nmpcWorkspace.Dy[lRun1 * 12 + 1]*nmpcVariables.W[13];
tmpDy[2] = + nmpcWorkspace.Dy[lRun1 * 12 + 2]*nmpcVariables.W[26];
tmpDy[3] = + nmpcWorkspace.Dy[lRun1 * 12 + 3]*nmpcVariables.W[39];
tmpDy[4] = + nmpcWorkspace.Dy[lRun1 * 12 + 4]*nmpcVariables.W[52];
tmpDy[5] = + nmpcWorkspace.Dy[lRun1 * 12 + 5]*nmpcVariables.W[65];
tmpDy[6] = + nmpcWorkspace.Dy[lRun1 * 12 + 6]*nmpcVariables.W[78];
tmpDy[7] = + nmpcWorkspace.Dy[lRun1 * 12 + 7]*nmpcVariables.W[91];
tmpDy[8] = + nmpcWorkspace.Dy[lRun1 * 12 + 8]*nmpcVariables.W[104];
tmpDy[9] = + nmpcWorkspace.Dy[lRun1 * 12 + 9]*nmpcVariables.W[117];
tmpDy[10] = + nmpcWorkspace.Dy[lRun1 * 12 + 10]*nmpcVariables.W[130];
tmpDy[11] = + nmpcWorkspace.Dy[lRun1 * 12 + 11]*nmpcVariables.W[143];
objVal += + nmpcWorkspace.Dy[lRun1 * 12]*tmpDy[0] + nmpcWorkspace.Dy[lRun1 * 12 + 1]*tmpDy[1] + nmpcWorkspace.Dy[lRun1 * 12 + 2]*tmpDy[2] + nmpcWorkspace.Dy[lRun1 * 12 + 3]*tmpDy[3] + nmpcWorkspace.Dy[lRun1 * 12 + 4]*tmpDy[4] + nmpcWorkspace.Dy[lRun1 * 12 + 5]*tmpDy[5] + nmpcWorkspace.Dy[lRun1 * 12 + 6]*tmpDy[6] + nmpcWorkspace.Dy[lRun1 * 12 + 7]*tmpDy[7] + nmpcWorkspace.Dy[lRun1 * 12 + 8]*tmpDy[8] + nmpcWorkspace.Dy[lRun1 * 12 + 9]*tmpDy[9] + nmpcWorkspace.Dy[lRun1 * 12 + 10]*tmpDy[10] + nmpcWorkspace.Dy[lRun1 * 12 + 11]*tmpDy[11];
}

tmpDyN[0] = + nmpcWorkspace.DyN[0]*nmpcVariables.WN[0];
tmpDyN[1] = + nmpcWorkspace.DyN[1]*nmpcVariables.WN[9];
tmpDyN[2] = + nmpcWorkspace.DyN[2]*nmpcVariables.WN[18];
tmpDyN[3] = + nmpcWorkspace.DyN[3]*nmpcVariables.WN[27];
tmpDyN[4] = + nmpcWorkspace.DyN[4]*nmpcVariables.WN[36];
tmpDyN[5] = + nmpcWorkspace.DyN[5]*nmpcVariables.WN[45];
tmpDyN[6] = + nmpcWorkspace.DyN[6]*nmpcVariables.WN[54];
tmpDyN[7] = + nmpcWorkspace.DyN[7]*nmpcVariables.WN[63];
objVal += + nmpcWorkspace.DyN[0]*tmpDyN[0] + nmpcWorkspace.DyN[1]*tmpDyN[1] + nmpcWorkspace.DyN[2]*tmpDyN[2] + nmpcWorkspace.DyN[3]*tmpDyN[3] + nmpcWorkspace.DyN[4]*tmpDyN[4] + nmpcWorkspace.DyN[5]*tmpDyN[5] + nmpcWorkspace.DyN[6]*tmpDyN[6] + nmpcWorkspace.DyN[7]*tmpDyN[7];

objVal *= 0.5;
return objVal;
}

