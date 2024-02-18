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
for (lRun1 = 0; lRun1 < 10; ++lRun1)
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
for (runObj = 0; runObj < 10; ++runObj)
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
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[80];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[81];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[82];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[83];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[84];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[85];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[86];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[87];
nmpcWorkspace.objValueIn[8] = nmpcVariables.od[30];
nmpcWorkspace.objValueIn[9] = nmpcVariables.od[31];
nmpcWorkspace.objValueIn[10] = nmpcVariables.od[32];
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
nmpcWorkspace.H[(iRow * 192 + 384) + (iCol * 4 + 8)] += + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28];
nmpcWorkspace.H[(iRow * 192 + 384) + (iCol * 4 + 9)] += + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29];
nmpcWorkspace.H[(iRow * 192 + 384) + (iCol * 4 + 10)] += + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30];
nmpcWorkspace.H[(iRow * 192 + 384) + (iCol * 4 + 11)] += + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31];
nmpcWorkspace.H[(iRow * 192 + 432) + (iCol * 4 + 8)] += + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28];
nmpcWorkspace.H[(iRow * 192 + 432) + (iCol * 4 + 9)] += + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29];
nmpcWorkspace.H[(iRow * 192 + 432) + (iCol * 4 + 10)] += + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30];
nmpcWorkspace.H[(iRow * 192 + 432) + (iCol * 4 + 11)] += + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31];
nmpcWorkspace.H[(iRow * 192 + 480) + (iCol * 4 + 8)] += + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28];
nmpcWorkspace.H[(iRow * 192 + 480) + (iCol * 4 + 9)] += + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29];
nmpcWorkspace.H[(iRow * 192 + 480) + (iCol * 4 + 10)] += + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30];
nmpcWorkspace.H[(iRow * 192 + 480) + (iCol * 4 + 11)] += + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31];
nmpcWorkspace.H[(iRow * 192 + 528) + (iCol * 4 + 8)] += + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28];
nmpcWorkspace.H[(iRow * 192 + 528) + (iCol * 4 + 9)] += + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29];
nmpcWorkspace.H[(iRow * 192 + 528) + (iCol * 4 + 10)] += + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30];
nmpcWorkspace.H[(iRow * 192 + 528) + (iCol * 4 + 11)] += + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31];
}

void nmpc_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
nmpcWorkspace.H[(iRow * 192 + 384) + (iCol * 4 + 8)] = R11[0];
nmpcWorkspace.H[(iRow * 192 + 384) + (iCol * 4 + 9)] = R11[1];
nmpcWorkspace.H[(iRow * 192 + 384) + (iCol * 4 + 10)] = R11[2];
nmpcWorkspace.H[(iRow * 192 + 384) + (iCol * 4 + 11)] = R11[3];
nmpcWorkspace.H[(iRow * 192 + 432) + (iCol * 4 + 8)] = R11[4];
nmpcWorkspace.H[(iRow * 192 + 432) + (iCol * 4 + 9)] = R11[5];
nmpcWorkspace.H[(iRow * 192 + 432) + (iCol * 4 + 10)] = R11[6];
nmpcWorkspace.H[(iRow * 192 + 432) + (iCol * 4 + 11)] = R11[7];
nmpcWorkspace.H[(iRow * 192 + 480) + (iCol * 4 + 8)] = R11[8];
nmpcWorkspace.H[(iRow * 192 + 480) + (iCol * 4 + 9)] = R11[9];
nmpcWorkspace.H[(iRow * 192 + 480) + (iCol * 4 + 10)] = R11[10];
nmpcWorkspace.H[(iRow * 192 + 480) + (iCol * 4 + 11)] = R11[11];
nmpcWorkspace.H[(iRow * 192 + 528) + (iCol * 4 + 8)] = R11[12];
nmpcWorkspace.H[(iRow * 192 + 528) + (iCol * 4 + 9)] = R11[13];
nmpcWorkspace.H[(iRow * 192 + 528) + (iCol * 4 + 10)] = R11[14];
nmpcWorkspace.H[(iRow * 192 + 528) + (iCol * 4 + 11)] = R11[15];
}

void nmpc_zeroBlockH11( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 192 + 384) + (iCol * 4 + 8)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 192 + 384) + (iCol * 4 + 9)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 192 + 384) + (iCol * 4 + 10)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 192 + 384) + (iCol * 4 + 11)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 192 + 432) + (iCol * 4 + 8)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 192 + 432) + (iCol * 4 + 9)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 192 + 432) + (iCol * 4 + 10)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 192 + 432) + (iCol * 4 + 11)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 192 + 480) + (iCol * 4 + 8)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 192 + 480) + (iCol * 4 + 9)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 192 + 480) + (iCol * 4 + 10)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 192 + 480) + (iCol * 4 + 11)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 192 + 528) + (iCol * 4 + 8)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 192 + 528) + (iCol * 4 + 9)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 192 + 528) + (iCol * 4 + 10)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 192 + 528) + (iCol * 4 + 11)] = 0.0000000000000000e+00;
}

void nmpc_copyHTH( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 192 + 384) + (iCol * 4 + 8)] = nmpcWorkspace.H[(iCol * 192 + 384) + (iRow * 4 + 8)];
nmpcWorkspace.H[(iRow * 192 + 384) + (iCol * 4 + 9)] = nmpcWorkspace.H[(iCol * 192 + 432) + (iRow * 4 + 8)];
nmpcWorkspace.H[(iRow * 192 + 384) + (iCol * 4 + 10)] = nmpcWorkspace.H[(iCol * 192 + 480) + (iRow * 4 + 8)];
nmpcWorkspace.H[(iRow * 192 + 384) + (iCol * 4 + 11)] = nmpcWorkspace.H[(iCol * 192 + 528) + (iRow * 4 + 8)];
nmpcWorkspace.H[(iRow * 192 + 432) + (iCol * 4 + 8)] = nmpcWorkspace.H[(iCol * 192 + 384) + (iRow * 4 + 9)];
nmpcWorkspace.H[(iRow * 192 + 432) + (iCol * 4 + 9)] = nmpcWorkspace.H[(iCol * 192 + 432) + (iRow * 4 + 9)];
nmpcWorkspace.H[(iRow * 192 + 432) + (iCol * 4 + 10)] = nmpcWorkspace.H[(iCol * 192 + 480) + (iRow * 4 + 9)];
nmpcWorkspace.H[(iRow * 192 + 432) + (iCol * 4 + 11)] = nmpcWorkspace.H[(iCol * 192 + 528) + (iRow * 4 + 9)];
nmpcWorkspace.H[(iRow * 192 + 480) + (iCol * 4 + 8)] = nmpcWorkspace.H[(iCol * 192 + 384) + (iRow * 4 + 10)];
nmpcWorkspace.H[(iRow * 192 + 480) + (iCol * 4 + 9)] = nmpcWorkspace.H[(iCol * 192 + 432) + (iRow * 4 + 10)];
nmpcWorkspace.H[(iRow * 192 + 480) + (iCol * 4 + 10)] = nmpcWorkspace.H[(iCol * 192 + 480) + (iRow * 4 + 10)];
nmpcWorkspace.H[(iRow * 192 + 480) + (iCol * 4 + 11)] = nmpcWorkspace.H[(iCol * 192 + 528) + (iRow * 4 + 10)];
nmpcWorkspace.H[(iRow * 192 + 528) + (iCol * 4 + 8)] = nmpcWorkspace.H[(iCol * 192 + 384) + (iRow * 4 + 11)];
nmpcWorkspace.H[(iRow * 192 + 528) + (iCol * 4 + 9)] = nmpcWorkspace.H[(iCol * 192 + 432) + (iRow * 4 + 11)];
nmpcWorkspace.H[(iRow * 192 + 528) + (iCol * 4 + 10)] = nmpcWorkspace.H[(iCol * 192 + 480) + (iRow * 4 + 11)];
nmpcWorkspace.H[(iRow * 192 + 528) + (iCol * 4 + 11)] = nmpcWorkspace.H[(iCol * 192 + 528) + (iRow * 4 + 11)];
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
nmpcWorkspace.H[48] = 0.0000000000000000e+00;
nmpcWorkspace.H[49] = 0.0000000000000000e+00;
nmpcWorkspace.H[50] = 0.0000000000000000e+00;
nmpcWorkspace.H[51] = 0.0000000000000000e+00;
nmpcWorkspace.H[52] = 0.0000000000000000e+00;
nmpcWorkspace.H[53] = 0.0000000000000000e+00;
nmpcWorkspace.H[54] = 0.0000000000000000e+00;
nmpcWorkspace.H[55] = 0.0000000000000000e+00;
nmpcWorkspace.H[96] = 0.0000000000000000e+00;
nmpcWorkspace.H[97] = 0.0000000000000000e+00;
nmpcWorkspace.H[98] = 0.0000000000000000e+00;
nmpcWorkspace.H[99] = 0.0000000000000000e+00;
nmpcWorkspace.H[100] = 0.0000000000000000e+00;
nmpcWorkspace.H[101] = 0.0000000000000000e+00;
nmpcWorkspace.H[102] = 0.0000000000000000e+00;
nmpcWorkspace.H[103] = 0.0000000000000000e+00;
nmpcWorkspace.H[144] = 0.0000000000000000e+00;
nmpcWorkspace.H[145] = 0.0000000000000000e+00;
nmpcWorkspace.H[146] = 0.0000000000000000e+00;
nmpcWorkspace.H[147] = 0.0000000000000000e+00;
nmpcWorkspace.H[148] = 0.0000000000000000e+00;
nmpcWorkspace.H[149] = 0.0000000000000000e+00;
nmpcWorkspace.H[150] = 0.0000000000000000e+00;
nmpcWorkspace.H[151] = 0.0000000000000000e+00;
nmpcWorkspace.H[192] = 0.0000000000000000e+00;
nmpcWorkspace.H[193] = 0.0000000000000000e+00;
nmpcWorkspace.H[194] = 0.0000000000000000e+00;
nmpcWorkspace.H[195] = 0.0000000000000000e+00;
nmpcWorkspace.H[196] = 0.0000000000000000e+00;
nmpcWorkspace.H[197] = 0.0000000000000000e+00;
nmpcWorkspace.H[198] = 0.0000000000000000e+00;
nmpcWorkspace.H[199] = 0.0000000000000000e+00;
nmpcWorkspace.H[240] = 0.0000000000000000e+00;
nmpcWorkspace.H[241] = 0.0000000000000000e+00;
nmpcWorkspace.H[242] = 0.0000000000000000e+00;
nmpcWorkspace.H[243] = 0.0000000000000000e+00;
nmpcWorkspace.H[244] = 0.0000000000000000e+00;
nmpcWorkspace.H[245] = 0.0000000000000000e+00;
nmpcWorkspace.H[246] = 0.0000000000000000e+00;
nmpcWorkspace.H[247] = 0.0000000000000000e+00;
nmpcWorkspace.H[288] = 0.0000000000000000e+00;
nmpcWorkspace.H[289] = 0.0000000000000000e+00;
nmpcWorkspace.H[290] = 0.0000000000000000e+00;
nmpcWorkspace.H[291] = 0.0000000000000000e+00;
nmpcWorkspace.H[292] = 0.0000000000000000e+00;
nmpcWorkspace.H[293] = 0.0000000000000000e+00;
nmpcWorkspace.H[294] = 0.0000000000000000e+00;
nmpcWorkspace.H[295] = 0.0000000000000000e+00;
nmpcWorkspace.H[336] = 0.0000000000000000e+00;
nmpcWorkspace.H[337] = 0.0000000000000000e+00;
nmpcWorkspace.H[338] = 0.0000000000000000e+00;
nmpcWorkspace.H[339] = 0.0000000000000000e+00;
nmpcWorkspace.H[340] = 0.0000000000000000e+00;
nmpcWorkspace.H[341] = 0.0000000000000000e+00;
nmpcWorkspace.H[342] = 0.0000000000000000e+00;
nmpcWorkspace.H[343] = 0.0000000000000000e+00;
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
nmpcWorkspace.H[48] += + Gx1[1]*Gx2[0] + Gx1[9]*Gx2[8] + Gx1[17]*Gx2[16] + Gx1[25]*Gx2[24] + Gx1[33]*Gx2[32] + Gx1[41]*Gx2[40] + Gx1[49]*Gx2[48] + Gx1[57]*Gx2[56];
nmpcWorkspace.H[49] += + Gx1[1]*Gx2[1] + Gx1[9]*Gx2[9] + Gx1[17]*Gx2[17] + Gx1[25]*Gx2[25] + Gx1[33]*Gx2[33] + Gx1[41]*Gx2[41] + Gx1[49]*Gx2[49] + Gx1[57]*Gx2[57];
nmpcWorkspace.H[50] += + Gx1[1]*Gx2[2] + Gx1[9]*Gx2[10] + Gx1[17]*Gx2[18] + Gx1[25]*Gx2[26] + Gx1[33]*Gx2[34] + Gx1[41]*Gx2[42] + Gx1[49]*Gx2[50] + Gx1[57]*Gx2[58];
nmpcWorkspace.H[51] += + Gx1[1]*Gx2[3] + Gx1[9]*Gx2[11] + Gx1[17]*Gx2[19] + Gx1[25]*Gx2[27] + Gx1[33]*Gx2[35] + Gx1[41]*Gx2[43] + Gx1[49]*Gx2[51] + Gx1[57]*Gx2[59];
nmpcWorkspace.H[52] += + Gx1[1]*Gx2[4] + Gx1[9]*Gx2[12] + Gx1[17]*Gx2[20] + Gx1[25]*Gx2[28] + Gx1[33]*Gx2[36] + Gx1[41]*Gx2[44] + Gx1[49]*Gx2[52] + Gx1[57]*Gx2[60];
nmpcWorkspace.H[53] += + Gx1[1]*Gx2[5] + Gx1[9]*Gx2[13] + Gx1[17]*Gx2[21] + Gx1[25]*Gx2[29] + Gx1[33]*Gx2[37] + Gx1[41]*Gx2[45] + Gx1[49]*Gx2[53] + Gx1[57]*Gx2[61];
nmpcWorkspace.H[54] += + Gx1[1]*Gx2[6] + Gx1[9]*Gx2[14] + Gx1[17]*Gx2[22] + Gx1[25]*Gx2[30] + Gx1[33]*Gx2[38] + Gx1[41]*Gx2[46] + Gx1[49]*Gx2[54] + Gx1[57]*Gx2[62];
nmpcWorkspace.H[55] += + Gx1[1]*Gx2[7] + Gx1[9]*Gx2[15] + Gx1[17]*Gx2[23] + Gx1[25]*Gx2[31] + Gx1[33]*Gx2[39] + Gx1[41]*Gx2[47] + Gx1[49]*Gx2[55] + Gx1[57]*Gx2[63];
nmpcWorkspace.H[96] += + Gx1[2]*Gx2[0] + Gx1[10]*Gx2[8] + Gx1[18]*Gx2[16] + Gx1[26]*Gx2[24] + Gx1[34]*Gx2[32] + Gx1[42]*Gx2[40] + Gx1[50]*Gx2[48] + Gx1[58]*Gx2[56];
nmpcWorkspace.H[97] += + Gx1[2]*Gx2[1] + Gx1[10]*Gx2[9] + Gx1[18]*Gx2[17] + Gx1[26]*Gx2[25] + Gx1[34]*Gx2[33] + Gx1[42]*Gx2[41] + Gx1[50]*Gx2[49] + Gx1[58]*Gx2[57];
nmpcWorkspace.H[98] += + Gx1[2]*Gx2[2] + Gx1[10]*Gx2[10] + Gx1[18]*Gx2[18] + Gx1[26]*Gx2[26] + Gx1[34]*Gx2[34] + Gx1[42]*Gx2[42] + Gx1[50]*Gx2[50] + Gx1[58]*Gx2[58];
nmpcWorkspace.H[99] += + Gx1[2]*Gx2[3] + Gx1[10]*Gx2[11] + Gx1[18]*Gx2[19] + Gx1[26]*Gx2[27] + Gx1[34]*Gx2[35] + Gx1[42]*Gx2[43] + Gx1[50]*Gx2[51] + Gx1[58]*Gx2[59];
nmpcWorkspace.H[100] += + Gx1[2]*Gx2[4] + Gx1[10]*Gx2[12] + Gx1[18]*Gx2[20] + Gx1[26]*Gx2[28] + Gx1[34]*Gx2[36] + Gx1[42]*Gx2[44] + Gx1[50]*Gx2[52] + Gx1[58]*Gx2[60];
nmpcWorkspace.H[101] += + Gx1[2]*Gx2[5] + Gx1[10]*Gx2[13] + Gx1[18]*Gx2[21] + Gx1[26]*Gx2[29] + Gx1[34]*Gx2[37] + Gx1[42]*Gx2[45] + Gx1[50]*Gx2[53] + Gx1[58]*Gx2[61];
nmpcWorkspace.H[102] += + Gx1[2]*Gx2[6] + Gx1[10]*Gx2[14] + Gx1[18]*Gx2[22] + Gx1[26]*Gx2[30] + Gx1[34]*Gx2[38] + Gx1[42]*Gx2[46] + Gx1[50]*Gx2[54] + Gx1[58]*Gx2[62];
nmpcWorkspace.H[103] += + Gx1[2]*Gx2[7] + Gx1[10]*Gx2[15] + Gx1[18]*Gx2[23] + Gx1[26]*Gx2[31] + Gx1[34]*Gx2[39] + Gx1[42]*Gx2[47] + Gx1[50]*Gx2[55] + Gx1[58]*Gx2[63];
nmpcWorkspace.H[144] += + Gx1[3]*Gx2[0] + Gx1[11]*Gx2[8] + Gx1[19]*Gx2[16] + Gx1[27]*Gx2[24] + Gx1[35]*Gx2[32] + Gx1[43]*Gx2[40] + Gx1[51]*Gx2[48] + Gx1[59]*Gx2[56];
nmpcWorkspace.H[145] += + Gx1[3]*Gx2[1] + Gx1[11]*Gx2[9] + Gx1[19]*Gx2[17] + Gx1[27]*Gx2[25] + Gx1[35]*Gx2[33] + Gx1[43]*Gx2[41] + Gx1[51]*Gx2[49] + Gx1[59]*Gx2[57];
nmpcWorkspace.H[146] += + Gx1[3]*Gx2[2] + Gx1[11]*Gx2[10] + Gx1[19]*Gx2[18] + Gx1[27]*Gx2[26] + Gx1[35]*Gx2[34] + Gx1[43]*Gx2[42] + Gx1[51]*Gx2[50] + Gx1[59]*Gx2[58];
nmpcWorkspace.H[147] += + Gx1[3]*Gx2[3] + Gx1[11]*Gx2[11] + Gx1[19]*Gx2[19] + Gx1[27]*Gx2[27] + Gx1[35]*Gx2[35] + Gx1[43]*Gx2[43] + Gx1[51]*Gx2[51] + Gx1[59]*Gx2[59];
nmpcWorkspace.H[148] += + Gx1[3]*Gx2[4] + Gx1[11]*Gx2[12] + Gx1[19]*Gx2[20] + Gx1[27]*Gx2[28] + Gx1[35]*Gx2[36] + Gx1[43]*Gx2[44] + Gx1[51]*Gx2[52] + Gx1[59]*Gx2[60];
nmpcWorkspace.H[149] += + Gx1[3]*Gx2[5] + Gx1[11]*Gx2[13] + Gx1[19]*Gx2[21] + Gx1[27]*Gx2[29] + Gx1[35]*Gx2[37] + Gx1[43]*Gx2[45] + Gx1[51]*Gx2[53] + Gx1[59]*Gx2[61];
nmpcWorkspace.H[150] += + Gx1[3]*Gx2[6] + Gx1[11]*Gx2[14] + Gx1[19]*Gx2[22] + Gx1[27]*Gx2[30] + Gx1[35]*Gx2[38] + Gx1[43]*Gx2[46] + Gx1[51]*Gx2[54] + Gx1[59]*Gx2[62];
nmpcWorkspace.H[151] += + Gx1[3]*Gx2[7] + Gx1[11]*Gx2[15] + Gx1[19]*Gx2[23] + Gx1[27]*Gx2[31] + Gx1[35]*Gx2[39] + Gx1[43]*Gx2[47] + Gx1[51]*Gx2[55] + Gx1[59]*Gx2[63];
nmpcWorkspace.H[192] += + Gx1[4]*Gx2[0] + Gx1[12]*Gx2[8] + Gx1[20]*Gx2[16] + Gx1[28]*Gx2[24] + Gx1[36]*Gx2[32] + Gx1[44]*Gx2[40] + Gx1[52]*Gx2[48] + Gx1[60]*Gx2[56];
nmpcWorkspace.H[193] += + Gx1[4]*Gx2[1] + Gx1[12]*Gx2[9] + Gx1[20]*Gx2[17] + Gx1[28]*Gx2[25] + Gx1[36]*Gx2[33] + Gx1[44]*Gx2[41] + Gx1[52]*Gx2[49] + Gx1[60]*Gx2[57];
nmpcWorkspace.H[194] += + Gx1[4]*Gx2[2] + Gx1[12]*Gx2[10] + Gx1[20]*Gx2[18] + Gx1[28]*Gx2[26] + Gx1[36]*Gx2[34] + Gx1[44]*Gx2[42] + Gx1[52]*Gx2[50] + Gx1[60]*Gx2[58];
nmpcWorkspace.H[195] += + Gx1[4]*Gx2[3] + Gx1[12]*Gx2[11] + Gx1[20]*Gx2[19] + Gx1[28]*Gx2[27] + Gx1[36]*Gx2[35] + Gx1[44]*Gx2[43] + Gx1[52]*Gx2[51] + Gx1[60]*Gx2[59];
nmpcWorkspace.H[196] += + Gx1[4]*Gx2[4] + Gx1[12]*Gx2[12] + Gx1[20]*Gx2[20] + Gx1[28]*Gx2[28] + Gx1[36]*Gx2[36] + Gx1[44]*Gx2[44] + Gx1[52]*Gx2[52] + Gx1[60]*Gx2[60];
nmpcWorkspace.H[197] += + Gx1[4]*Gx2[5] + Gx1[12]*Gx2[13] + Gx1[20]*Gx2[21] + Gx1[28]*Gx2[29] + Gx1[36]*Gx2[37] + Gx1[44]*Gx2[45] + Gx1[52]*Gx2[53] + Gx1[60]*Gx2[61];
nmpcWorkspace.H[198] += + Gx1[4]*Gx2[6] + Gx1[12]*Gx2[14] + Gx1[20]*Gx2[22] + Gx1[28]*Gx2[30] + Gx1[36]*Gx2[38] + Gx1[44]*Gx2[46] + Gx1[52]*Gx2[54] + Gx1[60]*Gx2[62];
nmpcWorkspace.H[199] += + Gx1[4]*Gx2[7] + Gx1[12]*Gx2[15] + Gx1[20]*Gx2[23] + Gx1[28]*Gx2[31] + Gx1[36]*Gx2[39] + Gx1[44]*Gx2[47] + Gx1[52]*Gx2[55] + Gx1[60]*Gx2[63];
nmpcWorkspace.H[240] += + Gx1[5]*Gx2[0] + Gx1[13]*Gx2[8] + Gx1[21]*Gx2[16] + Gx1[29]*Gx2[24] + Gx1[37]*Gx2[32] + Gx1[45]*Gx2[40] + Gx1[53]*Gx2[48] + Gx1[61]*Gx2[56];
nmpcWorkspace.H[241] += + Gx1[5]*Gx2[1] + Gx1[13]*Gx2[9] + Gx1[21]*Gx2[17] + Gx1[29]*Gx2[25] + Gx1[37]*Gx2[33] + Gx1[45]*Gx2[41] + Gx1[53]*Gx2[49] + Gx1[61]*Gx2[57];
nmpcWorkspace.H[242] += + Gx1[5]*Gx2[2] + Gx1[13]*Gx2[10] + Gx1[21]*Gx2[18] + Gx1[29]*Gx2[26] + Gx1[37]*Gx2[34] + Gx1[45]*Gx2[42] + Gx1[53]*Gx2[50] + Gx1[61]*Gx2[58];
nmpcWorkspace.H[243] += + Gx1[5]*Gx2[3] + Gx1[13]*Gx2[11] + Gx1[21]*Gx2[19] + Gx1[29]*Gx2[27] + Gx1[37]*Gx2[35] + Gx1[45]*Gx2[43] + Gx1[53]*Gx2[51] + Gx1[61]*Gx2[59];
nmpcWorkspace.H[244] += + Gx1[5]*Gx2[4] + Gx1[13]*Gx2[12] + Gx1[21]*Gx2[20] + Gx1[29]*Gx2[28] + Gx1[37]*Gx2[36] + Gx1[45]*Gx2[44] + Gx1[53]*Gx2[52] + Gx1[61]*Gx2[60];
nmpcWorkspace.H[245] += + Gx1[5]*Gx2[5] + Gx1[13]*Gx2[13] + Gx1[21]*Gx2[21] + Gx1[29]*Gx2[29] + Gx1[37]*Gx2[37] + Gx1[45]*Gx2[45] + Gx1[53]*Gx2[53] + Gx1[61]*Gx2[61];
nmpcWorkspace.H[246] += + Gx1[5]*Gx2[6] + Gx1[13]*Gx2[14] + Gx1[21]*Gx2[22] + Gx1[29]*Gx2[30] + Gx1[37]*Gx2[38] + Gx1[45]*Gx2[46] + Gx1[53]*Gx2[54] + Gx1[61]*Gx2[62];
nmpcWorkspace.H[247] += + Gx1[5]*Gx2[7] + Gx1[13]*Gx2[15] + Gx1[21]*Gx2[23] + Gx1[29]*Gx2[31] + Gx1[37]*Gx2[39] + Gx1[45]*Gx2[47] + Gx1[53]*Gx2[55] + Gx1[61]*Gx2[63];
nmpcWorkspace.H[288] += + Gx1[6]*Gx2[0] + Gx1[14]*Gx2[8] + Gx1[22]*Gx2[16] + Gx1[30]*Gx2[24] + Gx1[38]*Gx2[32] + Gx1[46]*Gx2[40] + Gx1[54]*Gx2[48] + Gx1[62]*Gx2[56];
nmpcWorkspace.H[289] += + Gx1[6]*Gx2[1] + Gx1[14]*Gx2[9] + Gx1[22]*Gx2[17] + Gx1[30]*Gx2[25] + Gx1[38]*Gx2[33] + Gx1[46]*Gx2[41] + Gx1[54]*Gx2[49] + Gx1[62]*Gx2[57];
nmpcWorkspace.H[290] += + Gx1[6]*Gx2[2] + Gx1[14]*Gx2[10] + Gx1[22]*Gx2[18] + Gx1[30]*Gx2[26] + Gx1[38]*Gx2[34] + Gx1[46]*Gx2[42] + Gx1[54]*Gx2[50] + Gx1[62]*Gx2[58];
nmpcWorkspace.H[291] += + Gx1[6]*Gx2[3] + Gx1[14]*Gx2[11] + Gx1[22]*Gx2[19] + Gx1[30]*Gx2[27] + Gx1[38]*Gx2[35] + Gx1[46]*Gx2[43] + Gx1[54]*Gx2[51] + Gx1[62]*Gx2[59];
nmpcWorkspace.H[292] += + Gx1[6]*Gx2[4] + Gx1[14]*Gx2[12] + Gx1[22]*Gx2[20] + Gx1[30]*Gx2[28] + Gx1[38]*Gx2[36] + Gx1[46]*Gx2[44] + Gx1[54]*Gx2[52] + Gx1[62]*Gx2[60];
nmpcWorkspace.H[293] += + Gx1[6]*Gx2[5] + Gx1[14]*Gx2[13] + Gx1[22]*Gx2[21] + Gx1[30]*Gx2[29] + Gx1[38]*Gx2[37] + Gx1[46]*Gx2[45] + Gx1[54]*Gx2[53] + Gx1[62]*Gx2[61];
nmpcWorkspace.H[294] += + Gx1[6]*Gx2[6] + Gx1[14]*Gx2[14] + Gx1[22]*Gx2[22] + Gx1[30]*Gx2[30] + Gx1[38]*Gx2[38] + Gx1[46]*Gx2[46] + Gx1[54]*Gx2[54] + Gx1[62]*Gx2[62];
nmpcWorkspace.H[295] += + Gx1[6]*Gx2[7] + Gx1[14]*Gx2[15] + Gx1[22]*Gx2[23] + Gx1[30]*Gx2[31] + Gx1[38]*Gx2[39] + Gx1[46]*Gx2[47] + Gx1[54]*Gx2[55] + Gx1[62]*Gx2[63];
nmpcWorkspace.H[336] += + Gx1[7]*Gx2[0] + Gx1[15]*Gx2[8] + Gx1[23]*Gx2[16] + Gx1[31]*Gx2[24] + Gx1[39]*Gx2[32] + Gx1[47]*Gx2[40] + Gx1[55]*Gx2[48] + Gx1[63]*Gx2[56];
nmpcWorkspace.H[337] += + Gx1[7]*Gx2[1] + Gx1[15]*Gx2[9] + Gx1[23]*Gx2[17] + Gx1[31]*Gx2[25] + Gx1[39]*Gx2[33] + Gx1[47]*Gx2[41] + Gx1[55]*Gx2[49] + Gx1[63]*Gx2[57];
nmpcWorkspace.H[338] += + Gx1[7]*Gx2[2] + Gx1[15]*Gx2[10] + Gx1[23]*Gx2[18] + Gx1[31]*Gx2[26] + Gx1[39]*Gx2[34] + Gx1[47]*Gx2[42] + Gx1[55]*Gx2[50] + Gx1[63]*Gx2[58];
nmpcWorkspace.H[339] += + Gx1[7]*Gx2[3] + Gx1[15]*Gx2[11] + Gx1[23]*Gx2[19] + Gx1[31]*Gx2[27] + Gx1[39]*Gx2[35] + Gx1[47]*Gx2[43] + Gx1[55]*Gx2[51] + Gx1[63]*Gx2[59];
nmpcWorkspace.H[340] += + Gx1[7]*Gx2[4] + Gx1[15]*Gx2[12] + Gx1[23]*Gx2[20] + Gx1[31]*Gx2[28] + Gx1[39]*Gx2[36] + Gx1[47]*Gx2[44] + Gx1[55]*Gx2[52] + Gx1[63]*Gx2[60];
nmpcWorkspace.H[341] += + Gx1[7]*Gx2[5] + Gx1[15]*Gx2[13] + Gx1[23]*Gx2[21] + Gx1[31]*Gx2[29] + Gx1[39]*Gx2[37] + Gx1[47]*Gx2[45] + Gx1[55]*Gx2[53] + Gx1[63]*Gx2[61];
nmpcWorkspace.H[342] += + Gx1[7]*Gx2[6] + Gx1[15]*Gx2[14] + Gx1[23]*Gx2[22] + Gx1[31]*Gx2[30] + Gx1[39]*Gx2[38] + Gx1[47]*Gx2[46] + Gx1[55]*Gx2[54] + Gx1[63]*Gx2[62];
nmpcWorkspace.H[343] += + Gx1[7]*Gx2[7] + Gx1[15]*Gx2[15] + Gx1[23]*Gx2[23] + Gx1[31]*Gx2[31] + Gx1[39]*Gx2[39] + Gx1[47]*Gx2[47] + Gx1[55]*Gx2[55] + Gx1[63]*Gx2[63];
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
nmpc_moveGuE( nmpcWorkspace.evGu, nmpcWorkspace.E );
nmpc_moveGxT( &(nmpcWorkspace.evGx[ 64 ]), nmpcWorkspace.T );
nmpc_multGxd( nmpcWorkspace.d, &(nmpcWorkspace.evGx[ 64 ]), &(nmpcWorkspace.d[ 8 ]) );
nmpc_multGxGx( nmpcWorkspace.T, nmpcWorkspace.evGx, &(nmpcWorkspace.evGx[ 64 ]) );

nmpc_multGxGu( nmpcWorkspace.T, nmpcWorkspace.E, &(nmpcWorkspace.E[ 32 ]) );

nmpc_moveGuE( &(nmpcWorkspace.evGu[ 32 ]), &(nmpcWorkspace.E[ 64 ]) );

nmpc_moveGxT( &(nmpcWorkspace.evGx[ 128 ]), nmpcWorkspace.T );
nmpc_multGxd( &(nmpcWorkspace.d[ 8 ]), &(nmpcWorkspace.evGx[ 128 ]), &(nmpcWorkspace.d[ 16 ]) );
nmpc_multGxGx( nmpcWorkspace.T, &(nmpcWorkspace.evGx[ 64 ]), &(nmpcWorkspace.evGx[ 128 ]) );

nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 32 ]), &(nmpcWorkspace.E[ 96 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 64 ]), &(nmpcWorkspace.E[ 128 ]) );

nmpc_moveGuE( &(nmpcWorkspace.evGu[ 64 ]), &(nmpcWorkspace.E[ 160 ]) );

nmpc_moveGxT( &(nmpcWorkspace.evGx[ 192 ]), nmpcWorkspace.T );
nmpc_multGxd( &(nmpcWorkspace.d[ 16 ]), &(nmpcWorkspace.evGx[ 192 ]), &(nmpcWorkspace.d[ 24 ]) );
nmpc_multGxGx( nmpcWorkspace.T, &(nmpcWorkspace.evGx[ 128 ]), &(nmpcWorkspace.evGx[ 192 ]) );

nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 96 ]), &(nmpcWorkspace.E[ 192 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 128 ]), &(nmpcWorkspace.E[ 224 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 160 ]), &(nmpcWorkspace.E[ 256 ]) );

nmpc_moveGuE( &(nmpcWorkspace.evGu[ 96 ]), &(nmpcWorkspace.E[ 288 ]) );

nmpc_moveGxT( &(nmpcWorkspace.evGx[ 256 ]), nmpcWorkspace.T );
nmpc_multGxd( &(nmpcWorkspace.d[ 24 ]), &(nmpcWorkspace.evGx[ 256 ]), &(nmpcWorkspace.d[ 32 ]) );
nmpc_multGxGx( nmpcWorkspace.T, &(nmpcWorkspace.evGx[ 192 ]), &(nmpcWorkspace.evGx[ 256 ]) );

nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 192 ]), &(nmpcWorkspace.E[ 320 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 224 ]), &(nmpcWorkspace.E[ 352 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 256 ]), &(nmpcWorkspace.E[ 384 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 288 ]), &(nmpcWorkspace.E[ 416 ]) );

nmpc_moveGuE( &(nmpcWorkspace.evGu[ 128 ]), &(nmpcWorkspace.E[ 448 ]) );

nmpc_moveGxT( &(nmpcWorkspace.evGx[ 320 ]), nmpcWorkspace.T );
nmpc_multGxd( &(nmpcWorkspace.d[ 32 ]), &(nmpcWorkspace.evGx[ 320 ]), &(nmpcWorkspace.d[ 40 ]) );
nmpc_multGxGx( nmpcWorkspace.T, &(nmpcWorkspace.evGx[ 256 ]), &(nmpcWorkspace.evGx[ 320 ]) );

nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 320 ]), &(nmpcWorkspace.E[ 480 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 352 ]), &(nmpcWorkspace.E[ 512 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 384 ]), &(nmpcWorkspace.E[ 544 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 416 ]), &(nmpcWorkspace.E[ 576 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 448 ]), &(nmpcWorkspace.E[ 608 ]) );

nmpc_moveGuE( &(nmpcWorkspace.evGu[ 160 ]), &(nmpcWorkspace.E[ 640 ]) );

nmpc_moveGxT( &(nmpcWorkspace.evGx[ 384 ]), nmpcWorkspace.T );
nmpc_multGxd( &(nmpcWorkspace.d[ 40 ]), &(nmpcWorkspace.evGx[ 384 ]), &(nmpcWorkspace.d[ 48 ]) );
nmpc_multGxGx( nmpcWorkspace.T, &(nmpcWorkspace.evGx[ 320 ]), &(nmpcWorkspace.evGx[ 384 ]) );

nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 480 ]), &(nmpcWorkspace.E[ 672 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 512 ]), &(nmpcWorkspace.E[ 704 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 544 ]), &(nmpcWorkspace.E[ 736 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 576 ]), &(nmpcWorkspace.E[ 768 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 608 ]), &(nmpcWorkspace.E[ 800 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 640 ]), &(nmpcWorkspace.E[ 832 ]) );

nmpc_moveGuE( &(nmpcWorkspace.evGu[ 192 ]), &(nmpcWorkspace.E[ 864 ]) );

nmpc_moveGxT( &(nmpcWorkspace.evGx[ 448 ]), nmpcWorkspace.T );
nmpc_multGxd( &(nmpcWorkspace.d[ 48 ]), &(nmpcWorkspace.evGx[ 448 ]), &(nmpcWorkspace.d[ 56 ]) );
nmpc_multGxGx( nmpcWorkspace.T, &(nmpcWorkspace.evGx[ 384 ]), &(nmpcWorkspace.evGx[ 448 ]) );

nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 672 ]), &(nmpcWorkspace.E[ 896 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 704 ]), &(nmpcWorkspace.E[ 928 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 736 ]), &(nmpcWorkspace.E[ 960 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 768 ]), &(nmpcWorkspace.E[ 992 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 800 ]), &(nmpcWorkspace.E[ 1024 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 832 ]), &(nmpcWorkspace.E[ 1056 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 864 ]), &(nmpcWorkspace.E[ 1088 ]) );

nmpc_moveGuE( &(nmpcWorkspace.evGu[ 224 ]), &(nmpcWorkspace.E[ 1120 ]) );

nmpc_moveGxT( &(nmpcWorkspace.evGx[ 512 ]), nmpcWorkspace.T );
nmpc_multGxd( &(nmpcWorkspace.d[ 56 ]), &(nmpcWorkspace.evGx[ 512 ]), &(nmpcWorkspace.d[ 64 ]) );
nmpc_multGxGx( nmpcWorkspace.T, &(nmpcWorkspace.evGx[ 448 ]), &(nmpcWorkspace.evGx[ 512 ]) );

nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 896 ]), &(nmpcWorkspace.E[ 1152 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 928 ]), &(nmpcWorkspace.E[ 1184 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 960 ]), &(nmpcWorkspace.E[ 1216 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 992 ]), &(nmpcWorkspace.E[ 1248 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 1024 ]), &(nmpcWorkspace.E[ 1280 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 1056 ]), &(nmpcWorkspace.E[ 1312 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 1088 ]), &(nmpcWorkspace.E[ 1344 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 1120 ]), &(nmpcWorkspace.E[ 1376 ]) );

nmpc_moveGuE( &(nmpcWorkspace.evGu[ 256 ]), &(nmpcWorkspace.E[ 1408 ]) );

nmpc_moveGxT( &(nmpcWorkspace.evGx[ 576 ]), nmpcWorkspace.T );
nmpc_multGxd( &(nmpcWorkspace.d[ 64 ]), &(nmpcWorkspace.evGx[ 576 ]), &(nmpcWorkspace.d[ 72 ]) );
nmpc_multGxGx( nmpcWorkspace.T, &(nmpcWorkspace.evGx[ 512 ]), &(nmpcWorkspace.evGx[ 576 ]) );

nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 1152 ]), &(nmpcWorkspace.E[ 1440 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 1184 ]), &(nmpcWorkspace.E[ 1472 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 1216 ]), &(nmpcWorkspace.E[ 1504 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 1248 ]), &(nmpcWorkspace.E[ 1536 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 1280 ]), &(nmpcWorkspace.E[ 1568 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 1312 ]), &(nmpcWorkspace.E[ 1600 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 1344 ]), &(nmpcWorkspace.E[ 1632 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 1376 ]), &(nmpcWorkspace.E[ 1664 ]) );
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ 1408 ]), &(nmpcWorkspace.E[ 1696 ]) );

nmpc_moveGuE( &(nmpcWorkspace.evGu[ 288 ]), &(nmpcWorkspace.E[ 1728 ]) );

nmpc_multGxGx( &(nmpcWorkspace.Q1[ 64 ]), nmpcWorkspace.evGx, nmpcWorkspace.QGx );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 128 ]), &(nmpcWorkspace.evGx[ 64 ]), &(nmpcWorkspace.QGx[ 64 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 192 ]), &(nmpcWorkspace.evGx[ 128 ]), &(nmpcWorkspace.QGx[ 128 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 256 ]), &(nmpcWorkspace.evGx[ 192 ]), &(nmpcWorkspace.QGx[ 192 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 320 ]), &(nmpcWorkspace.evGx[ 256 ]), &(nmpcWorkspace.QGx[ 256 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 384 ]), &(nmpcWorkspace.evGx[ 320 ]), &(nmpcWorkspace.QGx[ 320 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 448 ]), &(nmpcWorkspace.evGx[ 384 ]), &(nmpcWorkspace.QGx[ 384 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 512 ]), &(nmpcWorkspace.evGx[ 448 ]), &(nmpcWorkspace.QGx[ 448 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 576 ]), &(nmpcWorkspace.evGx[ 512 ]), &(nmpcWorkspace.QGx[ 512 ]) );
nmpc_multGxGx( nmpcWorkspace.QN1, &(nmpcWorkspace.evGx[ 576 ]), &(nmpcWorkspace.QGx[ 576 ]) );

nmpc_multGxGu( &(nmpcWorkspace.Q1[ 64 ]), nmpcWorkspace.E, nmpcWorkspace.QE );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 128 ]), &(nmpcWorkspace.E[ 32 ]), &(nmpcWorkspace.QE[ 32 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 128 ]), &(nmpcWorkspace.E[ 64 ]), &(nmpcWorkspace.QE[ 64 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 192 ]), &(nmpcWorkspace.E[ 96 ]), &(nmpcWorkspace.QE[ 96 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 192 ]), &(nmpcWorkspace.E[ 128 ]), &(nmpcWorkspace.QE[ 128 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 192 ]), &(nmpcWorkspace.E[ 160 ]), &(nmpcWorkspace.QE[ 160 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 256 ]), &(nmpcWorkspace.E[ 192 ]), &(nmpcWorkspace.QE[ 192 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 256 ]), &(nmpcWorkspace.E[ 224 ]), &(nmpcWorkspace.QE[ 224 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 256 ]), &(nmpcWorkspace.E[ 256 ]), &(nmpcWorkspace.QE[ 256 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 256 ]), &(nmpcWorkspace.E[ 288 ]), &(nmpcWorkspace.QE[ 288 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 320 ]), &(nmpcWorkspace.E[ 320 ]), &(nmpcWorkspace.QE[ 320 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 320 ]), &(nmpcWorkspace.E[ 352 ]), &(nmpcWorkspace.QE[ 352 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 320 ]), &(nmpcWorkspace.E[ 384 ]), &(nmpcWorkspace.QE[ 384 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 320 ]), &(nmpcWorkspace.E[ 416 ]), &(nmpcWorkspace.QE[ 416 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 320 ]), &(nmpcWorkspace.E[ 448 ]), &(nmpcWorkspace.QE[ 448 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 384 ]), &(nmpcWorkspace.E[ 480 ]), &(nmpcWorkspace.QE[ 480 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 384 ]), &(nmpcWorkspace.E[ 512 ]), &(nmpcWorkspace.QE[ 512 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 384 ]), &(nmpcWorkspace.E[ 544 ]), &(nmpcWorkspace.QE[ 544 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 384 ]), &(nmpcWorkspace.E[ 576 ]), &(nmpcWorkspace.QE[ 576 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 384 ]), &(nmpcWorkspace.E[ 608 ]), &(nmpcWorkspace.QE[ 608 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 384 ]), &(nmpcWorkspace.E[ 640 ]), &(nmpcWorkspace.QE[ 640 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 448 ]), &(nmpcWorkspace.E[ 672 ]), &(nmpcWorkspace.QE[ 672 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 448 ]), &(nmpcWorkspace.E[ 704 ]), &(nmpcWorkspace.QE[ 704 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 448 ]), &(nmpcWorkspace.E[ 736 ]), &(nmpcWorkspace.QE[ 736 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 448 ]), &(nmpcWorkspace.E[ 768 ]), &(nmpcWorkspace.QE[ 768 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 448 ]), &(nmpcWorkspace.E[ 800 ]), &(nmpcWorkspace.QE[ 800 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 448 ]), &(nmpcWorkspace.E[ 832 ]), &(nmpcWorkspace.QE[ 832 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 448 ]), &(nmpcWorkspace.E[ 864 ]), &(nmpcWorkspace.QE[ 864 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 512 ]), &(nmpcWorkspace.E[ 896 ]), &(nmpcWorkspace.QE[ 896 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 512 ]), &(nmpcWorkspace.E[ 928 ]), &(nmpcWorkspace.QE[ 928 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 512 ]), &(nmpcWorkspace.E[ 960 ]), &(nmpcWorkspace.QE[ 960 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 512 ]), &(nmpcWorkspace.E[ 992 ]), &(nmpcWorkspace.QE[ 992 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 512 ]), &(nmpcWorkspace.E[ 1024 ]), &(nmpcWorkspace.QE[ 1024 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 512 ]), &(nmpcWorkspace.E[ 1056 ]), &(nmpcWorkspace.QE[ 1056 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 512 ]), &(nmpcWorkspace.E[ 1088 ]), &(nmpcWorkspace.QE[ 1088 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 512 ]), &(nmpcWorkspace.E[ 1120 ]), &(nmpcWorkspace.QE[ 1120 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 576 ]), &(nmpcWorkspace.E[ 1152 ]), &(nmpcWorkspace.QE[ 1152 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 576 ]), &(nmpcWorkspace.E[ 1184 ]), &(nmpcWorkspace.QE[ 1184 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 576 ]), &(nmpcWorkspace.E[ 1216 ]), &(nmpcWorkspace.QE[ 1216 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 576 ]), &(nmpcWorkspace.E[ 1248 ]), &(nmpcWorkspace.QE[ 1248 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 576 ]), &(nmpcWorkspace.E[ 1280 ]), &(nmpcWorkspace.QE[ 1280 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 576 ]), &(nmpcWorkspace.E[ 1312 ]), &(nmpcWorkspace.QE[ 1312 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 576 ]), &(nmpcWorkspace.E[ 1344 ]), &(nmpcWorkspace.QE[ 1344 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 576 ]), &(nmpcWorkspace.E[ 1376 ]), &(nmpcWorkspace.QE[ 1376 ]) );
nmpc_multGxGu( &(nmpcWorkspace.Q1[ 576 ]), &(nmpcWorkspace.E[ 1408 ]), &(nmpcWorkspace.QE[ 1408 ]) );
nmpc_multGxGu( nmpcWorkspace.QN1, &(nmpcWorkspace.E[ 1440 ]), &(nmpcWorkspace.QE[ 1440 ]) );
nmpc_multGxGu( nmpcWorkspace.QN1, &(nmpcWorkspace.E[ 1472 ]), &(nmpcWorkspace.QE[ 1472 ]) );
nmpc_multGxGu( nmpcWorkspace.QN1, &(nmpcWorkspace.E[ 1504 ]), &(nmpcWorkspace.QE[ 1504 ]) );
nmpc_multGxGu( nmpcWorkspace.QN1, &(nmpcWorkspace.E[ 1536 ]), &(nmpcWorkspace.QE[ 1536 ]) );
nmpc_multGxGu( nmpcWorkspace.QN1, &(nmpcWorkspace.E[ 1568 ]), &(nmpcWorkspace.QE[ 1568 ]) );
nmpc_multGxGu( nmpcWorkspace.QN1, &(nmpcWorkspace.E[ 1600 ]), &(nmpcWorkspace.QE[ 1600 ]) );
nmpc_multGxGu( nmpcWorkspace.QN1, &(nmpcWorkspace.E[ 1632 ]), &(nmpcWorkspace.QE[ 1632 ]) );
nmpc_multGxGu( nmpcWorkspace.QN1, &(nmpcWorkspace.E[ 1664 ]), &(nmpcWorkspace.QE[ 1664 ]) );
nmpc_multGxGu( nmpcWorkspace.QN1, &(nmpcWorkspace.E[ 1696 ]), &(nmpcWorkspace.QE[ 1696 ]) );
nmpc_multGxGu( nmpcWorkspace.QN1, &(nmpcWorkspace.E[ 1728 ]), &(nmpcWorkspace.QE[ 1728 ]) );

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

nmpc_zeroBlockH10( nmpcWorkspace.H10 );
nmpc_multQETGx( nmpcWorkspace.QE, nmpcWorkspace.evGx, nmpcWorkspace.H10 );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 32 ]), &(nmpcWorkspace.evGx[ 64 ]), nmpcWorkspace.H10 );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 96 ]), &(nmpcWorkspace.evGx[ 128 ]), nmpcWorkspace.H10 );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 192 ]), &(nmpcWorkspace.evGx[ 192 ]), nmpcWorkspace.H10 );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 320 ]), &(nmpcWorkspace.evGx[ 256 ]), nmpcWorkspace.H10 );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 480 ]), &(nmpcWorkspace.evGx[ 320 ]), nmpcWorkspace.H10 );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 672 ]), &(nmpcWorkspace.evGx[ 384 ]), nmpcWorkspace.H10 );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 896 ]), &(nmpcWorkspace.evGx[ 448 ]), nmpcWorkspace.H10 );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1152 ]), &(nmpcWorkspace.evGx[ 512 ]), nmpcWorkspace.H10 );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1440 ]), &(nmpcWorkspace.evGx[ 576 ]), nmpcWorkspace.H10 );
nmpc_zeroBlockH10( &(nmpcWorkspace.H10[ 32 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 64 ]), &(nmpcWorkspace.evGx[ 64 ]), &(nmpcWorkspace.H10[ 32 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 128 ]), &(nmpcWorkspace.evGx[ 128 ]), &(nmpcWorkspace.H10[ 32 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 224 ]), &(nmpcWorkspace.evGx[ 192 ]), &(nmpcWorkspace.H10[ 32 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 352 ]), &(nmpcWorkspace.evGx[ 256 ]), &(nmpcWorkspace.H10[ 32 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 512 ]), &(nmpcWorkspace.evGx[ 320 ]), &(nmpcWorkspace.H10[ 32 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 704 ]), &(nmpcWorkspace.evGx[ 384 ]), &(nmpcWorkspace.H10[ 32 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 928 ]), &(nmpcWorkspace.evGx[ 448 ]), &(nmpcWorkspace.H10[ 32 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1184 ]), &(nmpcWorkspace.evGx[ 512 ]), &(nmpcWorkspace.H10[ 32 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1472 ]), &(nmpcWorkspace.evGx[ 576 ]), &(nmpcWorkspace.H10[ 32 ]) );
nmpc_zeroBlockH10( &(nmpcWorkspace.H10[ 64 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 160 ]), &(nmpcWorkspace.evGx[ 128 ]), &(nmpcWorkspace.H10[ 64 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 256 ]), &(nmpcWorkspace.evGx[ 192 ]), &(nmpcWorkspace.H10[ 64 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 384 ]), &(nmpcWorkspace.evGx[ 256 ]), &(nmpcWorkspace.H10[ 64 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 544 ]), &(nmpcWorkspace.evGx[ 320 ]), &(nmpcWorkspace.H10[ 64 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 736 ]), &(nmpcWorkspace.evGx[ 384 ]), &(nmpcWorkspace.H10[ 64 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 960 ]), &(nmpcWorkspace.evGx[ 448 ]), &(nmpcWorkspace.H10[ 64 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1216 ]), &(nmpcWorkspace.evGx[ 512 ]), &(nmpcWorkspace.H10[ 64 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1504 ]), &(nmpcWorkspace.evGx[ 576 ]), &(nmpcWorkspace.H10[ 64 ]) );
nmpc_zeroBlockH10( &(nmpcWorkspace.H10[ 96 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 288 ]), &(nmpcWorkspace.evGx[ 192 ]), &(nmpcWorkspace.H10[ 96 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 416 ]), &(nmpcWorkspace.evGx[ 256 ]), &(nmpcWorkspace.H10[ 96 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 576 ]), &(nmpcWorkspace.evGx[ 320 ]), &(nmpcWorkspace.H10[ 96 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 768 ]), &(nmpcWorkspace.evGx[ 384 ]), &(nmpcWorkspace.H10[ 96 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 992 ]), &(nmpcWorkspace.evGx[ 448 ]), &(nmpcWorkspace.H10[ 96 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1248 ]), &(nmpcWorkspace.evGx[ 512 ]), &(nmpcWorkspace.H10[ 96 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1536 ]), &(nmpcWorkspace.evGx[ 576 ]), &(nmpcWorkspace.H10[ 96 ]) );
nmpc_zeroBlockH10( &(nmpcWorkspace.H10[ 128 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 448 ]), &(nmpcWorkspace.evGx[ 256 ]), &(nmpcWorkspace.H10[ 128 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 608 ]), &(nmpcWorkspace.evGx[ 320 ]), &(nmpcWorkspace.H10[ 128 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 800 ]), &(nmpcWorkspace.evGx[ 384 ]), &(nmpcWorkspace.H10[ 128 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1024 ]), &(nmpcWorkspace.evGx[ 448 ]), &(nmpcWorkspace.H10[ 128 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1280 ]), &(nmpcWorkspace.evGx[ 512 ]), &(nmpcWorkspace.H10[ 128 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1568 ]), &(nmpcWorkspace.evGx[ 576 ]), &(nmpcWorkspace.H10[ 128 ]) );
nmpc_zeroBlockH10( &(nmpcWorkspace.H10[ 160 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 640 ]), &(nmpcWorkspace.evGx[ 320 ]), &(nmpcWorkspace.H10[ 160 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 832 ]), &(nmpcWorkspace.evGx[ 384 ]), &(nmpcWorkspace.H10[ 160 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1056 ]), &(nmpcWorkspace.evGx[ 448 ]), &(nmpcWorkspace.H10[ 160 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1312 ]), &(nmpcWorkspace.evGx[ 512 ]), &(nmpcWorkspace.H10[ 160 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1600 ]), &(nmpcWorkspace.evGx[ 576 ]), &(nmpcWorkspace.H10[ 160 ]) );
nmpc_zeroBlockH10( &(nmpcWorkspace.H10[ 192 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 864 ]), &(nmpcWorkspace.evGx[ 384 ]), &(nmpcWorkspace.H10[ 192 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1088 ]), &(nmpcWorkspace.evGx[ 448 ]), &(nmpcWorkspace.H10[ 192 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1344 ]), &(nmpcWorkspace.evGx[ 512 ]), &(nmpcWorkspace.H10[ 192 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1632 ]), &(nmpcWorkspace.evGx[ 576 ]), &(nmpcWorkspace.H10[ 192 ]) );
nmpc_zeroBlockH10( &(nmpcWorkspace.H10[ 224 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1120 ]), &(nmpcWorkspace.evGx[ 448 ]), &(nmpcWorkspace.H10[ 224 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1376 ]), &(nmpcWorkspace.evGx[ 512 ]), &(nmpcWorkspace.H10[ 224 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1664 ]), &(nmpcWorkspace.evGx[ 576 ]), &(nmpcWorkspace.H10[ 224 ]) );
nmpc_zeroBlockH10( &(nmpcWorkspace.H10[ 256 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1408 ]), &(nmpcWorkspace.evGx[ 512 ]), &(nmpcWorkspace.H10[ 256 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1696 ]), &(nmpcWorkspace.evGx[ 576 ]), &(nmpcWorkspace.H10[ 256 ]) );
nmpc_zeroBlockH10( &(nmpcWorkspace.H10[ 288 ]) );
nmpc_multQETGx( &(nmpcWorkspace.QE[ 1728 ]), &(nmpcWorkspace.evGx[ 576 ]), &(nmpcWorkspace.H10[ 288 ]) );

for (lRun1 = 0;lRun1 < 8; ++lRun1)
for (lRun2 = 0;lRun2 < 40; ++lRun2)
nmpcWorkspace.H[(lRun1 * 48) + (lRun2 + 8)] = nmpcWorkspace.H10[(lRun2 * 8) + (lRun1)];

nmpc_setBlockH11_R1( 0, 0, nmpcWorkspace.R1 );
nmpc_setBlockH11( 0, 0, nmpcWorkspace.E, nmpcWorkspace.QE );
nmpc_setBlockH11( 0, 0, &(nmpcWorkspace.E[ 32 ]), &(nmpcWorkspace.QE[ 32 ]) );
nmpc_setBlockH11( 0, 0, &(nmpcWorkspace.E[ 96 ]), &(nmpcWorkspace.QE[ 96 ]) );
nmpc_setBlockH11( 0, 0, &(nmpcWorkspace.E[ 192 ]), &(nmpcWorkspace.QE[ 192 ]) );
nmpc_setBlockH11( 0, 0, &(nmpcWorkspace.E[ 320 ]), &(nmpcWorkspace.QE[ 320 ]) );
nmpc_setBlockH11( 0, 0, &(nmpcWorkspace.E[ 480 ]), &(nmpcWorkspace.QE[ 480 ]) );
nmpc_setBlockH11( 0, 0, &(nmpcWorkspace.E[ 672 ]), &(nmpcWorkspace.QE[ 672 ]) );
nmpc_setBlockH11( 0, 0, &(nmpcWorkspace.E[ 896 ]), &(nmpcWorkspace.QE[ 896 ]) );
nmpc_setBlockH11( 0, 0, &(nmpcWorkspace.E[ 1152 ]), &(nmpcWorkspace.QE[ 1152 ]) );
nmpc_setBlockH11( 0, 0, &(nmpcWorkspace.E[ 1440 ]), &(nmpcWorkspace.QE[ 1440 ]) );

nmpc_zeroBlockH11( 0, 1 );
nmpc_setBlockH11( 0, 1, &(nmpcWorkspace.E[ 32 ]), &(nmpcWorkspace.QE[ 64 ]) );
nmpc_setBlockH11( 0, 1, &(nmpcWorkspace.E[ 96 ]), &(nmpcWorkspace.QE[ 128 ]) );
nmpc_setBlockH11( 0, 1, &(nmpcWorkspace.E[ 192 ]), &(nmpcWorkspace.QE[ 224 ]) );
nmpc_setBlockH11( 0, 1, &(nmpcWorkspace.E[ 320 ]), &(nmpcWorkspace.QE[ 352 ]) );
nmpc_setBlockH11( 0, 1, &(nmpcWorkspace.E[ 480 ]), &(nmpcWorkspace.QE[ 512 ]) );
nmpc_setBlockH11( 0, 1, &(nmpcWorkspace.E[ 672 ]), &(nmpcWorkspace.QE[ 704 ]) );
nmpc_setBlockH11( 0, 1, &(nmpcWorkspace.E[ 896 ]), &(nmpcWorkspace.QE[ 928 ]) );
nmpc_setBlockH11( 0, 1, &(nmpcWorkspace.E[ 1152 ]), &(nmpcWorkspace.QE[ 1184 ]) );
nmpc_setBlockH11( 0, 1, &(nmpcWorkspace.E[ 1440 ]), &(nmpcWorkspace.QE[ 1472 ]) );

nmpc_zeroBlockH11( 0, 2 );
nmpc_setBlockH11( 0, 2, &(nmpcWorkspace.E[ 96 ]), &(nmpcWorkspace.QE[ 160 ]) );
nmpc_setBlockH11( 0, 2, &(nmpcWorkspace.E[ 192 ]), &(nmpcWorkspace.QE[ 256 ]) );
nmpc_setBlockH11( 0, 2, &(nmpcWorkspace.E[ 320 ]), &(nmpcWorkspace.QE[ 384 ]) );
nmpc_setBlockH11( 0, 2, &(nmpcWorkspace.E[ 480 ]), &(nmpcWorkspace.QE[ 544 ]) );
nmpc_setBlockH11( 0, 2, &(nmpcWorkspace.E[ 672 ]), &(nmpcWorkspace.QE[ 736 ]) );
nmpc_setBlockH11( 0, 2, &(nmpcWorkspace.E[ 896 ]), &(nmpcWorkspace.QE[ 960 ]) );
nmpc_setBlockH11( 0, 2, &(nmpcWorkspace.E[ 1152 ]), &(nmpcWorkspace.QE[ 1216 ]) );
nmpc_setBlockH11( 0, 2, &(nmpcWorkspace.E[ 1440 ]), &(nmpcWorkspace.QE[ 1504 ]) );

nmpc_zeroBlockH11( 0, 3 );
nmpc_setBlockH11( 0, 3, &(nmpcWorkspace.E[ 192 ]), &(nmpcWorkspace.QE[ 288 ]) );
nmpc_setBlockH11( 0, 3, &(nmpcWorkspace.E[ 320 ]), &(nmpcWorkspace.QE[ 416 ]) );
nmpc_setBlockH11( 0, 3, &(nmpcWorkspace.E[ 480 ]), &(nmpcWorkspace.QE[ 576 ]) );
nmpc_setBlockH11( 0, 3, &(nmpcWorkspace.E[ 672 ]), &(nmpcWorkspace.QE[ 768 ]) );
nmpc_setBlockH11( 0, 3, &(nmpcWorkspace.E[ 896 ]), &(nmpcWorkspace.QE[ 992 ]) );
nmpc_setBlockH11( 0, 3, &(nmpcWorkspace.E[ 1152 ]), &(nmpcWorkspace.QE[ 1248 ]) );
nmpc_setBlockH11( 0, 3, &(nmpcWorkspace.E[ 1440 ]), &(nmpcWorkspace.QE[ 1536 ]) );

nmpc_zeroBlockH11( 0, 4 );
nmpc_setBlockH11( 0, 4, &(nmpcWorkspace.E[ 320 ]), &(nmpcWorkspace.QE[ 448 ]) );
nmpc_setBlockH11( 0, 4, &(nmpcWorkspace.E[ 480 ]), &(nmpcWorkspace.QE[ 608 ]) );
nmpc_setBlockH11( 0, 4, &(nmpcWorkspace.E[ 672 ]), &(nmpcWorkspace.QE[ 800 ]) );
nmpc_setBlockH11( 0, 4, &(nmpcWorkspace.E[ 896 ]), &(nmpcWorkspace.QE[ 1024 ]) );
nmpc_setBlockH11( 0, 4, &(nmpcWorkspace.E[ 1152 ]), &(nmpcWorkspace.QE[ 1280 ]) );
nmpc_setBlockH11( 0, 4, &(nmpcWorkspace.E[ 1440 ]), &(nmpcWorkspace.QE[ 1568 ]) );

nmpc_zeroBlockH11( 0, 5 );
nmpc_setBlockH11( 0, 5, &(nmpcWorkspace.E[ 480 ]), &(nmpcWorkspace.QE[ 640 ]) );
nmpc_setBlockH11( 0, 5, &(nmpcWorkspace.E[ 672 ]), &(nmpcWorkspace.QE[ 832 ]) );
nmpc_setBlockH11( 0, 5, &(nmpcWorkspace.E[ 896 ]), &(nmpcWorkspace.QE[ 1056 ]) );
nmpc_setBlockH11( 0, 5, &(nmpcWorkspace.E[ 1152 ]), &(nmpcWorkspace.QE[ 1312 ]) );
nmpc_setBlockH11( 0, 5, &(nmpcWorkspace.E[ 1440 ]), &(nmpcWorkspace.QE[ 1600 ]) );

nmpc_zeroBlockH11( 0, 6 );
nmpc_setBlockH11( 0, 6, &(nmpcWorkspace.E[ 672 ]), &(nmpcWorkspace.QE[ 864 ]) );
nmpc_setBlockH11( 0, 6, &(nmpcWorkspace.E[ 896 ]), &(nmpcWorkspace.QE[ 1088 ]) );
nmpc_setBlockH11( 0, 6, &(nmpcWorkspace.E[ 1152 ]), &(nmpcWorkspace.QE[ 1344 ]) );
nmpc_setBlockH11( 0, 6, &(nmpcWorkspace.E[ 1440 ]), &(nmpcWorkspace.QE[ 1632 ]) );

nmpc_zeroBlockH11( 0, 7 );
nmpc_setBlockH11( 0, 7, &(nmpcWorkspace.E[ 896 ]), &(nmpcWorkspace.QE[ 1120 ]) );
nmpc_setBlockH11( 0, 7, &(nmpcWorkspace.E[ 1152 ]), &(nmpcWorkspace.QE[ 1376 ]) );
nmpc_setBlockH11( 0, 7, &(nmpcWorkspace.E[ 1440 ]), &(nmpcWorkspace.QE[ 1664 ]) );

nmpc_zeroBlockH11( 0, 8 );
nmpc_setBlockH11( 0, 8, &(nmpcWorkspace.E[ 1152 ]), &(nmpcWorkspace.QE[ 1408 ]) );
nmpc_setBlockH11( 0, 8, &(nmpcWorkspace.E[ 1440 ]), &(nmpcWorkspace.QE[ 1696 ]) );

nmpc_zeroBlockH11( 0, 9 );
nmpc_setBlockH11( 0, 9, &(nmpcWorkspace.E[ 1440 ]), &(nmpcWorkspace.QE[ 1728 ]) );

nmpc_setBlockH11_R1( 1, 1, &(nmpcWorkspace.R1[ 16 ]) );
nmpc_setBlockH11( 1, 1, &(nmpcWorkspace.E[ 64 ]), &(nmpcWorkspace.QE[ 64 ]) );
nmpc_setBlockH11( 1, 1, &(nmpcWorkspace.E[ 128 ]), &(nmpcWorkspace.QE[ 128 ]) );
nmpc_setBlockH11( 1, 1, &(nmpcWorkspace.E[ 224 ]), &(nmpcWorkspace.QE[ 224 ]) );
nmpc_setBlockH11( 1, 1, &(nmpcWorkspace.E[ 352 ]), &(nmpcWorkspace.QE[ 352 ]) );
nmpc_setBlockH11( 1, 1, &(nmpcWorkspace.E[ 512 ]), &(nmpcWorkspace.QE[ 512 ]) );
nmpc_setBlockH11( 1, 1, &(nmpcWorkspace.E[ 704 ]), &(nmpcWorkspace.QE[ 704 ]) );
nmpc_setBlockH11( 1, 1, &(nmpcWorkspace.E[ 928 ]), &(nmpcWorkspace.QE[ 928 ]) );
nmpc_setBlockH11( 1, 1, &(nmpcWorkspace.E[ 1184 ]), &(nmpcWorkspace.QE[ 1184 ]) );
nmpc_setBlockH11( 1, 1, &(nmpcWorkspace.E[ 1472 ]), &(nmpcWorkspace.QE[ 1472 ]) );

nmpc_zeroBlockH11( 1, 2 );
nmpc_setBlockH11( 1, 2, &(nmpcWorkspace.E[ 128 ]), &(nmpcWorkspace.QE[ 160 ]) );
nmpc_setBlockH11( 1, 2, &(nmpcWorkspace.E[ 224 ]), &(nmpcWorkspace.QE[ 256 ]) );
nmpc_setBlockH11( 1, 2, &(nmpcWorkspace.E[ 352 ]), &(nmpcWorkspace.QE[ 384 ]) );
nmpc_setBlockH11( 1, 2, &(nmpcWorkspace.E[ 512 ]), &(nmpcWorkspace.QE[ 544 ]) );
nmpc_setBlockH11( 1, 2, &(nmpcWorkspace.E[ 704 ]), &(nmpcWorkspace.QE[ 736 ]) );
nmpc_setBlockH11( 1, 2, &(nmpcWorkspace.E[ 928 ]), &(nmpcWorkspace.QE[ 960 ]) );
nmpc_setBlockH11( 1, 2, &(nmpcWorkspace.E[ 1184 ]), &(nmpcWorkspace.QE[ 1216 ]) );
nmpc_setBlockH11( 1, 2, &(nmpcWorkspace.E[ 1472 ]), &(nmpcWorkspace.QE[ 1504 ]) );

nmpc_zeroBlockH11( 1, 3 );
nmpc_setBlockH11( 1, 3, &(nmpcWorkspace.E[ 224 ]), &(nmpcWorkspace.QE[ 288 ]) );
nmpc_setBlockH11( 1, 3, &(nmpcWorkspace.E[ 352 ]), &(nmpcWorkspace.QE[ 416 ]) );
nmpc_setBlockH11( 1, 3, &(nmpcWorkspace.E[ 512 ]), &(nmpcWorkspace.QE[ 576 ]) );
nmpc_setBlockH11( 1, 3, &(nmpcWorkspace.E[ 704 ]), &(nmpcWorkspace.QE[ 768 ]) );
nmpc_setBlockH11( 1, 3, &(nmpcWorkspace.E[ 928 ]), &(nmpcWorkspace.QE[ 992 ]) );
nmpc_setBlockH11( 1, 3, &(nmpcWorkspace.E[ 1184 ]), &(nmpcWorkspace.QE[ 1248 ]) );
nmpc_setBlockH11( 1, 3, &(nmpcWorkspace.E[ 1472 ]), &(nmpcWorkspace.QE[ 1536 ]) );

nmpc_zeroBlockH11( 1, 4 );
nmpc_setBlockH11( 1, 4, &(nmpcWorkspace.E[ 352 ]), &(nmpcWorkspace.QE[ 448 ]) );
nmpc_setBlockH11( 1, 4, &(nmpcWorkspace.E[ 512 ]), &(nmpcWorkspace.QE[ 608 ]) );
nmpc_setBlockH11( 1, 4, &(nmpcWorkspace.E[ 704 ]), &(nmpcWorkspace.QE[ 800 ]) );
nmpc_setBlockH11( 1, 4, &(nmpcWorkspace.E[ 928 ]), &(nmpcWorkspace.QE[ 1024 ]) );
nmpc_setBlockH11( 1, 4, &(nmpcWorkspace.E[ 1184 ]), &(nmpcWorkspace.QE[ 1280 ]) );
nmpc_setBlockH11( 1, 4, &(nmpcWorkspace.E[ 1472 ]), &(nmpcWorkspace.QE[ 1568 ]) );

nmpc_zeroBlockH11( 1, 5 );
nmpc_setBlockH11( 1, 5, &(nmpcWorkspace.E[ 512 ]), &(nmpcWorkspace.QE[ 640 ]) );
nmpc_setBlockH11( 1, 5, &(nmpcWorkspace.E[ 704 ]), &(nmpcWorkspace.QE[ 832 ]) );
nmpc_setBlockH11( 1, 5, &(nmpcWorkspace.E[ 928 ]), &(nmpcWorkspace.QE[ 1056 ]) );
nmpc_setBlockH11( 1, 5, &(nmpcWorkspace.E[ 1184 ]), &(nmpcWorkspace.QE[ 1312 ]) );
nmpc_setBlockH11( 1, 5, &(nmpcWorkspace.E[ 1472 ]), &(nmpcWorkspace.QE[ 1600 ]) );

nmpc_zeroBlockH11( 1, 6 );
nmpc_setBlockH11( 1, 6, &(nmpcWorkspace.E[ 704 ]), &(nmpcWorkspace.QE[ 864 ]) );
nmpc_setBlockH11( 1, 6, &(nmpcWorkspace.E[ 928 ]), &(nmpcWorkspace.QE[ 1088 ]) );
nmpc_setBlockH11( 1, 6, &(nmpcWorkspace.E[ 1184 ]), &(nmpcWorkspace.QE[ 1344 ]) );
nmpc_setBlockH11( 1, 6, &(nmpcWorkspace.E[ 1472 ]), &(nmpcWorkspace.QE[ 1632 ]) );

nmpc_zeroBlockH11( 1, 7 );
nmpc_setBlockH11( 1, 7, &(nmpcWorkspace.E[ 928 ]), &(nmpcWorkspace.QE[ 1120 ]) );
nmpc_setBlockH11( 1, 7, &(nmpcWorkspace.E[ 1184 ]), &(nmpcWorkspace.QE[ 1376 ]) );
nmpc_setBlockH11( 1, 7, &(nmpcWorkspace.E[ 1472 ]), &(nmpcWorkspace.QE[ 1664 ]) );

nmpc_zeroBlockH11( 1, 8 );
nmpc_setBlockH11( 1, 8, &(nmpcWorkspace.E[ 1184 ]), &(nmpcWorkspace.QE[ 1408 ]) );
nmpc_setBlockH11( 1, 8, &(nmpcWorkspace.E[ 1472 ]), &(nmpcWorkspace.QE[ 1696 ]) );

nmpc_zeroBlockH11( 1, 9 );
nmpc_setBlockH11( 1, 9, &(nmpcWorkspace.E[ 1472 ]), &(nmpcWorkspace.QE[ 1728 ]) );

nmpc_setBlockH11_R1( 2, 2, &(nmpcWorkspace.R1[ 32 ]) );
nmpc_setBlockH11( 2, 2, &(nmpcWorkspace.E[ 160 ]), &(nmpcWorkspace.QE[ 160 ]) );
nmpc_setBlockH11( 2, 2, &(nmpcWorkspace.E[ 256 ]), &(nmpcWorkspace.QE[ 256 ]) );
nmpc_setBlockH11( 2, 2, &(nmpcWorkspace.E[ 384 ]), &(nmpcWorkspace.QE[ 384 ]) );
nmpc_setBlockH11( 2, 2, &(nmpcWorkspace.E[ 544 ]), &(nmpcWorkspace.QE[ 544 ]) );
nmpc_setBlockH11( 2, 2, &(nmpcWorkspace.E[ 736 ]), &(nmpcWorkspace.QE[ 736 ]) );
nmpc_setBlockH11( 2, 2, &(nmpcWorkspace.E[ 960 ]), &(nmpcWorkspace.QE[ 960 ]) );
nmpc_setBlockH11( 2, 2, &(nmpcWorkspace.E[ 1216 ]), &(nmpcWorkspace.QE[ 1216 ]) );
nmpc_setBlockH11( 2, 2, &(nmpcWorkspace.E[ 1504 ]), &(nmpcWorkspace.QE[ 1504 ]) );

nmpc_zeroBlockH11( 2, 3 );
nmpc_setBlockH11( 2, 3, &(nmpcWorkspace.E[ 256 ]), &(nmpcWorkspace.QE[ 288 ]) );
nmpc_setBlockH11( 2, 3, &(nmpcWorkspace.E[ 384 ]), &(nmpcWorkspace.QE[ 416 ]) );
nmpc_setBlockH11( 2, 3, &(nmpcWorkspace.E[ 544 ]), &(nmpcWorkspace.QE[ 576 ]) );
nmpc_setBlockH11( 2, 3, &(nmpcWorkspace.E[ 736 ]), &(nmpcWorkspace.QE[ 768 ]) );
nmpc_setBlockH11( 2, 3, &(nmpcWorkspace.E[ 960 ]), &(nmpcWorkspace.QE[ 992 ]) );
nmpc_setBlockH11( 2, 3, &(nmpcWorkspace.E[ 1216 ]), &(nmpcWorkspace.QE[ 1248 ]) );
nmpc_setBlockH11( 2, 3, &(nmpcWorkspace.E[ 1504 ]), &(nmpcWorkspace.QE[ 1536 ]) );

nmpc_zeroBlockH11( 2, 4 );
nmpc_setBlockH11( 2, 4, &(nmpcWorkspace.E[ 384 ]), &(nmpcWorkspace.QE[ 448 ]) );
nmpc_setBlockH11( 2, 4, &(nmpcWorkspace.E[ 544 ]), &(nmpcWorkspace.QE[ 608 ]) );
nmpc_setBlockH11( 2, 4, &(nmpcWorkspace.E[ 736 ]), &(nmpcWorkspace.QE[ 800 ]) );
nmpc_setBlockH11( 2, 4, &(nmpcWorkspace.E[ 960 ]), &(nmpcWorkspace.QE[ 1024 ]) );
nmpc_setBlockH11( 2, 4, &(nmpcWorkspace.E[ 1216 ]), &(nmpcWorkspace.QE[ 1280 ]) );
nmpc_setBlockH11( 2, 4, &(nmpcWorkspace.E[ 1504 ]), &(nmpcWorkspace.QE[ 1568 ]) );

nmpc_zeroBlockH11( 2, 5 );
nmpc_setBlockH11( 2, 5, &(nmpcWorkspace.E[ 544 ]), &(nmpcWorkspace.QE[ 640 ]) );
nmpc_setBlockH11( 2, 5, &(nmpcWorkspace.E[ 736 ]), &(nmpcWorkspace.QE[ 832 ]) );
nmpc_setBlockH11( 2, 5, &(nmpcWorkspace.E[ 960 ]), &(nmpcWorkspace.QE[ 1056 ]) );
nmpc_setBlockH11( 2, 5, &(nmpcWorkspace.E[ 1216 ]), &(nmpcWorkspace.QE[ 1312 ]) );
nmpc_setBlockH11( 2, 5, &(nmpcWorkspace.E[ 1504 ]), &(nmpcWorkspace.QE[ 1600 ]) );

nmpc_zeroBlockH11( 2, 6 );
nmpc_setBlockH11( 2, 6, &(nmpcWorkspace.E[ 736 ]), &(nmpcWorkspace.QE[ 864 ]) );
nmpc_setBlockH11( 2, 6, &(nmpcWorkspace.E[ 960 ]), &(nmpcWorkspace.QE[ 1088 ]) );
nmpc_setBlockH11( 2, 6, &(nmpcWorkspace.E[ 1216 ]), &(nmpcWorkspace.QE[ 1344 ]) );
nmpc_setBlockH11( 2, 6, &(nmpcWorkspace.E[ 1504 ]), &(nmpcWorkspace.QE[ 1632 ]) );

nmpc_zeroBlockH11( 2, 7 );
nmpc_setBlockH11( 2, 7, &(nmpcWorkspace.E[ 960 ]), &(nmpcWorkspace.QE[ 1120 ]) );
nmpc_setBlockH11( 2, 7, &(nmpcWorkspace.E[ 1216 ]), &(nmpcWorkspace.QE[ 1376 ]) );
nmpc_setBlockH11( 2, 7, &(nmpcWorkspace.E[ 1504 ]), &(nmpcWorkspace.QE[ 1664 ]) );

nmpc_zeroBlockH11( 2, 8 );
nmpc_setBlockH11( 2, 8, &(nmpcWorkspace.E[ 1216 ]), &(nmpcWorkspace.QE[ 1408 ]) );
nmpc_setBlockH11( 2, 8, &(nmpcWorkspace.E[ 1504 ]), &(nmpcWorkspace.QE[ 1696 ]) );

nmpc_zeroBlockH11( 2, 9 );
nmpc_setBlockH11( 2, 9, &(nmpcWorkspace.E[ 1504 ]), &(nmpcWorkspace.QE[ 1728 ]) );

nmpc_setBlockH11_R1( 3, 3, &(nmpcWorkspace.R1[ 48 ]) );
nmpc_setBlockH11( 3, 3, &(nmpcWorkspace.E[ 288 ]), &(nmpcWorkspace.QE[ 288 ]) );
nmpc_setBlockH11( 3, 3, &(nmpcWorkspace.E[ 416 ]), &(nmpcWorkspace.QE[ 416 ]) );
nmpc_setBlockH11( 3, 3, &(nmpcWorkspace.E[ 576 ]), &(nmpcWorkspace.QE[ 576 ]) );
nmpc_setBlockH11( 3, 3, &(nmpcWorkspace.E[ 768 ]), &(nmpcWorkspace.QE[ 768 ]) );
nmpc_setBlockH11( 3, 3, &(nmpcWorkspace.E[ 992 ]), &(nmpcWorkspace.QE[ 992 ]) );
nmpc_setBlockH11( 3, 3, &(nmpcWorkspace.E[ 1248 ]), &(nmpcWorkspace.QE[ 1248 ]) );
nmpc_setBlockH11( 3, 3, &(nmpcWorkspace.E[ 1536 ]), &(nmpcWorkspace.QE[ 1536 ]) );

nmpc_zeroBlockH11( 3, 4 );
nmpc_setBlockH11( 3, 4, &(nmpcWorkspace.E[ 416 ]), &(nmpcWorkspace.QE[ 448 ]) );
nmpc_setBlockH11( 3, 4, &(nmpcWorkspace.E[ 576 ]), &(nmpcWorkspace.QE[ 608 ]) );
nmpc_setBlockH11( 3, 4, &(nmpcWorkspace.E[ 768 ]), &(nmpcWorkspace.QE[ 800 ]) );
nmpc_setBlockH11( 3, 4, &(nmpcWorkspace.E[ 992 ]), &(nmpcWorkspace.QE[ 1024 ]) );
nmpc_setBlockH11( 3, 4, &(nmpcWorkspace.E[ 1248 ]), &(nmpcWorkspace.QE[ 1280 ]) );
nmpc_setBlockH11( 3, 4, &(nmpcWorkspace.E[ 1536 ]), &(nmpcWorkspace.QE[ 1568 ]) );

nmpc_zeroBlockH11( 3, 5 );
nmpc_setBlockH11( 3, 5, &(nmpcWorkspace.E[ 576 ]), &(nmpcWorkspace.QE[ 640 ]) );
nmpc_setBlockH11( 3, 5, &(nmpcWorkspace.E[ 768 ]), &(nmpcWorkspace.QE[ 832 ]) );
nmpc_setBlockH11( 3, 5, &(nmpcWorkspace.E[ 992 ]), &(nmpcWorkspace.QE[ 1056 ]) );
nmpc_setBlockH11( 3, 5, &(nmpcWorkspace.E[ 1248 ]), &(nmpcWorkspace.QE[ 1312 ]) );
nmpc_setBlockH11( 3, 5, &(nmpcWorkspace.E[ 1536 ]), &(nmpcWorkspace.QE[ 1600 ]) );

nmpc_zeroBlockH11( 3, 6 );
nmpc_setBlockH11( 3, 6, &(nmpcWorkspace.E[ 768 ]), &(nmpcWorkspace.QE[ 864 ]) );
nmpc_setBlockH11( 3, 6, &(nmpcWorkspace.E[ 992 ]), &(nmpcWorkspace.QE[ 1088 ]) );
nmpc_setBlockH11( 3, 6, &(nmpcWorkspace.E[ 1248 ]), &(nmpcWorkspace.QE[ 1344 ]) );
nmpc_setBlockH11( 3, 6, &(nmpcWorkspace.E[ 1536 ]), &(nmpcWorkspace.QE[ 1632 ]) );

nmpc_zeroBlockH11( 3, 7 );
nmpc_setBlockH11( 3, 7, &(nmpcWorkspace.E[ 992 ]), &(nmpcWorkspace.QE[ 1120 ]) );
nmpc_setBlockH11( 3, 7, &(nmpcWorkspace.E[ 1248 ]), &(nmpcWorkspace.QE[ 1376 ]) );
nmpc_setBlockH11( 3, 7, &(nmpcWorkspace.E[ 1536 ]), &(nmpcWorkspace.QE[ 1664 ]) );

nmpc_zeroBlockH11( 3, 8 );
nmpc_setBlockH11( 3, 8, &(nmpcWorkspace.E[ 1248 ]), &(nmpcWorkspace.QE[ 1408 ]) );
nmpc_setBlockH11( 3, 8, &(nmpcWorkspace.E[ 1536 ]), &(nmpcWorkspace.QE[ 1696 ]) );

nmpc_zeroBlockH11( 3, 9 );
nmpc_setBlockH11( 3, 9, &(nmpcWorkspace.E[ 1536 ]), &(nmpcWorkspace.QE[ 1728 ]) );

nmpc_setBlockH11_R1( 4, 4, &(nmpcWorkspace.R1[ 64 ]) );
nmpc_setBlockH11( 4, 4, &(nmpcWorkspace.E[ 448 ]), &(nmpcWorkspace.QE[ 448 ]) );
nmpc_setBlockH11( 4, 4, &(nmpcWorkspace.E[ 608 ]), &(nmpcWorkspace.QE[ 608 ]) );
nmpc_setBlockH11( 4, 4, &(nmpcWorkspace.E[ 800 ]), &(nmpcWorkspace.QE[ 800 ]) );
nmpc_setBlockH11( 4, 4, &(nmpcWorkspace.E[ 1024 ]), &(nmpcWorkspace.QE[ 1024 ]) );
nmpc_setBlockH11( 4, 4, &(nmpcWorkspace.E[ 1280 ]), &(nmpcWorkspace.QE[ 1280 ]) );
nmpc_setBlockH11( 4, 4, &(nmpcWorkspace.E[ 1568 ]), &(nmpcWorkspace.QE[ 1568 ]) );

nmpc_zeroBlockH11( 4, 5 );
nmpc_setBlockH11( 4, 5, &(nmpcWorkspace.E[ 608 ]), &(nmpcWorkspace.QE[ 640 ]) );
nmpc_setBlockH11( 4, 5, &(nmpcWorkspace.E[ 800 ]), &(nmpcWorkspace.QE[ 832 ]) );
nmpc_setBlockH11( 4, 5, &(nmpcWorkspace.E[ 1024 ]), &(nmpcWorkspace.QE[ 1056 ]) );
nmpc_setBlockH11( 4, 5, &(nmpcWorkspace.E[ 1280 ]), &(nmpcWorkspace.QE[ 1312 ]) );
nmpc_setBlockH11( 4, 5, &(nmpcWorkspace.E[ 1568 ]), &(nmpcWorkspace.QE[ 1600 ]) );

nmpc_zeroBlockH11( 4, 6 );
nmpc_setBlockH11( 4, 6, &(nmpcWorkspace.E[ 800 ]), &(nmpcWorkspace.QE[ 864 ]) );
nmpc_setBlockH11( 4, 6, &(nmpcWorkspace.E[ 1024 ]), &(nmpcWorkspace.QE[ 1088 ]) );
nmpc_setBlockH11( 4, 6, &(nmpcWorkspace.E[ 1280 ]), &(nmpcWorkspace.QE[ 1344 ]) );
nmpc_setBlockH11( 4, 6, &(nmpcWorkspace.E[ 1568 ]), &(nmpcWorkspace.QE[ 1632 ]) );

nmpc_zeroBlockH11( 4, 7 );
nmpc_setBlockH11( 4, 7, &(nmpcWorkspace.E[ 1024 ]), &(nmpcWorkspace.QE[ 1120 ]) );
nmpc_setBlockH11( 4, 7, &(nmpcWorkspace.E[ 1280 ]), &(nmpcWorkspace.QE[ 1376 ]) );
nmpc_setBlockH11( 4, 7, &(nmpcWorkspace.E[ 1568 ]), &(nmpcWorkspace.QE[ 1664 ]) );

nmpc_zeroBlockH11( 4, 8 );
nmpc_setBlockH11( 4, 8, &(nmpcWorkspace.E[ 1280 ]), &(nmpcWorkspace.QE[ 1408 ]) );
nmpc_setBlockH11( 4, 8, &(nmpcWorkspace.E[ 1568 ]), &(nmpcWorkspace.QE[ 1696 ]) );

nmpc_zeroBlockH11( 4, 9 );
nmpc_setBlockH11( 4, 9, &(nmpcWorkspace.E[ 1568 ]), &(nmpcWorkspace.QE[ 1728 ]) );

nmpc_setBlockH11_R1( 5, 5, &(nmpcWorkspace.R1[ 80 ]) );
nmpc_setBlockH11( 5, 5, &(nmpcWorkspace.E[ 640 ]), &(nmpcWorkspace.QE[ 640 ]) );
nmpc_setBlockH11( 5, 5, &(nmpcWorkspace.E[ 832 ]), &(nmpcWorkspace.QE[ 832 ]) );
nmpc_setBlockH11( 5, 5, &(nmpcWorkspace.E[ 1056 ]), &(nmpcWorkspace.QE[ 1056 ]) );
nmpc_setBlockH11( 5, 5, &(nmpcWorkspace.E[ 1312 ]), &(nmpcWorkspace.QE[ 1312 ]) );
nmpc_setBlockH11( 5, 5, &(nmpcWorkspace.E[ 1600 ]), &(nmpcWorkspace.QE[ 1600 ]) );

nmpc_zeroBlockH11( 5, 6 );
nmpc_setBlockH11( 5, 6, &(nmpcWorkspace.E[ 832 ]), &(nmpcWorkspace.QE[ 864 ]) );
nmpc_setBlockH11( 5, 6, &(nmpcWorkspace.E[ 1056 ]), &(nmpcWorkspace.QE[ 1088 ]) );
nmpc_setBlockH11( 5, 6, &(nmpcWorkspace.E[ 1312 ]), &(nmpcWorkspace.QE[ 1344 ]) );
nmpc_setBlockH11( 5, 6, &(nmpcWorkspace.E[ 1600 ]), &(nmpcWorkspace.QE[ 1632 ]) );

nmpc_zeroBlockH11( 5, 7 );
nmpc_setBlockH11( 5, 7, &(nmpcWorkspace.E[ 1056 ]), &(nmpcWorkspace.QE[ 1120 ]) );
nmpc_setBlockH11( 5, 7, &(nmpcWorkspace.E[ 1312 ]), &(nmpcWorkspace.QE[ 1376 ]) );
nmpc_setBlockH11( 5, 7, &(nmpcWorkspace.E[ 1600 ]), &(nmpcWorkspace.QE[ 1664 ]) );

nmpc_zeroBlockH11( 5, 8 );
nmpc_setBlockH11( 5, 8, &(nmpcWorkspace.E[ 1312 ]), &(nmpcWorkspace.QE[ 1408 ]) );
nmpc_setBlockH11( 5, 8, &(nmpcWorkspace.E[ 1600 ]), &(nmpcWorkspace.QE[ 1696 ]) );

nmpc_zeroBlockH11( 5, 9 );
nmpc_setBlockH11( 5, 9, &(nmpcWorkspace.E[ 1600 ]), &(nmpcWorkspace.QE[ 1728 ]) );

nmpc_setBlockH11_R1( 6, 6, &(nmpcWorkspace.R1[ 96 ]) );
nmpc_setBlockH11( 6, 6, &(nmpcWorkspace.E[ 864 ]), &(nmpcWorkspace.QE[ 864 ]) );
nmpc_setBlockH11( 6, 6, &(nmpcWorkspace.E[ 1088 ]), &(nmpcWorkspace.QE[ 1088 ]) );
nmpc_setBlockH11( 6, 6, &(nmpcWorkspace.E[ 1344 ]), &(nmpcWorkspace.QE[ 1344 ]) );
nmpc_setBlockH11( 6, 6, &(nmpcWorkspace.E[ 1632 ]), &(nmpcWorkspace.QE[ 1632 ]) );

nmpc_zeroBlockH11( 6, 7 );
nmpc_setBlockH11( 6, 7, &(nmpcWorkspace.E[ 1088 ]), &(nmpcWorkspace.QE[ 1120 ]) );
nmpc_setBlockH11( 6, 7, &(nmpcWorkspace.E[ 1344 ]), &(nmpcWorkspace.QE[ 1376 ]) );
nmpc_setBlockH11( 6, 7, &(nmpcWorkspace.E[ 1632 ]), &(nmpcWorkspace.QE[ 1664 ]) );

nmpc_zeroBlockH11( 6, 8 );
nmpc_setBlockH11( 6, 8, &(nmpcWorkspace.E[ 1344 ]), &(nmpcWorkspace.QE[ 1408 ]) );
nmpc_setBlockH11( 6, 8, &(nmpcWorkspace.E[ 1632 ]), &(nmpcWorkspace.QE[ 1696 ]) );

nmpc_zeroBlockH11( 6, 9 );
nmpc_setBlockH11( 6, 9, &(nmpcWorkspace.E[ 1632 ]), &(nmpcWorkspace.QE[ 1728 ]) );

nmpc_setBlockH11_R1( 7, 7, &(nmpcWorkspace.R1[ 112 ]) );
nmpc_setBlockH11( 7, 7, &(nmpcWorkspace.E[ 1120 ]), &(nmpcWorkspace.QE[ 1120 ]) );
nmpc_setBlockH11( 7, 7, &(nmpcWorkspace.E[ 1376 ]), &(nmpcWorkspace.QE[ 1376 ]) );
nmpc_setBlockH11( 7, 7, &(nmpcWorkspace.E[ 1664 ]), &(nmpcWorkspace.QE[ 1664 ]) );

nmpc_zeroBlockH11( 7, 8 );
nmpc_setBlockH11( 7, 8, &(nmpcWorkspace.E[ 1376 ]), &(nmpcWorkspace.QE[ 1408 ]) );
nmpc_setBlockH11( 7, 8, &(nmpcWorkspace.E[ 1664 ]), &(nmpcWorkspace.QE[ 1696 ]) );

nmpc_zeroBlockH11( 7, 9 );
nmpc_setBlockH11( 7, 9, &(nmpcWorkspace.E[ 1664 ]), &(nmpcWorkspace.QE[ 1728 ]) );

nmpc_setBlockH11_R1( 8, 8, &(nmpcWorkspace.R1[ 128 ]) );
nmpc_setBlockH11( 8, 8, &(nmpcWorkspace.E[ 1408 ]), &(nmpcWorkspace.QE[ 1408 ]) );
nmpc_setBlockH11( 8, 8, &(nmpcWorkspace.E[ 1696 ]), &(nmpcWorkspace.QE[ 1696 ]) );

nmpc_zeroBlockH11( 8, 9 );
nmpc_setBlockH11( 8, 9, &(nmpcWorkspace.E[ 1696 ]), &(nmpcWorkspace.QE[ 1728 ]) );

nmpc_setBlockH11_R1( 9, 9, &(nmpcWorkspace.R1[ 144 ]) );
nmpc_setBlockH11( 9, 9, &(nmpcWorkspace.E[ 1728 ]), &(nmpcWorkspace.QE[ 1728 ]) );


nmpc_copyHTH( 1, 0 );
nmpc_copyHTH( 2, 0 );
nmpc_copyHTH( 2, 1 );
nmpc_copyHTH( 3, 0 );
nmpc_copyHTH( 3, 1 );
nmpc_copyHTH( 3, 2 );
nmpc_copyHTH( 4, 0 );
nmpc_copyHTH( 4, 1 );
nmpc_copyHTH( 4, 2 );
nmpc_copyHTH( 4, 3 );
nmpc_copyHTH( 5, 0 );
nmpc_copyHTH( 5, 1 );
nmpc_copyHTH( 5, 2 );
nmpc_copyHTH( 5, 3 );
nmpc_copyHTH( 5, 4 );
nmpc_copyHTH( 6, 0 );
nmpc_copyHTH( 6, 1 );
nmpc_copyHTH( 6, 2 );
nmpc_copyHTH( 6, 3 );
nmpc_copyHTH( 6, 4 );
nmpc_copyHTH( 6, 5 );
nmpc_copyHTH( 7, 0 );
nmpc_copyHTH( 7, 1 );
nmpc_copyHTH( 7, 2 );
nmpc_copyHTH( 7, 3 );
nmpc_copyHTH( 7, 4 );
nmpc_copyHTH( 7, 5 );
nmpc_copyHTH( 7, 6 );
nmpc_copyHTH( 8, 0 );
nmpc_copyHTH( 8, 1 );
nmpc_copyHTH( 8, 2 );
nmpc_copyHTH( 8, 3 );
nmpc_copyHTH( 8, 4 );
nmpc_copyHTH( 8, 5 );
nmpc_copyHTH( 8, 6 );
nmpc_copyHTH( 8, 7 );
nmpc_copyHTH( 9, 0 );
nmpc_copyHTH( 9, 1 );
nmpc_copyHTH( 9, 2 );
nmpc_copyHTH( 9, 3 );
nmpc_copyHTH( 9, 4 );
nmpc_copyHTH( 9, 5 );
nmpc_copyHTH( 9, 6 );
nmpc_copyHTH( 9, 7 );
nmpc_copyHTH( 9, 8 );

for (lRun1 = 0;lRun1 < 40; ++lRun1)
for (lRun2 = 0;lRun2 < 8; ++lRun2)
nmpcWorkspace.H[(lRun1 * 48 + 384) + (lRun2)] = nmpcWorkspace.H10[(lRun1 * 8) + (lRun2)];

nmpc_multQ1d( &(nmpcWorkspace.Q1[ 64 ]), nmpcWorkspace.d, nmpcWorkspace.Qd );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 128 ]), &(nmpcWorkspace.d[ 8 ]), &(nmpcWorkspace.Qd[ 8 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 192 ]), &(nmpcWorkspace.d[ 16 ]), &(nmpcWorkspace.Qd[ 16 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 256 ]), &(nmpcWorkspace.d[ 24 ]), &(nmpcWorkspace.Qd[ 24 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 320 ]), &(nmpcWorkspace.d[ 32 ]), &(nmpcWorkspace.Qd[ 32 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 384 ]), &(nmpcWorkspace.d[ 40 ]), &(nmpcWorkspace.Qd[ 40 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 448 ]), &(nmpcWorkspace.d[ 48 ]), &(nmpcWorkspace.Qd[ 48 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 512 ]), &(nmpcWorkspace.d[ 56 ]), &(nmpcWorkspace.Qd[ 56 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 576 ]), &(nmpcWorkspace.d[ 64 ]), &(nmpcWorkspace.Qd[ 64 ]) );
nmpc_multQN1d( nmpcWorkspace.QN1, &(nmpcWorkspace.d[ 72 ]), &(nmpcWorkspace.Qd[ 72 ]) );

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
nmpc_macETSlu( nmpcWorkspace.QE, &(nmpcWorkspace.g[ 8 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 32 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 96 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 192 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 320 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 480 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 672 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 896 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1152 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1440 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 64 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 128 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 224 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 352 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 512 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 704 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 928 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1184 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1472 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 160 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 256 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 384 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 544 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 736 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 960 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1216 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1504 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 288 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 416 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 576 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 768 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 992 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1248 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1536 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 448 ]), &(nmpcWorkspace.g[ 24 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 608 ]), &(nmpcWorkspace.g[ 24 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 800 ]), &(nmpcWorkspace.g[ 24 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1024 ]), &(nmpcWorkspace.g[ 24 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1280 ]), &(nmpcWorkspace.g[ 24 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1568 ]), &(nmpcWorkspace.g[ 24 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 640 ]), &(nmpcWorkspace.g[ 28 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 832 ]), &(nmpcWorkspace.g[ 28 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1056 ]), &(nmpcWorkspace.g[ 28 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1312 ]), &(nmpcWorkspace.g[ 28 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1600 ]), &(nmpcWorkspace.g[ 28 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 864 ]), &(nmpcWorkspace.g[ 32 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1088 ]), &(nmpcWorkspace.g[ 32 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1344 ]), &(nmpcWorkspace.g[ 32 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1632 ]), &(nmpcWorkspace.g[ 32 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1120 ]), &(nmpcWorkspace.g[ 36 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1376 ]), &(nmpcWorkspace.g[ 36 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1664 ]), &(nmpcWorkspace.g[ 36 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1408 ]), &(nmpcWorkspace.g[ 40 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1696 ]), &(nmpcWorkspace.g[ 40 ]) );
nmpc_macETSlu( &(nmpcWorkspace.QE[ 1728 ]), &(nmpcWorkspace.g[ 44 ]) );
nmpcWorkspace.lb[8] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[0];
nmpcWorkspace.lb[9] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[1];
nmpcWorkspace.lb[10] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[2];
nmpcWorkspace.lb[11] = (real_t)-1.0000000000000000e+02 - nmpcVariables.u[3];
nmpcWorkspace.lb[12] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[4];
nmpcWorkspace.lb[13] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[5];
nmpcWorkspace.lb[14] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[6];
nmpcWorkspace.lb[15] = (real_t)-1.0000000000000000e+02 - nmpcVariables.u[7];
nmpcWorkspace.lb[16] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[8];
nmpcWorkspace.lb[17] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[9];
nmpcWorkspace.lb[18] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[10];
nmpcWorkspace.lb[19] = (real_t)-1.0000000000000000e+02 - nmpcVariables.u[11];
nmpcWorkspace.lb[20] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[12];
nmpcWorkspace.lb[21] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[13];
nmpcWorkspace.lb[22] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[14];
nmpcWorkspace.lb[23] = (real_t)-1.0000000000000000e+02 - nmpcVariables.u[15];
nmpcWorkspace.lb[24] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[16];
nmpcWorkspace.lb[25] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[17];
nmpcWorkspace.lb[26] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[18];
nmpcWorkspace.lb[27] = (real_t)-1.0000000000000000e+02 - nmpcVariables.u[19];
nmpcWorkspace.lb[28] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[20];
nmpcWorkspace.lb[29] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[21];
nmpcWorkspace.lb[30] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[22];
nmpcWorkspace.lb[31] = (real_t)-1.0000000000000000e+02 - nmpcVariables.u[23];
nmpcWorkspace.lb[32] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[24];
nmpcWorkspace.lb[33] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[25];
nmpcWorkspace.lb[34] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[26];
nmpcWorkspace.lb[35] = (real_t)-1.0000000000000000e+02 - nmpcVariables.u[27];
nmpcWorkspace.lb[36] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[28];
nmpcWorkspace.lb[37] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[29];
nmpcWorkspace.lb[38] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[30];
nmpcWorkspace.lb[39] = (real_t)-1.0000000000000000e+02 - nmpcVariables.u[31];
nmpcWorkspace.lb[40] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[32];
nmpcWorkspace.lb[41] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[33];
nmpcWorkspace.lb[42] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[34];
nmpcWorkspace.lb[43] = (real_t)-1.0000000000000000e+02 - nmpcVariables.u[35];
nmpcWorkspace.lb[44] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[36];
nmpcWorkspace.lb[45] = (real_t)-4.0000000000000000e+01 - nmpcVariables.u[37];
nmpcWorkspace.lb[46] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[38];
nmpcWorkspace.lb[47] = (real_t)-1.0000000000000000e+02 - nmpcVariables.u[39];
nmpcWorkspace.ub[8] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[0];
nmpcWorkspace.ub[9] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[1];
nmpcWorkspace.ub[10] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[2];
nmpcWorkspace.ub[11] = (real_t)1.0000000000000000e+02 - nmpcVariables.u[3];
nmpcWorkspace.ub[12] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[4];
nmpcWorkspace.ub[13] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[5];
nmpcWorkspace.ub[14] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[6];
nmpcWorkspace.ub[15] = (real_t)1.0000000000000000e+02 - nmpcVariables.u[7];
nmpcWorkspace.ub[16] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[8];
nmpcWorkspace.ub[17] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[9];
nmpcWorkspace.ub[18] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[10];
nmpcWorkspace.ub[19] = (real_t)1.0000000000000000e+02 - nmpcVariables.u[11];
nmpcWorkspace.ub[20] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[12];
nmpcWorkspace.ub[21] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[13];
nmpcWorkspace.ub[22] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[14];
nmpcWorkspace.ub[23] = (real_t)1.0000000000000000e+02 - nmpcVariables.u[15];
nmpcWorkspace.ub[24] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[16];
nmpcWorkspace.ub[25] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[17];
nmpcWorkspace.ub[26] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[18];
nmpcWorkspace.ub[27] = (real_t)1.0000000000000000e+02 - nmpcVariables.u[19];
nmpcWorkspace.ub[28] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[20];
nmpcWorkspace.ub[29] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[21];
nmpcWorkspace.ub[30] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[22];
nmpcWorkspace.ub[31] = (real_t)1.0000000000000000e+02 - nmpcVariables.u[23];
nmpcWorkspace.ub[32] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[24];
nmpcWorkspace.ub[33] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[25];
nmpcWorkspace.ub[34] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[26];
nmpcWorkspace.ub[35] = (real_t)1.0000000000000000e+02 - nmpcVariables.u[27];
nmpcWorkspace.ub[36] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[28];
nmpcWorkspace.ub[37] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[29];
nmpcWorkspace.ub[38] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[30];
nmpcWorkspace.ub[39] = (real_t)1.0000000000000000e+02 - nmpcVariables.u[31];
nmpcWorkspace.ub[40] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[32];
nmpcWorkspace.ub[41] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[33];
nmpcWorkspace.ub[42] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[34];
nmpcWorkspace.ub[43] = (real_t)1.0000000000000000e+02 - nmpcVariables.u[35];
nmpcWorkspace.ub[44] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[36];
nmpcWorkspace.ub[45] = (real_t)4.0000000000000000e+01 - nmpcVariables.u[37];
nmpcWorkspace.ub[46] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[38];
nmpcWorkspace.ub[47] = (real_t)1.0000000000000000e+02 - nmpcVariables.u[39];

}

void nmpc_condenseFdb(  )
{
nmpcWorkspace.Dx0[0] = nmpcVariables.x0[0] - nmpcVariables.x[0];
nmpcWorkspace.Dx0[1] = nmpcVariables.x0[1] - nmpcVariables.x[1];
nmpcWorkspace.Dx0[2] = nmpcVariables.x0[2] - nmpcVariables.x[2];
nmpcWorkspace.Dx0[3] = nmpcVariables.x0[3] - nmpcVariables.x[3];
nmpcWorkspace.Dx0[4] = nmpcVariables.x0[4] - nmpcVariables.x[4];
nmpcWorkspace.Dx0[5] = nmpcVariables.x0[5] - nmpcVariables.x[5];
nmpcWorkspace.Dx0[6] = nmpcVariables.x0[6] - nmpcVariables.x[6];
nmpcWorkspace.Dx0[7] = nmpcVariables.x0[7] - nmpcVariables.x[7];

nmpcWorkspace.Dy[0] -= nmpcVariables.y[0];
nmpcWorkspace.Dy[1] -= nmpcVariables.y[1];
nmpcWorkspace.Dy[2] -= nmpcVariables.y[2];
nmpcWorkspace.Dy[3] -= nmpcVariables.y[3];
nmpcWorkspace.Dy[4] -= nmpcVariables.y[4];
nmpcWorkspace.Dy[5] -= nmpcVariables.y[5];
nmpcWorkspace.Dy[6] -= nmpcVariables.y[6];
nmpcWorkspace.Dy[7] -= nmpcVariables.y[7];
nmpcWorkspace.Dy[8] -= nmpcVariables.y[8];
nmpcWorkspace.Dy[9] -= nmpcVariables.y[9];
nmpcWorkspace.Dy[10] -= nmpcVariables.y[10];
nmpcWorkspace.Dy[11] -= nmpcVariables.y[11];
nmpcWorkspace.Dy[12] -= nmpcVariables.y[12];
nmpcWorkspace.Dy[13] -= nmpcVariables.y[13];
nmpcWorkspace.Dy[14] -= nmpcVariables.y[14];
nmpcWorkspace.Dy[15] -= nmpcVariables.y[15];
nmpcWorkspace.Dy[16] -= nmpcVariables.y[16];
nmpcWorkspace.Dy[17] -= nmpcVariables.y[17];
nmpcWorkspace.Dy[18] -= nmpcVariables.y[18];
nmpcWorkspace.Dy[19] -= nmpcVariables.y[19];
nmpcWorkspace.Dy[20] -= nmpcVariables.y[20];
nmpcWorkspace.Dy[21] -= nmpcVariables.y[21];
nmpcWorkspace.Dy[22] -= nmpcVariables.y[22];
nmpcWorkspace.Dy[23] -= nmpcVariables.y[23];
nmpcWorkspace.Dy[24] -= nmpcVariables.y[24];
nmpcWorkspace.Dy[25] -= nmpcVariables.y[25];
nmpcWorkspace.Dy[26] -= nmpcVariables.y[26];
nmpcWorkspace.Dy[27] -= nmpcVariables.y[27];
nmpcWorkspace.Dy[28] -= nmpcVariables.y[28];
nmpcWorkspace.Dy[29] -= nmpcVariables.y[29];
nmpcWorkspace.Dy[30] -= nmpcVariables.y[30];
nmpcWorkspace.Dy[31] -= nmpcVariables.y[31];
nmpcWorkspace.Dy[32] -= nmpcVariables.y[32];
nmpcWorkspace.Dy[33] -= nmpcVariables.y[33];
nmpcWorkspace.Dy[34] -= nmpcVariables.y[34];
nmpcWorkspace.Dy[35] -= nmpcVariables.y[35];
nmpcWorkspace.Dy[36] -= nmpcVariables.y[36];
nmpcWorkspace.Dy[37] -= nmpcVariables.y[37];
nmpcWorkspace.Dy[38] -= nmpcVariables.y[38];
nmpcWorkspace.Dy[39] -= nmpcVariables.y[39];
nmpcWorkspace.Dy[40] -= nmpcVariables.y[40];
nmpcWorkspace.Dy[41] -= nmpcVariables.y[41];
nmpcWorkspace.Dy[42] -= nmpcVariables.y[42];
nmpcWorkspace.Dy[43] -= nmpcVariables.y[43];
nmpcWorkspace.Dy[44] -= nmpcVariables.y[44];
nmpcWorkspace.Dy[45] -= nmpcVariables.y[45];
nmpcWorkspace.Dy[46] -= nmpcVariables.y[46];
nmpcWorkspace.Dy[47] -= nmpcVariables.y[47];
nmpcWorkspace.Dy[48] -= nmpcVariables.y[48];
nmpcWorkspace.Dy[49] -= nmpcVariables.y[49];
nmpcWorkspace.Dy[50] -= nmpcVariables.y[50];
nmpcWorkspace.Dy[51] -= nmpcVariables.y[51];
nmpcWorkspace.Dy[52] -= nmpcVariables.y[52];
nmpcWorkspace.Dy[53] -= nmpcVariables.y[53];
nmpcWorkspace.Dy[54] -= nmpcVariables.y[54];
nmpcWorkspace.Dy[55] -= nmpcVariables.y[55];
nmpcWorkspace.Dy[56] -= nmpcVariables.y[56];
nmpcWorkspace.Dy[57] -= nmpcVariables.y[57];
nmpcWorkspace.Dy[58] -= nmpcVariables.y[58];
nmpcWorkspace.Dy[59] -= nmpcVariables.y[59];
nmpcWorkspace.Dy[60] -= nmpcVariables.y[60];
nmpcWorkspace.Dy[61] -= nmpcVariables.y[61];
nmpcWorkspace.Dy[62] -= nmpcVariables.y[62];
nmpcWorkspace.Dy[63] -= nmpcVariables.y[63];
nmpcWorkspace.Dy[64] -= nmpcVariables.y[64];
nmpcWorkspace.Dy[65] -= nmpcVariables.y[65];
nmpcWorkspace.Dy[66] -= nmpcVariables.y[66];
nmpcWorkspace.Dy[67] -= nmpcVariables.y[67];
nmpcWorkspace.Dy[68] -= nmpcVariables.y[68];
nmpcWorkspace.Dy[69] -= nmpcVariables.y[69];
nmpcWorkspace.Dy[70] -= nmpcVariables.y[70];
nmpcWorkspace.Dy[71] -= nmpcVariables.y[71];
nmpcWorkspace.Dy[72] -= nmpcVariables.y[72];
nmpcWorkspace.Dy[73] -= nmpcVariables.y[73];
nmpcWorkspace.Dy[74] -= nmpcVariables.y[74];
nmpcWorkspace.Dy[75] -= nmpcVariables.y[75];
nmpcWorkspace.Dy[76] -= nmpcVariables.y[76];
nmpcWorkspace.Dy[77] -= nmpcVariables.y[77];
nmpcWorkspace.Dy[78] -= nmpcVariables.y[78];
nmpcWorkspace.Dy[79] -= nmpcVariables.y[79];
nmpcWorkspace.Dy[80] -= nmpcVariables.y[80];
nmpcWorkspace.Dy[81] -= nmpcVariables.y[81];
nmpcWorkspace.Dy[82] -= nmpcVariables.y[82];
nmpcWorkspace.Dy[83] -= nmpcVariables.y[83];
nmpcWorkspace.Dy[84] -= nmpcVariables.y[84];
nmpcWorkspace.Dy[85] -= nmpcVariables.y[85];
nmpcWorkspace.Dy[86] -= nmpcVariables.y[86];
nmpcWorkspace.Dy[87] -= nmpcVariables.y[87];
nmpcWorkspace.Dy[88] -= nmpcVariables.y[88];
nmpcWorkspace.Dy[89] -= nmpcVariables.y[89];
nmpcWorkspace.Dy[90] -= nmpcVariables.y[90];
nmpcWorkspace.Dy[91] -= nmpcVariables.y[91];
nmpcWorkspace.Dy[92] -= nmpcVariables.y[92];
nmpcWorkspace.Dy[93] -= nmpcVariables.y[93];
nmpcWorkspace.Dy[94] -= nmpcVariables.y[94];
nmpcWorkspace.Dy[95] -= nmpcVariables.y[95];
nmpcWorkspace.Dy[96] -= nmpcVariables.y[96];
nmpcWorkspace.Dy[97] -= nmpcVariables.y[97];
nmpcWorkspace.Dy[98] -= nmpcVariables.y[98];
nmpcWorkspace.Dy[99] -= nmpcVariables.y[99];
nmpcWorkspace.Dy[100] -= nmpcVariables.y[100];
nmpcWorkspace.Dy[101] -= nmpcVariables.y[101];
nmpcWorkspace.Dy[102] -= nmpcVariables.y[102];
nmpcWorkspace.Dy[103] -= nmpcVariables.y[103];
nmpcWorkspace.Dy[104] -= nmpcVariables.y[104];
nmpcWorkspace.Dy[105] -= nmpcVariables.y[105];
nmpcWorkspace.Dy[106] -= nmpcVariables.y[106];
nmpcWorkspace.Dy[107] -= nmpcVariables.y[107];
nmpcWorkspace.Dy[108] -= nmpcVariables.y[108];
nmpcWorkspace.Dy[109] -= nmpcVariables.y[109];
nmpcWorkspace.Dy[110] -= nmpcVariables.y[110];
nmpcWorkspace.Dy[111] -= nmpcVariables.y[111];
nmpcWorkspace.Dy[112] -= nmpcVariables.y[112];
nmpcWorkspace.Dy[113] -= nmpcVariables.y[113];
nmpcWorkspace.Dy[114] -= nmpcVariables.y[114];
nmpcWorkspace.Dy[115] -= nmpcVariables.y[115];
nmpcWorkspace.Dy[116] -= nmpcVariables.y[116];
nmpcWorkspace.Dy[117] -= nmpcVariables.y[117];
nmpcWorkspace.Dy[118] -= nmpcVariables.y[118];
nmpcWorkspace.Dy[119] -= nmpcVariables.y[119];
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

nmpcWorkspace.QDy[80] = + nmpcWorkspace.QN2[0]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[1]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[2]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[3]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[4]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[5]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[6]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[7]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[81] = + nmpcWorkspace.QN2[8]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[9]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[10]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[11]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[12]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[13]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[14]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[15]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[82] = + nmpcWorkspace.QN2[16]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[17]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[18]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[19]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[20]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[21]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[22]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[23]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[83] = + nmpcWorkspace.QN2[24]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[25]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[26]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[27]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[28]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[29]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[30]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[31]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[84] = + nmpcWorkspace.QN2[32]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[33]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[34]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[35]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[36]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[37]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[38]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[39]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[85] = + nmpcWorkspace.QN2[40]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[41]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[42]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[43]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[44]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[45]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[46]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[47]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[86] = + nmpcWorkspace.QN2[48]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[49]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[50]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[51]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[52]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[53]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[54]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[55]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[87] = + nmpcWorkspace.QN2[56]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[57]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[58]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[59]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[60]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[61]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[62]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[63]*nmpcWorkspace.DyN[7];

nmpcWorkspace.QDy[8] += nmpcWorkspace.Qd[0];
nmpcWorkspace.QDy[9] += nmpcWorkspace.Qd[1];
nmpcWorkspace.QDy[10] += nmpcWorkspace.Qd[2];
nmpcWorkspace.QDy[11] += nmpcWorkspace.Qd[3];
nmpcWorkspace.QDy[12] += nmpcWorkspace.Qd[4];
nmpcWorkspace.QDy[13] += nmpcWorkspace.Qd[5];
nmpcWorkspace.QDy[14] += nmpcWorkspace.Qd[6];
nmpcWorkspace.QDy[15] += nmpcWorkspace.Qd[7];
nmpcWorkspace.QDy[16] += nmpcWorkspace.Qd[8];
nmpcWorkspace.QDy[17] += nmpcWorkspace.Qd[9];
nmpcWorkspace.QDy[18] += nmpcWorkspace.Qd[10];
nmpcWorkspace.QDy[19] += nmpcWorkspace.Qd[11];
nmpcWorkspace.QDy[20] += nmpcWorkspace.Qd[12];
nmpcWorkspace.QDy[21] += nmpcWorkspace.Qd[13];
nmpcWorkspace.QDy[22] += nmpcWorkspace.Qd[14];
nmpcWorkspace.QDy[23] += nmpcWorkspace.Qd[15];
nmpcWorkspace.QDy[24] += nmpcWorkspace.Qd[16];
nmpcWorkspace.QDy[25] += nmpcWorkspace.Qd[17];
nmpcWorkspace.QDy[26] += nmpcWorkspace.Qd[18];
nmpcWorkspace.QDy[27] += nmpcWorkspace.Qd[19];
nmpcWorkspace.QDy[28] += nmpcWorkspace.Qd[20];
nmpcWorkspace.QDy[29] += nmpcWorkspace.Qd[21];
nmpcWorkspace.QDy[30] += nmpcWorkspace.Qd[22];
nmpcWorkspace.QDy[31] += nmpcWorkspace.Qd[23];
nmpcWorkspace.QDy[32] += nmpcWorkspace.Qd[24];
nmpcWorkspace.QDy[33] += nmpcWorkspace.Qd[25];
nmpcWorkspace.QDy[34] += nmpcWorkspace.Qd[26];
nmpcWorkspace.QDy[35] += nmpcWorkspace.Qd[27];
nmpcWorkspace.QDy[36] += nmpcWorkspace.Qd[28];
nmpcWorkspace.QDy[37] += nmpcWorkspace.Qd[29];
nmpcWorkspace.QDy[38] += nmpcWorkspace.Qd[30];
nmpcWorkspace.QDy[39] += nmpcWorkspace.Qd[31];
nmpcWorkspace.QDy[40] += nmpcWorkspace.Qd[32];
nmpcWorkspace.QDy[41] += nmpcWorkspace.Qd[33];
nmpcWorkspace.QDy[42] += nmpcWorkspace.Qd[34];
nmpcWorkspace.QDy[43] += nmpcWorkspace.Qd[35];
nmpcWorkspace.QDy[44] += nmpcWorkspace.Qd[36];
nmpcWorkspace.QDy[45] += nmpcWorkspace.Qd[37];
nmpcWorkspace.QDy[46] += nmpcWorkspace.Qd[38];
nmpcWorkspace.QDy[47] += nmpcWorkspace.Qd[39];
nmpcWorkspace.QDy[48] += nmpcWorkspace.Qd[40];
nmpcWorkspace.QDy[49] += nmpcWorkspace.Qd[41];
nmpcWorkspace.QDy[50] += nmpcWorkspace.Qd[42];
nmpcWorkspace.QDy[51] += nmpcWorkspace.Qd[43];
nmpcWorkspace.QDy[52] += nmpcWorkspace.Qd[44];
nmpcWorkspace.QDy[53] += nmpcWorkspace.Qd[45];
nmpcWorkspace.QDy[54] += nmpcWorkspace.Qd[46];
nmpcWorkspace.QDy[55] += nmpcWorkspace.Qd[47];
nmpcWorkspace.QDy[56] += nmpcWorkspace.Qd[48];
nmpcWorkspace.QDy[57] += nmpcWorkspace.Qd[49];
nmpcWorkspace.QDy[58] += nmpcWorkspace.Qd[50];
nmpcWorkspace.QDy[59] += nmpcWorkspace.Qd[51];
nmpcWorkspace.QDy[60] += nmpcWorkspace.Qd[52];
nmpcWorkspace.QDy[61] += nmpcWorkspace.Qd[53];
nmpcWorkspace.QDy[62] += nmpcWorkspace.Qd[54];
nmpcWorkspace.QDy[63] += nmpcWorkspace.Qd[55];
nmpcWorkspace.QDy[64] += nmpcWorkspace.Qd[56];
nmpcWorkspace.QDy[65] += nmpcWorkspace.Qd[57];
nmpcWorkspace.QDy[66] += nmpcWorkspace.Qd[58];
nmpcWorkspace.QDy[67] += nmpcWorkspace.Qd[59];
nmpcWorkspace.QDy[68] += nmpcWorkspace.Qd[60];
nmpcWorkspace.QDy[69] += nmpcWorkspace.Qd[61];
nmpcWorkspace.QDy[70] += nmpcWorkspace.Qd[62];
nmpcWorkspace.QDy[71] += nmpcWorkspace.Qd[63];
nmpcWorkspace.QDy[72] += nmpcWorkspace.Qd[64];
nmpcWorkspace.QDy[73] += nmpcWorkspace.Qd[65];
nmpcWorkspace.QDy[74] += nmpcWorkspace.Qd[66];
nmpcWorkspace.QDy[75] += nmpcWorkspace.Qd[67];
nmpcWorkspace.QDy[76] += nmpcWorkspace.Qd[68];
nmpcWorkspace.QDy[77] += nmpcWorkspace.Qd[69];
nmpcWorkspace.QDy[78] += nmpcWorkspace.Qd[70];
nmpcWorkspace.QDy[79] += nmpcWorkspace.Qd[71];
nmpcWorkspace.QDy[80] += nmpcWorkspace.Qd[72];
nmpcWorkspace.QDy[81] += nmpcWorkspace.Qd[73];
nmpcWorkspace.QDy[82] += nmpcWorkspace.Qd[74];
nmpcWorkspace.QDy[83] += nmpcWorkspace.Qd[75];
nmpcWorkspace.QDy[84] += nmpcWorkspace.Qd[76];
nmpcWorkspace.QDy[85] += nmpcWorkspace.Qd[77];
nmpcWorkspace.QDy[86] += nmpcWorkspace.Qd[78];
nmpcWorkspace.QDy[87] += nmpcWorkspace.Qd[79];

nmpcWorkspace.g[0] = + nmpcWorkspace.evGx[0]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[8]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[16]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[24]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[32]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[40]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[48]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[56]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[64]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[72]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[80]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[88]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[96]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[104]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[112]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[120]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[128]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[136]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[144]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[152]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[160]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[168]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[176]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[184]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[192]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[200]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[208]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[216]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[224]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[232]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[240]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[248]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[256]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[264]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[272]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[280]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[288]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[296]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[304]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[312]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[320]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[328]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[336]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[344]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[352]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[360]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[368]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[376]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[384]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[392]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[400]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[408]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[416]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[424]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[432]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[440]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[448]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[456]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[464]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[472]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[480]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[488]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[496]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[504]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[512]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[520]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[528]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[536]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[544]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[552]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[560]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[568]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[576]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[584]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[592]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[600]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[608]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[616]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[624]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[632]*nmpcWorkspace.QDy[87];
nmpcWorkspace.g[1] = + nmpcWorkspace.evGx[1]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[9]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[17]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[25]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[33]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[41]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[49]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[57]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[65]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[73]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[81]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[89]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[97]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[105]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[113]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[121]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[129]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[137]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[145]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[153]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[161]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[169]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[177]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[185]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[193]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[201]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[209]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[217]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[225]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[233]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[241]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[249]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[257]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[265]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[273]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[281]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[289]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[297]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[305]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[313]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[321]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[329]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[337]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[345]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[353]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[361]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[369]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[377]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[385]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[393]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[401]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[409]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[417]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[425]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[433]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[441]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[449]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[457]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[465]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[473]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[481]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[489]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[497]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[505]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[513]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[521]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[529]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[537]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[545]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[553]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[561]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[569]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[577]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[585]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[593]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[601]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[609]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[617]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[625]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[633]*nmpcWorkspace.QDy[87];
nmpcWorkspace.g[2] = + nmpcWorkspace.evGx[2]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[10]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[18]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[26]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[34]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[42]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[50]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[58]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[66]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[74]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[82]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[90]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[98]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[106]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[114]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[122]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[130]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[138]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[146]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[154]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[162]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[170]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[178]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[186]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[194]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[202]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[210]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[218]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[226]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[234]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[242]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[250]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[258]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[266]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[274]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[282]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[290]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[298]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[306]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[314]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[322]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[330]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[338]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[346]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[354]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[362]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[370]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[378]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[386]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[394]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[402]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[410]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[418]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[426]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[434]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[442]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[450]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[458]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[466]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[474]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[482]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[490]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[498]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[506]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[514]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[522]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[530]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[538]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[546]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[554]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[562]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[570]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[578]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[586]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[594]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[602]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[610]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[618]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[626]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[634]*nmpcWorkspace.QDy[87];
nmpcWorkspace.g[3] = + nmpcWorkspace.evGx[3]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[11]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[19]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[27]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[35]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[43]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[51]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[59]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[67]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[75]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[83]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[91]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[99]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[107]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[115]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[123]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[131]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[139]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[147]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[155]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[163]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[171]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[179]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[187]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[195]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[203]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[211]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[219]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[227]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[235]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[243]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[251]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[259]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[267]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[275]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[283]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[291]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[299]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[307]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[315]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[323]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[331]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[339]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[347]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[355]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[363]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[371]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[379]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[387]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[395]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[403]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[411]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[419]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[427]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[435]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[443]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[451]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[459]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[467]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[475]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[483]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[491]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[499]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[507]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[515]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[523]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[531]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[539]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[547]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[555]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[563]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[571]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[579]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[587]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[595]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[603]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[611]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[619]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[627]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[635]*nmpcWorkspace.QDy[87];
nmpcWorkspace.g[4] = + nmpcWorkspace.evGx[4]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[12]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[20]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[28]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[36]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[44]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[52]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[60]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[68]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[76]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[84]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[92]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[100]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[108]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[116]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[124]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[132]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[140]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[148]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[156]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[164]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[172]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[180]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[188]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[196]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[204]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[212]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[220]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[228]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[236]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[244]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[252]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[260]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[268]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[276]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[284]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[292]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[300]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[308]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[316]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[324]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[332]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[340]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[348]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[356]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[364]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[372]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[380]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[388]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[396]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[404]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[412]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[420]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[428]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[436]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[444]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[452]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[460]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[468]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[476]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[484]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[492]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[500]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[508]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[516]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[524]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[532]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[540]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[548]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[556]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[564]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[572]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[580]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[588]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[596]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[604]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[612]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[620]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[628]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[636]*nmpcWorkspace.QDy[87];
nmpcWorkspace.g[5] = + nmpcWorkspace.evGx[5]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[13]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[21]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[29]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[37]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[45]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[53]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[61]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[69]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[77]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[85]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[93]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[101]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[109]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[117]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[125]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[133]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[141]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[149]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[157]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[165]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[173]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[181]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[189]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[197]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[205]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[213]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[221]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[229]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[237]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[245]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[253]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[261]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[269]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[277]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[285]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[293]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[301]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[309]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[317]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[325]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[333]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[341]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[349]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[357]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[365]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[373]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[381]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[389]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[397]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[405]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[413]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[421]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[429]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[437]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[445]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[453]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[461]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[469]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[477]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[485]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[493]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[501]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[509]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[517]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[525]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[533]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[541]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[549]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[557]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[565]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[573]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[581]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[589]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[597]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[605]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[613]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[621]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[629]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[637]*nmpcWorkspace.QDy[87];
nmpcWorkspace.g[6] = + nmpcWorkspace.evGx[6]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[14]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[22]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[30]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[38]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[46]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[54]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[62]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[70]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[78]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[86]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[94]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[102]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[110]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[118]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[126]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[134]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[142]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[150]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[158]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[166]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[174]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[182]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[190]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[198]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[206]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[214]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[222]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[230]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[238]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[246]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[254]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[262]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[270]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[278]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[286]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[294]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[302]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[310]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[318]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[326]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[334]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[342]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[350]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[358]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[366]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[374]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[382]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[390]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[398]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[406]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[414]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[422]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[430]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[438]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[446]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[454]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[462]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[470]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[478]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[486]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[494]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[502]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[510]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[518]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[526]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[534]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[542]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[550]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[558]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[566]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[574]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[582]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[590]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[598]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[606]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[614]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[622]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[630]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[638]*nmpcWorkspace.QDy[87];
nmpcWorkspace.g[7] = + nmpcWorkspace.evGx[7]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[15]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[23]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[31]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[39]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[47]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[55]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[63]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[71]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[79]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[87]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[95]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[103]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[111]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[119]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[127]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[135]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[143]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[151]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[159]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[167]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[175]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[183]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[191]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[199]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[207]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[215]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[223]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[231]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[239]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[247]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[255]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[263]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[271]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[279]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[287]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[295]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[303]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[311]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[319]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[327]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[335]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[343]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[351]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[359]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[367]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[375]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[383]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[391]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[399]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[407]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[415]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[423]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[431]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[439]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[447]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[455]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[463]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[471]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[479]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[487]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[495]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[503]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[511]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[519]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[527]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[535]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[543]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[551]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[559]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[567]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[575]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[583]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[591]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[599]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[607]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[615]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[623]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[631]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[639]*nmpcWorkspace.QDy[87];


nmpc_multEQDy( nmpcWorkspace.E, &(nmpcWorkspace.QDy[ 8 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 32 ]), &(nmpcWorkspace.QDy[ 16 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 96 ]), &(nmpcWorkspace.QDy[ 24 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 192 ]), &(nmpcWorkspace.QDy[ 32 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 320 ]), &(nmpcWorkspace.QDy[ 40 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 480 ]), &(nmpcWorkspace.QDy[ 48 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 672 ]), &(nmpcWorkspace.QDy[ 56 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 896 ]), &(nmpcWorkspace.QDy[ 64 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1152 ]), &(nmpcWorkspace.QDy[ 72 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1440 ]), &(nmpcWorkspace.QDy[ 80 ]), &(nmpcWorkspace.g[ 8 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 64 ]), &(nmpcWorkspace.QDy[ 16 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 128 ]), &(nmpcWorkspace.QDy[ 24 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 224 ]), &(nmpcWorkspace.QDy[ 32 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 352 ]), &(nmpcWorkspace.QDy[ 40 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 512 ]), &(nmpcWorkspace.QDy[ 48 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 704 ]), &(nmpcWorkspace.QDy[ 56 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 928 ]), &(nmpcWorkspace.QDy[ 64 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1184 ]), &(nmpcWorkspace.QDy[ 72 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1472 ]), &(nmpcWorkspace.QDy[ 80 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 160 ]), &(nmpcWorkspace.QDy[ 24 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 256 ]), &(nmpcWorkspace.QDy[ 32 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 384 ]), &(nmpcWorkspace.QDy[ 40 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 544 ]), &(nmpcWorkspace.QDy[ 48 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 736 ]), &(nmpcWorkspace.QDy[ 56 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 960 ]), &(nmpcWorkspace.QDy[ 64 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1216 ]), &(nmpcWorkspace.QDy[ 72 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1504 ]), &(nmpcWorkspace.QDy[ 80 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 288 ]), &(nmpcWorkspace.QDy[ 32 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 416 ]), &(nmpcWorkspace.QDy[ 40 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 576 ]), &(nmpcWorkspace.QDy[ 48 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 768 ]), &(nmpcWorkspace.QDy[ 56 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 992 ]), &(nmpcWorkspace.QDy[ 64 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1248 ]), &(nmpcWorkspace.QDy[ 72 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1536 ]), &(nmpcWorkspace.QDy[ 80 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 448 ]), &(nmpcWorkspace.QDy[ 40 ]), &(nmpcWorkspace.g[ 24 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 608 ]), &(nmpcWorkspace.QDy[ 48 ]), &(nmpcWorkspace.g[ 24 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 800 ]), &(nmpcWorkspace.QDy[ 56 ]), &(nmpcWorkspace.g[ 24 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1024 ]), &(nmpcWorkspace.QDy[ 64 ]), &(nmpcWorkspace.g[ 24 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1280 ]), &(nmpcWorkspace.QDy[ 72 ]), &(nmpcWorkspace.g[ 24 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1568 ]), &(nmpcWorkspace.QDy[ 80 ]), &(nmpcWorkspace.g[ 24 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 640 ]), &(nmpcWorkspace.QDy[ 48 ]), &(nmpcWorkspace.g[ 28 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 832 ]), &(nmpcWorkspace.QDy[ 56 ]), &(nmpcWorkspace.g[ 28 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1056 ]), &(nmpcWorkspace.QDy[ 64 ]), &(nmpcWorkspace.g[ 28 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1312 ]), &(nmpcWorkspace.QDy[ 72 ]), &(nmpcWorkspace.g[ 28 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1600 ]), &(nmpcWorkspace.QDy[ 80 ]), &(nmpcWorkspace.g[ 28 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 864 ]), &(nmpcWorkspace.QDy[ 56 ]), &(nmpcWorkspace.g[ 32 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1088 ]), &(nmpcWorkspace.QDy[ 64 ]), &(nmpcWorkspace.g[ 32 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1344 ]), &(nmpcWorkspace.QDy[ 72 ]), &(nmpcWorkspace.g[ 32 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1632 ]), &(nmpcWorkspace.QDy[ 80 ]), &(nmpcWorkspace.g[ 32 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1120 ]), &(nmpcWorkspace.QDy[ 64 ]), &(nmpcWorkspace.g[ 36 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1376 ]), &(nmpcWorkspace.QDy[ 72 ]), &(nmpcWorkspace.g[ 36 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1664 ]), &(nmpcWorkspace.QDy[ 80 ]), &(nmpcWorkspace.g[ 36 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1408 ]), &(nmpcWorkspace.QDy[ 72 ]), &(nmpcWorkspace.g[ 40 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1696 ]), &(nmpcWorkspace.QDy[ 80 ]), &(nmpcWorkspace.g[ 40 ]) );
nmpc_multEQDy( &(nmpcWorkspace.E[ 1728 ]), &(nmpcWorkspace.QDy[ 80 ]), &(nmpcWorkspace.g[ 44 ]) );

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
nmpcVariables.x[0] += nmpcWorkspace.x[0];
nmpcVariables.x[1] += nmpcWorkspace.x[1];
nmpcVariables.x[2] += nmpcWorkspace.x[2];
nmpcVariables.x[3] += nmpcWorkspace.x[3];
nmpcVariables.x[4] += nmpcWorkspace.x[4];
nmpcVariables.x[5] += nmpcWorkspace.x[5];
nmpcVariables.x[6] += nmpcWorkspace.x[6];
nmpcVariables.x[7] += nmpcWorkspace.x[7];

nmpcVariables.u[0] += nmpcWorkspace.x[8];
nmpcVariables.u[1] += nmpcWorkspace.x[9];
nmpcVariables.u[2] += nmpcWorkspace.x[10];
nmpcVariables.u[3] += nmpcWorkspace.x[11];
nmpcVariables.u[4] += nmpcWorkspace.x[12];
nmpcVariables.u[5] += nmpcWorkspace.x[13];
nmpcVariables.u[6] += nmpcWorkspace.x[14];
nmpcVariables.u[7] += nmpcWorkspace.x[15];
nmpcVariables.u[8] += nmpcWorkspace.x[16];
nmpcVariables.u[9] += nmpcWorkspace.x[17];
nmpcVariables.u[10] += nmpcWorkspace.x[18];
nmpcVariables.u[11] += nmpcWorkspace.x[19];
nmpcVariables.u[12] += nmpcWorkspace.x[20];
nmpcVariables.u[13] += nmpcWorkspace.x[21];
nmpcVariables.u[14] += nmpcWorkspace.x[22];
nmpcVariables.u[15] += nmpcWorkspace.x[23];
nmpcVariables.u[16] += nmpcWorkspace.x[24];
nmpcVariables.u[17] += nmpcWorkspace.x[25];
nmpcVariables.u[18] += nmpcWorkspace.x[26];
nmpcVariables.u[19] += nmpcWorkspace.x[27];
nmpcVariables.u[20] += nmpcWorkspace.x[28];
nmpcVariables.u[21] += nmpcWorkspace.x[29];
nmpcVariables.u[22] += nmpcWorkspace.x[30];
nmpcVariables.u[23] += nmpcWorkspace.x[31];
nmpcVariables.u[24] += nmpcWorkspace.x[32];
nmpcVariables.u[25] += nmpcWorkspace.x[33];
nmpcVariables.u[26] += nmpcWorkspace.x[34];
nmpcVariables.u[27] += nmpcWorkspace.x[35];
nmpcVariables.u[28] += nmpcWorkspace.x[36];
nmpcVariables.u[29] += nmpcWorkspace.x[37];
nmpcVariables.u[30] += nmpcWorkspace.x[38];
nmpcVariables.u[31] += nmpcWorkspace.x[39];
nmpcVariables.u[32] += nmpcWorkspace.x[40];
nmpcVariables.u[33] += nmpcWorkspace.x[41];
nmpcVariables.u[34] += nmpcWorkspace.x[42];
nmpcVariables.u[35] += nmpcWorkspace.x[43];
nmpcVariables.u[36] += nmpcWorkspace.x[44];
nmpcVariables.u[37] += nmpcWorkspace.x[45];
nmpcVariables.u[38] += nmpcWorkspace.x[46];
nmpcVariables.u[39] += nmpcWorkspace.x[47];

nmpcVariables.x[8] += + nmpcWorkspace.evGx[0]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[2]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[3]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[4]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[5]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[6]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[7]*nmpcWorkspace.x[7] + nmpcWorkspace.d[0];
nmpcVariables.x[9] += + nmpcWorkspace.evGx[8]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[9]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[10]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[11]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[12]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[13]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[14]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[15]*nmpcWorkspace.x[7] + nmpcWorkspace.d[1];
nmpcVariables.x[10] += + nmpcWorkspace.evGx[16]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[17]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[18]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[19]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[20]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[21]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[22]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[23]*nmpcWorkspace.x[7] + nmpcWorkspace.d[2];
nmpcVariables.x[11] += + nmpcWorkspace.evGx[24]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[25]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[26]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[27]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[28]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[29]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[30]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[31]*nmpcWorkspace.x[7] + nmpcWorkspace.d[3];
nmpcVariables.x[12] += + nmpcWorkspace.evGx[32]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[33]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[34]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[35]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[36]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[37]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[38]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[39]*nmpcWorkspace.x[7] + nmpcWorkspace.d[4];
nmpcVariables.x[13] += + nmpcWorkspace.evGx[40]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[41]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[42]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[43]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[44]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[45]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[46]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[47]*nmpcWorkspace.x[7] + nmpcWorkspace.d[5];
nmpcVariables.x[14] += + nmpcWorkspace.evGx[48]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[49]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[50]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[51]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[52]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[53]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[54]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[55]*nmpcWorkspace.x[7] + nmpcWorkspace.d[6];
nmpcVariables.x[15] += + nmpcWorkspace.evGx[56]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[57]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[58]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[59]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[60]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[61]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[62]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[63]*nmpcWorkspace.x[7] + nmpcWorkspace.d[7];
nmpcVariables.x[16] += + nmpcWorkspace.evGx[64]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[65]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[66]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[67]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[68]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[69]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[70]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[71]*nmpcWorkspace.x[7] + nmpcWorkspace.d[8];
nmpcVariables.x[17] += + nmpcWorkspace.evGx[72]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[73]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[74]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[75]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[76]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[77]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[78]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[79]*nmpcWorkspace.x[7] + nmpcWorkspace.d[9];
nmpcVariables.x[18] += + nmpcWorkspace.evGx[80]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[81]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[82]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[83]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[84]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[85]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[86]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[87]*nmpcWorkspace.x[7] + nmpcWorkspace.d[10];
nmpcVariables.x[19] += + nmpcWorkspace.evGx[88]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[89]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[90]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[91]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[92]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[93]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[94]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[95]*nmpcWorkspace.x[7] + nmpcWorkspace.d[11];
nmpcVariables.x[20] += + nmpcWorkspace.evGx[96]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[97]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[98]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[99]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[100]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[101]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[102]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[103]*nmpcWorkspace.x[7] + nmpcWorkspace.d[12];
nmpcVariables.x[21] += + nmpcWorkspace.evGx[104]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[105]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[106]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[107]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[108]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[109]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[110]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[111]*nmpcWorkspace.x[7] + nmpcWorkspace.d[13];
nmpcVariables.x[22] += + nmpcWorkspace.evGx[112]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[113]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[114]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[115]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[116]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[117]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[118]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[119]*nmpcWorkspace.x[7] + nmpcWorkspace.d[14];
nmpcVariables.x[23] += + nmpcWorkspace.evGx[120]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[121]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[122]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[123]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[124]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[125]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[126]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[127]*nmpcWorkspace.x[7] + nmpcWorkspace.d[15];
nmpcVariables.x[24] += + nmpcWorkspace.evGx[128]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[129]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[130]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[131]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[132]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[133]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[134]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[135]*nmpcWorkspace.x[7] + nmpcWorkspace.d[16];
nmpcVariables.x[25] += + nmpcWorkspace.evGx[136]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[137]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[138]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[139]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[140]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[141]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[142]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[143]*nmpcWorkspace.x[7] + nmpcWorkspace.d[17];
nmpcVariables.x[26] += + nmpcWorkspace.evGx[144]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[145]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[146]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[147]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[148]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[149]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[150]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[151]*nmpcWorkspace.x[7] + nmpcWorkspace.d[18];
nmpcVariables.x[27] += + nmpcWorkspace.evGx[152]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[153]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[154]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[155]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[156]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[157]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[158]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[159]*nmpcWorkspace.x[7] + nmpcWorkspace.d[19];
nmpcVariables.x[28] += + nmpcWorkspace.evGx[160]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[161]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[162]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[163]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[164]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[165]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[166]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[167]*nmpcWorkspace.x[7] + nmpcWorkspace.d[20];
nmpcVariables.x[29] += + nmpcWorkspace.evGx[168]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[169]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[170]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[171]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[172]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[173]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[174]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[175]*nmpcWorkspace.x[7] + nmpcWorkspace.d[21];
nmpcVariables.x[30] += + nmpcWorkspace.evGx[176]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[177]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[178]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[179]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[180]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[181]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[182]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[183]*nmpcWorkspace.x[7] + nmpcWorkspace.d[22];
nmpcVariables.x[31] += + nmpcWorkspace.evGx[184]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[185]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[186]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[187]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[188]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[189]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[190]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[191]*nmpcWorkspace.x[7] + nmpcWorkspace.d[23];
nmpcVariables.x[32] += + nmpcWorkspace.evGx[192]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[193]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[194]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[195]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[196]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[197]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[198]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[199]*nmpcWorkspace.x[7] + nmpcWorkspace.d[24];
nmpcVariables.x[33] += + nmpcWorkspace.evGx[200]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[201]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[202]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[203]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[204]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[205]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[206]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[207]*nmpcWorkspace.x[7] + nmpcWorkspace.d[25];
nmpcVariables.x[34] += + nmpcWorkspace.evGx[208]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[209]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[210]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[211]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[212]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[213]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[214]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[215]*nmpcWorkspace.x[7] + nmpcWorkspace.d[26];
nmpcVariables.x[35] += + nmpcWorkspace.evGx[216]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[217]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[218]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[219]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[220]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[221]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[222]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[223]*nmpcWorkspace.x[7] + nmpcWorkspace.d[27];
nmpcVariables.x[36] += + nmpcWorkspace.evGx[224]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[225]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[226]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[227]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[228]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[229]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[230]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[231]*nmpcWorkspace.x[7] + nmpcWorkspace.d[28];
nmpcVariables.x[37] += + nmpcWorkspace.evGx[232]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[233]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[234]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[235]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[236]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[237]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[238]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[239]*nmpcWorkspace.x[7] + nmpcWorkspace.d[29];
nmpcVariables.x[38] += + nmpcWorkspace.evGx[240]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[241]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[242]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[243]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[244]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[245]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[246]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[247]*nmpcWorkspace.x[7] + nmpcWorkspace.d[30];
nmpcVariables.x[39] += + nmpcWorkspace.evGx[248]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[249]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[250]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[251]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[252]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[253]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[254]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[255]*nmpcWorkspace.x[7] + nmpcWorkspace.d[31];
nmpcVariables.x[40] += + nmpcWorkspace.evGx[256]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[257]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[258]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[259]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[260]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[261]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[262]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[263]*nmpcWorkspace.x[7] + nmpcWorkspace.d[32];
nmpcVariables.x[41] += + nmpcWorkspace.evGx[264]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[265]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[266]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[267]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[268]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[269]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[270]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[271]*nmpcWorkspace.x[7] + nmpcWorkspace.d[33];
nmpcVariables.x[42] += + nmpcWorkspace.evGx[272]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[273]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[274]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[275]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[276]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[277]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[278]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[279]*nmpcWorkspace.x[7] + nmpcWorkspace.d[34];
nmpcVariables.x[43] += + nmpcWorkspace.evGx[280]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[281]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[282]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[283]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[284]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[285]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[286]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[287]*nmpcWorkspace.x[7] + nmpcWorkspace.d[35];
nmpcVariables.x[44] += + nmpcWorkspace.evGx[288]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[289]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[290]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[291]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[292]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[293]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[294]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[295]*nmpcWorkspace.x[7] + nmpcWorkspace.d[36];
nmpcVariables.x[45] += + nmpcWorkspace.evGx[296]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[297]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[298]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[299]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[300]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[301]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[302]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[303]*nmpcWorkspace.x[7] + nmpcWorkspace.d[37];
nmpcVariables.x[46] += + nmpcWorkspace.evGx[304]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[305]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[306]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[307]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[308]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[309]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[310]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[311]*nmpcWorkspace.x[7] + nmpcWorkspace.d[38];
nmpcVariables.x[47] += + nmpcWorkspace.evGx[312]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[313]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[314]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[315]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[316]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[317]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[318]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[319]*nmpcWorkspace.x[7] + nmpcWorkspace.d[39];
nmpcVariables.x[48] += + nmpcWorkspace.evGx[320]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[321]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[322]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[323]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[324]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[325]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[326]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[327]*nmpcWorkspace.x[7] + nmpcWorkspace.d[40];
nmpcVariables.x[49] += + nmpcWorkspace.evGx[328]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[329]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[330]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[331]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[332]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[333]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[334]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[335]*nmpcWorkspace.x[7] + nmpcWorkspace.d[41];
nmpcVariables.x[50] += + nmpcWorkspace.evGx[336]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[337]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[338]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[339]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[340]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[341]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[342]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[343]*nmpcWorkspace.x[7] + nmpcWorkspace.d[42];
nmpcVariables.x[51] += + nmpcWorkspace.evGx[344]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[345]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[346]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[347]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[348]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[349]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[350]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[351]*nmpcWorkspace.x[7] + nmpcWorkspace.d[43];
nmpcVariables.x[52] += + nmpcWorkspace.evGx[352]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[353]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[354]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[355]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[356]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[357]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[358]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[359]*nmpcWorkspace.x[7] + nmpcWorkspace.d[44];
nmpcVariables.x[53] += + nmpcWorkspace.evGx[360]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[361]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[362]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[363]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[364]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[365]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[366]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[367]*nmpcWorkspace.x[7] + nmpcWorkspace.d[45];
nmpcVariables.x[54] += + nmpcWorkspace.evGx[368]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[369]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[370]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[371]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[372]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[373]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[374]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[375]*nmpcWorkspace.x[7] + nmpcWorkspace.d[46];
nmpcVariables.x[55] += + nmpcWorkspace.evGx[376]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[377]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[378]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[379]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[380]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[381]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[382]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[383]*nmpcWorkspace.x[7] + nmpcWorkspace.d[47];
nmpcVariables.x[56] += + nmpcWorkspace.evGx[384]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[385]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[386]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[387]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[388]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[389]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[390]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[391]*nmpcWorkspace.x[7] + nmpcWorkspace.d[48];
nmpcVariables.x[57] += + nmpcWorkspace.evGx[392]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[393]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[394]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[395]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[396]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[397]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[398]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[399]*nmpcWorkspace.x[7] + nmpcWorkspace.d[49];
nmpcVariables.x[58] += + nmpcWorkspace.evGx[400]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[401]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[402]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[403]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[404]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[405]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[406]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[407]*nmpcWorkspace.x[7] + nmpcWorkspace.d[50];
nmpcVariables.x[59] += + nmpcWorkspace.evGx[408]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[409]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[410]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[411]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[412]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[413]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[414]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[415]*nmpcWorkspace.x[7] + nmpcWorkspace.d[51];
nmpcVariables.x[60] += + nmpcWorkspace.evGx[416]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[417]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[418]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[419]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[420]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[421]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[422]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[423]*nmpcWorkspace.x[7] + nmpcWorkspace.d[52];
nmpcVariables.x[61] += + nmpcWorkspace.evGx[424]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[425]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[426]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[427]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[428]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[429]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[430]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[431]*nmpcWorkspace.x[7] + nmpcWorkspace.d[53];
nmpcVariables.x[62] += + nmpcWorkspace.evGx[432]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[433]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[434]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[435]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[436]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[437]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[438]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[439]*nmpcWorkspace.x[7] + nmpcWorkspace.d[54];
nmpcVariables.x[63] += + nmpcWorkspace.evGx[440]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[441]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[442]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[443]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[444]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[445]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[446]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[447]*nmpcWorkspace.x[7] + nmpcWorkspace.d[55];
nmpcVariables.x[64] += + nmpcWorkspace.evGx[448]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[449]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[450]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[451]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[452]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[453]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[454]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[455]*nmpcWorkspace.x[7] + nmpcWorkspace.d[56];
nmpcVariables.x[65] += + nmpcWorkspace.evGx[456]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[457]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[458]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[459]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[460]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[461]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[462]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[463]*nmpcWorkspace.x[7] + nmpcWorkspace.d[57];
nmpcVariables.x[66] += + nmpcWorkspace.evGx[464]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[465]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[466]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[467]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[468]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[469]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[470]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[471]*nmpcWorkspace.x[7] + nmpcWorkspace.d[58];
nmpcVariables.x[67] += + nmpcWorkspace.evGx[472]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[473]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[474]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[475]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[476]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[477]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[478]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[479]*nmpcWorkspace.x[7] + nmpcWorkspace.d[59];
nmpcVariables.x[68] += + nmpcWorkspace.evGx[480]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[481]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[482]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[483]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[484]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[485]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[486]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[487]*nmpcWorkspace.x[7] + nmpcWorkspace.d[60];
nmpcVariables.x[69] += + nmpcWorkspace.evGx[488]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[489]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[490]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[491]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[492]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[493]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[494]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[495]*nmpcWorkspace.x[7] + nmpcWorkspace.d[61];
nmpcVariables.x[70] += + nmpcWorkspace.evGx[496]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[497]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[498]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[499]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[500]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[501]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[502]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[503]*nmpcWorkspace.x[7] + nmpcWorkspace.d[62];
nmpcVariables.x[71] += + nmpcWorkspace.evGx[504]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[505]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[506]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[507]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[508]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[509]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[510]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[511]*nmpcWorkspace.x[7] + nmpcWorkspace.d[63];
nmpcVariables.x[72] += + nmpcWorkspace.evGx[512]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[513]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[514]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[515]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[516]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[517]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[518]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[519]*nmpcWorkspace.x[7] + nmpcWorkspace.d[64];
nmpcVariables.x[73] += + nmpcWorkspace.evGx[520]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[521]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[522]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[523]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[524]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[525]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[526]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[527]*nmpcWorkspace.x[7] + nmpcWorkspace.d[65];
nmpcVariables.x[74] += + nmpcWorkspace.evGx[528]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[529]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[530]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[531]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[532]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[533]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[534]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[535]*nmpcWorkspace.x[7] + nmpcWorkspace.d[66];
nmpcVariables.x[75] += + nmpcWorkspace.evGx[536]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[537]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[538]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[539]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[540]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[541]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[542]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[543]*nmpcWorkspace.x[7] + nmpcWorkspace.d[67];
nmpcVariables.x[76] += + nmpcWorkspace.evGx[544]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[545]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[546]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[547]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[548]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[549]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[550]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[551]*nmpcWorkspace.x[7] + nmpcWorkspace.d[68];
nmpcVariables.x[77] += + nmpcWorkspace.evGx[552]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[553]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[554]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[555]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[556]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[557]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[558]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[559]*nmpcWorkspace.x[7] + nmpcWorkspace.d[69];
nmpcVariables.x[78] += + nmpcWorkspace.evGx[560]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[561]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[562]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[563]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[564]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[565]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[566]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[567]*nmpcWorkspace.x[7] + nmpcWorkspace.d[70];
nmpcVariables.x[79] += + nmpcWorkspace.evGx[568]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[569]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[570]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[571]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[572]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[573]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[574]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[575]*nmpcWorkspace.x[7] + nmpcWorkspace.d[71];
nmpcVariables.x[80] += + nmpcWorkspace.evGx[576]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[577]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[578]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[579]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[580]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[581]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[582]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[583]*nmpcWorkspace.x[7] + nmpcWorkspace.d[72];
nmpcVariables.x[81] += + nmpcWorkspace.evGx[584]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[585]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[586]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[587]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[588]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[589]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[590]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[591]*nmpcWorkspace.x[7] + nmpcWorkspace.d[73];
nmpcVariables.x[82] += + nmpcWorkspace.evGx[592]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[593]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[594]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[595]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[596]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[597]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[598]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[599]*nmpcWorkspace.x[7] + nmpcWorkspace.d[74];
nmpcVariables.x[83] += + nmpcWorkspace.evGx[600]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[601]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[602]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[603]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[604]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[605]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[606]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[607]*nmpcWorkspace.x[7] + nmpcWorkspace.d[75];
nmpcVariables.x[84] += + nmpcWorkspace.evGx[608]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[609]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[610]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[611]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[612]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[613]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[614]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[615]*nmpcWorkspace.x[7] + nmpcWorkspace.d[76];
nmpcVariables.x[85] += + nmpcWorkspace.evGx[616]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[617]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[618]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[619]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[620]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[621]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[622]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[623]*nmpcWorkspace.x[7] + nmpcWorkspace.d[77];
nmpcVariables.x[86] += + nmpcWorkspace.evGx[624]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[625]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[626]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[627]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[628]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[629]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[630]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[631]*nmpcWorkspace.x[7] + nmpcWorkspace.d[78];
nmpcVariables.x[87] += + nmpcWorkspace.evGx[632]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[633]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[634]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[635]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[636]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[637]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[638]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[639]*nmpcWorkspace.x[7] + nmpcWorkspace.d[79];

nmpc_multEDu( nmpcWorkspace.E, &(nmpcWorkspace.x[ 8 ]), &(nmpcVariables.x[ 8 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 32 ]), &(nmpcWorkspace.x[ 8 ]), &(nmpcVariables.x[ 16 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 64 ]), &(nmpcWorkspace.x[ 12 ]), &(nmpcVariables.x[ 16 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 96 ]), &(nmpcWorkspace.x[ 8 ]), &(nmpcVariables.x[ 24 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 128 ]), &(nmpcWorkspace.x[ 12 ]), &(nmpcVariables.x[ 24 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 160 ]), &(nmpcWorkspace.x[ 16 ]), &(nmpcVariables.x[ 24 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 192 ]), &(nmpcWorkspace.x[ 8 ]), &(nmpcVariables.x[ 32 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 224 ]), &(nmpcWorkspace.x[ 12 ]), &(nmpcVariables.x[ 32 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 256 ]), &(nmpcWorkspace.x[ 16 ]), &(nmpcVariables.x[ 32 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 288 ]), &(nmpcWorkspace.x[ 20 ]), &(nmpcVariables.x[ 32 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 320 ]), &(nmpcWorkspace.x[ 8 ]), &(nmpcVariables.x[ 40 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 352 ]), &(nmpcWorkspace.x[ 12 ]), &(nmpcVariables.x[ 40 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 384 ]), &(nmpcWorkspace.x[ 16 ]), &(nmpcVariables.x[ 40 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 416 ]), &(nmpcWorkspace.x[ 20 ]), &(nmpcVariables.x[ 40 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 448 ]), &(nmpcWorkspace.x[ 24 ]), &(nmpcVariables.x[ 40 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 480 ]), &(nmpcWorkspace.x[ 8 ]), &(nmpcVariables.x[ 48 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 512 ]), &(nmpcWorkspace.x[ 12 ]), &(nmpcVariables.x[ 48 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 544 ]), &(nmpcWorkspace.x[ 16 ]), &(nmpcVariables.x[ 48 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 576 ]), &(nmpcWorkspace.x[ 20 ]), &(nmpcVariables.x[ 48 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 608 ]), &(nmpcWorkspace.x[ 24 ]), &(nmpcVariables.x[ 48 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 640 ]), &(nmpcWorkspace.x[ 28 ]), &(nmpcVariables.x[ 48 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 672 ]), &(nmpcWorkspace.x[ 8 ]), &(nmpcVariables.x[ 56 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 704 ]), &(nmpcWorkspace.x[ 12 ]), &(nmpcVariables.x[ 56 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 736 ]), &(nmpcWorkspace.x[ 16 ]), &(nmpcVariables.x[ 56 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 768 ]), &(nmpcWorkspace.x[ 20 ]), &(nmpcVariables.x[ 56 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 800 ]), &(nmpcWorkspace.x[ 24 ]), &(nmpcVariables.x[ 56 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 832 ]), &(nmpcWorkspace.x[ 28 ]), &(nmpcVariables.x[ 56 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 864 ]), &(nmpcWorkspace.x[ 32 ]), &(nmpcVariables.x[ 56 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 896 ]), &(nmpcWorkspace.x[ 8 ]), &(nmpcVariables.x[ 64 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 928 ]), &(nmpcWorkspace.x[ 12 ]), &(nmpcVariables.x[ 64 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 960 ]), &(nmpcWorkspace.x[ 16 ]), &(nmpcVariables.x[ 64 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 992 ]), &(nmpcWorkspace.x[ 20 ]), &(nmpcVariables.x[ 64 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1024 ]), &(nmpcWorkspace.x[ 24 ]), &(nmpcVariables.x[ 64 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1056 ]), &(nmpcWorkspace.x[ 28 ]), &(nmpcVariables.x[ 64 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1088 ]), &(nmpcWorkspace.x[ 32 ]), &(nmpcVariables.x[ 64 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1120 ]), &(nmpcWorkspace.x[ 36 ]), &(nmpcVariables.x[ 64 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1152 ]), &(nmpcWorkspace.x[ 8 ]), &(nmpcVariables.x[ 72 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1184 ]), &(nmpcWorkspace.x[ 12 ]), &(nmpcVariables.x[ 72 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1216 ]), &(nmpcWorkspace.x[ 16 ]), &(nmpcVariables.x[ 72 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1248 ]), &(nmpcWorkspace.x[ 20 ]), &(nmpcVariables.x[ 72 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1280 ]), &(nmpcWorkspace.x[ 24 ]), &(nmpcVariables.x[ 72 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1312 ]), &(nmpcWorkspace.x[ 28 ]), &(nmpcVariables.x[ 72 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1344 ]), &(nmpcWorkspace.x[ 32 ]), &(nmpcVariables.x[ 72 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1376 ]), &(nmpcWorkspace.x[ 36 ]), &(nmpcVariables.x[ 72 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1408 ]), &(nmpcWorkspace.x[ 40 ]), &(nmpcVariables.x[ 72 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1440 ]), &(nmpcWorkspace.x[ 8 ]), &(nmpcVariables.x[ 80 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1472 ]), &(nmpcWorkspace.x[ 12 ]), &(nmpcVariables.x[ 80 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1504 ]), &(nmpcWorkspace.x[ 16 ]), &(nmpcVariables.x[ 80 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1536 ]), &(nmpcWorkspace.x[ 20 ]), &(nmpcVariables.x[ 80 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1568 ]), &(nmpcWorkspace.x[ 24 ]), &(nmpcVariables.x[ 80 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1600 ]), &(nmpcWorkspace.x[ 28 ]), &(nmpcVariables.x[ 80 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1632 ]), &(nmpcWorkspace.x[ 32 ]), &(nmpcVariables.x[ 80 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1664 ]), &(nmpcWorkspace.x[ 36 ]), &(nmpcVariables.x[ 80 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1696 ]), &(nmpcWorkspace.x[ 40 ]), &(nmpcVariables.x[ 80 ]) );
nmpc_multEDu( &(nmpcWorkspace.E[ 1728 ]), &(nmpcWorkspace.x[ 44 ]), &(nmpcVariables.x[ 80 ]) );
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
for (index = 0; index < 10; ++index)
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
for (index = 0; index < 10; ++index)
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
nmpcVariables.x[80] = xEnd[0];
nmpcVariables.x[81] = xEnd[1];
nmpcVariables.x[82] = xEnd[2];
nmpcVariables.x[83] = xEnd[3];
nmpcVariables.x[84] = xEnd[4];
nmpcVariables.x[85] = xEnd[5];
nmpcVariables.x[86] = xEnd[6];
nmpcVariables.x[87] = xEnd[7];
}
else if (strategy == 2) 
{
nmpcWorkspace.state[0] = nmpcVariables.x[80];
nmpcWorkspace.state[1] = nmpcVariables.x[81];
nmpcWorkspace.state[2] = nmpcVariables.x[82];
nmpcWorkspace.state[3] = nmpcVariables.x[83];
nmpcWorkspace.state[4] = nmpcVariables.x[84];
nmpcWorkspace.state[5] = nmpcVariables.x[85];
nmpcWorkspace.state[6] = nmpcVariables.x[86];
nmpcWorkspace.state[7] = nmpcVariables.x[87];
if (uEnd != 0)
{
nmpcWorkspace.state[104] = uEnd[0];
nmpcWorkspace.state[105] = uEnd[1];
nmpcWorkspace.state[106] = uEnd[2];
nmpcWorkspace.state[107] = uEnd[3];
}
else
{
nmpcWorkspace.state[104] = nmpcVariables.u[36];
nmpcWorkspace.state[105] = nmpcVariables.u[37];
nmpcWorkspace.state[106] = nmpcVariables.u[38];
nmpcWorkspace.state[107] = nmpcVariables.u[39];
}
nmpcWorkspace.state[108] = nmpcVariables.od[30];
nmpcWorkspace.state[109] = nmpcVariables.od[31];
nmpcWorkspace.state[110] = nmpcVariables.od[32];

nmpc_integrate(nmpcWorkspace.state, 1);

nmpcVariables.x[80] = nmpcWorkspace.state[0];
nmpcVariables.x[81] = nmpcWorkspace.state[1];
nmpcVariables.x[82] = nmpcWorkspace.state[2];
nmpcVariables.x[83] = nmpcWorkspace.state[3];
nmpcVariables.x[84] = nmpcWorkspace.state[4];
nmpcVariables.x[85] = nmpcWorkspace.state[5];
nmpcVariables.x[86] = nmpcWorkspace.state[6];
nmpcVariables.x[87] = nmpcWorkspace.state[7];
}
}

void nmpc_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 9; ++index)
{
nmpcVariables.u[index * 4] = nmpcVariables.u[index * 4 + 4];
nmpcVariables.u[index * 4 + 1] = nmpcVariables.u[index * 4 + 5];
nmpcVariables.u[index * 4 + 2] = nmpcVariables.u[index * 4 + 6];
nmpcVariables.u[index * 4 + 3] = nmpcVariables.u[index * 4 + 7];
}

if (uEnd != 0)
{
nmpcVariables.u[36] = uEnd[0];
nmpcVariables.u[37] = uEnd[1];
nmpcVariables.u[38] = uEnd[2];
nmpcVariables.u[39] = uEnd[3];
}
}

real_t nmpc_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + nmpcWorkspace.g[0]*nmpcWorkspace.x[0] + nmpcWorkspace.g[1]*nmpcWorkspace.x[1] + nmpcWorkspace.g[2]*nmpcWorkspace.x[2] + nmpcWorkspace.g[3]*nmpcWorkspace.x[3] + nmpcWorkspace.g[4]*nmpcWorkspace.x[4] + nmpcWorkspace.g[5]*nmpcWorkspace.x[5] + nmpcWorkspace.g[6]*nmpcWorkspace.x[6] + nmpcWorkspace.g[7]*nmpcWorkspace.x[7] + nmpcWorkspace.g[8]*nmpcWorkspace.x[8] + nmpcWorkspace.g[9]*nmpcWorkspace.x[9] + nmpcWorkspace.g[10]*nmpcWorkspace.x[10] + nmpcWorkspace.g[11]*nmpcWorkspace.x[11] + nmpcWorkspace.g[12]*nmpcWorkspace.x[12] + nmpcWorkspace.g[13]*nmpcWorkspace.x[13] + nmpcWorkspace.g[14]*nmpcWorkspace.x[14] + nmpcWorkspace.g[15]*nmpcWorkspace.x[15] + nmpcWorkspace.g[16]*nmpcWorkspace.x[16] + nmpcWorkspace.g[17]*nmpcWorkspace.x[17] + nmpcWorkspace.g[18]*nmpcWorkspace.x[18] + nmpcWorkspace.g[19]*nmpcWorkspace.x[19] + nmpcWorkspace.g[20]*nmpcWorkspace.x[20] + nmpcWorkspace.g[21]*nmpcWorkspace.x[21] + nmpcWorkspace.g[22]*nmpcWorkspace.x[22] + nmpcWorkspace.g[23]*nmpcWorkspace.x[23] + nmpcWorkspace.g[24]*nmpcWorkspace.x[24] + nmpcWorkspace.g[25]*nmpcWorkspace.x[25] + nmpcWorkspace.g[26]*nmpcWorkspace.x[26] + nmpcWorkspace.g[27]*nmpcWorkspace.x[27] + nmpcWorkspace.g[28]*nmpcWorkspace.x[28] + nmpcWorkspace.g[29]*nmpcWorkspace.x[29] + nmpcWorkspace.g[30]*nmpcWorkspace.x[30] + nmpcWorkspace.g[31]*nmpcWorkspace.x[31] + nmpcWorkspace.g[32]*nmpcWorkspace.x[32] + nmpcWorkspace.g[33]*nmpcWorkspace.x[33] + nmpcWorkspace.g[34]*nmpcWorkspace.x[34] + nmpcWorkspace.g[35]*nmpcWorkspace.x[35] + nmpcWorkspace.g[36]*nmpcWorkspace.x[36] + nmpcWorkspace.g[37]*nmpcWorkspace.x[37] + nmpcWorkspace.g[38]*nmpcWorkspace.x[38] + nmpcWorkspace.g[39]*nmpcWorkspace.x[39] + nmpcWorkspace.g[40]*nmpcWorkspace.x[40] + nmpcWorkspace.g[41]*nmpcWorkspace.x[41] + nmpcWorkspace.g[42]*nmpcWorkspace.x[42] + nmpcWorkspace.g[43]*nmpcWorkspace.x[43] + nmpcWorkspace.g[44]*nmpcWorkspace.x[44] + nmpcWorkspace.g[45]*nmpcWorkspace.x[45] + nmpcWorkspace.g[46]*nmpcWorkspace.x[46] + nmpcWorkspace.g[47]*nmpcWorkspace.x[47];
kkt = fabs( kkt );
for (index = 0; index < 48; ++index)
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

for (lRun1 = 0; lRun1 < 10; ++lRun1)
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
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[80];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[81];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[82];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[83];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[84];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[85];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[86];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[87];
nmpcWorkspace.objValueIn[8] = nmpcVariables.od[30];
nmpcWorkspace.objValueIn[9] = nmpcVariables.od[31];
nmpcWorkspace.objValueIn[10] = nmpcVariables.od[32];
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
for (lRun1 = 0; lRun1 < 10; ++lRun1)
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

