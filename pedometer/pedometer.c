/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

#include "msp430.h"
#include "b_filter_coeff.h"
#include "pedometer.h"

void b_filter(signed int *, signed int *, unsigned short);
void b2_filter(signed int *, signed int *, unsigned short);
void do_q15_mult(signed int, signed int *);

unsigned char index = 0;

#pragma NOINIT(x)
signed int x[50];

#pragma NOINIT(y)
signed int y[50];

#pragma NOINIT(z)
signed int z[50];

#pragma NOINIT(g)
signed int g[38], g_tmp, g_prev;
signed int vcnt_up=0,vcnt_dn=0;

#pragma NOINIT(x_temp)
signed int x_temp[24];

#pragma NOINIT(y_temp)
signed int y_temp[24];

#pragma NOINIT(z_temp)
signed int z_temp[24];

#pragma NOINIT(g_temp)
signed int g_temp[12];
unsigned short i=0,j, g_index=0;
unsigned short step_cnt=0,cnt_flg=1, cnt_trip=46;
unsigned short cnt=0, prev_cnt=0, no_cnt_flg, disp_trip=0;
unsigned short first_loop=1,up_flg=1,dn_flg=0;
unsigned short length_g=14, reset_time=99,pos_flg=1,strt_trip=0;


/**
* @brief <b>Description:</b> Updates pedometer sample every time accelerometer is read
* @param[in] *p_data
* @return result
**/
char ped_update_sample(short* p_data)
{
	// WE COME HERE ON EVERY ACCEL SAMPLE
	static unsigned int sample_index = 0;
	unsigned char result = 0;

	x[sample_index] = (unsigned int) *p_data++;		// x
	y[sample_index] = (unsigned int) *p_data++;		// y
	z[sample_index] = (unsigned int) *p_data;		//z

	sample_index++;

	if(sample_index > 49)
	{
		sample_index = 24;
		result = 1;
	}
	return(result);
}

/**
* @brief <b>Description:</b> Initialize pedometer variables and structures
**/
void ped_step_detect_init(void)
{
	for(index=0; index < 50; index++)
	{
		x[index] = 0;
		y[index] = 0;
		z[index] = 0;
	}

	index = 0;
	first_loop = 1;
	step_cnt=0;
	cnt=0;
	prev_cnt=0;
	cnt_flg=1;
	cnt_trip=46;
	disp_trip=0;
	i=0;
	g_index=0;
	reset_time=99;
	length_g=14;
	pos_flg=1;
	vcnt_up=0;
	vcnt_dn=0;
	strt_trip=0;
	up_flg=1;
	dn_flg=0;

}

/**
* @brief <b>Description:</b> This is the algorithm that processes real-time data input
* @return step count
**/
unsigned short ped_step_detect(void)
{

	// do processing
	b_filter(&x[0], &x_temp[0], first_loop);
	b_filter(&y[0], &y_temp[0], first_loop);
	b_filter(&z[0], &z_temp[0], first_loop);


	for(j=0;j<26;j++)
	{
		if (x[j] < 0)
		{do_q15_mult(0x8000,&x[j]);} // mult by -1
		if (y[j] < 0)
		{do_q15_mult(0x8000,&y[j]);}
		if (z[j] < 0)
		{do_q15_mult(0x8000,&z[j]);}
		g[j+g_index] = x[j]+y[j]+z[j];
	}

	b2_filter(&g[0], &g_temp[0], first_loop);

	if (first_loop==1)
	{
		g_prev=g[0];
	}

	for (j=0;j<length_g;j++)
	{
		if (g[j]-g_prev > 0)
		{
			pos_flg=1;
		}
		else if (g[j]-g_prev < 0)
		{
			pos_flg=0;
		}

		if (cnt_flg==1 && pos_flg==1)
		{
			if( (cnt_trip>11 && vcnt_up > 91) || (cnt_trip>15 && vcnt_up > 71)||(cnt_trip>19 && vcnt_up > 54) || (cnt_trip>23 && vcnt_up > 41) || (cnt_trip>29 && vcnt_up>33) )
			{
				cnt_flg=0;
                cnt=cnt+1;
                cnt_trip=0;
                vcnt_up=0;
                up_flg=0;
                strt_trip=1;
			}
		}
		else if(cnt_flg==0)
		{
			if ((cnt_trip>5) && (vcnt_dn > 3)) //15
			{
				cnt_flg=1;
				vcnt_dn=0;
	           	dn_flg=0;
			}

			else if (cnt_trip>59)
			{
				cnt_flg=1;
				vcnt_dn=0;
	            dn_flg=0;
			}
		}

		if (pos_flg==1 && cnt_flg==1)
		{
            up_flg=1;
		}
        else if (pos_flg==0 && cnt_flg==0)
        {
            dn_flg=1;
            if (strt_trip==1)
            {
                cnt_trip=0;
                strt_trip=0;
                vcnt_up=0;
            }
        }

        if (up_flg==1)
        {
            vcnt_up=vcnt_up+(g[j]-g_prev);
        }
        else if (dn_flg==1)
        {
            vcnt_dn=vcnt_dn+(g_prev-g[j]);
        }

        if (vcnt_up < 0)
        {
            vcnt_up=0;
            up_flg=0;
        }
        else if (vcnt_dn < 0)
        {
            vcnt_dn=0;
            dn_flg=0;
        }

		cnt_trip++;
        g_prev=g[j];

        if (prev_cnt==cnt)
        {
        	no_cnt_flg=1;
        }
        else
        {
            no_cnt_flg=0;
            prev_cnt=cnt;
        }

        if ((no_cnt_flg==1) && (cnt_trip > reset_time))
        {
            disp_trip=0;
            cnt=step_cnt;
            reset_time=99;
		}
	}

    	if (disp_trip > 15)
        {
        	step_cnt=cnt;
        	reset_time=85;
        }

    	if (disp_trip < 16)
        	disp_trip++;


		first_loop=0;
		length_g=26;
		g_index=12;

	return(step_cnt);

}

/**
* @brief <b>Description:</b> B filter algorithm
* @param[in] *d
* @param[in] *t
* @param[in] first_loop
**/
void b_filter(signed int *d, signed int *t, unsigned short first_loop)
{
	unsigned short i,j;
	signed int R=0;

	WDTCTL = WDTPW+WDTHOLD;                   // Stop WDT
  	MPY32CTL0 = MPYFRAC;                      // Set fractional mode
  	
  	// initialize data
  	if(first_loop==1)
  	{
	  	for(i=0;i<24;i++)
	  	{
	  		*(t+i)= *(d+26+i);
	  	}
  	}
  	else
  	{
  		for (i=0;i<24;i++)
  		{
  			*(d+i) = *(t+i);
  			*(t+i) = *(d+26+i);
  		}
  	}
  	
  	for(j=24;j<50;j++)
  	{
  		R=0;
  		for(i=0;i<25;i++)
  		{
  			MPYS = FILTER_COEFF_b[i];                            // Load first operand
  			OP2 = d[j-i];                             // Load second operand
  			R += RESHI;
  		}  
  		d[j-24]=R;         
  	}
  	
  	MPY32CTL0 &= ~MPYFRAC;  	
}

/**
* @brief <b>Description: B2 filter algorithm</b>
* @param[in] *d
* @param[in] *t
* @param[in] first_loop
**/
void b2_filter(signed int *d, signed int *t, unsigned short first_loop)
{
	unsigned short i,j,nn=26;
	signed int R=0;

	WDTCTL = WDTPW+WDTHOLD;                   // Stop WDT
  	MPY32CTL0 = MPYFRAC;                      // Set fractional mode
  	
  	// initialize data
  	if(first_loop==1)
  	{
	  	for(i=0;i<12;i++)
	  	{
	  		*(t+i)= *(d+14+i);
	  	}
  	}
  	else
  	{
  		for (i=0;i<12;i++)
  		{
  			*(d+i) = *(t+i);
  			*(t+i) = *(d+26+i);
  		}
  		nn=38;
  	}
  	
  	for(j=12;j<nn;j++)
  	{
  		R=0;
  		for(i=0;i<13;i++)
  		{
  			MPYS = 0x09D8;                            // Load first operand
  			OP2 = d[j-i];                             // Load second operand
  			R += RESHI;
  		}  
  		d[j-12]=R;         
  	}
  	
  	MPY32CTL0 &= ~MPYFRAC;  	
}


/**
* @brief <b>Description: Q15 multiplication</b>
* @param[in] a
* @param[in] *var
**/
void do_q15_mult(signed int a, signed int * var)
{
	WDTCTL = WDTPW+WDTHOLD;                   // Stop WDT

  	MPY32CTL0 = MPYFRAC;                      // Set fractional mode
  	MPYS = a;                            // Load first operand
  	OP2 = *(var);                             // Load second operand
  	*(var) = RESHI;                       // Q15 result
  
  	MPY32CTL0 &= ~MPYFRAC;
}


