#include "../Header/win_api.h"
#include <math.h>
#include <stdio.h>
#define PI (3.1415926)

static void conjugate_complex(int n, Complexfloat in[], Complexfloat out[])
{
	int i = 0;
	for (i = 0; i < n; i++)
	{
		out[i].image = -in[i].image;
		out[i].real = in[i].real;
	}
}
static void c_abs(Complexfloat f[], float out[], int n)
{
	int i = 0;
	float t;
	for (i = 0; i < n; i++)
	{
		t = f[i].real * f[i].real + f[i].image * f[i].image;
		out[i] = sqrt(t);
	}
}
static void c_plus(Complexfloat a, Complexfloat b, Complexfloat* c)
{
	c->real = a.real + b.real;
	c->image = a.image + b.image;
}
static void c_sub(Complexfloat a, Complexfloat b, Complexfloat* c)
{
	c->real = a.real - b.real;
	c->image = a.image - b.image;
}
static void c_mul(Complexfloat a, Complexfloat b, Complexfloat* c)
{
	c->real = a.real * b.real - a.image * b.image;
	c->image = a.real * b.image + a.image * b.real;
}
static void c_div(Complexfloat a, Complexfloat b, Complexfloat* c)
{
	c->real = (a.real * b.real + a.image * b.image) / (b.real * b.real + b.image * b.image);
	c->image = (a.image * b.real - a.real * b.image) / (b.real * b.real + b.image * b.image);
}
static void Wn_i(int n, int i, Complexfloat* Wn, char flag)
{
	Wn->real = (float)cos(2 * PI * i / n);
	if (flag == 1)
		Wn->image = (float)-sin(2 * PI * i / n);
	else if (flag == 0)
		Wn->image = (float)-sin(2 * PI * i / n);
}

int get_shift(Complexfloat f[], int N)
{
	float min, max;
	int shift;
	min = 1.E+10;
	max = -1.E+10;
	shift = 0;
	for (int k = 0; k < N; k++)
	{
		if (f[k].real > max)
		{
			max = f[k].real;
		}
		if (f[k].image > max)
		{
			max = f[k].image;
		}
		if (f[k].real < min)
		{
			min = f[k].real;
		}
		if (f[k].image < min)
		{
			min = f[k].image;
		}
	}
	while (max > 32767)
	{
		max = max / 2;
		min = min / 2;
		shift += 1;
	}
	while (min < -32767)
	{
		min = min / 2;
		shift += 1;
	}
	return shift;
}


int win_fft(int N, Complexfloat f[], short scaleFactor, double gat)
{
	Complexfloat t, wn;
	int i, j, k, m, n, l, r, M, s;
	int la, lb, lc;
	for (i = N, M = 1; (i = i / 2) != 1; M++);
	for (i = 1, j = N / 2; i <= N - 2; i++)
	{
		if (i < j)
		{
			t = f[j];
			f[j] = f[i];
			f[i] = t;
		}
		k = N / 2;
		while (k <= j)
		{
			j = j - k;
			k = k / 2;
		}
		j = j + k;
	}
	for (m = 1; m <= M; m++)
	{
		la = (int)(1U << m);
		lb = la / 2;
		for (l = 1; l <= lb; l++)
		{
			r = (l - 1) * (int)pow(2.0, M - m);
			for (n = l - 1; n < N - 1; n = n + la)
			{
				lc = n + lb;
				Wn_i(N, r, &wn, 1);
				c_mul(f[lc], wn, &t);
				c_sub(f[n], t, &(f[lc]));
				c_plus(f[n], t, &(f[n]));
			}
		}
	}
	float factor = (float)(1U << scaleFactor);
	for (s = 0; s < N; s++)
	{
		f[s].real = f[s].real / factor;
		f[s].image = f[s].image / factor;

	}
#if 0
	for (s = 0; s < N; s++)
	{
		if (fabs(f[s].real) > gat || fabs(f[s].image) > gat)
		{
			printf("%s overflow::shift=%d (%.1f || %.1f) > %.0f )\n", __FUNCTION__,
				s, f[s].real, f[s].image, gat);
			return -1;
		}
	}
#endif

	return 0;
}