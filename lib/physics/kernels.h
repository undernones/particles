/******************************************************************************
Copyright (c) 2007 Bart Adams (bart.adams@gmail.com)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software. The authors shall be
acknowledged in scientific publications resulting from using the Software
by referencing the ACM SIGGRAPH 2007 paper "Adaptively Sampled Particle
Fluids".

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
******************************************************************************/

#ifndef __KERNELS_H__
#define __KERNELS_H__

#define MYPI 3.14159265359f
#define KERNEL_3D 1
//#define OFFLINE_SIMULATION 1

class Kernels {

public:

static inline double spikykernel(const double R, const double r) {
   const double q = r/R;
   if (q>1.0) return 0.0;
   else {
      const double d = 1.0-q;
      const double RR = R*R;
#ifndef KERNEL_3D
      return 10.0/(MYPI * RR) * d * d * d;
#else
      return 15.0/(MYPI * RR * R) * d * d * d;
#endif

   }
}

static inline double standardkernel(const double R, const double r) {
   const double RR = R*R;
   const double qq = r*r/RR;

      if (qq > 1)
         return 0;
      else
      {
         const double dd = 1 - qq;
#ifndef KERNEL_3D
            // alpha = 4 / (Pi * h^2)
            return 4.0/(MYPI * RR)*dd*dd*dd;
#else
            // alpha = 315 / (Pi * h^3 * 64)
         return 315.0/(64.0*MYPI*RR*R)*dd*dd*dd;
#endif
      }

}

static inline double smoothkernel(const double R, const double r) {
   const double q = r/R;
   if (q>1.0) return 0.0;
   else {
      return 1.0-q*q;
   
   }
}

static inline double spikykernelgradient(const double R, const double r) {
   const double q = r/R;
   if (q>1.0) return 0.0;
   //else if (r==0.0) return 0.0;
   else {
      const double d = 1.0-q;
      const double RR = R*R;
#ifndef KERNEL_3D
      return -30.0 / (MYPI * RR) *d*d;
#else
      return -45.0 / (MYPI * RR*R) *d*d;
#endif
   }
}

static inline double viscositykernellaplacian(const double R, const double r) {
   const double q = r/R;
   if (q>1.0) return 0.0;
   else {
      const double d = 1.0-q;
      const double RR = R*R;
#ifndef KERNEL_3D
   return 60.0/(11.0 * MYPI * RR) *d;
#else
   return 45.0/(13.0 * MYPI * RR *R) *d;
#endif
   }
}

static inline void gradientlaplace(const double R, const double r, double &grad, double &lap) {
   const double q = r/R;
   if (q>1.0 || r==0.0) grad=lap=0.0;
   else {
      const double RR = R*R;
      const double dd = 1-q*q;
#ifndef KERNEL_3D
      const double alpha = 24.0 / (MYPI * RR);
#else
      const double alpha = 945.0 / (32.0 * MYPI * RR *R);
#endif
      lap = alpha * (4.0*q*q*dd-dd*dd);
      grad = -alpha * q * dd * dd;
   }
}

static inline double gradient(const double R, const double r, double &grad) {
   const double q = r/R;
   if (q>1.0 || r==0.0) grad=0.0;
   else {
      const double RR = R*R;
      const double dd = 1-q*q;
#ifndef KERNEL_3D
      const double alpha = 24.0 / (MYPI * RR);
#else
      const double alpha = 945.0 / (32.0 * MYPI * RR *R);
#endif
      grad = -alpha * q * dd * dd;
   }
   return grad;
}

static inline void smoothgradient(const double R, const double r, double &grad) {
   const double q = r/R;
   if (q>1.0 || r==0.0) grad=0.0;
   else {
      const double RR = R*R;
      const double dd = 1-q*q;
#ifndef KERNEL_3D
      const double alpha = 24.0 / (MYPI * RR);
#else
      const double alpha = 945.0 / (32.0 * MYPI * RR *R);
#endif
      grad = -alpha * dd;
   }
}

};

#endif
