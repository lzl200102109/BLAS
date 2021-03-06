#include<iostream>
#include<math.h>
#include <mgl2/mgl.h>
#include <mgl2/qt.h>
#include <mgl2/glut.h>

using namespace std;

void mgls_prepare2v(mglData *a, mglData *b)
{
  register long i,j,n=20,m=30,i0;
  if(a) a->Create(n,m);   if(b) b->Create(n,m);
  mreal x,y;
  for(i=0;i<n;i++)  for(j=0;j<m;j++)
  {
    x=i/(n-1.); y=j/(m-1.); i0 = i+n*j;
    if(a) a->a[i0] = 0.6*sin(2*M_PI*x)*sin(3*M_PI*y)+0.4*cos(3*M_PI*x*y);
    if(b) b->a[i0] = 0.6*cos(2*M_PI*x)*cos(3*M_PI*y)+0.4*cos(3*M_PI*x*y);
  }
}
void mgls_prepare3v(mglData *ex, mglData *ey, mglData *ez)
{
  register long i,j,k,n=10,i0;
  if(!ex || !ey || !ez) return;
  ex->Create(n,n,n);  ey->Create(n,n,n);  ez->Create(n,n,n);
  mreal x,y,z, r1,r2;
  for(i=0;i<n;i++)  for(j=0;j<n;j++)  for(k=0;k<n;k++)
  {
    x=2*i/(n-1.)-1; y=2*j/(n-1.)-1; z=2*k/(n-1.)-1; i0 = i+n*(j+k*n);
    r1 = pow(x*x+y*y+(z-0.3)*(z-0.3)+0.03,1.5);
    r2 = pow(x*x+y*y+(z+0.3)*(z+0.3)+0.03,1.5);
    ex->a[i0]=0.2*x/r1 - 0.2*x/r2;
    ey->a[i0]=0.2*y/r1 - 0.2*y/r2;
    ez->a[i0]=0.2*(z-0.3)/r1 - 0.2*(z+0.3)/r2;
  }
}

int sample1(mglGraph *gr)
{
  mglData a,b;  mgls_prepare2v(&a,&b);
  gr->SubPlot(3,2,0,""); gr->Title("Vect plot (default)");
  gr->Box();  gr->Vect(a,b);

  gr->SubPlot(3,2,1,"");  gr->Title("'.' style; '=' style");
  gr->Box();  gr->Vect(a,b,"=.");

  gr->SubPlot(3,2,2,"");  gr->Title("'f' style");
  gr->Box();  gr->Vect(a,b,"f");

  gr->SubPlot(3,2,3,"");  gr->Title("'>' style");
  gr->Box();  gr->Vect(a,b,">");

  gr->SubPlot(3,2,4,"");  gr->Title("'<' style");
  gr->Box();  gr->Vect(a,b,"<");

  mglData ex,ey,ez; mgls_prepare3v(&ex,&ey,&ez);
  gr->SubPlot(3,2,5); gr->Title("3d variant");  gr->Rotate(50,60);
  gr->Box();  gr->Vect(ex,ey,ez);
  return 0;
}


int main() {

  mglQT gr(sample1,"MathGL examples");
  return gr.Run();
}
