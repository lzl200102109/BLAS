#include<iostream>
#include<math.h>
#include <mgl2/mgl.h>
#include <mgl2/qt.h>
#include <mgl2/glut.h>

using namespace std;

int sample1(mglGraph *gr)
{
  mglData y0(50);   y0.Modify("sin(pi*(2*x-1))");
  gr->SubPlot(2,2,0);
  gr->Plot(y0);     gr->Box();

  gr->SubPlot(2,2,1);
  mglData y1(50,2);
  y1.Modify("sin(pi*2*x-pi)");
  y1.Modify("cos(pi*2*x-pi)/2",1);
  gr->Plot(y1);     gr->Box();

  mglData x(50);    x.Modify("cos(pi*2*x-pi)");
  gr->Plot(x,y0,"Y+");
  gr->Plot(y1.SubData(-1,0),y1.SubData(-1,1),"q|");

  gr->SubPlot(2,2,2); gr->Rotate(60,40);
  mglData z(50);    z.Modify("2*x-1");
  gr->Plot(x,y0,z); gr->Box();

  mglData y2(10,3); y2.Modify("cos(pi*(2*x-1+y))");
  y2.Modify("2*x-1",2);
  gr->Plot(y2.SubData(-1,0),y2.SubData(-1,1),y2.SubData(-1,2),"bo ");

  gr->SubPlot(2,2,3); gr->Rotate(60,40);
  gr->Bars(x,y0,z,"r"); gr->Box();

  return 0;
}


int sample2(mglGraph *gr)
{
  gr->Light(true);

  mglData a0(50,40);
  a0.Modify("0.6*sin(2*pi*x)*sin(3*pi*y)+0.4*cos(3*pi*(x*y))");
  gr->SubPlot(2,2,0); gr->Rotate(60,40);
  gr->Surf(a0);   gr->Box();

  mglData x(50,40),y(50,40),z(50,40);
  x.Modify("0.8*sin(2*pi*x)*sin(pi*y)");
  y.Modify("0.8*cos(2*pi*x)*sin(pi*y)");
  z.Modify("0.8*cos(pi*y)");
  gr->SubPlot(2,2,1); gr->Rotate(60,40);
  gr->Surf(x,y,z,"BbwrR");gr->Box();

  mglData a1(50,40,3);
  a1.Modify("0.6*sin(2*pi*x)*sin(3*pi*y)+0.4*cos(3*pi*(x*y))");
  a1.Modify("0.6*cos(2*pi*x)*cos(3*pi*y)+0.4*sin(3*pi*(x*y))",1);
  a1.Modify("0.6*cos(2*pi*x)*cos(3*pi*y)+0.4*cos(3*pi*(x*y))",2);
  gr->SubPlot(2,2,2); gr->Rotate(60,40);
  gr->Alpha(true);
  gr->Surf(a1);   gr->Box();

  gr->SubPlot(2,2,3); gr->Rotate(60,40);
  gr->Dens(a1);   gr->Box();

  return 0;
}

int sample3(mglGraph *gr)
{
  gr->Alpha(true);  gr->Light(true);
  mglData a(30,20);
  a.Modify("0.6*sin(2*pi*x)*sin(3*pi*y) + 0.4*cos(3*pi*(x*y))");

  gr->SubPlot(2,2,0); gr->Rotate(40,60);
  gr->Surf(a,"BbcyrR#");    gr->Box();
  gr->SubPlot(2,2,1); gr->Rotate(40,60);
  gr->Dens(a,"BbcyrR#");    gr->Box();
  gr->SubPlot(2,2,2); gr->Rotate(40,60);
  gr->Cont(a,"BbcyrR#");    gr->Box();
  gr->SubPlot(2,2,3); gr->Rotate(40,60);
  gr->Axial(a,"BbcyrR#");   gr->Box();
  return 0;
}


int main() {

  mglQT gr(sample1,"MathGL examples");
  return gr.Run();
}
