#include<iostream>
#include<math.h>
#include <mgl2/mgl.h>
#include <mgl2/qt.h>

using namespace std;

int sample1(mglGraph *gr)
{
  gr->SetRanges(0,1,0,1,0,1);
  mglData a(30,40); a.Modify("x*y");
  gr->SubPlot(2,2,0); gr->Rotate(60,40);
  gr->Surf(a);    gr->Box();
  gr->Puts(mglPoint(0.7,1,1.2),"a(x,y)");
  gr->SubPlot(2,2,1); gr->Rotate(60,40);
  a.Diff("x");    gr->Surf(a);  gr->Box();
  gr->Puts(mglPoint(0.7,1,1.2),"da/dx");
  gr->SubPlot(2,2,2); gr->Rotate(60,40);
  a.Integral("xy"); gr->Surf(a);  gr->Box();
  gr->Puts(mglPoint(0.7,1,1.2),"\\int da/dx dxdy");
  gr->SubPlot(2,2,3); gr->Rotate(60,40);
  a.Diff2("y"); gr->Surf(a);  gr->Box();
  gr->Puts(mglPoint(0.7,1,1.2),"\\int {d^2}a/dxdy dx");
  return 0;
}

int sample2(mglGraph *gr)
{
  gr->SubPlot(2,2,0,"");  gr->Title("Envelop sample");
  mglData d1(1000); gr->Fill(d1,"exp(-8*x^2)*sin(10*pi*x)");
  gr->Axis();     gr->Plot(d1, "b");
  d1.Envelop('x');  gr->Plot(d1, "r");

  gr->SubPlot(2,2,1,"");  gr->Title("Smooth sample");
  mglData y0(30),y1,y2,y3;
  gr->SetRanges(0,1,0,1);
  gr->Fill(y0, "0.4*sin(pi*x) + 0.3*cos(1.5*pi*x) - 0.4*sin(2*pi*x)+0.5*rnd");

  y1=y0;  y1.Smooth("x3");
  y2=y0;  y2.Smooth("x5");
  y3=y0;  y3.Smooth("x");

  gr->Plot(y0,"{m7}:s", "legend 'none'"); //gr->AddLegend("none","k");
  gr->Plot(y1,"r", "legend ''3' style'");
  gr->Plot(y2,"g", "legend ''5' style'");
  gr->Plot(y3,"b", "legend 'default'");
  gr->Legend();   gr->Box();

  gr->SubPlot(2,2,2);   gr->Title("Sew sample");
  mglData d2(100, 100); gr->Fill(d2, "mod((y^2-(1-x)^2)/2,0.1)");
  gr->Rotate(50, 60);   gr->Light(true);  gr->Alpha(true);
  gr->Box();            gr->Surf(d2, "b");
  d2.Sew("xy", 0.1);  gr->Surf(d2, "r");

  gr->SubPlot(2,2,3);   gr->Title("Resize sample (interpolation)");
  mglData x0(10), v0(10), x1, v1;
  gr->Fill(x0,"rnd");     gr->Fill(v0,"rnd");
  x1 = x0.Resize(100);    v1 = v0.Resize(100);
  gr->Plot(x0,v0,"b+ ");  gr->Plot(x1,v1,"r-");
  gr->Label(x0,v0,"%n");
  return 0;
}

int sample3(mglGraph *gr)
{
  gr->SetRange('z',0,1);
  mglData x(20,30), y(20,30), z(20,30), xx,yy,zz;
  gr->Fill(x,"(x+2)/3*cos(pi*y)");
  gr->Fill(y,"(x+2)/3*sin(pi*y)");
  gr->Fill(z,"exp(-6*x^2-2*sin(pi*y)^2)");

  gr->SubPlot(2,1,0); gr->Title("Cartesian space");   gr->Rotate(30,-40);
  gr->Axis("xyzU");   gr->Box();  gr->Label('x',"x"); gr->Label('y',"y");
  gr->SetOrigin(1,1); gr->Grid("xy");
  gr->Mesh(x,y,z);

  // section along 'x' direction
  mglData u = x.Solve(0.5,'x');
  mglData v(u.nx);  v.Fill(0,1);
  xx = x.Evaluate(u,v);   yy = y.Evaluate(u,v);   zz = z.Evaluate(u,v);
  gr->Plot(xx,yy,zz,"k2o");

  // 1st section along 'y' direction
  mglData u1 = x.Solve(-0.5,'y');
  mglData v1(u1.nx);  v1.Fill(0,1);
  xx = x.Evaluate(v1,u1); yy = y.Evaluate(v1,u1); zz = z.Evaluate(v1,u1);
  gr->Plot(xx,yy,zz,"b2^");

  // 2nd section along 'y' direction
  mglData u2 = x.Solve(-0.5,'y',u1);
  xx = x.Evaluate(v1,u2); yy = y.Evaluate(v1,u2); zz = z.Evaluate(v1,u2);
  gr->Plot(xx,yy,zz,"r2v");

  gr->SubPlot(2,1,1); gr->Title("Accompanied space");
  gr->SetRanges(0,1,0,1); gr->SetOrigin(0,0);
  gr->Axis(); gr->Box();  gr->Label('x',"i"); gr->Label('y',"j");
  gr->Grid(z,"h");

  gr->Plot(u,v,"k2o");    gr->Line(mglPoint(0.4,0.5),mglPoint(0.8,0.5),"kA");
  gr->Plot(v1,u1,"b2^");  gr->Line(mglPoint(0.5,0.15),mglPoint(0.5,0.3),"bA");
  gr->Plot(v1,u2,"r2v");  gr->Line(mglPoint(0.5,0.7),mglPoint(0.5,0.85),"rA");
}

int main() {

  /* 1D data */
	// external array
	double *a1 = new double[50];
	for(int i=0;i<50;i++)   a1[i] = sin(M_PI*i/49.);

	mglData y1;
	y1.Set(a1,50);

	// direct array
	mglData y2(50);
	for(int i=0;i<50;i++)   y2.a[i] = sin(M_PI*i/49.);

	// textual formula
  mglData y3(50);
  y3.Modify("sin(pi*x)");

  mglData y4(50);
  y4.Fill(0,M_PI);
  y4.Modify("sin(u)");


  // load from file
  FILE *fp1=fopen("sin.dat","wt");   // create file first
  for(int i=0;i<50;i++)   fprintf(fp1,"%g\n",sin(M_PI*i/49.));
  fclose(fp1);

  mglData y5("sin.dat");             // load it

  // load part of file
  FILE *fp2=fopen("sin.dat","wt");   // create large file first
  for(int i=0;i<70;i++)   fprintf(fp2,"%g\n",sin(M_PI*i/49.));
  fclose(fp2);

  mglData y6;
  y6.Read("sin.dat",50);             // load it

  /* 2D data */
  mglData z1(30,40);
  for(int i=0;i<30;i++)   for(int j=0;j<40;j++)
    z1.a[i+30*j] = sin(M_PI*i/29.)*sin(M_PI*j/39.);

  mglData z2(30,40);
  z2.Modify("sin(pi*x)*cos(pi*y)");

  /* link */
  double *a2 = new double[50];
  for(int i=0;i<50;i++)   a2[i] = sin(M_PI*i/49.);

  mglData y;
  y.Link(a2,50);


  mglQT gr(sample3,"MathGL examples");
  return gr.Run();
}
