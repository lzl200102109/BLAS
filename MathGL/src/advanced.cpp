
/**
 * MATHGL EXAMPLES
 */

#include <mgl2/mgl.h>
#include <mgl2/qt.h>

int sample1(mglGraph *gr)
{
  gr->SubPlot(2,2,0); gr->Box();
  gr->Puts(mglPoint(-1,1.1),"Just box",":L");
  gr->InPlot(0.2,0.5,0.7,1,false);  gr->Box();
  gr->Puts(mglPoint(0,1.2),"InPlot example");
  gr->SubPlot(2,2,1); gr->Title("Rotate only");
  gr->Rotate(50,60);  gr->Box();
  gr->SubPlot(2,2,2); gr->Title("Rotate and Aspect");
  gr->Rotate(50,60);  gr->Aspect(1,1,2);  gr->Box();
  gr->SubPlot(2,2,3); gr->Title("Aspect in other direction");
  gr->Rotate(50,60);  gr->Aspect(1,2,2);  gr->Box();
  return 0;
}

int sample2(mglGraph *gr)
{
  gr->SubPlot(3,2,0); gr->Title("StickPlot");
  gr->StickPlot(3, 0, 20, 30);  gr->Box("r"); gr->Puts(mglPoint(0),"0","r");
  gr->StickPlot(3, 1, 20, 30);  gr->Box("g"); gr->Puts(mglPoint(0),"1","g");
  gr->StickPlot(3, 2, 20, 30);  gr->Box("b"); gr->Puts(mglPoint(0),"2","b");
  gr->SubPlot(3,2,3,"");  gr->Title("ColumnPlot");
  gr->ColumnPlot(3, 0); gr->Box("r"); gr->Puts(mglPoint(0),"0","r");
  gr->ColumnPlot(3, 1); gr->Box("g"); gr->Puts(mglPoint(0),"1","g");
  gr->ColumnPlot(3, 2); gr->Box("b"); gr->Puts(mglPoint(0),"2","b");
  gr->SubPlot(3,2,4,"");  gr->Title("GridPlot");
  gr->GridPlot(2, 2, 0);  gr->Box("r"); gr->Puts(mglPoint(0),"0","r");
  gr->GridPlot(2, 2, 1);  gr->Box("g"); gr->Puts(mglPoint(0),"1","g");
  gr->GridPlot(2, 2, 2);  gr->Box("b"); gr->Puts(mglPoint(0),"2","b");
  gr->GridPlot(2, 2, 3);  gr->Box("m"); gr->Puts(mglPoint(0),"3","m");
  gr->SubPlot(3,2,5,"");  gr->Title("InPlot");  gr->Box();
  gr->InPlot(0.4, 1, 0.6, 1, true); gr->Box("r");
  gr->MultiPlot(3,2,1, 2, 1,"");  gr->Title("MultiPlot"); gr->Box();
  return 0;
}

int sample3(mglGraph *gr)
{
  gr->SubPlot(2,2,0); gr->Title("Axis origin, Grid"); gr->SetOrigin(0,0);
  gr->Axis(); gr->Grid(); gr->FPlot("x^3");

  gr->SubPlot(2,2,1); gr->Title("2 axis");
  gr->SetRanges(-1,1,-1,1); gr->SetOrigin(-1,-1,-1);  // first axis
  gr->Axis(); gr->Label('y',"axis 1",0);  gr->FPlot("sin(pi*x)");
  gr->SetRanges(0,1,0,1);   gr->SetOrigin(1,1,1);   // second axis
  gr->Axis(); gr->Label('y',"axis 2",0);  gr->FPlot("cos(pi*x)");

  gr->SubPlot(2,2,3); gr->Title("More axis");
  gr->SetOrigin(NAN,NAN); gr->SetRange('x',-1,1);
  gr->Axis(); gr->Label('x',"x",0); gr->Label('y',"y_1",0);
  gr->FPlot("x^2","k");
  gr->SetRanges(-1,1,-1,1); gr->SetOrigin(-1.3,-1); // second axis
  gr->Axis("y","r");  gr->Label('y',"#r{y_2}",0.2);
  gr->FPlot("x^3","r");

  gr->SubPlot(2,2,2); gr->Title("4 segments, inverted axis");
  gr->SetOrigin(0,0);
  gr->InPlot(0.5,1,0.5,1);  gr->SetRanges(0,10,0,2);  gr->Axis();
  gr->FPlot("sqrt(x/2)");   gr->Label('x',"W",1); gr->Label('y',"U",1);
  gr->InPlot(0,0.5,0.5,1);  gr->SetRanges(1,0,0,2); gr->Axis("x");
  gr->FPlot("sqrt(x)+x^3"); gr->Label('x',"\\tau",-1);
  gr->InPlot(0.5,1,0,0.5);  gr->SetRanges(0,10,4,0);  gr->Axis("y");
  gr->FPlot("x/4"); gr->Label('y',"L",-1);
  gr->InPlot(0,0.5,0,0.5);  gr->SetRanges(1,0,4,0); gr->FPlot("4*x^2");
  return 0;
}

int sample4(mglGraph *gr)
{
  gr->SubPlot(3,3,0); gr->Title("Usual axis");  gr->Axis();
  gr->SubPlot(3,3,1); gr->Title("Too big/small range");
  gr->SetRanges(-1000,1000,0,0.001);  gr->Axis();
  gr->SubPlot(3,3,2); gr->Title("LaTeX-like labels");
  gr->Axis("F!");
  gr->SubPlot(3,3,3); gr->Title("Too narrow range");
  gr->SetRanges(100,100.1,10,10.01);  gr->Axis();
  gr->SubPlot(3,3,4); gr->Title("No tuning, manual '+'");
  // for version<2.3 you need first call gr->SetTuneTicks(0);
  gr->Axis("+!");
  gr->SubPlot(3,3,5); gr->Title("Template for ticks");
  gr->SetTickTempl('x',"xxx:%g"); gr->SetTickTempl('y',"y:%g");
  gr->Axis();
  // now switch it off for other plots
  gr->SetTickTempl('x',"");   gr->SetTickTempl('y',"");
  gr->SubPlot(3,3,6); gr->Title("No tuning, higher precision");
  gr->Axis("!4");
  gr->SubPlot(3,3,7); gr->Title("Manual ticks");  gr->SetRanges(-M_PI,M_PI, 0, 2);
  gr->SetTicks('x',M_PI,0,0,"\\pi");  gr->AddTick('x',0.886,"x^*");
  // alternatively you can use following lines
  //double val[]={-M_PI, -M_PI/2, 0, 0.886, M_PI/2, M_PI};
  //gr->SetTicksVal('x', mglData(6,val), "-\\pi\n-\\pi/2\n0\nx^*\n\\pi/2\n\\pi");
  gr->Axis();  gr->Grid();  gr->FPlot("2*cos(x^2)^2", "r2");
  gr->SubPlot(3,3,8); gr->Title("Time ticks");  gr->SetRange('x',0,3e5);
  gr->SetTicksTime('x',0);  gr->Axis();
}

int sample5(mglGraph *gr)
{
  gr->SubPlot(2,2,0,"<_");  gr->Title("Semi-log axis");
  gr->SetRanges(0.01,100,-1,1); gr->SetFunc("lg(x)","");
  gr->Axis(); gr->Grid("xy","g"); gr->FPlot("sin(1/x)");
  gr->Label('x',"x",0); gr->Label('y', "y = sin 1/x",0);

  gr->SubPlot(2,2,1,"<_");  gr->Title("Log-log axis");
  gr->SetRanges(0.01,100,0.1,100);  gr->SetFunc("lg(x)","lg(y)");
  gr->Axis(); gr->Grid("!","h=");   gr->Grid();
  gr->FPlot("sqrt(1+x^2)"); gr->Label('x',"x",0);
  gr->Label('y', "y = \\sqrt{1+x^2}",0);

  gr->SubPlot(2,2,2,"<_");  gr->Title("Minus-log axis");
  gr->SetRanges(-100,-0.01,-100,-0.1);  gr->SetFunc("-lg(-x)","-lg(-y)");
  gr->Axis(); gr->FPlot("-sqrt(1+x^2)");
  gr->Label('x',"x",0); gr->Label('y', "y = -\\sqrt{1+x^2}",0);

  gr->SubPlot(2,2,3,"<_");  gr->Title("Log-ticks");
  gr->SetRanges(0.1,100,0,100); gr->SetFunc("sqrt(x)","");
  gr->Axis(); gr->FPlot("x");
  gr->Label('x',"x",1); gr->Label('y', "y = x",0);
  return 0;
}

int sample6(mglGraph *gr)
{
  gr->SetOrigin(-1,1,-1);

  gr->SubPlot(2,2,0); gr->Title("Cartesian"); gr->Rotate(50,60);
  gr->FPlot("2*t-1","0.5","0","r2");
  gr->Axis(); gr->Grid();

  gr->SetFunc("y*sin(pi*x)","y*cos(pi*x)",0);
  gr->SubPlot(2,2,1); gr->Title("Cylindrical"); gr->Rotate(50,60);
  gr->FPlot("2*t-1","0.5","0","r2");
  gr->Axis(); gr->Grid();

  gr->SetFunc("2*y*x","y*y - x*x",0);
  gr->SubPlot(2,2,2); gr->Title("Parabolic"); gr->Rotate(50,60);
  gr->FPlot("2*t-1","0.5","0","r2");
  gr->Axis(); gr->Grid();

  gr->SetFunc("y*sin(pi*x)","y*cos(pi*x)","x+z");
  gr->SubPlot(2,2,3); gr->Title("Spiral");  gr->Rotate(50,60);
  gr->FPlot("2*t-1","0.5","0","r2");
  gr->Axis(); gr->Grid();
  gr->SetFunc(0,0,0); // set to default Cartesian
  return 0;
}

void mgls_prepare2d(mglData *a, mglData *b=0, mglData *v=0)
{
  register long i,j,n=50,m=40,i0;
  if(a) a->Create(n,m);   if(b) b->Create(n,m);
  if(v) { v->Create(9); v->Fill(-1,1);  }
  mreal x,y;
  for(i=0;i<n;i++)  for(j=0;j<m;j++)
  {
    x = i/(n-1.); y = j/(m-1.); i0 = i+n*j;
    if(a) a->a[i0] = 0.6*sin(2*M_PI*x)*sin(3*M_PI*y)+0.4*cos(3*M_PI*x*y);
    if(b) b->a[i0] = 0.6*cos(2*M_PI*x)*cos(3*M_PI*y)+0.4*cos(3*M_PI*x*y);
  }
}

int sample7(mglGraph *gr)
{
  gr->SubPlot(2,2,0); gr->Title("Colorbar out of box"); gr->Box();
  gr->Colorbar("<");  gr->Colorbar(">");
  gr->Colorbar("_");  gr->Colorbar("^");

  gr->SubPlot(2,2,1); gr->Title("Colorbar near box");   gr->Box();
  gr->Colorbar("<I"); gr->Colorbar(">I");
  gr->Colorbar("_I"); gr->Colorbar("^I");

  gr->SubPlot(2,2,2); gr->Title("manual colors");
  mglData a,v;  mgls_prepare2d(&a,0,&v);
  gr->Box();  gr->ContD(v,a);
  gr->Colorbar(v,"<");  gr->Colorbar(v,">");
  gr->Colorbar(v,"_");  gr->Colorbar(v,"^");

  gr->SubPlot(2,2,3);   gr->Title(" ");
  gr->Puts(mglPoint(-0.5,1.55),"Color positions",":C",-2);
  gr->Colorbar("bwr>",0.25,0);  gr->Puts(mglPoint(-0.9,1.2),"Default");
  gr->Colorbar("b{w,0.3}r>",0.5,0); gr->Puts(mglPoint(-0.1,1.2),"Manual");

  gr->Puts(mglPoint(1,1.55),"log-scale",":C",-2);
  gr->SetRange('c',0.01,1e3);
  gr->Colorbar(">",0.75,0);  gr->Puts(mglPoint(0.65,1.2),"Normal scale");
  gr->SetFunc("","","","lg(c)");
  gr->Colorbar(">");    gr->Puts(mglPoint(1.35,1.2),"Log scale");
  return 0;
}

int sample8(mglGraph *gr)
{
  gr->SubPlot(2,2,0); gr->Title("Box (default)"); gr->Rotate(50,60);
  gr->Box();
  gr->SubPlot(2,2,1); gr->Title("colored");   gr->Rotate(50,60);
  gr->Box("r");
  gr->SubPlot(2,2,2); gr->Title("with faces");  gr->Rotate(50,60);
  gr->Box("@");
  gr->SubPlot(2,2,3); gr->Title("both");  gr->Rotate(50,60);
  gr->Box("@cm");
  return 0;
}

int sample9(mglGraph *gr)
{
  gr->SetRanges(0,1,0,1,0,1);
  mglData x(50),y(50),z(50),rx(10),ry(10), a(20,30);
  a.Modify("30*x*y*(1-x-y)^2*(x+y<1)");
  x.Modify("0.25*(1+cos(2*pi*x))");
  y.Modify("0.25*(1+sin(2*pi*x))");
  rx.Modify("rnd"); ry.Modify("(1-v)*rnd",rx);
  z.Modify("x");

  gr->SubPlot(2,2,0); gr->Title("Ordinary axis 3D");
  gr->Rotate(50,60);    gr->Light(true);
  gr->Plot(x,y,z,"r2"); gr->Surf(a,"BbcyrR#");
  gr->Axis(); gr->Grid(); gr->Box();
  gr->Label('x',"B",1); gr->Label('y',"C",1); gr->Label('z',"Z",1);

  gr->SubPlot(2,2,1); gr->Title("Ternary axis (x+y+t=1)");
  gr->Ternary(1);
  gr->Plot(x,y,"r2"); gr->Plot(rx,ry,"q^ ");  gr->Cont(a,"BbcyrR");
  gr->Line(mglPoint(0.5,0), mglPoint(0,0.75), "g2");
  gr->Axis(); gr->Grid("xyz","B;");
  gr->Label('x',"B"); gr->Label('y',"C"); gr->Label('t',"A");

  gr->SubPlot(2,2,2); gr->Title("Quaternary axis 3D");
  gr->Rotate(50,60);    gr->Light(true);
  gr->Ternary(2);
  gr->Plot(x,y,z,"r2"); gr->Surf(a,"BbcyrR#");
  gr->Axis(); gr->Grid(); gr->Box();
  gr->Label('t',"A",1); gr->Label('x',"B",1);
  gr->Label('y',"C",1); gr->Label('z',"D",1);

  gr->SubPlot(2,2,3); gr->Title("Ternary axis 3D");
  gr->Rotate(50,60);    gr->Light(true);
  gr->Ternary(1);
  gr->Plot(x,y,z,"r2"); gr->Surf(a,"BbcyrR#");
  gr->Axis(); gr->Grid(); gr->Box();
  gr->Label('t',"A",1); gr->Label('x',"B",1);
  gr->Label('y',"C",1); gr->Label('z',"Z",1);
  return 0;
}

void mgls_prepare1d(mglData *y, mglData *y1=0, mglData *y2=0, mglData *x1=0, mglData *x2=0)
{
  register long i,n=50;
  if(y) y->Create(n,3);
  if(x1)  x1->Create(n);    if(x2)  x2->Create(n);
  if(y1)  y1->Create(n);    if(y2)  y2->Create(n);
  mreal xx;
  for(i=0;i<n;i++)
  {
    xx = i/(n-1.);
    if(y)
    {
      y->a[i] = 0.7*sin(2*M_PI*xx) + 0.5*cos(3*M_PI*xx) + 0.2*sin(M_PI*xx);
      y->a[i+n] = sin(2*M_PI*xx);
      y->a[i+2*n] = cos(2*M_PI*xx);
    }
    if(y1)  y1->a[i] = 0.5+0.3*cos(2*M_PI*xx);
    if(y2)  y2->a[i] = 0.3*sin(2*M_PI*xx);
    if(x1)  x1->a[i] = xx*2-1;
    if(x2)  x2->a[i] = 0.05+0.03*cos(2*M_PI*xx);
  }
}


int sample10(mglGraph *gr)
{
  gr->SubPlot(2,2,0,"");
  gr->Putsw(mglPoint(0,1),L"Text can be in ASCII and in Unicode");
  gr->Puts(mglPoint(0,0.6),"It can be \\wire{wire}, \\big{big} or #r{colored}");
  gr->Puts(mglPoint(0,0.2),"One can change style in string: "
  "\\b{bold}, \\i{italic, \\b{both}}");
  gr->Puts(mglPoint(0,-0.2),"Easy to \\a{overline} or "
  "\\u{underline}");
  gr->Puts(mglPoint(0,-0.6),"Easy to change indexes ^{up} _{down} @{center}");
  gr->Puts(mglPoint(0,-1),"It parse TeX: \\int \\alpha \\cdot "
  "\\sqrt3{sin(\\pi x)^2 + \\gamma_{i_k}} dx");

  gr->SubPlot(2,2,1,"");
  gr->Puts(mglPoint(0,0.5), "\\sqrt{\\frac{\\alpha^{\\gamma^2}+\\overset 1{\\big\\infty}}{\\sqrt3{2+b}}}", "@", -4);
  gr->Puts(mglPoint(0,-0.5),"Text can be printed\non several lines");

  gr->SubPlot(2,2,2,"");
  mglData y;  mgls_prepare1d(&y);
  gr->Box();  gr->Plot(y.SubData(-1,0));
  gr->Text(y,"This is very very long string drawn along a curve",":k");
  gr->Text(y,"Another string drawn under a curve","T:r");

  gr->SubPlot(2,2,3,"");
  gr->Line(mglPoint(-1,-1),mglPoint(1,-1),"rA");
  gr->Puts(mglPoint(0,-1),mglPoint(1,-1),"Horizontal");
  gr->Line(mglPoint(-1,-1),mglPoint(1,1),"rA");
  gr->Puts(mglPoint(0,0),mglPoint(1,1),"At angle","@");
  gr->Line(mglPoint(-1,-1),mglPoint(-1,1),"rA");
  gr->Puts(mglPoint(-1,0),mglPoint(-1,1),"Vertical");
  return 0;
}

int sample11(mglGraph *gr)
{
  double h=1.1, d=0.25;
  gr->LoadFont("STIX");     gr->Puts(mglPoint(0,h), "default font (STIX)");
  gr->LoadFont("adventor"); gr->Puts(mglPoint(0,h-d), "adventor font");
  gr->LoadFont("bonum");    gr->Puts(mglPoint(0,h-2*d), "bonum font");
  gr->LoadFont("chorus");   gr->Puts(mglPoint(0,h-3*d), "chorus font");
  gr->LoadFont("cursor");   gr->Puts(mglPoint(0,h-4*d), "cursor font");
  gr->LoadFont("heros");    gr->Puts(mglPoint(0,h-5*d), "heros font");
  gr->LoadFont("heroscn");  gr->Puts(mglPoint(0,h-6*d), "heroscn font");
  gr->LoadFont("pagella");  gr->Puts(mglPoint(0,h-7*d), "pagella font");
  gr->LoadFont("schola");   gr->Puts(mglPoint(0,h-8*d), "schola font");
  gr->LoadFont("termes");   gr->Puts(mglPoint(0,h-9*d), "termes font");
  return 0;
}

int sample12(mglGraph *gr)
{
  gr->AddLegend("sin(\\pi {x^2})","b");
  gr->AddLegend("sin(\\pi x)","g*");
  gr->AddLegend("sin(\\pi \\sqrt{x})","rd");
  gr->AddLegend("just text"," ");
  gr->AddLegend("no indent for this","");

  gr->SubPlot(2,2,0,""); gr->Title("Legend (default)");
  gr->Box();  gr->Legend();

  gr->Legend(3,"A#");
  gr->Puts(mglPoint(0.75,0.65),"Absolute position","A");

  gr->SubPlot(2,2,2,"");  gr->Title("coloring");  gr->Box();
  gr->Legend(0,"r#"); gr->Legend(1,"Wb#");  gr->Legend(2,"ygr#");

  gr->SubPlot(2,2,3,"");  gr->Title("manual position"); gr->Box();
  gr->Legend(0.5,1);  gr->Puts(mglPoint(0.5,0.55),"at x=0.5, y=1","a");
  gr->Legend(1,"#-"); gr->Puts(mglPoint(0.75,0.25),"Horizontal legend","a");
  return 0;
}

void mgls_prepare3d(mglData *a, mglData *b=0)
{
  register long i,j,k,n=61,m=50,l=40,i0;
  if(a) a->Create(n,m,l);   if(b) b->Create(n,m,l);
  mreal x,y,z;
  for(i=0;i<n;i++)  for(j=0;j<m;j++)  for(k=0;k<l;k++)
  {
    x=2*i/(n-1.)-1; y=2*j/(m-1.)-1; z=2*k/(l-1.)-1; i0 = i+n*(j+m*k);
    if(a) a->a[i0] = -2*(x*x + y*y + z*z*z*z - z*z - 0.1);
    if(b) b->a[i0] = 1-2*tanh((x+y)*(x+y));
  }
}

int sample13(mglGraph *gr)
{
  mglData a,c,v(1); mgls_prepare2d(&a); mgls_prepare3d(&c); v.a[0]=0.5;
  gr->SubPlot(2,2,0); gr->Title("Cut on (default)");
  gr->Rotate(50,60);  gr->Light(true);
  gr->Box();  gr->Surf(a,"","zrange -1 0.5");

  gr->SubPlot(2,2,1); gr->Title("Cut off");   gr->Rotate(50,60);
  gr->Box();  gr->Surf(a,"","zrange -1 0.5; cut off");

  gr->SubPlot(2,2,2); gr->Title("Cut in box");  gr->Rotate(50,60);
  gr->SetCutBox(mglPoint(0,-1,-1), mglPoint(1,0,1.1));
  gr->Alpha(true);  gr->Box();  gr->Surf3(c);
  gr->SetCutBox(mglPoint(0), mglPoint(0));  // switch it off

  gr->SubPlot(2,2,3); gr->Title("Cut by formula");  gr->Rotate(50,60);
  gr->CutOff("(z>(x+0.5*y-1)^2-1) & (z>(x-0.5*y-1)^2-1)");
  gr->Box();  gr->Surf3(c); gr->CutOff(""); // switch it off
  return 0;
}

int main(int argc,char **argv)
{
  mglQT gr(sample13,"MathGL examples");
  return gr.Run();
}
