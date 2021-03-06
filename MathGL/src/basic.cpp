
/**
 * MATHGL EXAMPLES
 */

#include <mgl2/mgl.h>

/* simple example */
//
//int main()
//{
//  mglGraph gr;
//  gr.FPlot("sin(pi*x)");
//  gr.WriteFrame("test.png");
//}

/* Using MathGL window */
// Qt
//#include <mgl2/qt.h>
//int sample(mglGraph *gr)
//{
//  gr->Rotate(60,40);
//  gr->Box();
//  return 0;
//}
//
//int main(int argc,char **argv)
//{
//  mglQT gr(sample,"MathGL examples");
//  return gr.Run();
//}

// GLUT - not working
//#include <mgl2/glut.h>
//int sample(mglGraph *gr)
//{
//  gr->Rotate(60,40);
//  gr->Box();
//  return 0;
//}
//int main(int argc,char **argv)
//{
//  mglGLUT gr(sample,"MathGL examples");
//  return 0;
//}

/* Drawing to file */
//
//int sample(mglGraph *gr)
//{
//  gr->Rotate(60,40);
//  gr->Box();
//  return 0;
//}
//int main(int ,char **)
//{
//
//  mglGraph gr;
//  gr.Alpha(true);
//  gr.Light(true);
//  sample(&gr);              // The same drawing function.
//  //png
//  gr.WritePNG("test.png");  // Don't forget to save the result!
//  //eps
//  gr.WriteEPS("test.eps");  // Don't forget to save the result!
//
//  return 0;
//}

/* Animation */

int main(int ,char **)
{
  mglGraph gr;
  mglData dat(100);
  char str[32];

  // gif
  gr.StartGIF("sample.gif");
  for(int i=0;i<40;i++)
  {
    gr.NewFrame();     // start frame
    gr.Box();          // some plotting
    for(int j=0;j<dat.nx;j++)
      dat.a[j]=sin(M_PI*j/dat.nx+M_PI*0.05*i);
    gr.Plot(dat,"b");
    gr.EndFrame();     // end frame
  }
  gr.CloseGIF();

  // jpg
  for(int i=0;i<40;i++)
  {
    gr.NewFrame();     // start frame
    gr.Box();          // some plotting
    for(int j=0;j<dat.nx;j++)
      dat.a[j]=sin(M_PI*j/dat.nx+M_PI*0.05*i);
    gr.Plot(dat,"b");
    gr.EndFrame();     // end frame
    gr.WriteFrame();   // save frame
  }
  return 0;
}

/* Drawing in memory - not working */
//
//#include<mgl2/qt.h>
//void paintEvent(QPaintEvent *)
//{
//  mglGraph gr(w(),h());
//
//  gr.Alpha(true);         // draws something using MathGL
//  gr.Light(true);         gr.Light(0,mglPoint(1,0,-1));
//  sample(&gr,NULL);
//
//  // Qt don't support RGB format as is. So, let convert it to BGRN.
//  long w=gr.GetWidth(), h=gr.GetHeight();
//  unsigned char *buf = new uchar[4*w*h];
//  gr.GetBGRN(buf, 4*w*h)
//  QPixmap pic = QPixmap::fromImage(QImage(*buf, w, h, QImage::Format_RGB32));
//
//  QPainter paint;
//  paint.begin(this);  paint.drawPixmap(0,0,pic);  paint.end();
//  delete []buf;
//}
//
//int main() {
//
//	return 0;
//}


