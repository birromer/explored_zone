#include <codac.h>

using namespace std;
using namespace codac;

int main()
{
  

  const Vector coord({2,1,M_PI/6});
  float size = 1;
float heading = 270;
  vibes::beginDrawing();
  VIBesFigTube fig("visualisation");
  vibes::drawVehicle(2, 1, heading,size, "black");
  Interval r(3.,4.);
  Interval theta(-M_PI/10.,M_PI/10.);
  //fig.draw_pie(2, 1, r, theta, "blue[cyan]");
  double r_min = 3, r_max=4;
  double th_min = (-35 + heading);
  double th_max = (35 + heading) ;
  vibes::drawPie(2, 1, r_min, r_max, th_min, th_max, "blue[blue]", vibesParams("figure", "visualisation"));
  fig.show();
  vibes::endDrawing();
}
