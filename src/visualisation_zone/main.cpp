#include <codac.h>

using namespace std;
using namespace codac;

int main()
{
  

  const Vector coord({2,1,M_PI/6});
  float size = 1;
  vibes::beginDrawing();
  VIBesFigTube fig("visualisation");
  vibes::drawVehicle(2, 1, M_PI/6,size, "black");
  Interval r(3.,4.);
  Interval theta(-M_PI/10.,M_PI/10.);
  fig.draw_pie(2, 1, r, theta, "blue[cyan]");
  fig.show();
  vibes::endDrawing();
}
