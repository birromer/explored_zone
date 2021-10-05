#include <codac.h>

using namespace std;
using namespace codac;

int main()
{
  

  Vector coord({2,1,M_PI/6});
  vibes::beginDrawing();
  VIBesFigTube fig("visualisation");
  fig.draw_vehicle(coord,0.5);
  fig.show();
  vibes::endDrawing();
}
