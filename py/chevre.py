from pyibex import *
from vibes import vibes


m1x=Interval(0,1)
m1y=Interval(2,10)
m2x=Interval(10,16)
m2y=Interval(0,1)
m = [ [m1x ,m1y  ],
      [m2x, m2y] ]
    
        
L=30    
      
      
f = Function("x","y","mx","my", "(x-mx)^2+(y-my)^2")
Sda=SepFwdBwd(f,[-oo,100])
S1a=SepProj(Sda,IntervalVector([m1x, m1y]))
S2a=SepProj(Sda,IntervalVector([m2x, m2y]))

Sdb=~SepFwdBwd(f,[-oo,100])
S1b=SepProj(Sdb,IntervalVector([m1x,m1y]))
S2b=SepProj(Sdb,IntervalVector([m2x,m2y]))
Sa=S1a|S2a
Sb=~S1b|~S2b
vibes.beginDrawing()
vibes.newFigure()
vibes.setFigureSize(1000,1000)
vibes.axisLimits(-L,L,-L,L)
params = {'color_in': 'black[orange]', 'color_out':'transparent[transparent]', 'color_maybe':'white[white]', 'use_patch': False}
pySIVIA([[-L,L],[-L,L]],Sa&~Sb,0.5,draw_boxes=True, save_result=False, **params)
params = {'color_in': 'red[magenta]', 'color_out':'transparent[transparent]', 'color_maybe':'white[white]', 'use_patch': False}
pySIVIA([[-L,L],[-L,L]],Sb,0.5,draw_boxes=True, save_result=False, **params)
params = {'color_in': 'transparent[transparent]', 'color_out':'black[blue]', 'color_maybe':'white[white]', 'use_patch': False}
pySIVIA([[-L,L],[-L,L]],Sa,0.5,draw_boxes=True, save_result=False, **params)

for m_ in m:
   vibes.drawBox(m_[0][0], m_[0][1], m_[1][0], m_[1][1], '[k]')
