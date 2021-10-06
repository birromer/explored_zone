import pyibex as pib
from vibes import vibes

m1x = pib.Interval(0,1)
m1y = pib.Interval(2,10)
m2x = pib.Interval(10,16)
m2y = pib.Interval(0,1)
m = [[m1x, m1y],
     [m2x, m2y]]

L = 30

f = pib.Function("x", "y", "mx", "my", "(x-mx)^2+(y-my)^2")

Sda = pib.SepFwdBwd(f, [-pib.oo,100])
S1a = pib.SepProj(Sda, pib.IntervalVector([m1x, m1y]))
S2a = pib.SepProj(Sda, pib.IntervalVector([m2x, m2y]))

Sdb = ~pib.SepFwdBwd(f, [-pib.oo,100])
S1b = pib.SepProj(Sdb, pib.IntervalVector([m1x, m1y]))
S2b = pib.SepProj(Sdb, pib.IntervalVector([m2x, m2y]))

Sa = S1a | S2a
Sb = ~S1b | ~S2b

vibes.beginDrawing()
vibes.newFigure()
vibes.setFigureSize(1000,1000)
vibes.axisLimits(-L,L,-L,L)

params = {'color_in': 'black[orange]', 'color_out':'transparent[transparent]', 'color_maybe':'white[white]', 'use_patch': False}
pib.pySIVIA([[-L,L],[-L,L]], Sa&~Sb, 0.5, draw_boxes=True, save_result=False, **params)

params = {'color_in': 'red[magenta]', 'color_out':'transparent[transparent]', 'color_maybe':'white[white]', 'use_patch': False}
pib.pySIVIA([[-L,L],[-L,L]], Sb, 0.5, draw_boxes=True, save_result=False, **params)

params = {'color_in': 'transparent[transparent]', 'color_out':'black[blue]', 'color_maybe':'white[white]', 'use_patch': False}
pib.pySIVIA([[-L,L],[-L,L]], Sa, 0.5, draw_boxes=True, save_result=False, **params)

for m_ in m:
    vibes.drawBox(m_[0][0], m_[0][1], m_[1][0], m_[1][1], '[k]')
