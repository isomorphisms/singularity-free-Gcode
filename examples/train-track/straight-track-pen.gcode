(ILLUSTRATIVE ONLY — abstract pen plotter, not machine-ready)
(@object track.A)
(G21 millimetres; G90 absolute coordinates; G94 feed per minute)
(Z5 means pen raised and Z0 means pen touching only for this abstract device)

G21
G90
G94
G0 Z5.000

(@source track.A.rail.left)
G0 X20.000 Y60.000
G1 Z0.000 F300
G1 X180.000 Y60.000 F600
G0 Z5.000

(@source track.A.rail.right)
G0 X180.000 Y40.000
G1 Z0.000 F300
G1 X20.000 Y40.000 F600
G0 Z5.000

(@source track.A.sleeper.01)
G0 X25.000 Y35.000
G1 Z0.000 F300
G1 X25.000 Y65.000 F600
G0 Z5.000

(@source track.A.sleeper.02)
G0 X45.000 Y65.000
G1 Z0.000 F300
G1 X45.000 Y35.000 F600
G0 Z5.000

(@source track.A.sleeper.03)
G0 X65.000 Y35.000
G1 Z0.000 F300
G1 X65.000 Y65.000 F600
G0 Z5.000

(@source track.A.sleeper.04)
G0 X85.000 Y65.000
G1 Z0.000 F300
G1 X85.000 Y35.000 F600
G0 Z5.000

(@source track.A.sleeper.05)
G0 X105.000 Y35.000
G1 Z0.000 F300
G1 X105.000 Y65.000 F600
G0 Z5.000

(@source track.A.sleeper.06)
G0 X125.000 Y65.000
G1 Z0.000 F300
G1 X125.000 Y35.000 F600
G0 Z5.000

(@source track.A.sleeper.07)
G0 X145.000 Y35.000
G1 Z0.000 F300
G1 X145.000 Y65.000 F600
G0 Z5.000

(@source track.A.sleeper.08)
G0 X165.000 Y65.000
G1 Z0.000 F300
G1 X165.000 Y35.000 F600
G0 Z5.000

(@endobject track.A)
G0 X0.000 Y0.000
M2
