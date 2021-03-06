<Qucs Schematic 0.0.17>
<Properties>
  <View=0,0,800,800,1,0,0>
  <Grid=10,10,1>
  <DataSet=ir-compensation.dat>
  <DataDisplay=ir-compensation.dpl>
  <OpenDisplay=1>
  <Script=ir-compensation.m>
  <RunScript=0>
  <showFrame=0>
  <FrameText0=Title>
  <FrameText1=Drawn By:>
  <FrameText2=Date:>
  <FrameText3=Revision:>
</Properties>
<Symbol>
  <.PortSym 40 20 1 0>
</Symbol>
<Components>
  <R R1 1 160 350 -23 -58 0 0 "1 kOhm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <GND * 1 260 480 0 0 0 0>
  <_BJT BC547 1 260 350 -30 -59 0 0 "npn" 0 "1.8e-14" 0 "0.9955" 0 "1.005" 0 "0.14" 0 "0.03" 0 "80" 0 "12.5" 0 "5e-14" 0 "1.46" 0 "1.72e-13" 0 "1.27" 0 "400" 0 "35.5" 0 "0" 0 "0" 0 "0.25" 0 "0.6" 0 "0.56" 0 "1.3e-11" 0 "0.75" 0 "0.33" 0 "4e-12" 0 "0.54" 0 "0.33" 0 "1" 0 "0" 0 "0.75" 0 "0" 0 "0.5" 0 "6.4e-10" 0 "0" 0 "0" 0 "0" 0 "5.072e-08" 0 "26.85" 0 "0" 0 "1" 0 "1" 0 "0" 0 "1" 0 "1" 0 "0" 0 "0" 0 "3" 0 "1.11" 0 "26.85" 0 "1" 0>
  <R R2 1 260 440 -110 -20 0 1 "1 kOhm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <R R3 1 360 500 -77 18 0 1 "50 Ohm" 1 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <GND * 1 380 550 0 0 0 0>
  <_BJT 2N3055 1 360 380 8 -26 0 0 "npn" 0 "4.66e-12" 0 "1" 0 "1" 0 "0.25" 0 "0" 0 "100" 0 "0" 0 "3.339e-11" 0 "1.5" 0 "5e-09" 0 "2" 0 "360" 0 "2" 0 "0.4" 0 "0.001" 0 "0.04" 0 "0" 0 "3" 0 "5.802e-10" 0 "1.2" 0 "0.45" 0 "2.121e-10" 0 "0.75" 0 "0.4" 0 "1" 0 "0" 0 "0.75" 0 "0" 0 "0.5" 0 "8e-08" 0 "1" 0 "0" 0 "3" 0 "2.55e-06" 0 "26.85" 0 "0" 0 "1" 0 "1" 0 "0" 0 "1" 0 "1" 0 "120" 0 "1" 0 "3" 0 "1.11" 0 "26.85" 0 "1" 0>
  <Diode D1 1 400 500 15 -26 0 1 "1e-15 A" 0 "1" 0 "10 fF" 0 "0.5" 0 "0.7 V" 0 "0.5" 0 "0.0 fF" 0 "0.0" 0 "2.0" 0 "0.0 Ohm" 0 "0.0 ps" 0 "0" 0 "0.0" 0 "1.0" 0 "1.0" 0 "0" 0 "1 mA" 0 "26.85" 0 "3.0" 0 "1.11" 0 "0.0" 0 "0.0" 0 "0.0" 0 "0.0" 0 "0.0" 0 "0.0" 0 "26.85" 0 "1.0" 0 "normal" 0>
  <Diode D2 1 360 160 19 13 0 3 "1e-15 A" 0 "1" 0 "10 fF" 0 "0.5" 0 "0.7 V" 0 "0.5" 0 "0.0 fF" 0 "0.0" 0 "2.0" 0 "0.0 Ohm" 0 "0.0 ps" 0 "0" 0 "0.0" 0 "1.0" 0 "1.0" 0 "0" 0 "1 mA" 0 "26.85" 0 "3.0" 0 "1.11" 0 "0.0" 0 "0.0" 0 "0.0" 0 "0.0" 0 "0.0" 0 "0.0" 0 "26.85" 0 "1.0" 0 "normal" 0>
  <L L1 1 330 140 -43 -7 0 3 "1 nH" 0 "" 0>
  <R R4 1 330 200 -45 -11 0 3 "1 Ohm" 0 "26.85" 0 "0.0" 0 "0.0" 0 "26.85" 0 "european" 0>
  <Port PWM_IN 1 130 350 -67 15 0 0 "1" 1 "analog" 0>
  <Vdc V1 1 510 140 18 -26 0 1 "12 V" 1>
  <C C1 1 410 140 17 -26 0 1 "22 uF" 1 "" 0 "neutral" 0>
  <GND * 1 460 190 0 0 0 0>
</Components>
<Wires>
  <260 380 260 410 "" 0 0 0 "">
  <260 320 360 320 "" 0 0 0 "">
  <190 350 230 350 "" 0 0 0 "">
  <360 470 400 470 "" 0 0 0 "">
  <360 530 380 530 "" 0 0 0 "">
  <380 530 400 530 "" 0 0 0 "">
  <380 530 380 550 "" 0 0 0 "">
  <360 410 360 470 "" 0 0 0 "">
  <360 320 360 350 "" 0 0 0 "">
  <260 380 330 380 "" 0 0 0 "">
  <360 230 360 320 "" 0 0 0 "">
  <330 110 360 110 "" 0 0 0 "">
  <360 110 410 110 "" 0 0 0 "">
  <360 190 360 230 "" 0 0 0 "">
  <360 110 360 130 "" 0 0 0 "">
  <330 230 360 230 "" 0 0 0 "">
  <410 170 460 170 "" 0 0 0 "">
  <410 110 510 110 "" 0 0 0 "">
  <460 170 510 170 "" 0 0 0 "">
  <460 170 460 190 "" 0 0 0 "">
  <260 470 260 480 "" 0 0 0 "">
</Wires>
<Diagrams>
</Diagrams>
<Paintings>
</Paintings>
