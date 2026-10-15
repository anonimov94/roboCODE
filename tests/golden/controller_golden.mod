MODULE MOD_Golden
  ! UWAGA: dane obciazenia narzedzia (1 kg, CoG Z50) sa domyslne - wpisz rzeczywiste
  ! (masa/srodek ciezkosci/bezwladnosc wrzeciona, np. z LoadIdentify) przed praca produkcyjna
  PERS tooldata rcTool := [TRUE,[[0,0,0],[1,0,0,0]],[1,[0,0,50],[1,0,0,0],0,0,0]];
  TASK PERS wobjdata rcWobj := [FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
  PROC Golden()
    ConfJ \Off; ConfL \Off;
    rcWobj.uframe := [[100.000,200.000,300.000],[0.707107,0.000000,0.000000,0.707107]]; ! Fixture
    rcTool.tframe := [[0.000,0.000,150.000],[1.000000,0.000000,0.000000,0.000000]]; ! Spindle
    ! speed 50.0 mm/s
    ! joint speed 30 deg/s (MoveAbsJ is limited by speeddata)
    MoveAbsJ [[0.0000,10.0000,-20.0000,30.0000,40.0000,-50.0000],[9E9,9E9,9E9,9E9,9E9,9E9]],[50.0,500,5000,1000],fine,rcTool;
    MoveL [[10.000,20.000,30.000],[0.923880,0.000000,0.000000,0.382683],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],[50.0,500,5000,1000],fine,rcTool\WObj:=rcWobj;
    MoveC [[15.000,25.000,35.000],[1.000000,0.000000,0.000000,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],[[20.000,30.000,40.000],[1.000000,0.000000,0.000000,0.000000],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]],[50.0,500,5000,1000],fine,rcTool\WObj:=rcWobj;
    TPWrite "Kontrola";
    SetDO do1,1;
    WaitDI di1,1\MaxTime:=2.5;
    WaitTime 0.500;
    CycleDone;
  ENDPROC
ENDMODULE
