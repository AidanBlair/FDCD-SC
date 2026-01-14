function dist = Calculate_Distance_New(SenLoc1, SenLoc2)
    x1 = SenLoc1(1);
    y1 = SenLoc1(2);
    x2 = SenLoc2(1);
    y2 = SenLoc2(2);
    dist = sqrt((x1 - x2) ^ 2 + (y1 - y2) ^ 2);
end