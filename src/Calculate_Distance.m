function dist = Calculate_Distance(SenLoc1, SenLoc2, u1, u2)
    step_size = 50;
    x1 = SenLoc1(1);
    y1 = SenLoc1(2);
    x2 = SenLoc2(1);
    y2 = SenLoc2(2);
    directions = 4;
    if u1 >= 4
        %x1 = x1 + step_size * real(1i ^ (8 - u1));
        x1 = x1 + step_size * cos((u1-4)*(2*pi / directions));
        %y1 = y1 + step_size * real(1i ^ (9 - u1));
        y1 = y1 + step_size * sin((u1-4)*(2*pi / directions));
    end
    if u2 >= 4
        %x2 = x2 + step_size * real(1i ^ (8 - u2));
        x2 = x2 + step_size * cos((u2-4)*(2*pi / directions));
        %y2 = y2 + step_size * real(1i ^ (9 - u2));
        x2 = x2 + step_size * cos((u2-4)*(2*pi / directions));
    end
    dist = sqrt((x1 - x2) ^ 2 + (y1 - y2) ^ 2);
end