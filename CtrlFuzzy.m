%Fuzzy controller
%Last modification: 05/02/2020
%Felipe Vasconcelos

function Kc  = CtrlFuzzy(Error,dError)

%% Initial paramenters

%membership functions to inputs
a = -0.06; b = -0.04; c = -0.02; d = 0; e = 0.02; f = 0.04; g = 0.06; 
a_ = -0.06; b_ = -0.04; c_ = -0.02; d_ = 0; e_ = 0.02; f_ = 0.04; g_ = 0.06; 
%a = -1; b = -0.6; c = -0.4; d = 0; e = 0.4; f = 0.6; g = 1; 
%a_ = -0.06; b_ = -0.04; c_ = -0.02; d_ = 0; e_ = 0.02; f_ = 0.04; g_ = 0.06; 
% a = -0.6; b = -0.4; c = -0.2; d = 0; e = 0.2; f = 0.4; g = 0.6; 
% a_ = -0.6; b_ = -0.4; c_ = -0.2; d_ = 0; e_ = 0.2; f_ = 0.4; g_ = 0.6; 
% a = -6; b = -4; c = -2; d = 0; e = 2; f = 4; g = 6; 
% a_ = -5; b_ = -2.5; c_ = -1.25; d_ = 0; e_ = 1.25; f_ = 2.5; g_ = 5; 

%membership functions to output control
BN = -1; MN = -0.4; LN = -0.2; ZE = 0; LP = 0.2; MP = 0.4; BP = 1;
% BN = -8; MN = -4; LN = -2; ZE = 0; LP = 2; MP = 4; BP = 8;

%% Rules

%pe = Error pertinence
%pde = Error derivative pertinence

num = 0; den = 0; %for defuzzification

low = 0; high = 1; %indicates the direction of the slope
% p1 = 0; p2 = 0; p3 = 0; p4 = 0; p5 = 0; 
% p6 = 0; p7 = 0; p8 = 0; p9 = 0; p10 = 0;

%% 1. If (Error is BN) and (dError is BN) then (control is BN) (1) 
if((Error <= b) && (dError <= b_)) 
    
    pe = mtrp(a,b,high,low,Error);

    pde = mtrp(a_,b_,high,low,dError);
    
    p1 = min(pe,pde);
    
    %Defuzzification
    num = num + p1*BN; den = den + p1;

end
%% 2. If (Error is BN) and (dError is MN) then (control is BN) (1) 
if((Error <= b) && ((dError >= a_) && (dError <= c_))) 
    
    pe = mtrp(a,b,high,low,Error);

    pde = mtri(a_,b_,c_,low,high,dError);
    
    p2 = min(pe,pde);
    
    %Defuzzification
    num = num + p2*BN; den = den + p2;

end
%% 3. If (Error is BN) and (dError is LN) then (control is BN) (1) 
if((Error <= b) && ((dError >= b_) && (dError <= d_))) 
    
    pe = mtrp(a,b,high,low,Error);

    pde = mtri(b_,c_,d_,low,high,dError);
    
    p3 = min(pe,pde);
    
    %Defuzzification
    num = num + p3*BN; den = den + p3;

end
%% 4. If (Error is BN) and (dError is ZE) then (control is BN) (1) 
if((Error <= b) && ((dError >= c_) && (dError <= e_))) 
    
    pe = mtrp(a,b,high,low,Error);

    pde = mtri(c_,d_,e_,low,high,dError);
    
    p4 = min(pe,pde);
    
    %Defuzzification
    num = num + p4*BN; den = den + p4;

end
%% 5. If (Error is BN) and (dError is LP) then (control is MN) (1) 
if((Error <= b) && ((dError >= d_) && (dError <= f_))) 
    
    pe = mtrp(a,b,high,low,Error);

    pde = mtri(d_,e_,f_,low,high,dError);
    
    p5 = min(pe,pde);
    
    %Defuzzification
    num = num + p5*MN; den = den + p5;

end
%% 6. If (Error is BN) and (dError is MP) then (control is LN) (1) 
if((Error <= b) && ((dError >= e_) && (dError <= g_))) 
    
    pe = mtrp(a,b,high,low,Error);

    pde = mtri(e_,f_,g_,low,high,dError);
    
    p6 = min(pe,pde);
    
    %Defuzzification
    num = num + p6*LN; den = den + p6;

end
%% 7. If (Error is BN) and (dError is BP) then (control is ZE) (1) 
if((Error <= b) && (dError >= f_)) 
    
    pe = mtrp(a,b,high,low,Error);

    pde = mtrp(f_,g_,low,high,dError);
    
    p7 = min(pe,pde);
    
    %Defuzzification
    num = num + p7*ZE; den = den + p7;

end
%% 8. If (Error is MN) and (dError is BN) then (control is BN) (1) 
if(((Error >= a) && (Error <= c)) && (dError <= b_)) 
    
    pe = mtri(a,b,c,low,high,Error);

    pde = mtrp(a_,b_,high,low,dError);
    
    p8 = min(pe,pde);
    
    %Defuzzification
    num = num + p8*BN; den = den + p8;

end
%% 9. If (Error is MN) and (dError is MN) then (control is BN) (1) 
if(((Error >= a) && (Error <= c)) && ((dError >= a_) && (dError <= c_))) 
    
    pe = mtri(a,b,c,low,high,Error);

    pde = mtri(a_,b_,c_,low,high,dError);
    
    p9 = min(pe,pde);
    
    %Defuzzification
    num = num + p9*BN; den = den + p9;

end
%% 10. If (Error is MN) and (dError is LN) then (control is BN) (1)
if(((Error >= a) && (Error <= c)) && ((dError >= b_) && (dError <= d_))) 
    
    pe = mtri(a,b,c,low,high,Error);

    pde = mtri(b_,c_,d_,low,high,dError);
    
    p10 = min(pe,pde);
    
    %Defuzzification
    num = num + p10*BN; den = den + p10;

end
%% 11. If (Error is MN) and (dError is ZE) then (control is MN) (1)
if(((Error >= a) && (Error <= c)) && ((dError >= c_) && (dError <= e_))) 
    
    pe = mtri(a,b,c,low,high,Error);

    pde = mtri(c_,d_,e_,low,high,dError);
    
    p11 = min(pe,pde);
    
    %Defuzzification
    num = num + p11*MN; den = den + p11;

end
%% 12. If (Error is MN) and (dError is LP) then (control is LN) (1)
if(((Error >= a) && (Error <= c)) && ((dError >= d_) && (dError <= f_))) 
    
    pe = mtri(a,b,c,low,high,Error);

    pde = mtri(d_,e_,f_,low,high,dError);
    
    p12 = min(pe,pde);
    
    %Defuzzification
    num = num + p12*LN; den = den + p12;

end
%% 13. If (Error is MN) and (dError is MP) then (control is ZE) (1)
if(((Error >= a) && (Error <= c)) && ((dError >= e_) && (dError <= g_))) 
    
    pe = mtri(a,b,c,low,high,Error);

    pde = mtri(e_,f_,g_,low,high,dError);
    
    p13 = min(pe,pde);
    
    %Defuzzification
    num = num + p13*ZE; den = den + p13;

end
%% 14. If (Error is MN) and (dError is BP) then (control is LP) (1)
if(((Error >= a) && (Error <= c)) && (dError >= f_)) 
    
    pe = mtri(a,b,c,low,high,Error);

    pde = mtrp(f_,g_,low,high,dError);
    
    p14 = min(pe,pde);
    
    %Defuzzification
    num = num + p14*LP; den = den + p14;

end
%% 15. If (Error is LN) and (dError is BN) then (control is BN) (1)
if(((Error >= b) && (Error <= d)) && (dError <= b_)) 
    
    pe = mtri(b,c,d,low,high,Error);

    pde = mtrp(a_,b_,high,low,dError);
    
    p15 = min(pe,pde);
    
    %Defuzzification
    num = num + p15*BN; den = den + p15;

end
%% 16. If (Error is LN) and (dError is MN) then (control is BN) (1)
if(((Error >= b) && (Error <= d)) && ((dError >= a_) && (dError <= c_))) 
    
    pe = mtri(b,c,d,low,high,Error);

    pde = mtri(a_,b_,c_,low,high,dError);
    
    p16 = min(pe,pde);
    
    %Defuzzification
    num = num + p16*BN; den = den + p16;

end
%% 17. If (Error is LN) and (dError is LN) then (control is MN) (1)
if(((Error >= b) && (Error <= d)) && ((dError >= b_) && (dError <= d_))) 
    
    pe = mtri(b,c,d,low,high,Error);

    pde = mtri(b_,c_,d_,low,high,dError);
    
    p17 = min(pe,pde);
    
    %Defuzzification
    num = num + p17*MN; den = den + p17;

end
%% 18. If (Error is LN) and (dError is ZE) then (control is LN) (1)
if(((Error >= b) && (Error <= d)) && ((dError >= c_) && (dError <= e_))) 
    
    pe = mtri(b,c,d,low,high,Error);

    pde = mtri(c_,d_,e_,low,high,dError);
    
    p18 = min(pe,pde);
    
    %Defuzzification
    num = num + p18*LN; den = den + p18;

end
%% 19. If (Error is LN) and (dError is LP) then (control is ZE) (1)
if(((Error >= b) && (Error <= d)) && ((dError >= d_) && (dError <= f_))) 
    
    pe = mtri(b,c,d,low,high,Error);

    pde = mtri(d_,e_,f_,low,high,dError);
    
    p19 = min(pe,pde);
    
    %Defuzzification
    num = num + p19*ZE; den = den + p19;

end
%% 20. If (Error is LN) and (dError is MP) then (control is LP) (1)
if(((Error >= b) && (Error <= d)) && ((dError >= e_) && (dError <= g_))) 
    
    pe = mtri(b,c,d,low,high,Error);

    pde = mtri(e_,f_,g_,low,high,dError);
    
    p20 = min(pe,pde);
    
    %Defuzzification
    num = num + p20*LP; den = den + p20;

end
%% 21. If (Error is LN) and (dError is BP) then (control is MP) (1)
if(((Error >= b) && (Error <= d)) && (dError >= f_)) 
    
    pe = mtri(b,c,d,low,high,Error);

    pde = mtrp(f_,g_,low,high,dError);
    
    p21 = min(pe,pde);
    
    %Defuzzification
    num = num + p21*MP; den = den + p21;

end
%% 22. If (Error is ZE) and (dError is BN) then (control is BN) (1)
if(((Error >= c) && (Error <= e)) && (dError <= b_)) 
    
    pe = mtri(c,d,e,low,high,Error);

    pde = mtrp(a_,b_,high,low,dError);
    
    p22 = min(pe,pde);
    
    %Defuzzification
    num = num + p22*BN; den = den + p22;

end
%% 23. If (Error is ZE) and (dError is MN) then (control is MN) (1)
if(((Error >= c) && (Error <= e)) && ((dError >= a_) && (dError <= c_))) 
    
    pe = mtri(c,d,e,low,high,Error);

    pde = mtri(a_,b_,c_,low,high,dError);
    
    p23 = min(pe,pde);
    
    %Defuzzification
    num = num + p23*MN; den = den + p23;

end
%% 24. If (Error is ZE) and (dError is LN) then (control is LN) (1)
if(((Error >= c) && (Error <= e)) && ((dError >= b_) && (dError <= d_))) 
    
    pe = mtri(c,d,e,low,high,Error);

    pde = mtri(b_,c_,d_,low,high,dError);
    
    p24 = min(pe,pde);
    
    %Defuzzification
    num = num + p24*LN; den = den + p24;

end
%% 25. If (Error is ZE) and (dError is ZE) then (control is ZE) (1)
if(((Error >= c) && (Error <= e)) && ((dError >= c_) && (dError <= e_))) 
    
    pe = mtri(c,d,e,low,high,Error);

    pde = mtri(c_,d_,e_,low,high,dError);
    
    p25 = min(pe,pde);
    
    %Defuzzification
    num = num + p25*ZE; den = den + p25;

end
%% 26. If (Error is ZE) and (dError is LP) then (control is LP) (1)
if(((Error >= c) && (Error <= e)) && ((dError >= d_) && (dError <= f_))) 
    
    pe = mtri(c,d,e,low,high,Error);

    pde = mtri(d_,e_,f_,low,high,dError);
    
    p26 = min(pe,pde);
    
    %Defuzzification
    num = num + p26*LP; den = den + p26;

end
%% 27. If (Error is ZE) and (dError is MP) then (control is MP) (1)
if(((Error >= c) && (Error <= e)) && ((dError >= e_) && (dError <= g_))) 
    
    pe = mtri(c,d,e,low,high,Error);

    pde = mtri(e_,f_,g_,low,high,dError);
    
    p27 = min(pe,pde);
    
    %Defuzzification
    num = num + p27*MP; den = den + p27;

end
%% 28. If (Error is ZE) and (dError is BP) then (control is BP) (1)
if(((Error >= c) && (Error <= e)) && (dError >= f_)) 
    
    pe = mtri(c,d,e,low,high,Error);

    pde = mtrp(f_,g_,low,high,dError);
    
    p28 = min(pe,pde);
    
    %Defuzzification
    num = num + p28*BP; den = den + p28;

end
%% 29. If (Error is LP) and (dError is BN) then (control is MN) (1)
if(((Error >= d) && (Error <= f)) && (dError <= b_)) 
    
    pe = mtri(d,e,f,low,high,Error);

    pde = mtrp(a_,b_,high,low,dError);
    
    p29 = min(pe,pde);
    
    %Defuzzification
    num = num + p29*MN; den = den + p29;

end
%% 30. If (Error is LP) and (dError is MN) then (control is LN) (1)
if(((Error >= d) && (Error <= f)) && ((dError >= a_) && (dError <= c_))) 
    
    pe = mtri(d,e,f,low,high,Error);

    pde = mtri(a_,b_,c_,low,high,dError);
    
    p30 = min(pe,pde);
    
    %Defuzzification
    num = num + p30*LN; den = den + p30;

end
%% 31. If (Error is LP) and (dError is LN) then (control is ZE) (1)
if(((Error >= d) && (Error <= f)) && ((dError >= b_) && (dError <= d_))) 
    
    pe = mtri(d,e,f,low,high,Error);

    pde = mtri(b_,c_,d_,low,high,dError);
    
    p31 = min(pe,pde);
    
    %Defuzzification
    num = num + p31*ZE; den = den + p31;

end
%% 32. If (Error is LP) and (dError is ZE) then (control is LP) (1)
if(((Error >= d) && (Error <= f)) && ((dError >= c_) && (dError <= e_))) 
    
    pe = mtri(d,e,f,low,high,Error);

    pde = mtri(c_,d_,e_,low,high,dError);
    
    p32 = min(pe,pde);
    
    %Defuzzification
    num = num + p32*LP; den = den + p32;

end
%% 33. If (Error is LP) and (dError is LP) then (control is MP) (1)
if(((Error >= d) && (Error <= f)) && ((dError >= d_) && (dError <= f_))) 
    
    pe = mtri(d,e,f,low,high,Error);

    pde = mtri(d_,e_,f_,low,high,dError);
    
    p33 = min(pe,pde);
    
    %Defuzzification
    num = num + p33*MP; den = den + p33;

end
%% 34. If (Error is LP) and (dError is MP) then (control is BP) (1)
if(((Error >= d) && (Error <= f)) && ((dError >= e_) && (dError <= g_))) 
    
    pe = mtri(d,e,f,low,high,Error);

    pde = mtri(e_,f_,g_,low,high,dError);
    
    p34 = min(pe,pde);
    
    %Defuzzification
    num = num + p34*BP; den = den + p34;

end
%% 35. If (Error is LP) and (dError is BP) then (control is BP) (1)
if(((Error >= d) && (Error <= f)) && (dError >= f_)) 
    
    pe = mtri(d,e,f,low,high,Error);

    pde = mtrp(f_,g_,low,high,dError);
    
    p35 = min(pe,pde);
    
    %Defuzzification
    num = num + p35*BP; den = den + p35;

end
%% 36. If (Error is MP) and (dError is BN) then (control is LN) (1)
if(((Error >= e) && (Error <= g)) && (dError <= b_)) 
    
    pe = mtri(e,f,g,low,high,Error);

    pde = mtrp(a_,b_,high,low,dError);
    
    p36 = min(pe,pde);
    
    %Defuzzification
    num = num + p36*LN; den = den + p36;

end
%% 37. If (Error is MP) and (dError is MN) then (control is ZE) (1)
if(((Error >= e) && (Error <= g)) && ((dError >= a_) && (dError <= c_))) 
    
    pe = mtri(e,f,g,low,high,Error);

    pde = mtri(a_,b_,c_,low,high,dError);
    
    p37 = min(pe,pde);
    
    %Defuzzification
    num = num + p37*ZE; den = den + p37;

end
%% 38. If (Error is MP) and (dError is LN) then (control is LP) (1)
if(((Error >= e) && (Error <= g)) && ((dError >= b_) && (dError <= d_))) 
    
    pe = mtri(e,f,g,low,high,Error);

    pde = mtri(b_,c_,d_,low,high,dError);
    
    p38 = min(pe,pde);
    
    %Defuzzification
    num = num + p38*LP; den = den + p38;

end
%% 39. If (Error is MP) and (dError is ZE) then (control is MP) (1)
if(((Error >= e) && (Error <= g)) && ((dError >= c_) && (dError <= e_))) 
    
    pe = mtri(e,f,g,low,high,Error);

    pde = mtri(c_,d_,e_,low,high,dError);
    
    p39 = min(pe,pde);
    
    %Defuzzification
    num = num + p39*MP; den = den + p39;

end
%% 40. If (Error is MP) and (dError is LP) then (control is BP) (1)
if(((Error >= e) && (Error <= g)) && ((dError >= d_) && (dError <= f_))) 
    
    pe = mtri(e,f,g,low,high,Error);

    pde = mtri(d_,e_,f_,low,high,dError);
    
    p40 = min(pe,pde);
    
    %Defuzzification
    num = num + p40*BP; den = den + p40;

end
%% 41. If (Error is MP) and (dError is MP) then (control is BP) (1)
if(((Error >= e) && (Error <= g)) && ((dError >= e_) && (dError <= g_))) 
    
    pe = mtri(e,f,g,low,high,Error);

    pde = mtri(e_,f_,g_,low,high,dError);
    
    p41 = min(pe,pde);
    
    %Defuzzification
    num = num + p41*BP; den = den + p41;

end
%% 42. If (Error is MP) and (dError is BP) then (control is BP) (1)
if(((Error >= e) && (Error <= g)) && (dError >= f_)) 
    
    pe = mtri(e,f,g,low,high,Error);

    pde = mtrp(f_,g_,low,high,dError);
    
    p42 = min(pe,pde);
    
    %Defuzzification
    num = num + p42*BP; den = den + p42;

end
%% 43. If (Error is BP) and (dError is BN) then (control is ZE) (1)
if((Error >= f) && (dError <= b_)) 
    
    pe = mtrp(f,g,low,high,Error);

    pde = mtrp(a_,b_,high,low,dError);
    
    p43 = min(pe,pde);
    
    %Defuzzification
    num = num + p43*ZE; den = den + p43;

end
%% 44. If (Error is BP) and (dError is MN) then (control is LP) (1)
if((Error >= f) && ((dError >= a_) && (dError <= c_))) 
    
    pe = mtrp(f,g,low,high,Error);

    pde = mtri(a_,b_,c_,low,high,dError);
    
    p44 = min(pe,pde);
    
    %Defuzzification
    num = num + p44*LP; den = den + p44;

end
%% 45. If (Error is BP) and (dError is LN) then (control is MP) (1)
if((Error >= f) && ((dError >= b_) && (dError <= d_))) 
    
    pe = mtrp(f,g,low,high,Error);

    pde = mtri(b_,c_,d_,low,high,dError);
    
    p45 = min(pe,pde);
    
    %Defuzzification
    num = num + p45*MP; den = den + p45;

end
%% 46. If (Error is BP) and (dError is ZE) then (control is BP) (1)
if((Error >= f) && ((dError >= c_) && (dError <= e_))) 
    
    pe = mtrp(f,g,low,high,Error);

    pde = mtri(c_,d_,e_,low,high,dError);
    
    p46 = min(pe,pde);
    
    %Defuzzification
    num = num + p46*BP; den = den + p46;

end
%% 47. If (Error is BP) and (dError is LP) then (control is BP) (1)
if((Error >= f) && ((dError >= d_) && (dError <= f_))) 
    
    pe = mtrp(f,g,low,high,Error);

    pde = mtri(d_,e_,f_,low,high,dError);
    
    p47 = min(pe,pde);
    
    %Defuzzification
    num = num + p47*BP; den = den + p47;

end
%% 48. If (Error is BP) and (dError is MP) then (control is BP) (1)
if((Error >= f) && ((dError >= e_) && (dError <= g_))) 
    
    pe = mtrp(f,g,low,high,Error);

    pde = mtri(e_,f_,g_,low,high,dError);
    
    p48 = min(pe,pde);
    
    %Defuzzification
    num = num + p48*BP; den = den + p48;

end
%% 49. If (Error is BP) and (dError is BP) then (control is BP) (1)
if((Error >= f) && (dError >= f_)) 
    
    pe = mtrp(f,g,low,high,Error);

    pde = mtrp(f_,g_,low,high,dError);
    
    p49 = min(pe,pde);
    
    %Defuzzification
    num = num + p49*BP; den = den + p49;

end

%% Control output
Kc = (num/den);
%end

function ptrp = mtrp(a,b,h1,h2,x)  %mtrp = membership trapezoidal    
    if a == b
            ptrp = 1;
    else
        if h1<h2  %raising line 
            if(x<b)
                m = (h2-h1)/(b-a);
                ptrp = abs(m)*(x-a)+0;
            else %x>=b
                ptrp = 1;
            end
        else% y1>y2 falling line
            if(x<=a)
                ptrp = 1;
            else %(x>a)  
                m = (h1-h2)/(b-a); 
                ptrp = abs(m)*(b-x)+0;        
            end   
        end
    end
%end

function ptri = mtri(a,b,c,h1,h2,x)  %mtrp = membership tringular
    ptri =0;
    if(x==b)
        ptri = 1;
    elseif(x<b) %raising line 
        m = (h2-h1)/(b-a);
        ptri = abs(m)*(x-a)+0;
    elseif(x>b) %falling line
        m = (h1-h2)/(c-b);
        ptri = abs(m)*(c-x)+0;
    end
%end
