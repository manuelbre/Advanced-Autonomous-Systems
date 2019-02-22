function [ u ] = control_input_p2_a(t)
    
    if (t >=10 && t <=20)
        u = 1;
    elseif (t >=21 && t <=30)
        u = 2;
    else
        u = 0;
    end
    
end