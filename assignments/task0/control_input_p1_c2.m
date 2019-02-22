function [ u ] = control_input_p1_c2(t)
    
    if (t >=1 && t <=2)
        u = 10;
    elseif (t >=3 && t <=4)
        u = 20;
    else
        u = 0;
    end
    
end

