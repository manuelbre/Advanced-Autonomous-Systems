function [C, r_squared] = CircleFit(P)
   X = P(:, 1);
   Y = P(:, 2);
    
   A = [ -2 * (X(1:end-1) - X(2:end)) ...
         -2 * (Y(1:end-1) - Y(2:end))];
     
   B = [ ( X(2:end).^2 - X(1:end-1).^2) + ( Y(2:end).^2 - Y(1:end-1).^2)];
   
   C = (pinv(A) * B);
   
   r_squared = sum((X-C(1)).^2 + (Y-C(2)).^2 , 1)/ (size(X,1));
   C = C.';
end