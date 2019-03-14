function [C, r_squared] = CircleFit(P)
    %% CircleFit
    % Manuel Breitenstein, 14.03.2019
    %
    % Fit a circle for 2D points by solving a linear system of equations.
    %
    % INPUT:
    %       - P (N x 2): 2D Points used for fitting the circle onto. First
    %                    column contains X-coordinates, second columnt
    %                    y-coordinates.
    %
    % OUTPUT:
    %       - C (1 x 2): Center of Circle.
    %       - r_squared (1 x 1): Squared radius of Circle.
    %%
   X = P(:, 1);
   Y = P(:, 2);
    
   A = [ -2 * (X(1:end-1) - X(2:end)) ...
         -2 * (Y(1:end-1) - Y(2:end))];
     
   B = [ ( X(2:end).^2 - X(1:end-1).^2) + ( Y(2:end).^2 - Y(1:end-1).^2)];
   
   C = A\B;
   
   r_squared = sum((X-C(1)).^2 + (Y-C(2)).^2 , 1)/ (size(X,1));
   C = C.';
end