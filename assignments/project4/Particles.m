classdef Particles < handle
   %% DESCRIPTION
   %
   % Class for particle swarm optimization. The swarm consists of multiple
   % particles, each has an assigned velocity and position.
   % This function assumes at the moment only a one decision variable but
   % can be extended to be suitable for multiple decision variables.
   %
   % Particle swarms can be used to find the best hyperparameter for a
   % given problem statement.
   
   %%
   properties
      velocities
      vel_range
      positions
      pos_range
      fitness_curr
      personalBests
      globalBest
      c_p
      c_g
      n
   end
   
   methods
      function obj = Particles(n, c_p, c_g, pos_range, vel_range)
          % INPUT: 
          %       - n (1 x 1): Number of particles to creat.
          %       - c_p (1 x 1): Personal best update coefficient.
          %       - c_g (1 x 1): Global best update coefficient.
          %       - pos_range (2 x 1): Range for particle positon
          %       - vel_range (2 x 1): Range for particle velocity
          
          assert(length(pos_range) == 2);
          assert(length(vel_range) == 2);
          
          obj.positions = min(pos_range) + abs(pos_range(1) - ...
                          pos_range(2)) * rand(n, 1);
          obj.velocities = min(vel_range) + abs(vel_range(1) - ...
                          vel_range(2)) * rand(n, 1);
                      
          obj.personalBests.val = NaN(n,1); % Fitness Value
          obj.personalBests.pos = obj.positions; % Position

          
          obj.globalBest.val = NaN; % Fitness Value
          obj.globalBest.idx = NaN; % Index
          obj.globalBest.pos = NaN; % Position

          
          obj.vel_range = sort(vel_range);
          obj.pos_range = sort(pos_range);
          
          obj.n = n;
          obj.c_p = c_p;
          obj.c_g = c_g;
          obj.fitness_curr = NaN(n, 1);

      end
      
      function [pos] = getPosition(obj, idx)
          pos = obj.positions(idx);
      end
      
      function setFitness(obj, idx, fitness)
        obj.fitness_curr(idx) = fitness;
      end
      
      function [] = updatePersonalBests(obj)
         assert(all(~isnan(obj.fitness_curr)), ...
           'Curent fitness contains NaN. Must assign fitness to all particles before evaluating new current bests')
         
         new_better = isnan(obj.personalBests.val) | ...
                                obj.personalBests.val < obj.fitness_curr;
         obj.personalBests.val(new_better) = obj.fitness_curr(new_better);
         obj.personalBests.pos(new_better) = obj.positions(new_better);
         
         % Reset current fitness.
         obj.fitness_curr = NaN(obj.n, 1);
      end
      
      function [] = updateGlobalBest(obj)
         [val, idx] = max(obj.personalBests.val);
         obj.globalBest.val = val;
         obj.globalBest.idx = idx;
         obj.globalBest.pos = obj.personalBests.pos(idx);
      end
      
      function [] = updateVelPos(obj)
         % Personal and global best residuals
         p_diff = (obj.positions - obj.personalBests.pos) * (-1);
         g_diff = (obj.positions - obj.globalBest.pos) * (-1);
         
         % Update velocities and positions
         obj.velocities = obj.velocities + ...
                          obj.c_p * rand(obj.n, 1) .* p_diff + ... 
                          obj.c_g * rand(obj.n, 1) .* g_diff;
         obj.velocities(obj.velocities < obj.vel_range(1)) = ...
                                                        obj.vel_range(1);
         obj.velocities(obj.velocities > obj.vel_range(2)) = ...
                                                        obj.vel_range(2);
                                                    
         obj.positions = obj.positions + obj.velocities;
         obj.positions(obj.positions < obj.pos_range(1)) = ...
                                                        obj.pos_range(1);
         obj.positions(obj.positions > obj.pos_range(2)) = ...
                                                        obj.pos_range(2);
      end
      
   end
end
