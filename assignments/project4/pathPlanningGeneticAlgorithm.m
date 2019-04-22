function [] = pathPlanningGeneticAlgorithm()
%% DESCRIPTION
%  
%  Path planning of a simulated mobile robot with help of genetic ...
%  algorithms.
%  The goal is to visit every landmark in a certain order such that the ...
%  path mhasinimial length.
%
%% MISC
clc; clear all; close all;

%% PARAMETERS

n_c = 60; % Number of chromosomes
assert(mod(n_c, 2) == 0, 'n_c most be even');
n_g = 500; % Number of generations
p_cross = 0.1; % Crossover probability
p_mut = 0.01; % Mutation probability

n_landmarks = 15; % Number of landmarks to visit
map_range = [ 0 100;...
              0 100]; % Size of Map

%% Initialization

p_rand = [randi(map_range(1,:),1,n_landmarks + 2) ; ...
          randi(map_range(2,:),1,n_landmarks + 2)]; % random points on map
S = p_rand(:,1); % Start location
T = p_rand(:,end); % Target location
landmarks = p_rand(:,2:end-1); % Landmarks to visit
chromosomes = []; % Chromosomes

for i = 1:n_c 
    chromosomes = [chromosomes; randperm(n_landmarks)];
end

best_dists_hist = zeros(n_g ,1); % Save intermediate distances of best chromosome
best_idx_hist = zeros(n_g ,1); % Save intermediate indexs of best chromosome
best_path_hist = zeros(n_g, n_landmarks + 2, 2); % Save intermediate indexs of best chromosome
best_chromosom_hist = zeros(n_g, n_landmarks);% Save intermediate best chromosom
%% FUNCTION

for gen = 1:n_g
    
    % Get Path from chromosom description
    paths = chromosomes2paths(chromosomes, S, T, landmarks);
    % Calculate distance from start to target
    dists = path_dists(paths);
    % Calculate fitness from distance, shorter distance is better fitness
    fitness = 1.0 ./dists;
    fitness = fitness ./ sum(fitness);
    % Save best chromosome
    [~, best_idx] = max(fitness);
    best_chromosom = chromosomes(best_idx, :);
    best_chromosom_hist(gen, :) = best_chromosom;
    best_dists_hist(gen) = dists(best_idx);
    best_idx_hist(gen) = best_idx;
    best_path_hist(gen, :) = paths(best_idx, :);
    assert(gen ==1 || best_dists_hist(gen-1) >= dists(best_idx), 'Distance must get non strictly smaller.')
    
    % Break loop if last iteration
    if gen == n_g
        break;
    end
    
    % Select, crossover and mutation of chromosomes
    chromosomes = select_from_fitness(chromosomes, fitness);
    chromosomes = crossover(chromosomes, p_cross);
    chromosomes = mutation(chromosomes, p_mut);
    % Check if best chromosom from before is still part of new final ...
    % chromosomes
    [~, index] = ismember(best_chromosom, chromosomes,'rows');
    if index == 0
        chromosomes(randi(n_landmarks),:) = best_chromosom;
    end 
end

% Plot results
figure()
title('Results');
subplot(2,1,1)
plot(1:n_g, best_dists_hist,'k')
xlabel('Number of Generation [-]') 
ylabel('Best Path Distance of Generation [m]') 

subplot(2,1,2)
p = plot(best_path_hist(end, :, 1), best_path_hist(end, :, 2),'-*k');
drawnow
cd = [uint8(jet(n_landmarks+2)*255) uint8(ones(n_landmarks+2,1))].';
drawnow
set(p.Edge, 'ColorBinding','interpolated', 'ColorData',cd)
hold on;
plot(S(1), S(2), '-dg')
t = text(S(1)+0.1,S(2), '\leftarrow Start');
t.Color = 'g';
plot(T(1), T(2), '-dr')
t = text(T(1)+0.1, T(2), '\leftarrow End');
t.Color = 'r';




xlabel('X [m]') 
ylabel('Y [m]') 
hold off;
end

function [paths] = chromosomes2paths(chromosomes, S, T, landmarks)
    %% DESCRIPTION
    % Get Paths from chromosomes IDs
    %
    % INPUT:
    %       - chromosomes (n_chromosomes x m_genes): Chromosome IDs
    % OUTPUT:
    %       - paths ( (n_chromosomes) x (m_genes + 2) x 2): Path
    %         coordinates
    %% FUNCTION
    
    n_chrom = size(chromosomes, 1);
    n_genes = size(chromosomes, 2);
    
    % Create 3D Landmark array
    lx = reshape(landmarks(1, chromosomes), n_chrom, n_genes);
    ly = reshape(landmarks(2, chromosomes), n_chrom, n_genes);
    l = cat(3, lx, ly);
    
    % Create 3d arrays for S and T
    S_3D = repmat(S(1), n_chrom, 1);
    S_3D(:,:,2) = S(2);
    T_3D = repmat(T(1), n_chrom, 1);
    T_3D(:,:,2) = T(2);
    paths = cat(2, S_3D, l, T_3D);

end

function [survivors] = select_from_fitness(chromosomes, fitness)
    %% DESCRIPTION
    % Select chromosomes according to its fitness value.
    %
    % INPUT:
    %       - chromosomes (n_chrom x m_genes): Chromosomes
    %       - fitness (n_chrom x 1): Fitness value of chromosomes
    % OUTPUT:
    %       - survivors (n_chrom x m_genes): Selected chromosomes
        
    %% INITIALIZATION
    
    n_chroms = size(chromosomes,1);
    
    %% SANITY CHECK
    
    assert(n_chroms == size(fitness,1));
    %% FUNCTIONALLITY

    selected = randsample(n_chroms, n_chroms, true, fitness);
    survivors = chromosomes(selected, :);
    assert(isequal(size(survivors), size(chromosomes)));
    
end

function [dists] = path_dists(paths)
%% DESCRIPTION
%
% Calculate length of path according to eucildean distance.
%
% INPUT:
%       - paths (n_chromosomes x (n_landmarks + 2) x 2): Current path of robot consisting of visited points.
%
% OUTPUT:
%       - dists (n_chromosomes x 1): Length of paths.
%
%% FUNCTION

% Sanity check
assert(size(paths,3) == 2);

% Variables
n_landmarks = size(paths,2);
n_chromosomes = size(paths,1);
% p_prev = reshape(paths(:,1,:), [n_chromosomes, 2]);
p_prev = squeeze(paths(:,1,:));
dists = zeros(n_chromosomes, 1);
dist_metric = 'euclidean';

for i = 2:n_landmarks
    p_cur = squeeze(paths(:,i,:));
    d_cur = [];
    
    for  ii = 1:n_chromosomes
        d_cur = [d_cur; pdist([p_cur(ii).'; p_prev(ii).'], dist_metric)];
    end
    dists = dists + d_cur;
    p_prev = p_cur;
end

end

function [crossed_chromosomes] = crossover(chromosomes, p_cross)
%% DESCRIPTION
% Crossover of chromosomes according to probability. This step of the ...
% genetic algorithm resambles the mixing of the chromosomes at the step ...
% of the reproduction.
%
%  INPUT:
%        - chromosomes (n_chrom x m_genes): Chromosomes
%        - p_cross (1 x 1): Crossover probabiliti
%
% OUTPUT:
%        - crossed_chromosomes (n_chrom x m_genes): Chromosomes after ...
%                                                   crossover.
%

%% INITIALIZATION

n_chroms = size(chromosomes,1);
crossed_chromosomes = chromosomes;

%% SANITY CHECK

assert(0 <= p_cross && p_cross <=1, "Probability must be between 0 and 1.")

%% FUNCTIONALITY

% Determin which chromosomes are crossed over
n_crossover = round(n_chroms * p_cross);
is_odd = mod(n_crossover,2) == 1;
n_crossover = (n_crossover + (is_odd));
crossover_pairs = randsample(n_chroms, n_crossover , false);
crossover_pairs = reshape(crossover_pairs, [length(crossover_pairs)/2 2]);
no_crossover = setdiff(1:n_chroms, crossover_pairs);

% Loop through chromosome pairs and do actual crossover
for i = 1:n_crossover./2
    chrom1 = chromosomes(crossover_pairs(i,1),:);
    chrom2 = chromosomes(crossover_pairs(i,2),:);
    
    [chrom1_crossover, chrom2_crossover] = ...
                                    crossover_chrompair(chrom1, chrom2);
    crossed_chromosomes(crossover_pairs(i,1),:) = chrom1_crossover;
    crossed_chromosomes(crossover_pairs(i,2),:) = chrom2_crossover;
    
end

assert(isequal(size(crossed_chromosomes), size(chromosomes)))

end

function [chrom1_crossover, chrom2_crossover] = crossover_chrompair(chrom1, chrom2)
%% DESCRIPTION
% Crossover of chromosomes according to genetic algorithm. Chromosomes are
% splitted at rendom point an recombined to produce new chromosomes.
% Duplicated genes are not allowed and replaced with feasible genes.
%
% INPUT:
%       - chrom1 (1xN): Chromosome 1
%       - chrom2 (1xN): Chromosome 2
%
% OUTPUT:
%       - chrom1_crossover (1xN): Chromosome 1 after crossover
%       - chrom2_crossover (1xN): Chromosome 2 after crossover
%% FUNCTION

% Sanity check
assert(isequal(size(chrom1), size(chrom2)));
% Init
N = length(chrom1);
chrom1_crossover = zeros(size(chrom1));
chrom2_crossover = zeros(size(chrom2));
x_cross = randi([1 N]); % Point at which crossover happens

% Crossover
chrom1_crossover(1:x_cross) = chrom1(1:x_cross);
chrom1_crossover(x_cross+1:end) = chrom2(x_cross+1:end);
chrom2_crossover(1:x_cross) = chrom2(1:x_cross);
chrom2_crossover(x_cross+1:end) = chrom1(x_cross+1:end);

chrom1_crossover = replace_duplicates(chrom1_crossover, N);
chrom2_crossover = replace_duplicates(chrom2_crossover, N);


function [chrom] = replace_duplicates(chrom, n_chroms)
    %% Replace duplicated genes
    if length(chrom) ~= length(unique(chrom))
        % Check what genes are not used
        not_used_genes = setdiff(1:n_chroms, chrom);
        not_used_genes(randperm(length(not_used_genes))) = not_used_genes;

        % Get non unique genes
        [~, ia, ~] = unique(chrom);
        idx_non_unique = setdiff(1:n_chroms, ia);
        values_non_unique = chrom(idx_non_unique);

        % Randomly select one of the non unique genes and assign not used gene.
        for i = 1:length(values_non_unique)
           v = values_non_unique(i);
           idx_non_unique = find(ismember(chrom, v));
           assert(length(idx_non_unique) == 2);
           replace = randsample(idx_non_unique, 1);
           chrom(replace) = not_used_genes(i);
        end
    end
    
end

end

function [mutated_chromosomes] = mutation(chromosomes, p_mut)
%% DESCRIPTION
% Mutation of chromosomes with probability. A mutation is modeled by
% randomly switching two genes of the chromosome.
%
%  INPUT:
%        - chromosomes (n_chrom x m_genes): Chromosomes
%        - p_cross (1 x 1): Mutation probability
%
% OUTPUT:
%        - mutated_chromosomes (n_chrom x m_genes): Chromosomes after ...
%                                                   mutation.
%

%% INITIALIZATION

n_chroms = size(chromosomes, 1);
n_genes = size(chromosomes, 2);

mutated_chromosomes = chromosomes;

%% SANITY CHECK

assert(0 <= p_mut && p_mut <=1, "Probability must be between 0 and 1.")

%% FUNCTIONALITY

% Determin which chromosomes are mutated 
n_mu = round(n_chroms * p_mut);
idx_mut = randsample(n_chroms, n_mu , false);

% Switch the genes for the mutation chromosomes
for i = 1:n_mu
    idx = idx_mut(i);
    mut_chromosom = chromosomes(idx, :);
    gene_mut = randsample(n_genes, 2 , false);
    mut_chromosom(gene_mut) = [mut_chromosom(gene_mut(2)), ...
                                               mut_chromosom(gene_mut(1))];
    mutated_chromosomes(idx, :) = mut_chromosom;
end

assert(isequal(size(mutated_chromosomes), size(chromosomes)))

end