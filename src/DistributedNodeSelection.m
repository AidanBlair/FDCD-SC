% Distributed Selection Algorithm
% Goal: Select (ideally minimal) number of nodes that's (partial)
% neighbourhood covers the entire network with no overlap
% Inputs: Each sensor knows the index of neighboring sensors
% Step 1: Each sensor broadcasts to neighbours it's own set of neighbouring
% indices and it's current selection
% Step 2: Each sensor chooses to either keep the same selection or change
% it to one of the neighboring sensors
% Step 3: Send message back to sensor that it has selected
% 
% Example broadcast message: (1, [1,2,4], 3, 3) (current selection,
% neighboring indices, weight (number of neighboring indices that have
% selected it), number of neighbours)
% Another example: (2, [1, 2, 4], 0) (current selection, neighboring
% indices, weight (0 because not selected itself), number of neighbours)
% After all broadcasts received in a time step, select neighbor with
% highest weight (number of neighbours if tied, random if that is tied)
% and set selected neighbor
% Note: At each time step, self with weight 1 is added to the pool

function [SelectedNodes, temp_neighbor_indices, currentNode] = DistributedNodeSelection(neighbor_indices)
    SelectedNodes = 0;
    Num_Sen = size(neighbor_indices, 1);
    currentNode = zeros(Num_Sen, 1);
    setSameSelected = cell(Num_Sen, 1);
    weight = zeros(Num_Sen, 1);
    numNeighbors = zeros(Num_Sen, 1);
    for s = 1:Num_Sen
        currentNode(s) = s;
        setSameSelected{s} = [s];
        weight(s) = 1;
        numNeighbors(s) = size(neighbor_indices{s}, 2);
    end
    newNode = zeros(Num_Sen, 1);
    newSetSameSelected = zeros(Num_Sen, 1);
    newWeight = zeros(Num_Sen, 1);
    for t = 1:5
        for s = 1:Num_Sen
            neighboring_weights = [];
            neighboring_tiebreakers = [];
            for i = 1:size(neighbor_indices{s}, 2)
                n = neighbor_indices{s}(i);
                neighboring_weights = [neighboring_weights, weight(n)];
                neighboring_tiebreakers = [neighboring_tiebreakers, numNeighbors(n)];
            end
            max_idx = find(neighboring_weights == max(neighboring_weights));
            for i = 1:size(neighboring_weights, 2)
                if ~ismember(i, max_idx)
                    neighboring_tiebreakers(i) = 0;
                end
            end
            if size(max_idx, 2) > 1
                tiebreaker = find(neighboring_tiebreakers == max(neighboring_tiebreakers));
                if size(tiebreaker, 2) > 1
                    max_idx = tiebreaker(1);
                else
                    max_idx = tiebreaker;
                end
            end
            if neighboring_weights(max_idx) == 0
                newNode(s) = s;
            else
                newNode(s) = neighbor_indices{s}(max_idx);
            end
        end
        for s = 1:Num_Sen
            currentNode(s) = newNode(s);
        end
        for s = 1:Num_Sen
            setSameSelected{s} = [];
            for i = 1:Num_Sen
                if currentNode(i) == s
                    setSameSelected{s} = [setSameSelected{s}, i];
                end
            end
            weight(s) = size(setSameSelected{s}, 2);
            if currentNode(s) ~= s
                weight(s) = 0;
            end
        end
    end
    SelectedNodes = unique(currentNode);
    temp_neighbor_indices = setSameSelected;
end