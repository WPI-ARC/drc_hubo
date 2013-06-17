function [ validationSamples ] = generate_val_data_turning(turningSamples)
%GENERATE_VAL_DATA_TURNING Generates validation data for turning trajectory
%samples.
%   Written for humanoids 2013 paper turning trajectories.,
noHeaderTurningSamples=turningSamples([2:size(turningSamples,1)],:);
[n,c]=size(noHeaderTurningSamples);
validationDataSize = round(n*0.15);
indices = round(1 + (n-1).*rand(validationDataSize,1));
% validationSamples
% [ pitch, height, traj_length, dist, left_x, left_y, left_z ]
validationSamples = noHeaderTurningSamples(indices,[2,3,4,5,7,8,9]);
end

