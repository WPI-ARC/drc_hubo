function [ testSamples ] = generate_test_data_turning(turningSamples)
%GENERATE_TEST_DATA_TURNING Generates test data for turning trajectory
%samples.
%   Written for humanoids 2013 paper turning trajectories.,
noHeaderTurningSamples=turningSamples([2:size(turningSamples,1)],:);
[n,c]=size(noHeaderTurningSamples);
testDataSize = round(n*0.15);

minPitch = -pi/2;
maxPitch = pi/2;
pitch = minPitch + (maxPitch - minPitch).*rand(testDataSize,1);

minHeight = 0.7;
maxHeight = 1.2;
height = minHeight + (maxHeight - minHeight).*rand(testDataSize,1);

minTrajLength = pi/8;
maxTrajLength = pi/4;
trajLength = minTrajLength + (maxTrajLength - minTrajLength).*rand(testDataSize,1);

minDist = 0.1;
maxDist = 0.5;
dist = minDist + (maxDist - minDist).*rand(testDataSize,1);

minX = min(turningSamples([2:size(turningSamples,1)],7));
maxX = max(turningSamples([2:size(turningSamples,1)],7));
lx = minX + (maxX - minX).*rand(testDataSize,1);

minY = min(turningSamples([2:size(turningSamples,1)],8));
maxY = max(turningSamples([2:size(turningSamples,1)],8));
ly = minY + (maxY - minY).*rand(testDataSize,1);

minZ = min(turningSamples([2:size(turningSamples,1)],9));
maxZ = max(turningSamples([2:size(turningSamples,1)],9));
lz = minZ + (maxZ - minZ).*rand(testDataSize,1);

testSamples = cat(2,pitch,height,trajLength,dist,lx,ly,lz);
end

