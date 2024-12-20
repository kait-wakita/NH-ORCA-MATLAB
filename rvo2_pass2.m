%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ORCA-MATLAB sample
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

pyrun("import pyrvo2")
pyrun("import numpy as np")
pyrun("import math")
pyrun("from pyrvo2 import *")

radius_std = 2.0;

agents(1).pos=[-200; 0];
agents(1).goal=[200; 0];
agents(1).radius = radius_std;
agents(2).pos=[200; 0];
agents(2).goal=[-200; 0];
agents(2).radius = radius_std;

close all;
figure('Position',[10 100 500 500]);
xlim([-250 250]);
ylim([-250 250]);


m_sim = pyrun("sim=RVOSimulator()","sim");
setupScenario(agents);
while(~reachedGoal(agents))
    setPreferredVelocities(agents);
    pyrun("sim.step()")
    agents = updateAgents(agents);
    visualize(agents);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function setupScenario(agents)
pyrun("sim.setTimeStep(0.25)")
pyrun("sim.setAgentDefaults(15.0, 10, 10.0, 10.0, 5.0, 2.0, np.array([0,0]))")

for i = 1:length(agents)
    pyrun("sim.addAgent(np.array(xy))",xy=agents(i).pos)
end
end

function status=reachedGoal(agents)
for i = 1:length(agents)
    if(norm(agents(i).pos-agents(i).goal)>agents(i).radius)
        status=false;
    else
        status=true;
    end
end
end

function setPreferredVelocities(agents)
for i = 1:length(agents)
    goalVector = agents(i).goal - agents(i).pos;
    if(norm(goalVector) > 1.0)
        goalVector = goalVector / norm(goalVector);
    end
    pyrun("sim.setAgentPrefVelocity(ii, np.array(gv))", ii=int32(i-1), gv=goalVector)
end
end

function agents_new = updateAgents(agents)
agents_new = agents;
for i = 1:length(agents)
    xy = pyrun("xy=sim.getAgentPosition(ii)","xy",ii=int32(i-1));
    agents_new(i).pos = double(xy)';
end
end

function visualize(agents)
clf;
hold on;
xlim([-250 250]);
ylim([-250 250]);

for i = 1:length(agents)
    x = agents(i).pos(1);
    y = agents(i).pos(2);
    r = agents(i).radius;
    rectangle('Position',[x-r,y-r,2*r,2*r],'Curvature',[1,1],'FaceColor','b')
    %fprintf("%f %f   ",x, y);
end
%fprintf("\n");
pause(0.0);

end
