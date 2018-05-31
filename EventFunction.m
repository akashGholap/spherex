function [value,isterminal,direction] = EventFunction(~,yiq)

z = yiq(3);

value = z;

%Stop if the hit the ground
isterminal = true;

%Should only be coming to the ground from above
direction = -1;

end