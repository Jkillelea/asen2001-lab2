% Copyright Jacob Killelea, ASEN 2001 Lab 2, Fall 2016
function m = magnitude(vector)
	% calculates the magnitude of the 3D vector [a, b, c] according to to the formula sqrt(a^2 + b^2 + c^2)
	a = vector(1).^2; % square them all
	b = vector(2).^2;
	c = vector(3).^2;
	m = sqrt(a + b + c);
end
