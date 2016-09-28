% Copyright Jacob Killelea, ASEN 2001 Lab 2, Fall 2016
function unit_vector = to_unit_vector(vector)
  % Returns the unit vector in the same direction as the given vector.
  % [u, j, k] = to_unit_vector([3, 4, 5])
  % this function is input-agnostic, and could be called as to_unit_vector({3, 4, 5})
  if isa(vector, 'cell')
    vector = cell2mat(vector)
  end

  dx = vector(1); % get values from vector
  dy = vector(2);
  dz = vector(3);

  vector_magnitude = magnitude([dx, dy, dz]);

  if vector_magnitude == 0 % if the magnitude is 0, then return the 0 vector
    unit_vector = [0, 0, 0];
    return; % exit function
  else
    x = dx/vector_magnitude;
    y = dy/vector_magnitude;
    z = dz/vector_magnitude;
    unit_vector = [x, y, z];
    return; % exit function
  end

end
